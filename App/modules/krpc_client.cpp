#include "krpc_client.hpp"
#include <config.hpp>
#include <umm_malloc.h>
#include <hw/usb.hpp>
#include <krpc_cnano.h>
#include <krpc_cnano/services/krpc.h>
#include <krpc_cnano/services/space_center.h>
#include <krpc_cnano/services/lidar.h>
#include <krpc_cnano/memory.h>
#include <krpc_cnano/communication.h>
#include <retarget_locks.h>
#include <tusb.h>
#include <defs.hpp>
#include <regions.h>

#include <FreeRTOS.h>
#include "semphr.h"
#include "task.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern "C" uint32_t _tinyusb_bss_run_addr[];
extern "C" uint32_t _krpc_bss_run_addr[];
extern "C" uint32_t _shared_bss_run_addr[];

#define KRPC_TEST(x) if (x) { state = InternalStates::Init; break; }

#define KRPC_BULK_START for (bulkState = BulkState::SEND; bulkState != BulkState::DISABLED; bulkState = (BulkState)((int)bulkState - 1)) { krpc_error_t ret;
#define KRPC_BULK_END } if (bulkState != BulkState::DISABLED) { bulkState = BulkState::DISABLED; state = InternalStates::Init; break; }
#define KRPC_BULK_TEST(x) if (((ret = x)) && ret != KRPC_ERROR_DECODING_FAILED) { break; }

static void krpc_memory_init(void);

namespace modules {
namespace krpc_client {

static TaskHandle_t px_krpc_client_TaskHandle __attribute__((section(".shared")));

void KrpcClient::task_krpc_client_Main_StaticWrapper(void *pvParameters) {
	(void) pvParameters;
	KrpcClient *_this = (KrpcClient *)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	_this->task_krpc_client_Main();
}

void KrpcClient::task_krpc_client_Main() {
	krpc_connection_t conn = (krpc_connection_t)this;
	krpc_SpaceCenter_Vessel_t vessel;
	krpc_LiDAR_Laser_t laser;
	krpc_SpaceCenter_Orbit_t orbit;
	krpc_SpaceCenter_CelestialBody_t body;
	krpc_SpaceCenter_ReferenceFrame_t body_frame;
	krpc_SpaceCenter_ReferenceFrame_t ref_frame;
	krpc_SpaceCenter_ReferenceFrame_t surface_ref_frame;
	krpc_SpaceCenter_ReferenceFrame_t flight_ref_frame;
	krpc_SpaceCenter_Flight_t flight;
	krpc_SpaceCenter_Control_t control;

	for (;;) {
		switch (state) {
		case InternalStates::Init: {
			::krpc_close(conn);
			::krpc_open(&conn, NULL);
			vTaskDelay(100);
			if (krpc_connect(conn, "Basic example")) { break; }
			krpc_schema_Status status;
			if (krpc_KRPC_GetStatus(conn, &status)) { break; }
			printf("Connected to kRPC server version %s\n", status.version);
			state = InternalStates::Connected;
		} break;
		case InternalStates::Connected: {
			krpc_KRPC_GameScene_t scene;
			KRPC_TEST(krpc_KRPC_CurrentGameScene(conn, &scene));
			if (scene == KRPC_KRPC_GAMESCENE_FLIGHT) {
				KRPC_TEST(krpc_SpaceCenter_ActiveVessel(conn, &vessel));
				{
					krpc_SpaceCenter_Parts_t vessel_parts;
					KRPC_TEST(krpc_SpaceCenter_Vessel_Parts(conn, &vessel_parts, vessel));
					krpc_list_object_t vessel_parts_lasers = { 0 };
					KRPC_TEST(krpc_SpaceCenter_Parts_WithName(conn, &vessel_parts_lasers, vessel_parts, "distometer100x"));
					KRPC_TEST(vessel_parts_lasers.size == 0);
					krpc_SpaceCenter_Part_t laser_part = vessel_parts_lasers.items[0];
					krpc_free(vessel_parts_lasers.items);
					KRPC_TEST(krpc_LiDAR_Laser(conn, &laser, laser_part));
				}
				KRPC_TEST(krpc_SpaceCenter_Vessel_Orbit(conn, &orbit, vessel));
				KRPC_TEST(krpc_SpaceCenter_Orbit_Body(conn, &body, orbit));
				KRPC_TEST(krpc_SpaceCenter_CelestialBody_ReferenceFrame(conn, &body_frame, body));
				KRPC_TEST(krpc_SpaceCenter_Vessel_ReferenceFrame(conn, &ref_frame, vessel));
				KRPC_TEST(krpc_SpaceCenter_Vessel_SurfaceReferenceFrame(conn, &surface_ref_frame, vessel));
				KRPC_TEST(krpc_SpaceCenter_ReferenceFrame_CreateHybrid(conn, &flight_ref_frame, body_frame, surface_ref_frame, body_frame, body_frame));
				KRPC_TEST(krpc_SpaceCenter_Vessel_Flight(conn, &flight, vessel, flight_ref_frame));
				KRPC_TEST(krpc_SpaceCenter_Vessel_Control(conn, &control, vessel));
				state = InternalStates::Flight;
			}
			else {
				vTaskDelay(1000);
			}
		} break;
		case InternalStates::Flight: {
			uint32_t flags = 0;
			if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG0_CYCLE | FLAG0_COMM_LINE_STATE, &flags, pdMS_TO_TICKS(1000)) == pdFALSE) {
				krpc_KRPC_GameScene_t scene;
				KRPC_TEST(krpc_KRPC_CurrentGameScene(conn, &scene));
				if (scene != KRPC_KRPC_GAMESCENE_FLIGHT) {
					state = InternalStates::Connected;
					break;
				}
			}
			else {
				if (flags & FLAG0_COMM_LINE_STATE) {
					if (!(tud_ready() && tud_cdc_n_get_line_state(0) & 0x02)) {
						state = InternalStates::Init;
						break;
					}
				}
				if (flags & FLAG0_CYCLE) {
					state = InternalStates::Processing;
					break;
				}
			}
		} break;
		case InternalStates::Processing: {
			uint32_t tickStart = xTaskGetTickCount();
			double latitude;
			double longitude;
			double mean_altitude;
			double speed;
			krpc_list_double_t laser_point_cloud = { 0 };
			// control
			KRPC_BULK_START;
//			KRPC_BULK_TEST(krpc_SpaceCenter_Control_set_Throttle(conn, control, plane_control.throttle));
			KRPC_BULK_TEST(krpc_SpaceCenter_Control_set_Pitch(conn, control, plane_control.pitch));
			KRPC_BULK_TEST(krpc_SpaceCenter_Control_set_Yaw(conn, control, plane_control.yaw));
			KRPC_BULK_TEST(krpc_SpaceCenter_Control_set_Roll(conn, control, plane_control.roll));
//			KRPC_BULK_TEST(krpc_SpaceCenter_Control_set_Gear(conn, control, plane_control.gear));

			// flight
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Pitch(conn, &plane_flight.pitch, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Heading(conn, &plane_flight.heading, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Roll(conn, &plane_flight.roll, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Latitude(conn, &latitude, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Longitude(conn, &longitude, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_MeanAltitude(conn, &mean_altitude, flight));
			KRPC_BULK_TEST(krpc_SpaceCenter_Flight_Speed(conn, &speed, flight));
			KRPC_BULK_TEST(krpc_LiDAR_Laser_Cloud(conn, &laser_point_cloud, laser));
			KRPC_BULK_END;

			plane_flight.latitude = (float)latitude;
			plane_flight.longitude = (float)longitude;
			plane_flight.mean_altitude = (float)mean_altitude;
			plane_flight.speed = (float)speed;
			if (laser_point_cloud.size > 0) {
				plane_flight.tof_distance = float(laser_point_cloud.items[0]);
			}
			krpc_free(laser_point_cloud.items);

			plane_flight.latency = xTaskGetTickCount() - tickStart;

			state = InternalStates::Flight;
		} break;
		default:
			Error_Handler();
		}
	}
}
static portSTACK_TYPE krpc_client_taskStack[1024] __attribute__((aligned(1024*4))) __attribute__((section(".stack")));
const TaskParameters_t KrpcClient::krpc_clientTaskDefinition =
{
	task_krpc_client_Main_StaticWrapper,
	"krpc",
	(configSTACK_DEPTH_TYPE)sizeof(krpc_client_taskStack) / sizeof(portSTACK_TYPE),
	NULL,
	1,
	krpc_client_taskStack,
	{
		/* Base address   Length                    Parameters */
		{ _tinyusb_bss_run_addr, __tinyusb_data_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_SRAM) },
		{ _krpc_bss_run_addr, __krpc_data_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_SRAM) },
		{ _shared_bss_run_addr, __shared_region_size__, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | CACHE_CONF(configTEX_S_C_B_TCMRAM) },
		{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER },
	}
};

void KrpcClient::Init() {
	krpc_memory_init();

	xTaskCreateRestricted(&krpc_clientTaskDefinition, &px_krpc_client_TaskHandle);
	hw::usb::GrantAccess(px_krpc_client_TaskHandle);
	retarget_locks_grant_access(px_krpc_client_TaskHandle);
	if (xTaskNotify(px_krpc_client_TaskHandle, (uint32_t)this, eSetValueWithOverwrite) != pdPASS) {
		Error_Handler();
	}
}

bool KrpcClient::Update() {
	return false;
}

void KrpcClient::Cycle() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 0, FLAG0_CYCLE, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void KrpcClient::NotifyCommRx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_RX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void KrpcClient::NotifyCommTx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_TX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void KrpcClient::NotifyCommLineState() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 0, FLAG0_COMM_LINE_STATE, eSetBits) != pdPASS) {
		Error_Handler();
	}
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_LINE_STATE, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

krpc_error_t KrpcClient::krpc_open() {
	for (;;) {
		if (tud_ready() && tud_cdc_n_get_line_state(0) & 0x02) {
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::KrpcClient::FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

krpc_error_t KrpcClient::krpc_close() {
	if (tud_ready()) {
		tud_cdc_n_read_flush(0);
		tud_cdc_n_write_clear(0);
	}
	return KRPC_OK;
}

krpc_error_t KrpcClient::krpc_read(uint8_t *buf, size_t count) {
	if (bulkState == BulkState::SEND) {
		return KRPC_ERROR_IO;
	}
	while (tud_cdc_n_write_available(0) != CFG_TUD_CDC_TX_BUFSIZE) {
		tud_cdc_n_write_flush(0);
	}
	size_t read = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(0) & 0x02)) {
			return KRPC_ERROR_CONNECTION_FAILED;
		}
		read += tud_cdc_n_read(0, buf + read, count - read);
		if (read == count) {
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG1_COMM_RX | FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

krpc_error_t KrpcClient::krpc_write(const uint8_t *buf, size_t count) {
	if (bulkState == BulkState::RECEIVE) {
		return KRPC_OK;
	}
	size_t written = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(0) & 0x02)) {
			return KRPC_ERROR_CONNECTION_FAILED;
		}
		written += tud_cdc_n_write(0, buf + written, count - written);
		if (written == count) {
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG1_COMM_TX | FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

}
}

/* -------------------------------------------------------------------------- */

static uint8_t krpc_heap_buf[4000] __attribute__((aligned(4)));
static umm_heap krpc_heap;

static void krpc_memory_init(void) {
	umm_multi_init_heap(&krpc_heap, krpc_heap_buf, sizeof(krpc_heap_buf));
}

void* krpc_malloc(size_t size) {
	return umm_multi_malloc(&krpc_heap, size);
}

void* krpc_calloc(size_t num, size_t size) {
	return umm_multi_calloc(&krpc_heap, num, size);
}

void* krpc_recalloc(void *ptr, size_t num, size_t inc, size_t size) {
	assert(inc > 0);
	ptr = umm_multi_realloc(&krpc_heap, ptr, (num + inc) * size);
	memset(((uint8_t*) ptr) + (num * size), 0, (inc * size));
	return ptr;
}

void krpc_free(void *ptr) {
	if (ptr)
		umm_multi_free(&krpc_heap, ptr);
}

/* -------------------------------------------------------------------------- */

krpc_error_t krpc_open(krpc_connection_t *connection, const krpc_connection_config_t *arg) {
	modules::krpc_client::KrpcClient *_this = (modules::krpc_client::KrpcClient *)*connection;
	return _this->krpc_open();
}

krpc_error_t krpc_close(krpc_connection_t connection) {
	modules::krpc_client::KrpcClient *_this = (modules::krpc_client::KrpcClient *)connection;
	return _this->krpc_close();
}

krpc_error_t krpc_read(krpc_connection_t connection, uint8_t *buf, size_t count) {
	modules::krpc_client::KrpcClient *_this = (modules::krpc_client::KrpcClient *)connection;
	return _this->krpc_read(buf, count);
}

krpc_error_t krpc_write(krpc_connection_t connection, const uint8_t *buf, size_t count) {
	modules::krpc_client::KrpcClient *_this = (modules::krpc_client::KrpcClient *)connection;
	return _this->krpc_write(buf, count);
}


