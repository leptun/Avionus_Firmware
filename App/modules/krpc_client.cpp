#include "krpc_client.hpp"
#include <config.hpp>
#include <umm_malloc.h>
#include <modules/usb.hpp>
#include <krpc_cnano.h>
#include <krpc_cnano/services/krpc.h>
#include <krpc_cnano/services/space_center.h>
#include <krpc_cnano/memory.h>
#include <krpc_cnano/communication.h>
#include <retarget_locks.h>
#include <tusb.h>
#include <modules/airplane.hpp>

#include <FreeRTOS.h>
#include "semphr.h"
#include "task.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern "C" uint32_t _tinyusb_bss_run_addr[];
extern "C" uint32_t _tinyusb_data_end[];
extern "C" uint32_t _krpc_bss_run_addr[];
extern "C" uint32_t _krpc_data_end[];

static void krpc_memory_init(void);

namespace modules {
namespace krpc_client {

enum FlagDef0 {
	FLAG0_CYCLE = 0x000001,
	FLAG0_COMM_LINE_STATE = 0x000002,
};

enum FlagDef1 {
	FLAG1_COMM_RX = 0x000001,
	FLAG1_COMM_TX = 0x000002,
	FLAG1_COMM_LINE_STATE = 0x000004,
};

enum class InternalStates {
	Init = 0,
	Connected,
	Flight,
	Processing,
} state;

static TaskHandle_t px_krpc_client_TaskHandle __attribute__((section(".shared")));
static modules::airplane::Flight plane_flight __attribute__((section(".shared")));
static modules::airplane::Control plane_control __attribute__((section(".shared")));


static void task_krpc_client_Main(void *pvParameters) {
	(void) pvParameters;
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for parent task to finish initializing this task

	krpc_connection_t conn;
	krpc_SpaceCenter_Vessel_t vessel;
	krpc_SpaceCenter_Orbit_t orbit;
	krpc_SpaceCenter_CelestialBody_t body;
	krpc_SpaceCenter_ReferenceFrame_t body_frame;
	krpc_SpaceCenter_Flight_t flight;
	krpc_SpaceCenter_Control_t control;

	for (;;) {
		switch (state) {
		case InternalStates::Init: {
			krpc_close(conn);
			krpc_open(&conn, NULL);
			vTaskDelay(100);
			if (krpc_connect(conn, "Basic example")) { break; }
			krpc_schema_Status status;
			if (krpc_KRPC_GetStatus(conn, &status)) { break; }
			printf("Connected to kRPC server version %s\n", status.version);
			state = InternalStates::Connected;
		} break;
		case InternalStates::Connected: {
			krpc_KRPC_GameScene_t scene;
			if (krpc_KRPC_CurrentGameScene(conn, &scene)) { state = InternalStates::Init; break; }
			if (scene == KRPC_KRPC_GAMESCENE_FLIGHT) {
				if (krpc_SpaceCenter_ActiveVessel(conn, &vessel)) { state = InternalStates::Init; break; }
				if (krpc_SpaceCenter_Vessel_Orbit(conn, &orbit, vessel)) { state = InternalStates::Init; break; }
				if (krpc_SpaceCenter_Orbit_Body(conn, &body, orbit)) { state = InternalStates::Init; break; }
				if (krpc_SpaceCenter_CelestialBody_ReferenceFrame(conn, &body_frame, body)) { state = InternalStates::Init; break; }
//				if (krpc_SpaceCenter_Vessel_SurfaceVelocityReferenceFrame(conn, &ref_frame, vessel)) { state = InternalStates::Init; break; }
				if (krpc_SpaceCenter_Vessel_Flight(conn, &flight, vessel, body_frame)) { state = InternalStates::Init; break; }
				if (krpc_SpaceCenter_Vessel_Control(conn, &control, vessel)) { state = InternalStates::Init; break; }
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
				if (krpc_KRPC_CurrentGameScene(conn, &scene)) { state = InternalStates::Init; break; }
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
			// control
//			if (krpc_SpaceCenter_Control_set_Throttle(conn, control, plane_control.throttle)) { state = InternalStates::Init; break; }
			if (krpc_SpaceCenter_Control_set_Pitch(conn, control, plane_control.pitch)) { state = InternalStates::Init; break; }
			if (krpc_SpaceCenter_Control_set_Yaw(conn, control, plane_control.yaw)) { state = InternalStates::Init; break; }
			if (krpc_SpaceCenter_Control_set_Roll(conn, control, plane_control.roll)) { state = InternalStates::Init; break; }
//			if (krpc_SpaceCenter_Control_set_Gear(conn, control, plane_control.gear)) { state = InternalStates::Init; break; }

			// flight
			krpc_tuple_double_double_double_double_t rotation;
			if (krpc_SpaceCenter_Flight_Rotation(conn, &rotation, flight)) { state = InternalStates::Init; break; }
			plane_flight.rotation.element.x = (float)rotation.e0;
			plane_flight.rotation.element.y = (float)rotation.e1;
			plane_flight.rotation.element.z = (float)rotation.e2;
			plane_flight.rotation.element.w = (float)rotation.e3;

			double latitude;
			if (krpc_SpaceCenter_Flight_Latitude(conn, &latitude, flight)) { state = InternalStates::Init; break; }
			plane_flight.latitude = (float)latitude;

			double longitude;
			if (krpc_SpaceCenter_Flight_Longitude(conn, &longitude, flight)) { state = InternalStates::Init; break; }
			plane_flight.longitude = (float)longitude;

			double mean_altitude;
			if (krpc_SpaceCenter_Flight_MeanAltitude(conn, &mean_altitude, flight)) { state = InternalStates::Init; break; }
			plane_flight.mean_altitude = (float)mean_altitude;

			double speed;
			if (krpc_SpaceCenter_Flight_Speed(conn, &speed, flight)) { state = InternalStates::Init; break; }
			plane_flight.speed = (float)speed;

			plane_flight.latency = xTaskGetTickCount() - tickStart;

			state = InternalStates::Flight;
		} break;
		default:
			Error_Handler();
		}
	}
}
static portSTACK_TYPE krpc_client_taskStack[1024] __attribute__((aligned(1024*4))) __attribute__((section(".stack")));


void Setup() {
	krpc_memory_init();
	const TaskParameters_t krpc_clientTaskDefinition =
	{
		task_krpc_client_Main,
		"krpc",
		(configSTACK_DEPTH_TYPE)sizeof(krpc_client_taskStack) / sizeof(portSTACK_TYPE),
		NULL,
		1,
		krpc_client_taskStack,
		{
			/* Base address   Length                    Parameters */
			{ (uint32_t*)(_tinyusb_bss_run_addr), (uint32_t)_tinyusb_data_end - (uint32_t)_tinyusb_bss_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
			{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER },
			{ (uint32_t*)(_krpc_bss_run_addr), (uint32_t)_krpc_data_end - (uint32_t)_krpc_bss_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
		}
	};

	xTaskCreateRestricted(&krpc_clientTaskDefinition, &px_krpc_client_TaskHandle);
	modules::usb::GrantAccess(px_krpc_client_TaskHandle);
	retarget_locks_grant_access(NULL);
	if (xTaskNotify(px_krpc_client_TaskHandle, 0, eNoAction) != pdPASS) {
		Error_Handler();
	}
}

void Cycle() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 0, FLAG0_CYCLE, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void NotifyCommRx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_RX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void NotifyCommTx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_TX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void NotifyCommLineState() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 0, FLAG0_COMM_LINE_STATE, eSetBits) != pdPASS) {
		Error_Handler();
	}
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG1_COMM_LINE_STATE, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

}
}

/* -------------------------------------------------------------------------- */

static uint8_t krpc_heap_buf[4032] __attribute__((aligned(4)));
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
	umm_multi_free(&krpc_heap, ptr);
}

/* -------------------------------------------------------------------------- */

krpc_error_t krpc_open(krpc_connection_t *connection,
		const krpc_connection_config_t *arg) {
	for (;;) {
		if (tud_ready() && tud_cdc_n_get_line_state(0) & 0x02) {
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

krpc_error_t krpc_close(krpc_connection_t connection) {
	tud_cdc_n_read_flush(0);
	tud_cdc_n_write_clear(0);
	return KRPC_OK;
}

krpc_error_t krpc_read(krpc_connection_t connection, uint8_t *buf,
		size_t count) {
	size_t read = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(0) & 0x02)) {
			return KRPC_ERROR_CONNECTION_FAILED;
		}
		read += tud_cdc_n_read(0, buf + read, count - read);
		if (read == count) {
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::FLAG1_COMM_RX | modules::krpc_client::FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

krpc_error_t krpc_write(krpc_connection_t connection, const uint8_t *buf,
		size_t count) {
	size_t written = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(0) & 0x02)) {
			return KRPC_ERROR_CONNECTION_FAILED;
		}
		written += tud_cdc_n_write(0, buf + written, count - written);
		if (written == count) {
			tud_cdc_n_write_flush(0);
			return KRPC_OK;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::FLAG1_COMM_TX | modules::krpc_client::FLAG1_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}
