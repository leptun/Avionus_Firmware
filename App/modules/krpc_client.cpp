#include "krpc_client.hpp"
#include <config.hpp>
#include <umm_malloc.h>
#include <modules/usb.hpp>
#include <krpc_cnano.h>
#include <krpc_cnano/services/krpc.h>
#include <krpc_cnano/memory.h>
#include <krpc_cnano/communication.h>
#include <retarget_locks.h>
#include <tusb.h>

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

enum FlagDef {
	FLAG_COMM_RX = 0x000001,
	FLAG_COMM_TX = 0x000002,
};

static TaskHandle_t px_krpc_client_TaskHandle;

static void task_krpc_client_Main(void *pvParameters) {
	(void) pvParameters;
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for parent task to finish initializing this task

	krpc_connection_t conn;
	krpc_open(&conn, NULL);
	krpc_connect(conn, "Basic example");
	krpc_schema_Status status;
	krpc_KRPC_GetStatus(conn, &status);
	printf("Connected to kRPC server version %s\n", status.version);

	for (;;) { vTaskDelay(1000); }
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

	// Create CDC task
	xTaskCreateRestricted(&krpc_clientTaskDefinition, &px_krpc_client_TaskHandle);
	modules::usb::GrantAccess(px_krpc_client_TaskHandle);
	retarget_locks_grant_access(NULL);
	if (xTaskNotify(px_krpc_client_TaskHandle, 0, eNoAction) != pdPASS) {
		Error_Handler();
	}
}

void NotifyCommRx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG_COMM_RX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

void NotifyCommTx() {
	if (px_krpc_client_TaskHandle && xTaskNotifyIndexed(px_krpc_client_TaskHandle, 1, FLAG_COMM_TX, eSetBits) != pdPASS) {
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
	cdc_line_coding_t config;
	for (;;) {
		tud_cdc_n_get_line_coding(0, &config);
		if (config.bit_rate == 69) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	return KRPC_OK;
}

krpc_error_t krpc_close(krpc_connection_t connection) {
	return KRPC_OK;
}

krpc_error_t krpc_read(krpc_connection_t connection, uint8_t *buf,
		size_t count) {
	size_t read = 0;
	while (true) {
		read += tud_cdc_n_read(0, buf + read, count - read);
		if (read == count)
			break;
		else
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::FLAG_COMM_RX, NULL, portMAX_DELAY);
	}
	return KRPC_OK;
}

krpc_error_t krpc_write(krpc_connection_t connection, const uint8_t *buf,
		size_t count) {
	size_t written = 0;
	while (true) {
		written += tud_cdc_n_write(0, buf + written, count - written);
		if (written == count)
			break;
		else
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, modules::krpc_client::FLAG_COMM_RX, NULL, portMAX_DELAY);
	}
	tud_cdc_n_write_flush(0);
	return KRPC_OK;
}
