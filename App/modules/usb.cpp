#include "usb.hpp"
#include "tusb.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include <util.hpp>
#include <retarget_locks.h>
#include <stream_buffer.h>
#include <config.hpp>
#include <math.h>

#define USBD_STACK_SIZE     256 * (CFG_TUSB_DEBUG ? 2 : 1)
#define CDC_STACK_SZIE      128

extern "C" uint32_t _tinyusb_bss_run_addr[];
extern "C" uint32_t _tinyusb_data_end[];

namespace modules {
namespace usb {

enum CDCThreadFlags {
	FLAG_KRPC_RX = 0x000001,
	FLAG_KRPC_TX = 0x000002,
};

StreamBufferHandle_t krpc_rx_stream;
StreamBufferHandle_t krpc_tx_stream;

static TaskHandle_t pxcdcTaskHandle;
static TaskHandle_t pxusbdTaskHandle;

static uint32_t UUID[3];

static void usb_device_task(void *pvParameters) {
	(void) pvParameters;

	// init device stack on configured roothub port
	// This should be called after scheduler/kernel is started.
	// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
	tud_init(BOARD_TUD_RHPORT);

	vGrantAccessToStreamBuffer(NULL, krpc_rx_stream);
	vGrantAccessToStreamBuffer(NULL, krpc_tx_stream);
	vGrantAccessToTask(NULL, pxcdcTaskHandle);
	retarget_locks_grant_access(NULL);
	vCloneAccessToKernelObjects(pxcdcTaskHandle, NULL);

	// start the cdc task
	if (xTaskNotify(pxcdcTaskHandle, 0, eNoAction) != pdPASS) {
		Error_Handler();
	}

	portSWITCH_TO_USER_MODE();

	// RTOS forever loop
	while (1) {
		// put this thread to waiting state until there is new events
		tud_task();

		// following code only run if tud_task() process at least 1 event
		tud_cdc_write_flush();
	}
}
static portSTACK_TYPE usb_device_taskStack[ USBD_STACK_SIZE ] __attribute__((aligned(USBD_STACK_SIZE*4))) __attribute__((section(".stack")));


static void cdc_task(void *pvParameters) {
	(void) pvParameters;

	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for parent task to finish initializing this task

	uint8_t buf[64];
	for (;;) {
		uint32_t pendingFlags = 0;
		util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_KRPC_RX | FLAG_KRPC_TX, &pendingFlags, portMAX_DELAY);
		if (pendingFlags & FLAG_KRPC_RX) {
			size_t count = tud_cdc_read(buf, std::min(sizeof(buf), xStreamBufferSpacesAvailable(krpc_tx_stream)));
			if (xStreamBufferSend(krpc_rx_stream, buf, count, 0) != count) {
				Error_Handler();
			}
		}
		if (pendingFlags & FLAG_KRPC_TX) {
			size_t count = xStreamBufferReceive(krpc_tx_stream, buf, std::min(sizeof(buf), (size_t)tud_cdc_write_available()), 0);
			tud_cdc_write(buf, count);
			tud_cdc_write_flush();
			if (xStreamBufferBytesAvailable(krpc_tx_stream) > 0) {
				if (xTaskNotifyIndexed(NULL, 1, FLAG_KRPC_TX, eSetBits) != pdPASS) {
					Error_Handler();
				}
			}
		}
	}
}
static portSTACK_TYPE cdc_taskStack[ CDC_STACK_SZIE ] __attribute__((aligned(CDC_STACK_SZIE*4))) __attribute__((section(".stack")));

extern "C"
void tud_cdc_rx_cb(uint8_t itf) {
	if (xTaskNotifyIndexed(pxcdcTaskHandle, 1, FLAG_KRPC_RX, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

static void krpc_rx_callback(StreamBufferHandle_t xStreamBuffer, BaseType_t xIsInsideISR, BaseType_t * const pxHigherPriorityTaskWoken) {
	if (xIsInsideISR) {
		if (xTaskNotifyIndexedFromISR(pxcdcTaskHandle, 1, FLAG_KRPC_RX, eSetBits, pxHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	} else {
		if (xTaskNotifyIndexed(pxcdcTaskHandle, 1, FLAG_KRPC_RX, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
}

static void krpc_tx_callback(StreamBufferHandle_t xStreamBuffer, BaseType_t xIsInsideISR, BaseType_t * const pxHigherPriorityTaskWoken) {
	if (xIsInsideISR) {
		if (xTaskNotifyIndexedFromISR(pxcdcTaskHandle, 1, FLAG_KRPC_TX, eSetBits, pxHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	} else {
		if (xTaskNotifyIndexed(pxcdcTaskHandle, 1, FLAG_KRPC_TX, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
}

void Setup() {
	memcpy(UUID, (const uint32_t*)UID_BASE, sizeof(UUID));

	LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**USB_OTG_HS GPIO Configuration
	PA4     ------> USB_OTG_HS_SOF
	PB13     ------> USB_OTG_HS_VBUS
	PB14     ------> USB_OTG_HS_DM
	PB15     ------> USB_OTG_HS_DP
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USB_OTG_HS clock enable */
	__HAL_RCC_USB_OTG_HS_CLK_ENABLE();

	/* USB_OTG_HS interrupt Init */
	HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(OTG_HS_IRQn);

	krpc_rx_stream = xStreamBufferCreateWithCallback(config::usb_stream_buffer_size, 0, NULL, krpc_rx_callback);
	krpc_tx_stream = xStreamBufferCreateWithCallback(config::usb_stream_buffer_size, 0, krpc_tx_callback, NULL);

	const TaskParameters_t cdc_taskTaskDefinition =
	{
		cdc_task,
		"cdc",
		(configSTACK_DEPTH_TYPE)sizeof(cdc_taskStack) / sizeof(portSTACK_TYPE),
		NULL,
		configMAX_PRIORITIES-2,
		cdc_taskStack,
		{
			/* Base address   Length                    Parameters */
			{ (uint32_t*)(_tinyusb_bss_run_addr), (uint32_t)_tinyusb_data_end - (uint32_t)_tinyusb_bss_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
			{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER },
		}
	};

	// Create CDC task
	xTaskCreateRestricted(&cdc_taskTaskDefinition, &pxcdcTaskHandle);

	const TaskParameters_t usb_device_taskTaskDefinition =
	{
		usb_device_task,
		"usbd",
		(configSTACK_DEPTH_TYPE)sizeof(usb_device_taskStack) / sizeof(portSTACK_TYPE),
		NULL,
		(configMAX_PRIORITIES-1) | portPRIVILEGE_BIT,
		usb_device_taskStack,
		{
			/* Base address   Length                    Parameters */
			{ _tinyusb_bss_run_addr, (uint32_t)_tinyusb_data_end - (uint32_t)_tinyusb_bss_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
			{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER },
		}
	};

	// Create a task for tinyusb device stack
	xTaskCreateRestricted(&usb_device_taskTaskDefinition, &pxusbdTaskHandle);
}

static size_t board_get_unique_id(uint8_t id[], size_t max_len) {
	(void) max_len;
	uint32_t* id32 = (uint32_t*) (uintptr_t) id;
	uint8_t const len = 12;

	id32[0] = UUID[0];
	id32[1] = UUID[1];
	id32[2] = UUID[2];

	return len;
}

// Get USB Serial number string from unique ID if available. Return number of character.
// Input is string descriptor from index 1 (index 0 is type + len)
extern "C" size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars) {
	uint8_t uid[16] TU_ATTR_ALIGNED(4);
	size_t uid_len;

	uid_len = board_get_unique_id(uid, sizeof(uid));

	if ( uid_len > max_chars / 2 ) uid_len = max_chars / 2;

	for ( size_t i = 0; i < uid_len; i++ ) {
		for ( size_t j = 0; j < 2; j++ ) {
			const char nibble_to_hex[16] = {
				'0', '1', '2', '3', '4', '5', '6', '7',
				'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
			};
			uint8_t const nibble = (uid[i] >> (j * 4)) & 0xf;
			desc_str1[i * 2 + (1 - j)] = nibble_to_hex[nibble]; // UTF-16-LE
		}
	}

	return 2 * uid_len;
}

extern "C"
void OTG_HS_IRQHandler(void) {
	tud_int_handler(1);
}



}
}
