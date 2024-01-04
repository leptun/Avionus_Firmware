#include <modules/system/usb.hpp>
#include "tusb.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#define USBD_STACK_SIZE    (2*configMINIMAL_STACK_SIZE) * (CFG_TUSB_DEBUG ? 2 : 1)
#define CDC_STACK_SZIE      configMINIMAL_STACK_SIZE

extern "C" uint32_t _tinyusb_data_run_addr[];
extern "C" uint32_t _tinyusb_bss_end[];

namespace system {
namespace usb {

static void usb_device_task(void *pvParameters) {
	(void) pvParameters;

	// init device stack on configured roothub port
	// This should be called after scheduler/kernel is started.
	// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
	tud_init(BOARD_TUD_RHPORT);

	// RTOS forever loop
	while (1) {
		// put this thread to waiting state until there is new events
		tud_task();

		// following code only run if tud_task() process at least 1 event
		tud_cdc_write_flush();
	}
}
static portSTACK_TYPE usb_device_taskStack[ USBD_STACK_SIZE ] __attribute__((aligned(USBD_STACK_SIZE*4)));


static void cdc_task(void *pvParameters) {
	(void) pvParameters;

	// RTOS forever loop
	while (1) {
		// connected() check for DTR bit
		// Most but not all terminal client set this when making connection
		if ( tud_cdc_connected() )
		{
			// There are data available
			while (tud_cdc_available()) {
				uint8_t buf[64];

				// read and echo back
				uint32_t count = tud_cdc_read(buf, sizeof(buf));

				// Echo back
				// Note: Skip echo by commenting out write() and write_flush()
				// for throughput test e.g
				//    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
				tud_cdc_write(buf, count);
			}

			tud_cdc_write_flush();
		}

		// For ESP32-Sx this delay is essential to allow idle how to run and reset watchdog
		vTaskDelay(1);
	}
}
static portSTACK_TYPE cdc_taskStack[ CDC_STACK_SZIE ] __attribute__((aligned(CDC_STACK_SZIE*4)));


void Setup() {
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

	const TaskParameters_t usb_device_taskTaskDefinition =
	{
		usb_device_task,
		"usbd",
		sizeof(usb_device_taskStack) / sizeof(portSTACK_TYPE),
		NULL,
		configMAX_PRIORITIES-1,
		usb_device_taskStack,
		{
			/* Base address   Length                    Parameters */
			{ _tinyusb_data_run_addr, (uint32_t)_tinyusb_bss_end - (uint32_t)_tinyusb_data_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
			{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
		}
	};

	// Create a task for tinyusb device stack
	xTaskCreateRestricted(&usb_device_taskTaskDefinition, NULL);

	const TaskParameters_t cdc_taskTaskDefinition =
	{
		cdc_task,
		"cdc",
		sizeof(cdc_taskStack) / sizeof(portSTACK_TYPE),
		NULL,
		configMAX_PRIORITIES-2,
		cdc_taskStack,
		{
			/* Base address   Length                    Parameters */
			{ (uint32_t*)(_tinyusb_data_run_addr), (uint32_t)_tinyusb_bss_end - (uint32_t)_tinyusb_data_run_addr, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_CACHEABLE_BUFFERABLE },
			{ (uint32_t*)(USB_OTG_HS_PERIPH_BASE), 0x40000, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
		}
	};

	// Create CDC task
	xTaskCreateRestricted(&cdc_taskTaskDefinition, NULL);
}

static size_t board_get_unique_id(uint8_t id[], size_t max_len) {
	(void) max_len;
	volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
	uint32_t* id32 = (uint32_t*) (uintptr_t) id;
	uint8_t const len = 12;

	id32[0] = stm32_uuid[0];
	id32[1] = stm32_uuid[1];
	id32[2] = stm32_uuid[2];

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
