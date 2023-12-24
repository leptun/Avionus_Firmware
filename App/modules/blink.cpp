#include "blink.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace blink {

void taskBlinkMain(void *pvParameters) {
	for (;;) {
		HAL_GPIO_TogglePin(UI_LED_GPS_GPIO_Port, UI_LED_GPS_Pin);
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xBlinkTaskStack[ 128 ] __attribute__((aligned(128*4)));
static const TaskParameters_t xBlinkTaskDefinition =
{
    taskBlinkMain,
	"Blinky",
    sizeof(xBlinkTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    1,
	xBlinkTaskStack,
    {
        /* Base address   Length                    Parameters */
        { (uint32_t*)(AHB1PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) }, //GPIOE
    }
};

void Setup() {
	xTaskCreateRestricted(&xBlinkTaskDefinition, NULL);
}

}
