#include "clock.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace system {
namespace clock {

static TaskHandle_t pxClockTaskHandle;

enum ClockThreadFlags {


};

void taskClockMain(void *pvParameters) {
	TaskHandle_t callingTaskHandle = (TaskHandle_t)pvParameters;

	vTaskDelay(500);

	if (xTaskNotify(callingTaskHandle, 0, eNoAction) != pdPASS)
		Error_Handler();

	for (;;) {
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xClockTaskStack[512] __attribute__((aligned(512*4)));
static TaskParameters_t xClockTaskDefinition =
{
	taskClockMain,
	"sys.clock",
    sizeof(xClockTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    0,
	xClockTaskStack,
    {
        /* Base address   Length                    Parameters */
//        { (uint32_t*)AHB1PERIPH_BASE, 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
    }
};

void Setup() {
	xClockTaskDefinition.pvParameters = xTaskGetCurrentTaskHandle();
	xTaskCreateRestricted(&xClockTaskDefinition, &pxClockTaskHandle);
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
}

}
}
