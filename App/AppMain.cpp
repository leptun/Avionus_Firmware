#include "AppMain.hpp"
#include "modules/blink.hpp"
#include "modules/system/clock.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

void taskAppMain(void *pvParameters) {
	system::clock::Setup();
	blink::Setup();

	for (;;) {
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xAppMainTaskStack[ 128 ] __attribute__((aligned(128*4)));
static TaskParameters_t xAppMainTaskDefinition =
{
	taskAppMain,
	"main",
    sizeof(xAppMainTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    0 | portPRIVILEGE_BIT,
	xAppMainTaskStack,
    {
        /* Base address   Length                    Parameters */
//        { (uint32_t*)AHB1PERIPH_BASE, 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
    }
};

void Setup() {
	xTaskCreateRestricted(&xAppMainTaskDefinition, NULL);
}

}
