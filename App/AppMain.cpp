#include "AppMain.hpp"
#include "modules/clock.hpp"
#include "modules/adc.hpp"
#include "modules/power.hpp"
#include "modules/usb.hpp"
#include "modules/blink.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

void taskAppMain(void *pvParameters) {
	modules::clock::Setup();
	modules::adc::Setup();
	modules::power::Setup();
	modules::usb::Setup();
	modules::blink::Setup();

	for (;;) {
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xAppMainTaskStack[ 128 ] __attribute__((aligned(128*4))) __attribute__((section(".stack")));
static const TaskParameters_t xAppMainTaskDefinition =
{
	taskAppMain,
	"main",
    sizeof(xAppMainTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    0 | portPRIVILEGE_BIT,
	xAppMainTaskStack,
    {
        /* Base address   Length                    Parameters */
//        { (uint32_t*)(AHB1PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
    }
};

void Setup() {
	xTaskCreateRestricted(&xAppMainTaskDefinition, NULL);
	LL_DBGMCU_APB1_GRP1_FreezePeriph(
			LL_DBGMCU_APB1_GRP1_TIM2_STOP |
			LL_DBGMCU_APB1_GRP1_TIM3_STOP |
			LL_DBGMCU_APB1_GRP1_TIM4_STOP |
			LL_DBGMCU_APB1_GRP1_TIM5_STOP |
			LL_DBGMCU_APB1_GRP1_TIM6_STOP |
			LL_DBGMCU_APB1_GRP1_RTC_STOP |
			LL_DBGMCU_APB1_GRP1_IWDG_STOP |
			LL_DBGMCU_APB1_GRP1_I2C2_STOP
	);
	LL_DBGMCU_APB2_GRP1_FreezePeriph(
			LL_DBGMCU_APB2_GRP1_TIM1_STOP |
			LL_DBGMCU_APB2_GRP1_TIM9_STOP |
			LL_DBGMCU_APB2_GRP1_TIM11_STOP
	);
}

}
