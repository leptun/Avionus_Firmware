#include "AppMain.hpp"
#include "hw/exti.hpp"
#include "hw/usart.hpp"
#include "hw/clock.hpp"
#include "hw/adc.hpp"
#include "hw/power.hpp"
#include "hw/usb.hpp"
#include "modules/module_manager.hpp"
#include "modules/modules.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

namespace AppMain {

TaskHandle_t pxTaskHandle;

static void taskAppMain(void *pvParameters) {
	hw::power::Setup();
	hw::exti::Setup();
	hw::clock::Setup();
	puts("start");
	hw::adc::Setup();
	hw::usb::Setup();
	modules::Init();
	modules::sv.SetPosition(15, 1500);

	for (;;) {
		uint32_t pulNotificationValue;
		if (xTaskNotifyWaitIndexed(0, 0, Flags::FLAG_MODULES, &pulNotificationValue, portMAX_DELAY) != pdPASS) {
			Error_Handler();
		}
		if (pulNotificationValue & Flags::FLAG_MODULES) {
			modules::Update();
		}
	}
}
static portSTACK_TYPE xAppMainTaskStack[ 256 ] __attribute__((aligned(256*4))) __attribute__((section(".stack")));

void Setup() {
	const TaskParameters_t xAppMainTaskDefinition = {
		taskAppMain,
		"main",
	    (configSTACK_DEPTH_TYPE)sizeof(xAppMainTaskStack) / sizeof(portSTACK_TYPE),
	    NULL,
	    0 | portPRIVILEGE_BIT,
		xAppMainTaskStack,
	    {
	        /* Base address   Length                    Parameters */
	//        { (uint32_t*)(AHB1PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
	    }
	};
	xTaskCreateRestricted(&xAppMainTaskDefinition, &pxTaskHandle);
	LL_DBGMCU_APB1_GRP1_FreezePeriph(
//			LL_DBGMCU_APB1_GRP1_TIM2_STOP |
//			LL_DBGMCU_APB1_GRP1_TIM3_STOP |
//			LL_DBGMCU_APB1_GRP1_TIM4_STOP |
			LL_DBGMCU_APB1_GRP1_TIM5_STOP |
			LL_DBGMCU_APB1_GRP1_TIM6_STOP |
//			LL_DBGMCU_APB1_GRP1_RTC_STOP |
			LL_DBGMCU_APB1_GRP1_IWDG_STOP |
			LL_DBGMCU_APB1_GRP1_I2C2_STOP
	);
	LL_DBGMCU_APB2_GRP1_FreezePeriph(
//			LL_DBGMCU_APB2_GRP1_TIM1_STOP |
			LL_DBGMCU_APB2_GRP1_TIM9_STOP |
			LL_DBGMCU_APB2_GRP1_TIM11_STOP
	);
}

}
