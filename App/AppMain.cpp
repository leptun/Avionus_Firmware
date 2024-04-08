#include "AppMain.hpp"
#include "hw/exti.hpp"
#include "hw/usart.hpp"
#include "hw/clock.hpp"
#include "hw/adc.hpp"
#include "hw/power.hpp"
#include "hw/usb.hpp"
#include "modules/module_manager.hpp"
#include "modules/modules.hpp"
#include "modules/logging.hpp"

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

	// servo test
	modules::sv.SetPosition(0, 1000);
	modules::sv.SetPosition(1, 1100);
	modules::sv.SetPosition(2, 1200);
	modules::sv.SetPosition(3, 1300);
	modules::sv.SetPosition(4, 1400);
	modules::sv.SetPosition(5, 1500);
	modules::sv.SetPosition(6, 1600);
	modules::sv.SetPosition(7, 1700);
	modules::sv.SetPosition(8, 1800);
	modules::sv.SetPosition(9, 1900);
	modules::sv.SetPosition(10, 2000);
	modules::sv.SetPosition(11, 2100);
	modules::sv.SetPosition(12, 2200);
	modules::sv.SetPosition(13, 2300);
	modules::sv.SetPosition(14, 2400);
	modules::sv.SetPosition(15, 2500);

	// sd test
//	hw::power::D3.Enable();
//	modules::logging::Setup();

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
static const TaskParameters_t xAppMainTaskDefinition = {
	taskAppMain,
	"AppMain",
    (configSTACK_DEPTH_TYPE)sizeof(xAppMainTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    0 | portPRIVILEGE_BIT,
	xAppMainTaskStack,
    {
        /* Base address   Length                    Parameters */
//        { (uint32_t*)(AHB1PERIPH_BASE), 0x400 * 8, portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER | (0b11101111 << MPU_RASR_SRD_Pos) },
    }
};

void Setup() {
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
