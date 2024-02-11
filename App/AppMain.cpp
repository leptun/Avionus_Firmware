#include "AppMain.hpp"
#include "hw/exti.hpp"
#include "hw/clock.hpp"
#include "hw/usart.hpp"
#include "hw/adc.hpp"
#include "hw/power.hpp"
#include "hw/usb.hpp"
#include <fatfs.h>
#include "Logging.hpp"
#include "modules/gps.hpp"
#include "modules/servo.hpp"
#include "modules/krpc_client.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

namespace AppMain {

static void taskAppMain(void *pvParameters) {
	hw::exti::Setup();
	hw::clock::Setup();
	puts("start");
	hw::usart::usart3.Setup();
	hw::usart::usart3.send((const uint8_t *)"test1", 5);
	hw::usart::usart3.send((const uint8_t *)"test2", 5);
	hw::usart::uart4.Setup();
	hw::adc::Setup();
	hw::power::Setup();
	hw::usb::Setup();
	fatfs_Init();
	Logging::Setup();
	modules::gps::Setup();
	modules::servo::Setup();
	modules::krpc_client::Setup();

	for (;;) {
		modules::krpc_client::Cycle();
		vTaskDelay(50);
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
	xTaskCreateRestricted(&xAppMainTaskDefinition, NULL);
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
