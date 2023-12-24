#include "clock.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "../../util.hpp"

namespace system {
namespace clock {

static TaskHandle_t pxClockTaskHandle;
static bool cssEnabled = false;

enum ClockThreadFlags {
	FLAG_LSIRDY = 0x000001,
	FLAG_LSERDY = 0x000002,
	FLAG_HSIRDY = 0x000004,
	FLAG_HSERDY = 0x000008,
	FLAG_PLLRDY = 0x000010,
	FLAG_PLLI2SRDY = 0x000020,
	FLAG_PLLSAIRDY = 0x000040,
	FLAG_CSS = 0x000080,
};

void taskClockMain(void *pvParameters) {
	TaskHandle_t callingTaskHandle = (TaskHandle_t)pvParameters;

	uint32_t retFlags;
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;

	// Enable backup SRAM regulator
	if (!LL_PWR_IsEnabledBkUpRegulator()) {
		LL_PWR_EnableBkUpAccess();
		LL_PWR_EnableBkUpRegulator();
		LL_PWR_DisableBkUpAccess();
	}

	// Set flash latency
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(2);
	while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_7) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	// Set voltage scaling
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

	// Enable overdrive mode
	LL_PWR_EnableOverDriveMode();
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(100);
	while (LL_PWR_IsActiveFlag_OD() == 0) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }
	LL_PWR_EnableOverDriveSwitching();
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(100);
	while (LL_PWR_IsActiveFlag_ODSW() == 0) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	// Set prescalers
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

	// Setup RCC interrupts
	LL_RCC_ClearFlag_LSIRDY();
	LL_RCC_EnableIT_LSIRDY();
	LL_RCC_ClearFlag_LSERDY();
	LL_RCC_EnableIT_LSERDY();
	LL_RCC_ClearFlag_HSIRDY();
	LL_RCC_EnableIT_HSIRDY();
	LL_RCC_ClearFlag_HSERDY();
	LL_RCC_EnableIT_HSERDY();
	LL_RCC_ClearFlag_PLLRDY();
	LL_RCC_EnableIT_PLLRDY();
	LL_RCC_ClearFlag_PLLI2SRDY();
	LL_RCC_EnableIT_PLLI2SRDY();
	LL_RCC_ClearFlag_PLLSAIRDY();
	LL_RCC_EnableIT_PLLSAIRDY();


	// Enable HSE
	LL_RCC_HSE_Enable();
	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_HSERDY, &retFlags, pdMS_TO_TICKS(HSE_STARTUP_TIMEOUT)) == pdFALSE) {
		//todo HSE timeout
		Error_Handler();
	}

	// Setup CSS
	taskENTER_CRITICAL();
	LL_RCC_ClearFlag_HSECSS();
	LL_RCC_HSE_EnableCSS();
	cssEnabled = true;
	taskEXIT_CRITICAL();

	// Configure main PLL using HSE
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 216, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();
	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_PLLRDY | FLAG_CSS, &retFlags, pdMS_TO_TICKS(PLL_TIMEOUT_VALUE)) == pdFALSE) {
		//todo PLL timeout
		Error_Handler();
	}
	if (retFlags & FLAG_CSS) {
		//todo CSS during PLL init
		Error_Handler();
	}

//	// wait for VOS to be ready
//	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(2);
//	while (LL_PWR_IsActiveFlag_VOS() == 0) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	// switch clock to PLL
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	do {
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE)
			//todo clock switch timeout
			Error_Handler();
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS, &retFlags, 0) == pdTRUE) {
			//todo CSS during clock switch
			Error_Handler();
		}
	} while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

	// Adjust system tick interrupt to new cpu frequency
	taskENTER_CRITICAL();
	LL_SetSystemCoreClock(216000000);
	HAL_InitTick(TICK_INT_PRIORITY);
	taskEXIT_CRITICAL();

//	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS, &retFlags, 0) == pdTRUE) {
//		//todo CSS after clock switch
//	}



//	HAL_RCC_OscConfig

	if (xTaskNotify(callingTaskHandle, 0, eNoAction) != pdPASS)
		Error_Handler();

	for (;;) {
		vTaskDelay(1000);
	}
}

static portSTACK_TYPE xClockTaskStack[128] __attribute__((aligned(128*4)));
static TaskParameters_t xClockTaskDefinition =
{
	taskClockMain,
	"sys.clock",
    sizeof(xClockTaskStack) / sizeof(portSTACK_TYPE),
    NULL,
    0 | portPRIVILEGE_BIT,
	xClockTaskStack,
    {
        /* Base address   Length                    Parameters */
    }
};

void Setup() {
	xClockTaskDefinition.pvParameters = xTaskGetCurrentTaskHandle();
	xTaskCreateRestricted(&xClockTaskDefinition, &pxClockTaskHandle);
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
}

extern "C"
void RCC_IRQHandler() {
	uint32_t flags = RCC->CIR; // read interrupt register
	flags &= flags >> 8; // mask enabled interrupts
	RCC->CIR |= flags << 16; //clear pending flags

	// handle CSS exception
	if (!READ_BIT(RCC->CR, RCC_CR_CSSON) && cssEnabled) {
		flags |= FLAG_CSS;
		cssEnabled = false;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyIndexedFromISR(pxClockTaskHandle, 0, flags, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

extern "C"
void NMI_Handler() {
	if (LL_RCC_IsActiveFlag_HSECSS()) {
		LL_RCC_ClearFlag_HSECSS();

		CLEAR_BIT(RCC->CR, RCC_CR_CSSON); // disable CSS in hardware, cssEnabled is still true
		HAL_NVIC_SetPendingIRQ(RCC_IRQn); // pend an RCC irq which will handle the disabled CSS

	}
	Error_Handler();
}

}
}
