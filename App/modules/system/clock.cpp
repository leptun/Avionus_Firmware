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

static bool configureLClocks(bool useExternalClock) {
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;

	// check if the RTC is already configured
	if ((LL_RCC_GetRTCClockSource() == (useExternalClock ? LL_RCC_RTC_CLKSOURCE_LSE : LL_RCC_RTC_CLKSOURCE_LSI)) && LL_RCC_IsEnabledRTC()) {
		return true;
	}

	LL_PWR_EnableBkUpAccess();

	LL_RCC_DisableRTC();
	LL_RCC_LSI_Disable();
	LL_RCC_LSE_Disable();
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	while(LL_RCC_LSI_IsReady() || LL_RCC_LSE_IsReady()) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	if (useExternalClock) {
		LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
		LL_RCC_LSE_DisableBypass();
		LL_RCC_LSE_Enable();
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);

		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_LSERDY, NULL, pdMS_TO_TICKS(LSE_STARTUP_TIMEOUT)) == pdFALSE) {
			// LSE timeout
			LL_PWR_DisableBkUpAccess();
			return false;
		}
	} else {
		LL_RCC_LSI_Enable();
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);

		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_LSIRDY, NULL, pdMS_TO_TICKS(LSI_TIMEOUT_VALUE)) == pdFALSE) {
			// LSE timeout
			LL_PWR_DisableBkUpAccess();
			return false;
		}
	}
	LL_RCC_EnableRTC();
	LL_PWR_DisableBkUpAccess();
	return true;
}

static bool configureHClocks(bool useExternalClock) {
	uint32_t retFlags;
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;
	if (useExternalClock) {
		// Enable HSE
		LL_RCC_HSE_Enable();
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_HSERDY, &retFlags, pdMS_TO_TICKS(HSE_STARTUP_TIMEOUT)) == pdFALSE) {
			// HSE timeout
			return false;
		}

		// Setup CSS
		taskENTER_CRITICAL();
		LL_RCC_ClearFlag_HSECSS();
		LL_RCC_HSE_EnableCSS();
		cssEnabled = true;
		taskEXIT_CRITICAL();
	}

	// Configure main PLL
	const uint32_t targetClockFreq = 216000000ul;
	const uint32_t pllClockInputFreq = 2000000ul;
	const uint32_t pllClockFreq = targetClockFreq * 2;
	const uint32_t pllm = (useExternalClock ? HSE_VALUE : HSI_VALUE) / pllClockInputFreq;
	const uint32_t plln = pllClockFreq / pllClockInputFreq;
	const uint32_t pllp = 2;
	const uint32_t pllq = pllClockFreq / 48000000ul;

	// configure main PLL
	MODIFY_REG(RCC->PLLCFGR
			, RCC_PLLCFGR_PLLSRC
			| RCC_PLLCFGR_PLLM
			| RCC_PLLCFGR_PLLN
			| RCC_PLLCFGR_PLLP
			| RCC_PLLCFGR_PLLQ
			, (useExternalClock ? LL_RCC_PLLSOURCE_HSE : LL_RCC_PLLSOURCE_HSI)
			| (pllm << RCC_PLLCFGR_PLLM_Pos)
			| (plln << RCC_PLLCFGR_PLLN_Pos)
			| (pllp << RCC_PLLCFGR_PLLP_Pos)
			| (pllq << RCC_PLLCFGR_PLLQ_Pos)
	);

	LL_RCC_PLL_Enable();
	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_PLLRDY | FLAG_CSS, &retFlags, pdMS_TO_TICKS(PLL_TIMEOUT_VALUE)) == pdFALSE) {
		//PLL timeout
		return false;
	}
	if (retFlags & FLAG_CSS) {
		//CSS during PLL init
		return false;
	}

	// switch clock to PLL
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	do {
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
			//clock switch timeout
			return false;
		}
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS, &retFlags, 0) == pdTRUE) {
			//CSS during clock switch
			return false;
		}
	} while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

	// Adjust system tick interrupt to new cpu frequency
	taskENTER_CRITICAL();
	LL_SetSystemCoreClock(targetClockFreq);
	HAL_InitTick(TICK_INT_PRIORITY);
	taskEXIT_CRITICAL();

	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS, &retFlags, 0) == pdTRUE) {
		//CSS after clock switch
		return false;
	}

	return true;
}

void taskClockMain(void *pvParameters) {
	TaskHandle_t callingTaskHandle = (TaskHandle_t)pvParameters;

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


	// try to setup the low speed clocks using the LSE
	if (!configureLClocks(true)) {
		// fallback to the LSI
		if (!configureLClocks(false)) {
			// wtf why is nothing working
			Error_Handler();
		}
	}

	// try to setup the high speed clocks using the HSE
	if (!configureHClocks(true)) {
		// fallback to the HSI
		if (!configureHClocks(false)) {
			// wtf why is nothing working
			Error_Handler();
		}
	}

//	HAL_RCC_OscConfig

	if (xTaskNotify(callingTaskHandle, 0, eNoAction) != pdPASS)
		Error_Handler();

	uint32_t lastSubSecond = LL_RTC_TIME_GetSubSecond(RTC);
	for (;;) {
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS, NULL, pdMS_TO_TICKS(1500)) == pdFALSE) {
			// time to check if the RTC is advancing
			uint32_t newSubSecond = LL_RTC_TIME_GetSubSecond(RTC);
			if (newSubSecond == lastSubSecond) {
				// rtc not advancing, switch to LSI
				if (!configureLClocks(false)) {
					Error_Handler();
				}
			}
			lastSubSecond = newSubSecond;
		}
		else {
			//CSS occured, try to reconfigure the clocks to internal
			if (!configureHClocks(false)) {
				Error_Handler();
			}
		}

	}
}

static portSTACK_TYPE xClockTaskStack[128] __attribute__((aligned(128*4)));
static const TaskParameters_t xClockTaskDefinition =
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
	TaskParameters_t _priv_xClockTaskDefinition = xClockTaskDefinition;
	_priv_xClockTaskDefinition.pvParameters = xTaskGetCurrentTaskHandle();
	xTaskCreateRestricted(&_priv_xClockTaskDefinition, &pxClockTaskHandle);
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

		// disable CSS in hardware, cssEnabled is still true
		CLEAR_BIT(RCC->CR, RCC_CR_CSSON);
		// pend an RCC irq which will handle the disabled CSS
		HAL_NVIC_SetPendingIRQ(RCC_IRQn);

		// reconfigure the systick to the new clock frequency
		LL_SetSystemCoreClock(HSI_VALUE);
		HAL_InitTick(TICK_INT_PRIORITY);
	}
	Error_Handler();
}

}
}
