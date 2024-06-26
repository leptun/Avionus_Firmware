#include "clock.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <util.hpp>
#include <config.hpp>
#include <ff.h>
#include <hw/exti.hpp>
#include <FatFs/fatfs.h>

namespace hw {
namespace clock {

static TaskHandle_t pxTaskHandle;
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
	FLAG_RTC_TICK = 0x000100,
};

static void rtcTickHandler() {
	fatfs_updateFatTime(
			(DWORD)(__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)) + 20) << 25 |
			(DWORD)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)) << 21 |
			(DWORD)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)) << 16 |
			(DWORD)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)) << 11 |
			(DWORD)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)) << 5 |
			(DWORD)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)) >> 1
	);
}

static void configureRTC() {
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;
	if (LL_RTC_IsActiveFlag_INITS(RTC)) {
		// already initialized
		return;
	}


	LL_RTC_DisableWriteProtection(RTC);

	LL_RTC_DisableIT_WUT(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	while (!LL_RTC_IsActiveFlag_WUTW(RTC)) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	LL_RTC_WAKEUP_SetAutoReload(RTC, 0); //1s tick period
	LL_RTC_WAKEUP_Enable(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableIT_WUT(RTC);

	LL_RTC_EnableWriteProtection(RTC);
}

static bool configureLClocks(bool useExternalClock) {
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;

	// check if the RTC is already configured
	if ((LL_RCC_GetRTCClockSource() == (useExternalClock ? LL_RCC_RTC_CLKSOURCE_LSE : LL_RCC_RTC_CLKSOURCE_LSI)) && LL_RCC_IsEnabledRTC()) {
		return true;
	}

	LL_RCC_ForceBackupDomainReset();
	LL_RCC_ReleaseBackupDomainReset();

	LL_RCC_LSI_Disable();
	LL_RCC_LSE_Disable();
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	while (LL_RCC_LSI_IsReady() || LL_RCC_LSE_IsReady()) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	if (useExternalClock) {
		LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
		LL_RCC_LSE_DisableBypass();
		LL_RCC_LSE_Enable();
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_LSERDY, NULL, pdMS_TO_TICKS(LSE_STARTUP_TIMEOUT)) == pdFALSE) {
			// LSE timeout
			return false;
		}
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	} else {
		LL_RCC_LSI_Enable();
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_LSIRDY, NULL, pdMS_TO_TICKS(LSI_TIMEOUT_VALUE)) == pdFALSE) {
			// LSI timeout
			return false;
		}
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
	}

	LL_RCC_EnableRTC();
	if (!useExternalClock) {
		LL_RTC_DisableWriteProtection(RTC);
		LL_RTC_SetAsynchPrescaler(RTC, (125-1)); // set prescaler for 32KHz clock
		LL_RTC_EnableWriteProtection(RTC);
	}

	return true;
}

static bool configureHClocks(bool useExternalClock) {
	uint32_t retFlags;
	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;

	// switch clock to HSI
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(10);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_HSI) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	// Adjust system tick interrupt to new cpu frequency
	taskENTER_CRITICAL();
	LL_SetSystemCoreClock(HSI_VALUE);
	HAL_InitTick(TICK_INT_PRIORITY);
	taskEXIT_CRITICAL();

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
	} else {
		// Disable HSE
		LL_RCC_HSE_Disable();
		vTaskSetTimeOutState(&xTimeOut); xTicksToWait = pdMS_TO_TICKS(2);
		while(LL_RCC_HSE_IsReady()) { if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) Error_Handler(); }

	}

	// Configure main PLL
	const uint32_t targetClockFreq = config::system_clock_frequency;
	const uint32_t pllClockInputFreq = 2000000ul;
	const uint32_t pllClockFreq = targetClockFreq * 2;
	const uint32_t pllm = (useExternalClock ? HSE_VALUE : HSI_VALUE) / pllClockInputFreq;
	const uint32_t plln = pllClockFreq / pllClockInputFreq;
	const uint32_t pllp_bits = LL_RCC_PLLP_DIV_2;
	const uint32_t pllq = 2;

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
			| (pllp_bits)
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

	// Configure PLLSAI1
	const uint32_t pllsaitargetClockFreq = config::clocks::pll48clk;
	const uint32_t pllsaiClockFreq = pllsaitargetClockFreq * 4;
	const uint32_t pllsain = pllsaiClockFreq / pllClockInputFreq;
	const uint32_t pllsaip_bits = LL_RCC_PLLSAIP_DIV_4;
	const uint32_t pllsaiq = 2;

	// configure main PLL
	MODIFY_REG(RCC->PLLSAICFGR
			, RCC_PLLSAICFGR_PLLSAIN
			| RCC_PLLSAICFGR_PLLSAIP
			| RCC_PLLSAICFGR_PLLSAIQ
			, (pllsain << RCC_PLLSAICFGR_PLLSAIN_Pos)
			| (pllsaip_bits)
			| (pllsaiq << RCC_PLLSAICFGR_PLLSAIQ_Pos)
	);

	LL_RCC_PLLSAI_Enable();
	if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_PLLSAIRDY | FLAG_CSS, &retFlags, pdMS_TO_TICKS(PLLSAI_TIMEOUT_VALUE)) == pdFALSE) {
		//PLL timeout
		return false;
	}
	if (retFlags & FLAG_CSS) {
		//CSS during PLL init
		return false;
	}

	return true;
}

static void taskClockMain(void *pvParameters) {
	TaskHandle_t callingTaskHandle = (TaskHandle_t)pvParameters;

	TimeOut_t xTimeOut;
	TickType_t xTicksToWait;

	LL_PWR_EnableBkUpAccess();

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

	// Set peripheral clock sources
	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_FOUR_TIMES); // Make all timers run at HCLK
	LL_RCC_SetCK48MClockSource(LL_RCC_CK48M_CLKSOURCE_PLLSAI);
	LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC2_CLKSOURCE_SYSCLK);

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

	// Setup wakeup interrupt
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_22);
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);


	// try to setup the low speed clocks using the LSE
	if (!configureLClocks(true)) {
		// fallback to the LSI
		if (!configureLClocks(false)) {
			// wtf why is nothing working
			Error_Handler();
		}
	}

	configureRTC();
	rtcTickHandler();

	// try to setup the high speed clocks using the HSE
	if (!configureHClocks(true)) {
		// fallback to the HSI
		if (!configureHClocks(false)) {
			// wtf why is nothing working
			Error_Handler();
		}
	}

	if (xTaskNotify(callingTaskHandle, 0, eNoAction) != pdPASS)
		Error_Handler();

	for (;;) {
		uint32_t retFlags;
		if (util::xTaskNotifyWaitBitsAnyIndexed(0, 0, FLAG_CSS | FLAG_RTC_TICK, &retFlags, pdMS_TO_TICKS(2000)) == pdFALSE) {
			// rtc not advancing, switch to LSI
			if (!configureLClocks(false)) {
				Error_Handler();
			}
			configureRTC();
		}
		else {
			if (retFlags & FLAG_CSS) {
				//CSS occured, try to reconfigure the clocks to internal
				if (!configureHClocks(false)) {
					Error_Handler();
				}
			}
			else if (retFlags & FLAG_RTC_TICK) {
				LL_RTC_DisableWriteProtection(RTC);
				LL_RTC_ClearFlag_WUT(RTC);
				LL_RTC_EnableWriteProtection(RTC);

				rtcTickHandler();
			}
		}
	}
}

static portSTACK_TYPE xClockTaskStack[128] __attribute__((aligned(128*4))) __attribute__((section(".stack")));
static const TaskParameters_t xClockTaskDefinition =
{
	taskClockMain,
	"clock",
	(configSTACK_DEPTH_TYPE)sizeof(xClockTaskStack) / sizeof(portSTACK_TYPE),
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
	xTaskCreateRestricted(&_priv_xClockTaskDefinition, &pxTaskHandle);
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
	xTaskNotifyIndexedFromISR(pxTaskHandle, 0, flags, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

extern "C"
void NMI_Handler() {
	if (LL_RCC_IsActiveFlag_HSECSS() && READ_BIT(RCC->CR, RCC_CR_CSSON)) {
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

void hw::exti::exti22_handler() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyIndexedFromISR(hw::clock::pxTaskHandle, 0, hw::clock::FLAG_RTC_TICK, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
