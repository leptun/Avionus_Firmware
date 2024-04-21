#include "adc.hpp"
#include <main.h>
#include <config.hpp>

namespace hw {
namespace adc {

void Setup() {
	// Fix stupid cubemx mistake
	LL_ADC_INJ_SetTrigAuto(ADC2, LL_ADC_INJ_TRIG_INDEPENDENT);
	LL_ADC_INJ_SetTrigAuto(ADC3, LL_ADC_INJ_TRIG_INDEPENDENT);

	// Initialize periodic trigger timer
	constexpr uint32_t timer_cnt_freq = 1000000;
	constexpr uint32_t timer_upd_freq = config::regular_adc_cycle_frequency;
	constexpr uint16_t psc = __LL_TIM_CALC_PSC(config::system_clock_frequency, timer_cnt_freq);
	constexpr uint16_t arr = __LL_TIM_CALC_ARR(config::system_clock_frequency, psc, timer_upd_freq);
	LL_TIM_SetPrescaler(config::adc_timer, psc);
	LL_TIM_SetAutoReload(config::adc_timer, arr);

	// Enable ADC
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
	LL_ADC_Enable(ADC3);

	LL_TIM_EnableCounter(config::adc_timer);
}

}
}
