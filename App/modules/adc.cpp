#include "adc.hpp"
#include <main.h>

namespace modules {
namespace adc {

void Setup() {
	LL_ADC_INJ_SetTrigAuto(ADC2, LL_ADC_INJ_TRIG_INDEPENDENT);
	LL_ADC_INJ_SetTrigAuto(ADC3, LL_ADC_INJ_TRIG_INDEPENDENT);
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
	LL_ADC_Enable(ADC3);

	LL_ADC_REG_StartConversionSWStart(ADC1);
}

}
}
