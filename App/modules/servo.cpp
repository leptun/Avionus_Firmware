#include "servo.hpp"
#include <main.h>
#include <config.hpp>

namespace modules {
namespace servo {

void Servo::Setup() {
	// Initialize servo timers
	constexpr uint32_t timer_cnt_freq = 1000000;
	constexpr uint32_t timer_upd_freq = config::servo_update_freq;
	constexpr uint16_t psc = __LL_TIM_CALC_PSC(config::system_clock_frequency, timer_cnt_freq);
	constexpr uint16_t arr = __LL_TIM_CALC_ARR(config::system_clock_frequency, psc, timer_upd_freq);

	for (uint32_t i = 0; i < COUNT_OF(config::servo_channels); i++) {
		util::TIM_CHAN_PAIR ch = config::servo_channels[i];
		if (!LL_TIM_IsEnabledCounter(ch.tim)) {
			LL_TIM_SetPrescaler(ch.tim, psc);
			LL_TIM_SetAutoReload(ch.tim, arr);
			if (IS_TIM_BREAK_INSTANCE(ch.tim)) {
				LL_TIM_EnableAllOutputs(ch.tim);
			}
			LL_TIM_GenerateEvent_UPDATE(ch.tim);
			LL_TIM_EnableCounter(ch.tim);
		}
		LL_TIM_CC_EnableChannel(ch.tim, ch.chan);
	}
}

int Servo::SetPosition(uint32_t servo, uint32_t pos_us) {
	if (servo >= COUNT_OF(config::servo_channels)) {
		return -1;
	}
	if (pos_us > 2500) {
		return -2;
	}
	if (pos_us < 500) {
		return -3;
	}

	util::TIM_CHAN_PAIR ch = config::servo_channels[servo];
	switch (ch.chan) {
	case LL_TIM_CHANNEL_CH1:
	case LL_TIM_CHANNEL_CH1N:
		LL_TIM_OC_SetCompareCH1(ch.tim, pos_us-1);
		break;
	case LL_TIM_CHANNEL_CH2:
	case LL_TIM_CHANNEL_CH2N:
		LL_TIM_OC_SetCompareCH2(ch.tim, pos_us-1);
		break;
	case LL_TIM_CHANNEL_CH3:
	case LL_TIM_CHANNEL_CH3N:
		LL_TIM_OC_SetCompareCH3(ch.tim, pos_us-1);
		break;
	case LL_TIM_CHANNEL_CH4:
		LL_TIM_OC_SetCompareCH4(ch.tim, pos_us-1);
		break;
	}

	return 0;
}

}
}
