#pragma once
#include "util.hpp"

namespace config {

static constexpr uint32_t system_clock_frequency = 216000000;

static constexpr uint32_t logic_cycle_frequency = 25;
static constexpr uint32_t regular_adc_sample_cnt = 400; //samples per cycle

static constexpr uint32_t servo_update_freq = 50;
static const util::TIM_CHAN_PAIR servo_channels[] = {
		{ TIM3, LL_TIM_CHANNEL_CH1 },
		{ TIM3, LL_TIM_CHANNEL_CH2 },
		{ TIM3, LL_TIM_CHANNEL_CH3 },
		{ TIM3, LL_TIM_CHANNEL_CH4 },
		{ TIM4, LL_TIM_CHANNEL_CH2 },
		{ TIM4, LL_TIM_CHANNEL_CH1 },
		{ TIM4, LL_TIM_CHANNEL_CH3 },
		{ TIM4, LL_TIM_CHANNEL_CH4 },
		{ TIM1, LL_TIM_CHANNEL_CH1 },
		{ TIM1, LL_TIM_CHANNEL_CH2 },
		{ TIM1, LL_TIM_CHANNEL_CH3 },
		{ TIM1, LL_TIM_CHANNEL_CH4 },
		{ TIM2, LL_TIM_CHANNEL_CH4 },
		{ TIM2, LL_TIM_CHANNEL_CH3 },
		{ TIM2, LL_TIM_CHANNEL_CH2 },
		{ TIM2, LL_TIM_CHANNEL_CH1 },
};

static TIM_TypeDef * const adc_timer = TIM6;

}
