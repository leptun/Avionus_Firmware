#pragma once
#include "util.hpp"
#include <inttypes.h>
#include <stddef.h>

namespace config {

static constexpr uint32_t system_clock_frequency = 216000000;

namespace clocks {
static constexpr uint32_t hclk = system_clock_frequency;
static constexpr uint32_t pclk1 = hclk / 4;
static constexpr uint32_t pclk2 = hclk / 2;
static constexpr uint32_t hsi = HSI_VALUE;
static constexpr uint32_t hse = HSE_VALUE;
static constexpr uint32_t lsi = LSI_VALUE;
static constexpr uint32_t lse = LSE_VALUE;
}

static constexpr uint32_t logic_cycle_frequency = 20;
static constexpr uint32_t regular_adc_sample_cnt = 500; //samples per cycle

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

static constexpr uint32_t usb_stream_buffer_size = 64;

static constexpr size_t gps_rxbuf_size = 256;
}
