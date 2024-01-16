#pragma once

namespace config {

static constexpr uint32_t system_clock_frequency = 216000000;

static constexpr uint32_t logic_cycle_frequency = 25;
static constexpr uint32_t regular_adc_sample_cnt = 400; //samples per cycle

static TIM_TypeDef * const adc_timer = TIM6;

}
