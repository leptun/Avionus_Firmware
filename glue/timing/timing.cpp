#include <main.h>
#include <config.hpp>
#include <utility>
#include "timing.hpp"
//#include "timer_defaults.h"
#include "FreeRTOS.h"
//#include <wdt.hpp>

#define TICK_TIMER TIM5

#define TIM_BASE_CLK_MHZ config::clocks::timclk
#define TICK_TIMER_CNT (TICK_TIMER->CNT)
#define TICK_TIMER_MX_INIT MX_##TICK_TIMER##_Init
#define TICK_TIMER_IRQ

// Large numbers to avoid number of 0s errors
constexpr const uint32_t thousand = 1000UL;
constexpr const uint32_t million = thousand * thousand;
constexpr const uint32_t billion = thousand * thousand * thousand;

static volatile uint32_t tick_cnt_s;

// macro xPortSysTickHandler in FreeRTOSConfig.h must be commented
extern "C" void xPortSysTickHandler(void);

// interrupt from ARM-CORE timer
extern "C" void SysTick_Handler(void) {
//    wdt_tick_1ms();
    xPortSysTickHandler();
}

#pragma GCC push_options
#pragma GCC optimize("Ofast")

/**
 * @brief Safely sample tick timer without the risk of race.
 * @param[out] sec seconds since boot
 * @param[out] subsec subseconds in TIM_BASE_CLK_MHZ, overflows every 1 second
 * @note Both subsec and sec need to be consistent, subsec will overlflow to 0 at the same time as sec increments.
 */
static void sample_timer(uint32_t &sec, uint32_t &subsec) {
    volatile uint32_t sec_1st_read;
    volatile uint32_t lower_cnt;
    volatile uint32_t sec_2nd_read;

    do {
        sec_1st_read = tick_cnt_s;
        lower_cnt = TICK_TIMER_CNT; // Will be in range 0 .. TIM_BASE_CLK_MHZ * million - 1
        sec_2nd_read = tick_cnt_s;
    } while (sec_1st_read != sec_2nd_read); // Repeat if overflow of the timer has happened

    sec = sec_1st_read;
    subsec = lower_cnt;
}

extern "C" int64_t get_timestamp_us() {
    uint32_t sec, subsec;
    sample_timer(sec, subsec);

    return static_cast<int64_t>(sec) * static_cast<int64_t>(million) + (subsec / TIM_BASE_CLK_MHZ);
}

extern "C" timestamp_t get_timestamp() {
    uint32_t sec, subsec;
    sample_timer(sec, subsec);

    return { sec, (subsec / TIM_BASE_CLK_MHZ) };
}

extern "C" uint32_t ticks_s() {
    return tick_cnt_s;
}

static uint32_t last_ms;
extern "C" uint32_t ticks_ms() {
    uint32_t sec, subsec;
    sample_timer(sec, subsec);

    last_ms = sec * thousand + subsec / (TIM_BASE_CLK_MHZ * thousand);
    return last_ms;
}

extern "C" uint32_t last_ticks_ms() {
    return last_ms;
}

extern "C" uint32_t ticks_us() {
    uint32_t sec, subsec;
    sample_timer(sec, subsec);

    return sec * million + subsec / TIM_BASE_CLK_MHZ;
}

extern "C" void TICK_TIMER_IRQHandler() {
    if (LL_TIM_IsActiveFlag_UPDATE(TICK_TIMER) && LL_TIM_IsEnabledIT_UPDATE(TICK_TIMER)) {
    	LL_TIM_ClearFlag_UPDATE(TICK_TIMER);
    	++tick_cnt_s;
    }
}

#pragma GCC pop_options

/**
 * @brief shadow weak function in HAL
 *        do nothing, SysTimer is owned by FreeRtos
 *
 * @param TickPriority
 * @return HAL_StatusTypeDef::HAL_OK
 */

/**
 * @brief Must use 32 bit timer
 *        ~12ns tick, 84MHz, 1s period
 *
 * @return HAL_StatusTypeDef
 */
void tick_timer_init() {
	LL_TIM_SetPrescaler(TICK_TIMER, 0);
	LL_TIM_SetCounter(TICK_TIMER, (TIM_BASE_CLK_MHZ * million) - 1);

	LL_TIM_ClearFlag_UPDATE(TICK_TIMER);
	LL_TIM_EnableIT_UPDATE(TICK_TIMER);
	LL_TIM_EnableCounter(TICK_TIMER);
}
