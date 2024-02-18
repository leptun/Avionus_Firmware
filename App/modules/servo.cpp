#include "servo.hpp"
#include <main.h>
#include <config.hpp>
#include "module_manager.hpp"

namespace modules {
namespace servo {

void Servo::Init() {
	Module::Init();
	// Initialize servo timers
	constexpr uint32_t timer_cnt_freq = 1000000;
	constexpr uint16_t psc = __LL_TIM_CALC_PSC(config::system_clock_frequency, timer_cnt_freq);

	for (uint32_t i = 0; i < COUNT_OF(config::servo_channels); i++) {
		util::TIM_CHAN_PAIR ch = config::servo_channels[i];
		LL_TIM_SetPrescaler(ch.tim, psc);
		LL_TIM_SetAutoReload(ch.tim, -1);
		if (IS_TIM_BREAK_INSTANCE(ch.tim)) {
			LL_TIM_EnableAllOutputs(ch.tim);
		}
		ch.SetCompare(-1);
		LL_TIM_GenerateEvent_UPDATE(ch.tim);
	}
	state = State::ready;

	// start periodic pushing of servo signals
	timer = xTimerCreate("servo", configTICK_RATE_HZ / config::servo_update_freq, pdTRUE, this, periodicTimerCallback);
	if (!timer || xTimerStart(timer, portMAX_DELAY) != pdPASS) {
		Error_Handler();
	}
}

bool Servo::Update() {
	if (state != State::ready) {
		return false;
	}
	uint32_t f = flags.Get();
	flags.Clear(f);
	if (f & FLAG_CHECK_POWER) {

		// todo
	}
	if (f & FLAG_PUSH_SERVO_SIGNALS) {
		ApplyPositions();
	}
	return false;
}

void Servo::Cycle() {
	if (state != State::ready) {
		return;
	}
	flags.Set(FLAG_CHECK_POWER);
	NotifyWork();
}

bool Servo::SetPosition(uint32_t servo, uint32_t pos_us) {
	if (servo >= COUNT_OF(config::servo_channels)) {
		return false;
	}
	servoPositions[servo] = pos_us;
	return true;
}

void Servo::ApplyPositions() {
	// if any of the timers are enabled already, that's no bueno. Skip this push cycle.
	for (const util::TIM_CHAN_PAIR &ch : config::servo_channels) {
		if (LL_TIM_IsEnabledCounter(ch.tim)) {
			return;
		}
	}

	// for each servo, push its position if it's valid. If not, then disable that timer channel
	for (uint32_t servo = 0; servo < COUNT_OF(config::servo_channels); servo++) {
		uint32_t pos_us = servoPositions[servo];
		util::TIM_CHAN_PAIR ch = config::servo_channels[servo];
		if (pos_us < config::servo_min || pos_us > config::servo_max) {
			LL_TIM_CC_DisableChannel(ch.tim, ch.chan);
			continue;
		}
		else {
			ch.SetCompare(pos_us - 1);
			LL_TIM_CC_EnableChannel(ch.tim, ch.chan);
		}

		uint32_t arr = pos_us + config::servo_right_porch - 1;
		if (LL_TIM_IsActiveFlag_UPDATE(ch.tim)) {
			// first time this timer is used in this cycle
			LL_TIM_SetAutoReload(ch.tim, arr);
			LL_TIM_ClearFlag_UPDATE(ch.tim);
		} else {
			// increase ARR if it's bigger than the currently set ARR
			if (arr > LL_TIM_GetAutoReload(ch.tim)) {
				LL_TIM_SetAutoReload(ch.tim, arr);
			}
		}
	}

	// start the pulse sequence for all timers
	for (uint32_t servo = 0; servo < COUNT_OF(config::servo_channels); servo++) {
		util::TIM_CHAN_PAIR ch = config::servo_channels[servo];
		if (!LL_TIM_IsEnabledCounter(ch.tim) && !LL_TIM_IsActiveFlag_UPDATE(ch.tim)) {
			LL_TIM_SetCounter(ch.tim, -config::servo_left_porch - 1);
			LL_TIM_EnableCounter(ch.tim);
		}
	}
}

void Servo::periodicTimerCallback(TimerHandle_t xTimer) {
	Servo *_this = (Servo *)pvTimerGetTimerID(xTimer);
	_this->flags.SetFromISR(FLAG_PUSH_SERVO_SIGNALS);
	NotifyWork();
}

}
}
