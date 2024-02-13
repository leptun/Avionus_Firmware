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
	constexpr uint32_t arr = config::servo_pulse_length - config::servo_idle_time - 1;

	for (uint32_t i = 0; i < COUNT_OF(config::servo_channels); i++) {
		util::TIM_CHAN_PAIR ch = config::servo_channels[i];
		LL_TIM_SetPrescaler(ch.tim, psc);
		LL_TIM_SetAutoReload(ch.tim, arr);
		if (IS_TIM_BREAK_INSTANCE(ch.tim)) {
			LL_TIM_EnableAllOutputs(ch.tim);
		}
		ch.SetCompare(arr + 1);
		servoPositions[i] = arr + 1;
		LL_TIM_GenerateEvent_UPDATE(ch.tim);
		LL_TIM_CC_EnableChannel(ch.tim, ch.chan);
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
	state = State::cycling;
	ApplyPositions();
	state = State::ready;
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
	for (uint32_t servo = 0; servo < COUNT_OF(config::servo_channels); servo++) {
		uint32_t pos_us = servoPositions[servo];
		if (pos_us < config::servo_min || pos_us > config::servo_max) {
			continue;
		}
		util::TIM_CHAN_PAIR ch = config::servo_channels[servo];
		LL_TIM_OC_DisablePreload(ch.tim, ch.chan);
		ch.SetCompare(pos_us - 1);
		LL_TIM_OC_EnablePreload(ch.tim, ch.chan);
		ch.SetCompare(config::servo_pulse_length - config::servo_idle_time); //arr+1
		LL_TIM_ClearFlag_UPDATE(ch.tim);
	}

	// start the pulse sequence
	for (uint32_t servo = 0; servo < COUNT_OF(config::servo_channels); servo++) {
		util::TIM_CHAN_PAIR ch = config::servo_channels[servo];
		if (!LL_TIM_IsEnabledCounter(ch.tim) && !LL_TIM_IsActiveFlag_UPDATE(ch.tim)) {
			LL_TIM_SetCounter(ch.tim, ch.MaxVal() - config::servo_idle_time + 1);
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
