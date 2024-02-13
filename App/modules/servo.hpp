#pragma once
#include <inttypes.h>
#include "module.hpp"
#include <config.hpp>
#include <util.hpp>
#include <FreeRTOS.h>
#include <timers.h>

namespace modules {
namespace servo {

class Servo : public Module {
public:
	void Init() override;
	bool Update() override;
	void Cycle() override;

	bool SetPosition(uint32_t servo, uint32_t pos_us);
private:
	enum Flags {
		FLAG_CHECK_POWER = 0x000001,
		FLAG_PUSH_SERVO_SIGNALS = 0x000002,
	};
	util::AtomicFlags flags;
	TimerHandle_t timer;
	uint32_t servoPositions[COUNT_OF(config::servo_channels)];
	void ApplyPositions();
	static void periodicTimerCallback(TimerHandle_t xTimer);
};

}
}
