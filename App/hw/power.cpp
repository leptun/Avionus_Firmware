#include "power.hpp"
#include <util.hpp>

namespace hw {
namespace power {

void PowerDomain::Enable() const {
	en_pin.Write(true);
	if (enableFunc) {
		enableFunc();
	}
}

void PowerDomain::Disable() const {
	if (disableFunc) {
		disableFunc();
	}
	en_pin.Write(false);
}

void Setup() {
	D1.Enable();
	D2.Disable();
	D3.Disable();
	D4.Disable();
	D5.Disable();
	pins::POWER::SERVO_5V_AUX_EN.Write(true);
	pins::POWER::LEDs_EN.Write(true);
}

}
}
