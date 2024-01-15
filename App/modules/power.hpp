#pragma once

namespace modules {
namespace power {

enum class PowerDomainControl {
	D1,
	D2,
	D3,
	D4,
	D5,
	SERVO_AUX,
};

void Setup();

void SetPowerLEDs(bool en);
void EnablePowerDomain(PowerDomainControl domain);
void DisablePowerDomain(PowerDomainControl domain);

}
}
