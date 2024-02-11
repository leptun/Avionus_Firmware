#include "power.hpp"
#include <util.hpp>

namespace hw {
namespace power {

static const util::IO EN_PINS[] = {
	{ POWER_D1_EN_GPIO_Port, POWER_D1_EN_Pin },
	{ POWER_D2_EN_GPIO_Port, POWER_D2_EN_Pin },
	{ POWER_D3_EN_GPIO_Port, POWER_D3_EN_Pin },
	{ POWER_D4_EN_GPIO_Port, POWER_D4_EN_Pin },
	{ POWER_D5_EN_GPIO_Port, POWER_D5_EN_Pin },
	{ POWER_SERVO_5V_AUX_EN_GPIO_Port, POWER_SERVO_5V_AUX_EN_Pin },
};

void Setup() {
	EnablePowerDomain(PowerDomainControl::D1);
	EnablePowerDomain(PowerDomainControl::D2);
	EnablePowerDomain(PowerDomainControl::D3);
	EnablePowerDomain(PowerDomainControl::D4);
	EnablePowerDomain(PowerDomainControl::D5);
	EnablePowerDomain(PowerDomainControl::SERVO_AUX);
	SetPowerLEDs(true);
}

void SetPowerLEDs(bool en) {
	HAL_GPIO_WritePin(POWER_LEDs_EN_GPIO_Port, POWER_LEDs_EN_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void EnablePowerDomain(PowerDomainControl domain) {
	const util::IO gpio = EN_PINS[(uint32_t)domain];
	HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_SET);
}

void DisablePowerDomain(PowerDomainControl domain) {
	const util::IO gpio = EN_PINS[(uint32_t)domain];
	HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_RESET);
}

}
}
