#include "power.hpp"
#include "adc.h"
#include <FreeRTOS.h>
#include <event_groups.h>
#include <timers.h>
#include <util.hpp>

static uint16_t adc_meas[12] __attribute__((aligned(8)));

namespace modules {
namespace power {

static const util::IO EN_PINS[] = {
	{ POWER_D1_EN_GPIO_Port, POWER_D1_EN_Pin },
	{ POWER_D2_EN_GPIO_Port, POWER_D2_EN_Pin },
	{ POWER_D3_EN_GPIO_Port, POWER_D3_EN_Pin },
	{ POWER_D4_EN_GPIO_Port, POWER_D4_EN_Pin },
	{ POWER_D5_EN_GPIO_Port, POWER_D5_EN_Pin },
	{ POWER_SERVO_5V_AUX_EN_GPIO_Port, POWER_SERVO_5V_AUX_EN_Pin },
};

//extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//
//}

void Setup() {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_meas, COUNT_OF(adc_meas));
	LL_TIM_EnableCounter(TIM6);

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
