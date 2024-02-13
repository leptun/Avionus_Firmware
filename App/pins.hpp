#pragma once
#include <main.h>

namespace pins {

struct GPIO {
	GPIO_TypeDef *port;
	uint32_t pin;

	void Write(bool val) const {
		if (val) {
			LL_GPIO_SetOutputPin(port, pin);
		}
		else {
			LL_GPIO_ResetOutputPin(port, pin);
		}
	}
	bool Read() const {
		return LL_GPIO_IsInputPinSet(port, pin);
	}
	void SetMode(uint32_t mode) const {
		LL_GPIO_SetPinMode(port, pin, mode);
	}
};

// D1

namespace POWER {
inline constexpr GPIO D1_EN = { POWER_D1_EN_GPIO_Port, POWER_D1_EN_Pin };
inline constexpr GPIO D2_EN = { POWER_D2_EN_GPIO_Port, POWER_D2_EN_Pin };
inline constexpr GPIO D3_EN = { POWER_D3_EN_GPIO_Port, POWER_D3_EN_Pin };
inline constexpr GPIO D4_EN = { POWER_D4_EN_GPIO_Port, POWER_D4_EN_Pin };
inline constexpr GPIO D5_EN = { POWER_D5_EN_GPIO_Port, POWER_D5_EN_Pin };
inline constexpr GPIO SERVO_5V_AUX_EN = { POWER_SERVO_5V_AUX_EN_GPIO_Port, POWER_SERVO_5V_AUX_EN_Pin };
inline constexpr GPIO LEDs_EN = { POWER_LEDs_EN_GPIO_Port, POWER_LEDs_EN_Pin };
}

namespace DBG {
inline constexpr GPIO GNDDetect = { DEBUG_GNDDetect_GPIO_Port, DEBUG_GNDDetect_Pin };
}

// D2

namespace GPS {
inline constexpr GPIO NRESET = { GPS_NRESET_GPIO_Port, GPS_NRESET_Pin };
inline constexpr GPIO TX = { GPS_NRESET_GPIO_Port, GPS_NRESET_Pin };
inline constexpr GPIO RX = { GPS_NRESET_GPIO_Port, GPS_NRESET_Pin };
inline constexpr GPIO SDA = { GPIOF, LL_GPIO_PIN_0 };
inline constexpr GPIO SCL = { GPIOF, LL_GPIO_PIN_1 };
}

namespace IMU {
inline constexpr GPIO MISO = { GPIOA, LL_GPIO_PIN_6 };
inline constexpr GPIO MOSI = { GPIOA, LL_GPIO_PIN_7 };
inline constexpr GPIO SCK = { GPIOA, LL_GPIO_PIN_5 };
inline constexpr GPIO NSS = { IMU_NSS_GPIO_Port, IMU_NSS_Pin };
inline constexpr GPIO INT1 = { IMU_INT1_GPIO_Port, IMU_INT1_Pin };
}

// D3

namespace SD_CARD {
inline constexpr GPIO CK = { GPIOD, LL_GPIO_PIN_6 };
inline constexpr GPIO CMD = { GPIOD, LL_GPIO_PIN_7 };
inline constexpr GPIO D0 = { GPIOG, LL_GPIO_PIN_9 };
inline constexpr GPIO D1 = { GPIOG, LL_GPIO_PIN_10 };
inline constexpr GPIO D2 = { GPIOG, LL_GPIO_PIN_11 };
inline constexpr GPIO D3 = { GPIOG, LL_GPIO_PIN_12 };
inline constexpr GPIO CD = { SD_CARD_CD_GPIO_Port, SD_CARD_CD_Pin };
inline constexpr GPIO WP = { SD_CARD_WP_GPIO_Port, SD_CARD_WP_Pin };
}

// D4

namespace RF {
inline constexpr GPIO MISO = { GPIOE, LL_GPIO_PIN_13 };
inline constexpr GPIO MOSI = { GPIOE, LL_GPIO_PIN_14 };
inline constexpr GPIO SCK = { GPIOE, LL_GPIO_PIN_12 };
inline constexpr GPIO NSS = { RF_NSS_GPIO_Port, RF_NSS_Pin };
inline constexpr GPIO BUSY = { RF_BUSY_GPIO_Port, RF_BUSY_Pin };
inline constexpr GPIO DIO1 = { RF_DIO1_GPIO_Port, RF_DIO1_Pin };
inline constexpr GPIO DIO2 = { RF_DIO2_GPIO_Port, RF_DIO2_Pin };
inline constexpr GPIO DIO3 = { RF_DIO3_GPIO_Port, RF_DIO3_Pin };
inline constexpr GPIO RX_SWITCH = { RF_RX_SWITCH_GPIO_Port, RF_RX_SWITCH_Pin };
inline constexpr GPIO TX_SWITCH = { RF_TX_SWITCH_GPIO_Port, RF_TX_SWITCH_Pin };
inline constexpr GPIO NRESET = { RF_NRESET_GPIO_Port, RF_NRESET_Pin };
}

// D5

namespace UI {
inline constexpr GPIO BUZZER = { UI_BUZZER_GPIO_Port, UI_BUZZER_Pin };
inline constexpr GPIO LED1 = { UI_LED_1_GPIO_Port, UI_LED_1_Pin };
inline constexpr GPIO LED2 = { UI_LED_2_GPIO_Port, UI_LED_2_Pin };
inline constexpr GPIO LED3 = { UI_LED_3_GPIO_Port, UI_LED_3_Pin };
inline constexpr GPIO LED4 = { UI_LED_4_GPIO_Port, UI_LED_4_Pin };
inline constexpr GPIO SW_USER = { UI_SW_USER_GPIO_Port, UI_SW_USER_Pin };
}

namespace LIDAR {
inline constexpr GPIO SDA = { GPIOF, LL_GPIO_PIN_0 };
inline constexpr GPIO SCL = { GPIOF, LL_GPIO_PIN_1 };
inline constexpr GPIO TX = { GPIOE, LL_GPIO_PIN_1 };
inline constexpr GPIO RX = { GPIOE, LL_GPIO_PIN_0 };
inline constexpr GPIO GPIO1 = { LIDAR_GPIO1_GPIO_Port, LIDAR_GPIO1_Pin };
inline constexpr GPIO XSHUT = { LIDAR_XSHUT_GPIO_Port, LIDAR_XSHUT_Pin };
}

namespace REMOTE {
inline constexpr GPIO PPM = { REMOTE_PPM_GPIO_Port, REMOTE_PPM_Pin };
inline constexpr GPIO iBUS = { REMOTE_iBUS_GPIO_Port, REMOTE_iBUS_Pin };
}

// SERVO

namespace SERVO {

}


}
