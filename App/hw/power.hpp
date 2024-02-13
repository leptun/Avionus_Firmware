#pragma once
#include <inttypes.h>
#include <pins.hpp>

namespace hw {
namespace power {

class PowerDomain {
	const pins::GPIO en_pin;
	void (*const enableFunc)(void);
	void (*const disableFunc)(void);

public:
	constexpr PowerDomain(const pins::GPIO en_pin, void (*const enableFunc)(void) = nullptr, void (*const disableFunc)(void) = nullptr)
		: en_pin(en_pin)
		, enableFunc(enableFunc)
		, disableFunc(disableFunc)
		{}
	void Enable() const;
	void Disable() const;
};

inline constexpr PowerDomain D1 = PowerDomain(pins::POWER::D1_EN);
inline constexpr PowerDomain D2 = PowerDomain(pins::POWER::D2_EN,
	[]() {
		pins::GPS::NRESET.Write(true);
		pins::GPS::TX.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::IMU::NSS.Write(true);
		pins::IMU::MISO.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::IMU::MOSI.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::IMU::SCK.SetMode(LL_GPIO_MODE_ALTERNATE);
	},
	[]() {
		pins::GPS::NRESET.Write(false);
		pins::GPS::TX.SetMode(LL_GPIO_MODE_INPUT);
		pins::IMU::NSS.Write(false);
		pins::IMU::MISO.SetMode(LL_GPIO_MODE_INPUT);
		pins::IMU::MOSI.SetMode(LL_GPIO_MODE_INPUT);
		pins::IMU::SCK.SetMode(LL_GPIO_MODE_INPUT);
	}
);
inline constexpr PowerDomain D3 = PowerDomain(pins::POWER::D3_EN,
	[]() {
		pins::SD_CARD::CK.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::SD_CARD::CMD.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::SD_CARD::D0.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::SD_CARD::D1.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::SD_CARD::D2.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::SD_CARD::D3.SetMode(LL_GPIO_MODE_ALTERNATE);
	},
	[]() {
		pins::SD_CARD::CK.SetMode(LL_GPIO_MODE_INPUT);
		pins::SD_CARD::CMD.SetMode(LL_GPIO_MODE_INPUT);
		pins::SD_CARD::D0.SetMode(LL_GPIO_MODE_INPUT);
		pins::SD_CARD::D1.SetMode(LL_GPIO_MODE_INPUT);
		pins::SD_CARD::D2.SetMode(LL_GPIO_MODE_INPUT);
		pins::SD_CARD::D3.SetMode(LL_GPIO_MODE_INPUT);
	}
);
inline constexpr PowerDomain D4 = PowerDomain(pins::POWER::D4_EN,
	[]() {
		pins::RF::NRESET.Write(true);
		pins::RF::NSS.Write(true);
		pins::RF::MISO.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::RF::MOSI.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::RF::SCK.SetMode(LL_GPIO_MODE_ALTERNATE);
	},
	[]() {
		pins::RF::NRESET.Write(false);
		pins::RF::RX_SWITCH.Write(false);
		pins::RF::TX_SWITCH.Write(false);
		pins::RF::NSS.Write(false);
		pins::RF::MISO.SetMode(LL_GPIO_MODE_INPUT);
		pins::RF::MOSI.SetMode(LL_GPIO_MODE_INPUT);
		pins::RF::SCK.SetMode(LL_GPIO_MODE_INPUT);
	}
);
inline constexpr PowerDomain D5 = PowerDomain(pins::POWER::D5_EN,
	[]() {
		pins::LIDAR::XSHUT.Write(true);
		pins::LIDAR::TX.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::REMOTE::iBUS.SetMode(LL_GPIO_MODE_ALTERNATE);
	},
	[]() {
		pins::LIDAR::XSHUT.Write(false);
		pins::LIDAR::TX.SetMode(LL_GPIO_MODE_INPUT);
		pins::REMOTE::iBUS.SetMode(LL_GPIO_MODE_INPUT);
	}
);

void Setup();

}
}
