#pragma once

#include <inttypes.h>
#include <FusionMath.h>

namespace modules {
namespace airplane {

struct Flight {
	float pitch;
	float heading;
	float roll;
	float latitude;
	float longitude;
	float mean_altitude;
	float speed;
	uint32_t latency;
};

struct Control {
	float throttle;
	float pitch;
	float yaw;
	float roll;
	float flaps;
	bool gear;
};

}
}
