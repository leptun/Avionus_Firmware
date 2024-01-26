#pragma once

#include <FusionMath.h>

namespace modules {
namespace airplane {

struct Flight {
	FusionQuaternion rotation;
	float latitude;
	float longitude;
	float mean_altitude;
	float speed;
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
