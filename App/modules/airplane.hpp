#pragma once

#include <FusionMath.h>

namespace modules {
namespace airplane {

class State {
	FusionEuler heading;
	float latitude;
	float longitude;
	float elevation;
	float velocity;
};

class Control {
	float throttle;
	float pitch;
	float yaw;
	float roll;
	float flaps;
	bool gear;
};

}
}
