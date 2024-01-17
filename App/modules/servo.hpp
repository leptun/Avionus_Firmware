#pragma once
#include <inttypes.h>

namespace modules {
namespace servo {

void Setup();
int SetServoPosition(uint32_t servo, uint32_t pos_us);

}
}
