#pragma once
#include <inttypes.h>
#include "module.hpp"

namespace modules {
namespace logging {

extern uint8_t aBuf[16384];
extern uint8_t bBuf[16384];

extern void Setup();

class Logging : public Module {
public:
private:
};

}
}
