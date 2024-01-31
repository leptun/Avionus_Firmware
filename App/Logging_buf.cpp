#include "Logging_buf.hpp"
#include <inttypes.h>

namespace Logging {

uint8_t aBuf[16384] __attribute__((aligned(16)));
uint8_t bBuf[16384] __attribute__((aligned(16)));

}
