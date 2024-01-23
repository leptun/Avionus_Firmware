#pragma once

#include "FreeRTOS.h"
#include <stream_buffer.h>

namespace modules {
namespace usb {

extern StreamBufferHandle_t krpc_rx_stream;
extern StreamBufferHandle_t krpc_tx_stream;

void Setup();

}
}
