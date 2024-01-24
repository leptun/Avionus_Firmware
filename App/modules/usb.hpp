#pragma once

#include "FreeRTOS.h"
#include <stream_buffer.h>
#include <task.h>

namespace modules {
namespace usb {

void Setup();
void GrantAccess(TaskHandle_t task);

}
}
