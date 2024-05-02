#pragma once

#include "FreeRTOS.h"
#include <task.h>

namespace hw {
namespace usb {

void Setup();
void GrantAccess(TaskHandle_t task);

}
}
