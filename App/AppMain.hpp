#pragma once
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

enum Flags {
	FLAG_MODULES = 0x000001,
};

extern TaskHandle_t pxTaskHandle;

extern void Setup();

}
