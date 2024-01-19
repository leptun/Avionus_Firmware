#pragma once

#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void ff_system_init(void);
extern void ff_system_grant_access(TaskHandle_t task);

#ifdef __cplusplus
}
#endif
