#pragma once

#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

void retarget_locks_init(void);
void retarget_locks_grant_access(TaskHandle_t task);

#ifdef __cplusplus
}
#endif
