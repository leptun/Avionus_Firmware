#pragma once

#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

void ff_meminit(void);
void ff_mem_grant_access(TaskHandle_t task);

#ifdef __cplusplus
}
#endif
