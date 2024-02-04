#pragma once

#include "ff.h"
#include <FreeRTOS.h>
#include <task.h>

/* Definitions of physical drive number for each drive */
#define DEV_SDMMC		0

#ifdef __cplusplus
extern "C" {
#endif

void fatfs_Init(void);
void fatfs_GrantAccess(TaskHandle_t task);


#ifdef __cplusplus
}
#endif
