#pragma once

#include "ff.h"
#include "diskio.h"
#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

void diskio_sdmmc_init(void);
void diskio_sdmmc_grant_access(TaskHandle_t task);

DSTATUS disk_sdmmc_status (void);
DSTATUS disk_sdmmc_initialize (void);
DRESULT disk_sdmmc_read (BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_sdmmc_write (const BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_sdmmc_ioctl (BYTE cmd, void* buff);

#ifdef __cplusplus
}
#endif
