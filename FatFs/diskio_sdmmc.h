#pragma once

#include "ff.h"
#include "diskio.h"

#ifdef __cplusplus
extern "C" {
#endif

DSTATUS disk_sdmmc_status (void);
DSTATUS disk_sdmmc_initialize (void);
DRESULT disk_sdmmc_read (BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_sdmmc_write (const BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_sdmmc_ioctl (BYTE cmd, void* buff);

#ifdef __cplusplus
}
#endif
