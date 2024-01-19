#include "fatfs.h"
#include "ff_mem.h"
#include "ffsystem.h"
#include "diskio_sdmmc.h"

void fatfs_Init(void) {
	ff_meminit();
	ff_system_init();
	diskio_sdmmc_init();
}

void fatfs_GrantAccess(TaskHandle_t task) {
	ff_system_grant_access(task);
	diskio_sdmmc_grant_access(task);
}

DWORD get_fattime(void) {
	return 0;
}
