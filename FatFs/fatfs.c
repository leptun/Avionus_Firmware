#include "fatfs.h"
#include "ff_mem.h"

void fatfs_Init(void) {
	ff_meminit();
}

DWORD get_fattime(void) {
	return 0;
}
