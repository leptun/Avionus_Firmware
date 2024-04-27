K = 1024,
M = 1024 * K,

XFLASH_BLOCK64_SIZE = 131072,
XFLASH_SECTOR_SIZE = 8192,
XFLASH_PAGE_SIZE = 512,


__privileged_functions_region_size__  = 64 * K,
__shared_region_size__                = 16 * K,
__fatfs_data_region_size__            = 4 * K,
__krpc_data_region_size__             = 4 * K,
__tinyusb_data_region_size__          = 2 * K,
__logging_data_region_size__          = 2 * K,
__buffers_data_region_size__          = 64 * K,
__backup_sram_region_size__           = 4 * K,
__xflash_crash_dump_region_size__     = 280 * K,
