#include "ff.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if FF_USE_LFN == 3	/* Use dynamic memory allocation */

/*------------------------------------------------------------------------*/
/* Allocate/Free a Memory Block                                           */
/*------------------------------------------------------------------------*/

#include <umm_malloc.h>		/* with POSIX API */

static uint8_t fatfs_heap_buf[1920] __attribute__((aligned(4)));
static umm_heap fatfs_heap;
static SemaphoreHandle_t fatfs_heap_lock;

void* ff_memalloc (	/* Returns pointer to the allocated memory block (null if not enough core) */
	UINT msize		/* Number of bytes to allocate */
)
{
	void *ptr;
	if (xSemaphoreTake(fatfs_heap_lock, portMAX_DELAY) != pdTRUE) {
		Error_Handler();
	}
	ptr = umm_multi_malloc(&fatfs_heap, (size_t)msize);	/* Allocate a new memory block */
	if (xSemaphoreGive(fatfs_heap_lock) != pdTRUE) {
		Error_Handler();
	}
	return ptr;
}


void ff_memfree (
	void* mblock	/* Pointer to the memory block to free (no effect if null) */
)
{
	if (xSemaphoreTake(fatfs_heap_lock, portMAX_DELAY) != pdTRUE) {
		Error_Handler();
	}
	umm_multi_free(&fatfs_heap, mblock);	/* Free the memory block */
	if (xSemaphoreGive(fatfs_heap_lock) != pdTRUE) {
		Error_Handler();
	}
}

void ff_meminit(void) {
	fatfs_heap_lock = xSemaphoreCreateMutex();
	umm_multi_init_heap(&fatfs_heap, fatfs_heap_buf, sizeof(fatfs_heap_buf));
}

void ff_mem_grant_access(TaskHandle_t task) {
	vGrantAccessToSemaphore(task, fatfs_heap_lock);
}

#endif
