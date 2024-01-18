#include "ff.h"

#if FF_USE_LFN == 3	/* Use dynamic memory allocation */

/*------------------------------------------------------------------------*/
/* Allocate/Free a Memory Block                                           */
/*------------------------------------------------------------------------*/

#include <umm_malloc.h>		/* with POSIX API */

static uint8_t fatfs_heap_buf[1024] __attribute__((aligned(4)));
static umm_heap fatfs_heap;

void* ff_memalloc (	/* Returns pointer to the allocated memory block (null if not enough core) */
	UINT msize		/* Number of bytes to allocate */
)
{
	return umm_multi_malloc(&fatfs_heap, (size_t)msize);	/* Allocate a new memory block */
}


void ff_memfree (
	void* mblock	/* Pointer to the memory block to free (no effect if null) */
)
{
	umm_multi_free(&fatfs_heap, mblock);	/* Free the memory block */
}

void ff_meminit(void) {
	umm_multi_init_heap(&fatfs_heap, fatfs_heap_buf, sizeof(fatfs_heap_buf));
}

#endif
