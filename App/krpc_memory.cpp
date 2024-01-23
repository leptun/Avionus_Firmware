#include <krpc_cnano/memory.h>
#include "krpc_memory.hpp"
#include <umm_malloc.h>

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef KRPC_CUSTOM_MEMORY_ALLOC

static uint8_t krpc_heap_buf[4032] __attribute__((aligned(4)));
static umm_heap krpc_heap;

void krpc_memory_init(void) {
	umm_multi_init_heap(&krpc_heap, krpc_heap_buf, sizeof(krpc_heap_buf));
}

void* krpc_malloc(size_t size) {
	return umm_multi_malloc(&krpc_heap, size);
}

void* krpc_calloc(size_t num, size_t size) {
	return umm_multi_calloc(&krpc_heap, num, size);
}

void* krpc_recalloc(void *ptr, size_t num, size_t inc, size_t size) {
	assert(inc > 0);
	ptr = umm_multi_realloc(&krpc_heap, ptr, (num + inc) * size);
	memset(((uint8_t*) ptr) + (num * size), 0, (inc * size));
	return ptr;
}

void krpc_free(void *ptr) {
	umm_multi_free(&krpc_heap, ptr);
}

#endif
