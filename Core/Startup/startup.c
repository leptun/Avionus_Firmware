#include <inttypes.h>

static void init_bss(uint32_t *start, uint32_t *end) {
	uint32_t cnt = end - start;
	while (cnt--)
		*(start++) = 0;
}

static void init_data(uint32_t *start, uint32_t *end, const uint32_t *loadStart) {
	uint32_t cnt = end - start;
	while (cnt--)
		*(start++) = *(loadStart++);
}

#define INIT_SECTION(name) do { \
	extern uint32_t _##name##_data_run_addr; \
	extern uint32_t _##name##_data_end; \
	extern const uint32_t _##name##_data_load_addr; \
	extern uint32_t _##name##_bss_run_addr; \
	extern uint32_t _##name##_bss_end; \
	init_data(&_##name##_data_run_addr, &_##name##_data_end, &_##name##_data_load_addr); \
	init_bss(&_##name##_bss_run_addr, &_##name##_bss_end); \
	} while (0)


void init_sections(void) {
	INIT_SECTION(freertos);
	INIT_SECTION(app);
	INIT_SECTION(tinyusb);
	INIT_SECTION(fatfs);
	INIT_SECTION(shared);
}
