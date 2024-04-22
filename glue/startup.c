#include <inttypes.h>
#include <main.h>

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
	extern uint32_t g_pfnVectors[];
	SCB->VTOR = (uint32_t)g_pfnVectors;

	HAL_DeInit();
    // reset clocks
    SET_BIT(RCC->CR, RCC_CR_HSION); /* Set HSION bit to the reset value */
    while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET) { } /* Wait till HSI is ready */
    CLEAR_REG(RCC->CFGR); /* Reset CFGR register */
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) { } /* Wait till clock switch is ready */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON); /* Clear PLLON bit */
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) { } /* Wait till PLL is disabled */
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR1 = 0x0000C000;
    PWR->CSR1 = 0x00000000;
    PWR->CR2 = 0x00000000;
    PWR->CSR2 = 0x00000000;
    __HAL_RCC_PWR_CLK_DISABLE();

	INIT_SECTION(freertos);
	INIT_SECTION(xflashmgr);
	INIT_SECTION(buffers);
	INIT_SECTION(app);
	INIT_SECTION(tinyusb);
	INIT_SECTION(fatfs);
	INIT_SECTION(krpc);
	INIT_SECTION(logging);
	INIT_SECTION(shared);
}
