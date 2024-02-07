#pragma once
#include "main.h"
#include <inttypes.h>
#include <FreeRTOS.h>

extern uint8_t null_ptr;

namespace util {

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

struct IO {
	GPIO_TypeDef *port;
	uint16_t pin;
};

struct TIM_CHAN_PAIR {
	TIM_TypeDef *tim;
	uint32_t chan;
};

struct LL_DMA_STREAM {
	DMA_TypeDef * const DMAx;
	const uint32_t Stream;
	volatile uint32_t * const ISR;
	volatile uint32_t * const IFCR;
	const uint8_t bitOffset;

private:
	static constexpr uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
public:
	constexpr LL_DMA_STREAM() :
			DMAx(0),
			Stream(0),
			ISR(0),
			IFCR(0),
			bitOffset(0) {}
	constexpr LL_DMA_STREAM(DMA_TypeDef *DMAx, uint32_t Stream) :
			DMAx(DMAx),
			Stream(Stream),
			ISR((Stream > 3) ? &DMAx->HISR : &DMAx->LISR),
			IFCR((Stream > 3) ? &DMAx->HIFCR : &DMAx->LIFCR),
			bitOffset(flagBitshiftOffset[Stream]) {}

	uint32_t irq_handler() const {
		uint32_t mask =
				(LL_DMA_IsEnabledIT_FE(DMAx, Stream) ? DMA_LISR_FEIF0 : 0) |
				(LL_DMA_IsEnabledIT_DME(DMAx, Stream) ? DMA_LISR_DMEIF0 : 0) |
				(LL_DMA_IsEnabledIT_TE(DMAx, Stream) ? DMA_LISR_TEIF0 : 0) |
				(LL_DMA_IsEnabledIT_HT(DMAx, Stream) ? DMA_LISR_HTIF0 : 0) |
				(LL_DMA_IsEnabledIT_TC(DMAx, Stream) ? DMA_LISR_TCIF0 : 0);
		uint32_t flags = *ISR & (mask << bitOffset);
		*IFCR = flags; //clear enabled flags
		return flags >> bitOffset;
	}

	void clearIRQ(uint32_t mask) const {
		*IFCR = mask << bitOffset;
	}
};

template<typename T>
constexpr T * ioCast(unsigned long addr) {
	return static_cast<T *>(static_cast<void *>(&null_ptr + addr));
}

BaseType_t xTaskNotifyWaitBitsAllIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);
BaseType_t xTaskNotifyWaitBitsAnyIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);

static inline bool isDebugging() {
	return !HAL_GPIO_ReadPin(DEBUG_GNDDetect_GPIO_Port, DEBUG_GNDDetect_Pin);
}

}

