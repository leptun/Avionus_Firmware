#pragma once
#include "main.h"
#include <FreeRTOS.h>

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

BaseType_t xTaskNotifyWaitBitsAllIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);
BaseType_t xTaskNotifyWaitBitsAnyIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);

}

