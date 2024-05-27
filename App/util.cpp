#include "util.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace util {

BaseType_t xTaskNotifyWaitBitsAllIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait) {
	BaseType_t ret = pdTRUE;
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	uint32_t currBits = 0;

	xTaskNotifyIndexed(xTaskGetCurrentTaskHandle(), uxIndexToWaitOn, 0, eNoAction);
	while (ulBitsToClearOnExit) {
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
			ret = pdFALSE;
			break;
		}
		uint32_t newBits = 0;
		if (xTaskNotifyWaitIndexed(uxIndexToWaitOn, ulBitsToClearOnEntry, ulBitsToClearOnExit, &newBits, xTicksToWait) == pdFALSE) {
			ret = pdFALSE;
			break;
		}
		ulBitsToClearOnEntry = 0; // clear bits only once on entry
		currBits |= newBits & ulBitsToClearOnExit; // cumulate cleared bits
		ulBitsToClearOnExit &= ~newBits; // clear bits only once on exit
	}
	if (pulNotificationValue) {
		*pulNotificationValue = currBits;
	}
	return ret;
}

BaseType_t xTaskNotifyWaitBitsAnyIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait) {
	BaseType_t ret = pdTRUE;
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	uint32_t currBits = 0;

	xTaskNotifyIndexed(xTaskGetCurrentTaskHandle(), uxIndexToWaitOn, 0, eNoAction);
	while (currBits == 0) {
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
			ret = pdFALSE;
			break;
		}
		uint32_t newBits = 0;
		if (xTaskNotifyWaitIndexed(uxIndexToWaitOn, ulBitsToClearOnEntry, ulBitsToClearOnExit, &newBits, xTicksToWait) == pdFALSE) {
			ret = pdFALSE;
			break;
		}
		ulBitsToClearOnEntry = 0; // clear bits only once on entry
		currBits |= newBits & ulBitsToClearOnExit; // cumulate cleared bits
	}
	if (pulNotificationValue) {
		*pulNotificationValue = currBits;
	}
	return ret;
}

}
