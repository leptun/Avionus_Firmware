#include "Avionus_QSPILoader.h"
#include <main.h> // must be before quadspi.h
#include <quadspi.h> // !!! it includes the wrong main.h
#include <FreeRTOS.h>
#include <task.h>


static uint32_t activeStreams;

void Avionus_QSPILoader_init() {
	return;
	if (CSP_QUADSPI_Init() != HAL_OK) {
		Error_Handler();
	}

	if (CSP_QSPI_EnableMemoryMappedMode() != HAL_OK) {
		Error_Handler();
	}
}

void Avionus_QSPILoader_idleTaskEnterLowPower() {
	return;
	// inside critical section by callee

	if (activeStreams) {
		activeStreams += ulTaskNotifyTakeIndexed(0, pdTRUE, 0); // add the number of streams started
		activeStreams -= ulTaskNotifyTakeIndexed(1, pdTRUE, 0); // subtract the number of streams ended
	}
	if (!activeStreams && (HAL_QSPI_GetState(&hqspi) & HAL_QSPI_STATE_BUSY_MEM_MAPPED)) {
		// no ahb masters are going to access the idle task during sleep, so abort memory mapped mode to make the chip select go high

		SET_BIT(hqspi.Instance->CR, QUADSPI_CR_ABORT);
		while (READ_BIT(hqspi.Instance->CR, QUADSPI_CR_ABORT)) {}
		SET_BIT(hqspi.Instance->FCR, QUADSPI_FCR_CTCF); // prevent accidental transfer complete interrupt if that is enabled for some reason
	}
}

void Avionus_QSPILoader_announceDmaStreamBegin() {
	return;
	if (xTaskNotifyGiveIndexed(xTaskGetIdleTaskHandle(), 0) != pdPASS) {
		Error_Handler();
	}
}

void Avionus_QSPILoader_announceDmaStreamEnd() {
	return;
	if (xTaskNotifyGiveIndexed(xTaskGetIdleTaskHandle(), 1) != pdPASS) {
		Error_Handler();
	}
}
