#include "diskio_sdmmc.h"
#include "diskio.h"
#include "ff.h"
#include <main.h>
#include "stm32f7xx_hal_sd_ext.h"
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <hw/exti.hpp>
#include <pins.hpp>


#define SDMMC_TIMEOUT_MS 2000
#define SDMMC_MAX_RETRY 2

extern SD_HandleTypeDef hsd2;
EventGroupHandle_t sd_diskio_flags;
enum FlagDef {
	SD_DISKIO_FLAG_NOINIT = STA_NOINIT,
	SD_DISKIO_FLAG_NODISK = STA_NODISK,
	SD_DISKIO_FLAG_PROTECT = STA_PROTECT,
	SD_DISKIO_TRANSFER_CPLT = 0x08,
	SD_DISKIO_TRANSFER_ERROR = 0x10,
	SD_DISKIO_TRANSFER_ABORTED = 0x20,
};

void diskio_sdmmc_init(void) {
	sd_diskio_flags = xEventGroupCreate();
	if (!sd_diskio_flags) {
		Error_Handler();
	}
	hsd2.Instance = SDMMC2;
	HAL_SD_MspInit(&hsd2);
}

void diskio_sdmmc_grant_access(TaskHandle_t task) {
	vGrantAccessToEventGroup(task, sd_diskio_flags);
}

static bool sdmmc_card_detected() {
	return !pins::SD_CARD::CD.Read();
}

static bool sdmmc_card_write_protected() {
	return pins::SD_CARD::WP.Read();
}

static DRESULT sdmmc_wait_ready() {
	TickType_t xTicksToWait = pdMS_TO_TICKS(SDMMC_TIMEOUT_MS);
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	while (HAL_SD_GetCardState(&hsd2) != HAL_SD_CARD_TRANSFER) {
		if (disk_sdmmc_status() & STA_NODISK) {
			return RES_NOTRDY;
		}
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
			return RES_ERROR;
		}
		taskYIELD();
	}
	return RES_OK;
}

DSTATUS disk_sdmmc_status (void) {
	if (sd_diskio_flags) {
		return xEventGroupGetBits(sd_diskio_flags) & (STA_NOINIT | STA_NODISK | STA_PROTECT);
	}
	else {
		return STA_NOINIT;
	}
}

DSTATUS disk_sdmmc_initialize(void) {
	if (sdmmc_card_write_protected()) {
		xEventGroupSetBits(sd_diskio_flags, STA_PROTECT);
	} else {
		xEventGroupClearBits(sd_diskio_flags, STA_PROTECT);
	}

	/* Check if the SD card is plugged in the slot */
	if (!sdmmc_card_detected()) {
		xEventGroupSetBits(sd_diskio_flags, STA_NOINIT | STA_NODISK);
		return disk_sdmmc_status();
	}
	else {
		xEventGroupClearBits(sd_diskio_flags, STA_NODISK);
	}

	/* uSD device interface configuration */
	hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_FALLING;
	hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	hsd2.Init.ClockDiv = 254;

	if (HAL_SD_Init(&hsd2) != HAL_OK) {
		xEventGroupSetBits(sd_diskio_flags, STA_NOINIT);
		return disk_sdmmc_status();
	}

	/* Configure SD Bus width */
	/* This is an essential Delay. Otherwise the following WideBusOperation can get stuck
	 * in optimized builds. It is unsure and irrelevant if this behavior is caused by software
	 * or hardware (RC-Delays, 4 bit lines in different lengths,...)
	 */
	vTaskDelay(1);

	hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd2.Init.ClockDiv = 2;

	/* Enable wide operation */
	if (HAL_SD_ConfigWideBusOperation(&hsd2, hsd2.Init.BusWide) != HAL_OK) {
		xEventGroupSetBits(sd_diskio_flags, STA_NOINIT);
	}

	return disk_sdmmc_status();
}

DRESULT disk_sdmmc_read (BYTE* buff, LBA_t sector, UINT count) {
	HAL_StatusTypeDef status;
	DRESULT res;
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}

	if ((uint32_t)buff & 0x1f) {
		Error_Handler();
	}

	for (uint32_t retryCounter = 0; retryCounter < SDMMC_MAX_RETRY; retryCounter++) {
interrupted:
		if (hsd2.State == HAL_SD_STATE_READY) {
			if ((res = sdmmc_wait_ready()) != RES_OK) {
				continue;
			}
		}

		status = HAL_SD_ReadBlocksUninterrupted_DMA(&hsd2, buff, (uint32_t)sector, count);
		if (status != HAL_OK) {
			if (status == HAL_BUSY) {
				goto interrupted;
			}
			res = RES_ERROR;
			continue;
		}

		if (xEventGroupWaitBits(
				sd_diskio_flags,
				SD_DISKIO_TRANSFER_CPLT | SD_DISKIO_TRANSFER_ERROR | SD_DISKIO_TRANSFER_ABORTED,
				pdTRUE,
				pdFALSE,
				pdMS_TO_TICKS(SDMMC_TIMEOUT_MS)
		) != SD_DISKIO_TRANSFER_CPLT) {
			res = RES_ERROR;
			continue;
		}

		res = RES_OK;
		break;
	}

	return res;
}

DRESULT disk_sdmmc_write (const BYTE* buff, LBA_t sector, UINT count) {
	HAL_StatusTypeDef status;
	DRESULT res;
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}

	if ((uint32_t)buff & 0x1f) {
		Error_Handler();
	}

	// Flush the write buffer to ram so that the DMA can push it to the SDMMC peripheral
	portCleanDCache_by_Addr(buff, count * BLOCKSIZE);

	for (uint32_t retryCounter = 0; retryCounter < SDMMC_MAX_RETRY; retryCounter++) {
interrupted:
		if (hsd2.State == HAL_SD_STATE_READY) {
			if ((res = sdmmc_wait_ready()) != RES_OK) {
				continue;
			}
		}

		status = HAL_SD_WriteBlocksUninterrupted_DMA(&hsd2, (uint8_t*)buff, (uint32_t)sector, count);
		if (status != HAL_OK) {
			if (status == HAL_BUSY) {
				goto interrupted;
			}
			res = RES_ERROR;
			continue;
		}

		if (xEventGroupWaitBits(
				sd_diskio_flags,
				SD_DISKIO_TRANSFER_CPLT | SD_DISKIO_TRANSFER_ERROR | SD_DISKIO_TRANSFER_ABORTED,
				pdTRUE,
				pdFALSE,
				pdMS_TO_TICKS(SDMMC_TIMEOUT_MS)
		) != SD_DISKIO_TRANSFER_CPLT) {
			res = RES_ERROR;
			continue;
		}

		res = RES_OK;
		break;
	}

	return res;
}

DRESULT disk_sdmmc_ioctl (BYTE cmd, void* buff) {
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}

	switch (cmd) {
	case CTRL_SYNC:
		if (HAL_SD_EXT_Sync(&hsd2) != HAL_OK) {
			return RES_ERROR;
		}
		return sdmmc_wait_ready();
	case GET_SECTOR_COUNT:
		*(DWORD*)buff = hsd2.SdCard.LogBlockNbr;
		return RES_OK;
	case GET_SECTOR_SIZE:
		*(WORD*)buff = hsd2.SdCard.LogBlockSize;
		return RES_OK;
	case GET_BLOCK_SIZE:
		*(DWORD*)buff = 1;
		return RES_OK;
	case CTRL_TRIM: {
		LBA_t *addr = (LBA_t *)buff;
		// finish ongoing operations
		if (HAL_SD_EXT_Sync(&hsd2) != HAL_OK) {
			return RES_ERROR;
		}
		DRESULT res;
		if ((res = sdmmc_wait_ready()) != RES_OK) {
			return res;
		}
		// erase sectors that are no longer used
		if (HAL_SD_Erase(&hsd2, addr[0], addr[1]) != HAL_OK) {
			return RES_ERROR;
		}
	} return RES_OK;
	default:
		return RES_PARERR;
	}
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = xEventGroupSetBitsFromISR(sd_diskio_flags, SD_DISKIO_TRANSFER_CPLT, &xHigherPriorityTaskWoken);
	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = xEventGroupSetBitsFromISR(sd_diskio_flags, SD_DISKIO_TRANSFER_CPLT, &xHigherPriorityTaskWoken);

	SCB_InvalidateDCache_by_Addr((uint32_t*)hsd->hdmarx->Instance->M0AR, (0xFFFF - hsd->hdmarx->Instance->NDTR) << 2);

	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = xEventGroupSetBitsFromISR(sd_diskio_flags, SD_DISKIO_TRANSFER_ERROR, &xHigherPriorityTaskWoken);
	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = xEventGroupSetBitsFromISR(sd_diskio_flags, SD_DISKIO_TRANSFER_ABORTED, &xHigherPriorityTaskWoken);
	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void hw::exti::exti13_handler() {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = xEventGroupSetBitsFromISR(sd_diskio_flags, SD_DISKIO_FLAG_NOINIT, &xHigherPriorityTaskWoken);
	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


