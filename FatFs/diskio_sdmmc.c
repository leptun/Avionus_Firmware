#include "diskio_sdmmc.h"
#include "diskio.h"
#include "ff.h"
#include <main.h>
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"


#define SDMMC_TIMEOUT_MS 5000

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

static int sdmmc_card_detected() {
	return HAL_GPIO_ReadPin(SD_CARD_CD_GPIO_Port, SD_CARD_CD_Pin) == GPIO_PIN_RESET;
}

static int sdmmc_card_write_protected() {
	return HAL_GPIO_ReadPin(SD_CARD_WP_GPIO_Port, SD_CARD_WP_Pin) == GPIO_PIN_SET;
}

static void sdmmc_config_dma_stream(const void *buff) {
	uint32_t tz = __CLZ(__RBIT((uint32_t)buff)); // get trailing zeros
	uint32_t sxcr;
	switch (tz) {
	case 0: sxcr = DMA_MBURST_SINGLE | DMA_MDATAALIGN_BYTE; break;
	case 1: sxcr = DMA_MBURST_SINGLE | DMA_MDATAALIGN_HALFWORD; break;
	case 2:
	case 3: sxcr = DMA_MBURST_SINGLE | DMA_MDATAALIGN_WORD; break;
	default: sxcr = DMA_MBURST_INC4 | DMA_MDATAALIGN_WORD;
	}

	MODIFY_REG(hsd2.hdmarx->Instance->CR, DMA_SxCR_MSIZE | DMA_SxCR_MBURST, sxcr);
}

static int sdmmc_wait_ready() {
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
	if (!sd_diskio_flags) {
		sd_diskio_flags = xEventGroupCreate();
		if (!sd_diskio_flags) {
			Error_Handler();
		}
		vGrantAccessToEventGroup(NULL, sd_diskio_flags);
	}

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
	hsd2.Instance = SDMMC2;
	hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	hsd2.Init.ClockDiv = SDMMC_INIT_CLK_DIV;

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

	hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd2.Init.ClockDiv = SDMMC_TRANSFER_CLK_DIV;

	/* Enable wide operation */
	if (HAL_SD_ConfigWideBusOperation(&hsd2, hsd2.Init.BusWide) != HAL_OK) {
		xEventGroupSetBits(sd_diskio_flags, STA_NOINIT);
	}

	return disk_sdmmc_status();
}

DRESULT disk_sdmmc_read (BYTE* buff, LBA_t sector, UINT count) {
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}
	DRESULT res;
	if ((res = sdmmc_wait_ready()) != RES_OK) {
		return res;
	}

	sdmmc_config_dma_stream(buff);

	if (HAL_SD_ReadBlocks_DMA(&hsd2, buff, (uint32_t)sector, count) != HAL_OK) {
		return RES_ERROR;
	}

	if (xEventGroupWaitBits(
			sd_diskio_flags,
			SD_DISKIO_TRANSFER_CPLT | SD_DISKIO_TRANSFER_ERROR | SD_DISKIO_TRANSFER_ABORTED,
			pdTRUE,
			pdFALSE,
			pdMS_TO_TICKS(SDMMC_TIMEOUT_MS)
	) != SD_DISKIO_TRANSFER_CPLT) {
		return RES_ERROR;
	}
	return RES_OK;
}

DRESULT disk_sdmmc_write (const BYTE* buff, LBA_t sector, UINT count) {
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}
	DRESULT res;
	if ((res = sdmmc_wait_ready()) != RES_OK) {
		return res;
	}

	sdmmc_config_dma_stream(buff);

	if (HAL_SD_WriteBlocks_DMA(&hsd2, (uint8_t*)buff, (uint32_t)sector, count) != HAL_OK) {
		return RES_ERROR;
	}

	if (xEventGroupWaitBits(
			sd_diskio_flags,
			SD_DISKIO_TRANSFER_CPLT | SD_DISKIO_TRANSFER_ERROR | SD_DISKIO_TRANSFER_ABORTED,
			pdTRUE,
			pdFALSE,
			pdMS_TO_TICKS(SDMMC_TIMEOUT_MS)
	) != SD_DISKIO_TRANSFER_CPLT) {
		return RES_ERROR;
	}
	return RES_OK;
}

DRESULT disk_sdmmc_ioctl (BYTE cmd, void* buff) {
	if (disk_sdmmc_status() & STA_NOINIT) {
		return RES_NOTRDY;
	}

	switch (cmd) {
	case CTRL_SYNC:
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
