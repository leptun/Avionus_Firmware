#include "diskio_sdmmc.h"
#include "diskio.h"
#include "ff.h"
#include <main.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"


extern SD_HandleTypeDef hsd2;
static volatile DSTATUS sta = STA_NOINIT;
EventGroupHandle_t sd_diskio_flags;
enum FlagDef {
	SD_DISKIO_TRANSFER_CPLT = 0x01,
};

static int sdmmc_card_detected() {
	return HAL_GPIO_ReadPin(SD_CARD_CD_GPIO_Port, SD_CARD_CD_Pin) == GPIO_PIN_RESET;
}

static int sdmmc_card_write_protected() {
	return HAL_GPIO_ReadPin(SD_CARD_WP_GPIO_Port, SD_CARD_WP_Pin) == GPIO_PIN_RESET;
}

static int sdmmc_wait_ready() {
	TickType_t xTicksToWait = pdMS_TO_TICKS(1000);
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	while (HAL_SD_GetCardState(&hsd2) != HAL_SD_CARD_TRANSFER) {
		if (sta & STA_NODISK) {
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
	return sta;
}

DSTATUS disk_sdmmc_initialize(void) {
	if (!sd_diskio_flags) {
		sd_diskio_flags = xEventGroupCreate();
		if (!sd_diskio_flags) {
			Error_Handler();
		}
		vGrantAccessToEventGroup(NULL, sd_diskio_flags);
	}

	sta = sdmmc_card_write_protected() ? STA_PROTECT : 0;
	/* uSD device interface configuration */
	hsd2.Instance = SDMMC2;
	hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	hsd2.Init.ClockDiv = SDMMC_INIT_CLK_DIV;

	/* Check if the SD card is plugged in the slot */
	if (!sdmmc_card_detected()) {
		sta = STA_NOINIT | STA_NODISK;
		return sta;
	}

	if (HAL_SD_Init(&hsd2) != HAL_OK) {
		sta = STA_NOINIT;
		return sta;
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
		sta = STA_NOINIT;
		return sta;
	}

	return sta;
}

DRESULT disk_sdmmc_read (BYTE* buff, LBA_t sector, UINT count) {
	if (sta & STA_NOINIT) {
		return RES_NOTRDY;
	}
	DRESULT res;
	if ((res = sdmmc_wait_ready()) != RES_OK) {
		return res;
	}

	uint32_t msize;
	if ((uint32_t)buff & 0x01) {
		msize = DMA_MDATAALIGN_BYTE;
	}
	else if ((uint32_t)buff & 0x02) {
		msize = DMA_MDATAALIGN_HALFWORD;
	}
	else {
		msize = DMA_MDATAALIGN_WORD;
	}
	MODIFY_REG(hsd2.hdmarx->Instance->CR, DMA_SxCR_MSIZE, msize);

	if (HAL_SD_ReadBlocks_DMA(&hsd2, buff, (uint32_t)sector, count) != HAL_OK) {
		return RES_ERROR;
	}

	if (xEventGroupWaitBits(sd_diskio_flags, SD_DISKIO_TRANSFER_CPLT, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)) != SD_DISKIO_TRANSFER_CPLT) {
		return RES_ERROR;
	}
	return RES_OK;
}

DRESULT disk_sdmmc_write (const BYTE* buff, LBA_t sector, UINT count) {
	if (sta & STA_NOINIT) {
		return RES_NOTRDY;
	}
	DRESULT res;
	if ((res = sdmmc_wait_ready()) != RES_OK) {
		return res;
	}

	uint32_t msize;
	if ((uint32_t)buff & 0x01) {
		msize = DMA_MDATAALIGN_BYTE;
	}
	else if ((uint32_t)buff & 0x02) {
		msize = DMA_MDATAALIGN_HALFWORD;
	}
	else {
		msize = DMA_MDATAALIGN_WORD;
	}
	MODIFY_REG(hsd2.hdmarx->Instance->CR, DMA_SxCR_MSIZE, msize);

	if (HAL_SD_WriteBlocks_DMA(&hsd2, (uint8_t*)buff, (uint32_t)sector, count) != HAL_OK) {
		return RES_ERROR;
	}

	if (xEventGroupWaitBits(sd_diskio_flags, SD_DISKIO_TRANSFER_CPLT, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)) != SD_DISKIO_TRANSFER_CPLT) {
		return RES_ERROR;
	}
	return RES_OK;
}

DRESULT disk_sdmmc_ioctl (BYTE cmd, void* buff) {
	if (sta & STA_NOINIT) {
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
