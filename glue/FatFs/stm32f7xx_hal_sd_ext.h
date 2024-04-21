#pragma once

#include <stm32f7xx_hal.h>

#define SD_CONTEXT_UNINTERRUPTED 0x00000100U

#ifdef __cplusplus
extern "C" {
#endif

extern HAL_StatusTypeDef HAL_SD_ReadBlocksUninterrupted_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
extern HAL_StatusTypeDef HAL_SD_WriteBlocksUninterrupted_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
extern HAL_StatusTypeDef HAL_SD_EXT_Sync(SD_HandleTypeDef *hsd);
extern uint8_t HAL_SD_EXT_IRQHandler(SD_HandleTypeDef *hsd);

#ifdef __cplusplus
}
#endif
