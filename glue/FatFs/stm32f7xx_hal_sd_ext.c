#include "stm32f7xx_hal_sd_ext.h"
#include <stm32f7xx_hal.h>

/**
  * @brief  DMA SD transmit process complete callback
  * @param  hdma: DMA handle
  * @retval None
  */
static void SD_DMATransmitUninterruptedCplt(DMA_HandleTypeDef *hdma)
{
  SD_HandleTypeDef* hsd = (SD_HandleTypeDef* )(hdma->Parent);

  /* Enable DATAEND Interrupt */
  __HAL_SD_ENABLE_IT(hsd, (SDMMC_IT_DATAEND));
}

/**
  * @brief  DMA SD receive process complete callback
  * @param  hdma: DMA handle
  * @retval None
  */
static void SD_DMAReceiveUninterruptedCplt(DMA_HandleTypeDef *hdma)
{
  SD_HandleTypeDef* hsd = (SD_HandleTypeDef* )(hdma->Parent);

  hsd->Instance->DCTRL &= ~(SDMMC_DCTRL_DMAEN | SDMMC_DCTRL_DTEN);

  __HAL_SD_DISABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR));

  /* Clear all the static flags */
  __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

  hsd->State = HAL_SD_STATE_TRANSFER;

#if (USE_HAL_SD_REGISTER_CALLBACKS == 1)
  hsd->RxCpltCallback(hsd);
#else
  HAL_SD_RxCpltCallback(hsd);
#endif
}

/**
  * @brief  DMA SD communication error callback
  * @param  hdma: DMA handle
  * @retval None
  */
static void SD_DMAUninterruptedError(DMA_HandleTypeDef *hdma)
{
  SD_HandleTypeDef* hsd = (SD_HandleTypeDef* )(hdma->Parent);
  uint32_t RxErrorCode, TxErrorCode;

  /* if DMA error is FIFO error ignore it */
  if(HAL_DMA_GetError(hdma) != HAL_DMA_ERROR_FE)
  {
    RxErrorCode = hsd->hdmarx->ErrorCode;
    TxErrorCode = hsd->hdmatx->ErrorCode;
    if((RxErrorCode == HAL_DMA_ERROR_TE) || (TxErrorCode == HAL_DMA_ERROR_TE))
    {
      /* Clear All flags */
      __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

      /* Disable All interrupts */
      __HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT|\
        SDMMC_IT_TXUNDERR| SDMMC_IT_RXOVERR);

      hsd->ErrorCode |= HAL_SD_ERROR_DMA;

      hsd->State = HAL_SD_STATE_ERROR;
      hsd->Context = SD_CONTEXT_NONE;
    }

#if (USE_HAL_SD_REGISTER_CALLBACKS == 1)
    hsd->ErrorCallback(hsd);
#else
    HAL_SD_ErrorCallback(hsd);
#endif
  }
}

static void SD_DMATxUninterruptedAbort(DMA_HandleTypeDef *hdma)
{
  SD_HandleTypeDef* hsd = (SD_HandleTypeDef* )(hdma->Parent);

  /* Clear All flags */
  __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

  hsd->State = HAL_SD_STATE_ERROR;
  hsd->Context = SD_CONTEXT_NONE;

#if (USE_HAL_SD_REGISTER_CALLBACKS == 1)
  hsd->AbortCpltCallback(hsd);
#else
  HAL_SD_AbortCallback(hsd);
#endif
}

static void SD_DMARxUninterruptedAbort(DMA_HandleTypeDef *hdma)
{
  SD_HandleTypeDef* hsd = (SD_HandleTypeDef* )(hdma->Parent);

  /* Clear All flags */
  __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

  hsd->State = HAL_SD_STATE_ERROR;
  hsd->Context = SD_CONTEXT_NONE;

#if (USE_HAL_SD_REGISTER_CALLBACKS == 1)
  hsd->AbortCpltCallback(hsd);
#else
  HAL_SD_AbortCallback(hsd);
#endif
}


HAL_StatusTypeDef HAL_SD_ReadBlocksUninterrupted_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
  SDMMC_DataInitTypeDef config;
  uint32_t errorstate;
  uint32_t add = BlockAdd;

  if(NULL == pData)
  {
    hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
    return HAL_ERROR;
  }

  if(hsd->State == HAL_SD_STATE_READY || hsd->State == HAL_SD_STATE_TRANSFER)
  {
    hsd->ErrorCode = HAL_SD_ERROR_NONE;

    if((add + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
    {
      hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
      return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_TRANSFER && (hsd->Context & (SD_CONTEXT_UNINTERRUPTED))) {
    	if ((hsd->Context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK)
    		|| ((hsd->Context & SD_CONTEXT_READ_MULTIPLE_BLOCK) && hsd->RxXferSize != BlockAdd)
    	) {
            hsd->Instance->DCTRL = 0U;

    		hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
    		hsd->State = HAL_SD_STATE_READY;
    		return HAL_BUSY; // retry after waiting for the TRANSFER state of the card
    	}
    }
    else {
        hsd->Instance->DCTRL = 0U;

    	hsd->State = HAL_SD_STATE_BUSY;
    }

    if (HAL_DMA_GetState(hsd->hdmarx) != HAL_DMA_STATE_READY) {
    	hsd->ErrorCode |= HAL_SD_ERROR_DMA;
		hsd->State = HAL_SD_STATE_ERROR;
		return HAL_ERROR;
    }

    __HAL_SD_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR));


    /* Set the DMA transfer complete callback */
    hsd->hdmarx->XferCpltCallback = SD_DMAReceiveUninterruptedCplt;

    /* Set the DMA error callback */
    hsd->hdmarx->XferErrorCallback = SD_DMAUninterruptedError;

    /* Set the DMA Abort callback */
    hsd->hdmarx->XferAbortCallback = SD_DMAUninterruptedError;

    /* Force DMA Direction */
    hsd->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    MODIFY_REG(hsd->hdmarx->Instance->CR, DMA_SxCR_DIR, hsd->hdmarx->Init.Direction);

    /* Enable the DMA Channel */
    if(HAL_DMA_Start_IT(hsd->hdmarx, (uint32_t)&hsd->Instance->FIFO, (uint32_t)pData, (uint32_t)(BLOCKSIZE * NumberOfBlocks)/4U) != HAL_OK)
    {
      __HAL_SD_DISABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR));
      __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
      hsd->ErrorCode |= HAL_SD_ERROR_DMA;
      hsd->State = HAL_SD_STATE_ERROR;
      return HAL_ERROR;
    }
    else
    {
      /* Enable SD DMA transfer */
      __HAL_SD_DMA_ENABLE(hsd);

      if(hsd->SdCard.CardType != CARD_SDHC_SDXC)
      {
        add *= 512U;
      }

      /* Configure the SD DPSM (Data Path State Machine) */
      config.DataTimeOut   = SDMMC_DATATIMEOUT;
      config.DataLength    = BLOCKSIZE * NumberOfBlocks;
      config.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
      config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
      config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
      config.DPSM          = SDMMC_DPSM_ENABLE;
      (void)SDMMC_ConfigData(hsd->Instance, &config);

      if (hsd->State == HAL_SD_STATE_BUSY) {
			/* Read Blocks in DMA mode */
			hsd->Context = (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_UNINTERRUPTED);

			/* Read Multi Block command */
			errorstate = SDMMC_CmdReadMultiBlock(hsd->Instance, add);
		  if(errorstate != HAL_SD_ERROR_NONE)
		  {
			/* Clear all the static flags */
			__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
			hsd->ErrorCode |= errorstate;
			hsd->State = HAL_SD_STATE_READY;
			hsd->Context = SD_CONTEXT_NONE;
			return HAL_ERROR;
		  }
      }

      hsd->RxXferSize = (BlockAdd + NumberOfBlocks);

      return HAL_OK;
    }
  }
  else
  {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef HAL_SD_WriteBlocksUninterrupted_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
  SDMMC_DataInitTypeDef config;
  uint32_t errorstate;
  uint32_t add = BlockAdd;

  if(NULL == pData)
  {
    hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
    return HAL_ERROR;
  }

  if(hsd->State == HAL_SD_STATE_READY || hsd->State == HAL_SD_STATE_TRANSFER)
  {
    hsd->ErrorCode = HAL_SD_ERROR_NONE;

    if((add + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
    {
      hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
      return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_TRANSFER && (hsd->Context & (SD_CONTEXT_UNINTERRUPTED))) {
    	if ((hsd->Context & SD_CONTEXT_READ_MULTIPLE_BLOCK)
    		|| ((hsd->Context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) && hsd->TxXferSize != BlockAdd)
    	) {
    		hsd->Instance->DCTRL = 0U;

    		hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
    		hsd->State = HAL_SD_STATE_READY;
    		return HAL_BUSY; // retry after waiting for the TRANSFER state of the card
    	}
    }
    else {
    	hsd->Instance->DCTRL = 0U;

    	hsd->State = HAL_SD_STATE_BUSY;
    }

    if (HAL_DMA_GetState(hsd->hdmatx) != HAL_DMA_STATE_READY) {
		hsd->ErrorCode |= HAL_SD_ERROR_DMA;
		hsd->State = HAL_SD_STATE_ERROR;
		return HAL_ERROR;
	}

    /* Enable SD Error interrupts */
    __HAL_SD_ENABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR));

    /* Set the DMA transfer complete callback */
    hsd->hdmatx->XferCpltCallback = SD_DMATransmitUninterruptedCplt;

    /* Set the DMA error callback */
    hsd->hdmatx->XferErrorCallback = SD_DMAUninterruptedError;

    /* Set the DMA Abort callback */
    hsd->hdmatx->XferAbortCallback = SD_DMAUninterruptedError;

    if(hsd->SdCard.CardType != CARD_SDHC_SDXC)
    {
      add *= 512U;
    }

    if (hsd->State == HAL_SD_STATE_BUSY) {
		/* Write Blocks in DMA mode */
		hsd->Context = (SD_CONTEXT_WRITE_MULTIPLE_BLOCK | SD_CONTEXT_UNINTERRUPTED);

		/* Write Multi Block command */
		errorstate = SDMMC_CmdWriteMultiBlock(hsd->Instance, add);

		if(errorstate != HAL_SD_ERROR_NONE)
		{
		  /* Clear all the static flags */
		  __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
		  hsd->ErrorCode |= errorstate;
		  hsd->State = HAL_SD_STATE_READY;
		  hsd->Context = SD_CONTEXT_NONE;
		  return HAL_ERROR;
		}
    }

    /* Enable SDMMC DMA transfer */
    __HAL_SD_DMA_ENABLE(hsd);

    /* Force DMA Direction */
    hsd->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    MODIFY_REG(hsd->hdmatx->Instance->CR, DMA_SxCR_DIR, hsd->hdmatx->Init.Direction);

    /* Enable the DMA Channel */
    if(HAL_DMA_Start_IT(hsd->hdmatx, (uint32_t)pData, (uint32_t)&hsd->Instance->FIFO, (uint32_t)(BLOCKSIZE * NumberOfBlocks)/4U) != HAL_OK)
    {
      __HAL_SD_DISABLE_IT(hsd, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR));
      __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
      hsd->ErrorCode |= HAL_SD_ERROR_DMA;
      hsd->State = HAL_SD_STATE_READY;
      hsd->Context = SD_CONTEXT_NONE;
      return HAL_ERROR;
    }
    else
    {
      /* Configure the SD DPSM (Data Path State Machine) */
      config.DataTimeOut   = SDMMC_DATATIMEOUT;
      config.DataLength    = BLOCKSIZE * NumberOfBlocks;
      config.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
      config.TransferDir   = SDMMC_TRANSFER_DIR_TO_CARD;
      config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
      config.DPSM          = SDMMC_DPSM_ENABLE;
      (void)SDMMC_ConfigData(hsd->Instance, &config);

      hsd->TxXferSize = (BlockAdd + NumberOfBlocks);

      return HAL_OK;
    }
  }
  else
  {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef HAL_SD_EXT_Sync(SD_HandleTypeDef *hsd) {
	if (hsd->State == HAL_SD_STATE_TRANSFER) {
		hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
		if (hsd->ErrorCode == HAL_SD_ERROR_NONE) {
			hsd->State = HAL_SD_STATE_READY;
			return HAL_OK;
		}
		else {
			return HAL_ERROR;
		}
	} else {
		return HAL_BUSY;
	}
}

uint8_t HAL_SD_EXT_IRQHandler(SD_HandleTypeDef *hsd) {
	uint32_t context = hsd->Context;
	if (context & SD_CONTEXT_UNINTERRUPTED) {
		if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DATAEND)) {
			if (context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) {
				hsd->Instance->DCTRL &= ~(SDMMC_DCTRL_DMAEN | SDMMC_DCTRL_DTEN);
					__HAL_SD_DISABLE_IT(hsd, (SDMMC_FLAG_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR));

					/* Clear all the static flags */
					__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

					hsd->State = HAL_SD_STATE_TRANSFER;

#if defined (USE_HAL_SD_REGISTER_CALLBACKS) && (USE_HAL_SD_REGISTER_CALLBACKS == 1U)
					hsd->TxCpltCallback(hsd);
#else
					HAL_SD_TxCpltCallback(hsd);
#endif /* USE_HAL_SD_REGISTER_CALLBACKS */
			}
			else {
				// DATAEND not used for uninterrupted block reads, dma TC flag is used instead
				__HAL_SD_DISABLE_IT(hsd, (SDMMC_FLAG_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR));
			}
		}
		else if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR | SDMMC_FLAG_TXUNDERR) != RESET) {
			__HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);
			__HAL_SD_DISABLE_IT(hsd, SDMMC_IT_DATAEND | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR);
			if (context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) {
				if (HAL_DMA_Abort_IT(hsd->hdmatx) != HAL_OK) {
					SD_DMATxUninterruptedAbort(hsd->hdmatx);
				}
			} else if (context & SD_CONTEXT_READ_MULTIPLE_BLOCK) {
				if(HAL_DMA_Abort_IT(hsd->hdmarx) != HAL_OK) {
					SD_DMARxUninterruptedAbort(hsd->hdmarx);
		        }
			} else {
				hsd->ErrorCode = HAL_SD_ERROR_NONE;
				hsd->State = HAL_SD_STATE_READY;
				hsd->Context = SD_CONTEXT_NONE;
#if defined (USE_HAL_SD_REGISTER_CALLBACKS) && (USE_HAL_SD_REGISTER_CALLBACKS == 1U)
				hsd->AbortCpltCallback(hsd);
#else
				HAL_SD_AbortCallback(hsd);
#endif /* USE_HAL_SD_REGISTER_CALLBACKS */
			}
		}
		return 1;
	}
	else {
		return 0;
	}
}
