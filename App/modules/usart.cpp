#include "usart.hpp"
#include <assert.h>
#include <stdio.h>

namespace modules {
namespace usart {

void USART::Setup(size_t stream_rx_size) {
	hwdef->MX_USARTx_Init();
	if (stream_rx_size > 0) {
		stream_rx = xStreamBufferCreate(stream_rx_size, 0);
		if (hwdef->rxBuf) {
			assert(hwdef->rxDMA.DMAx);
			// receive using DMA circular with interrupts for half, complete and UART idle
			LL_USART_EnableDMAReq_RX(hwdef->USARTx);
			LL_DMA_SetPeriphAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_RECEIVE));
			LL_DMA_SetMemoryAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream, (uint32_t)hwdef->rxBuf);
			LL_DMA_SetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream, hwdef->rxBufSize);
			hwdef->rxDMA.clearIRQ(DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0);
			LL_DMA_EnableIT_DME(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
			LL_DMA_EnableIT_TE(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
			LL_DMA_EnableIT_HT(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
			LL_DMA_EnableIT_TC(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
			LL_DMA_EnableStream(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
			LL_USART_ClearFlag_IDLE(hwdef->USARTx);
			LL_USART_EnableIT_IDLE(hwdef->USARTx);
		} else {
			// receive using UART rxne
			LL_USART_EnableIT_RXNE(hwdef->USARTx);
		}
		LL_USART_EnableDirectionRx(hwdef->USARTx);
	}
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA normal with interrupts for complete
		LL_USART_EnableDMAReq_TX(hwdef->USARTx);
		LL_DMA_SetPeriphAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Stream, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_TRANSMIT));
		hwdef->txDMA.clearIRQ(DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_TCIF0);
		LL_DMA_EnableIT_DME(hwdef->txDMA.DMAx, hwdef->txDMA.Stream);
		LL_DMA_EnableIT_TE(hwdef->txDMA.DMAx, hwdef->txDMA.Stream);
		LL_DMA_EnableIT_TC(hwdef->txDMA.DMAx, hwdef->txDMA.Stream);
	}
}

void USART::GrantAccess(TaskHandle_t task) {
	if (stream_rx) {
		vGrantAccessToTask(task, stream_rx);
	}
}

void USART::send(const uint8_t *buf, size_t len) {
	if (len <= 0) {
		return;
	}
	tx_task = xTaskGetCurrentTaskHandle();
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA
		LL_DMA_SetMemoryAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Stream, (uint32_t)buf);
		LL_DMA_SetDataLength(hwdef->txDMA.DMAx, hwdef->txDMA.Stream, len);
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_DMA_EnableStream(hwdef->txDMA.DMAx, hwdef->txDMA.Stream);
	}
	else {
		// transmit using interrupts
		txbuf = buf;
		txndtr = len;
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_USART_EnableIT_TXE(hwdef->USARTx);
	}
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //wait for transmission to complete
}

BaseType_t USART::rx_push() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t dropped = 0;
	uint32_t newTail = hwdef->rxBufSize - LL_DMA_GetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);

	if (newTail == rxTail) {
		// nothing to do
	}
	else if (newTail > rxTail || newTail == 0) {
		uint32_t len = newTail - rxTail;
		dropped = len;
		dropped -= xStreamBufferSendFromISR(stream_rx, hwdef->rxBuf + rxTail, len, &xHigherPriorityTaskWoken);
	}
	else if (newTail < rxTail) {
		uint32_t len = hwdef->rxBufSize - rxTail;
		dropped = len;
		dropped -= xStreamBufferSendFromISR(stream_rx, hwdef->rxBuf + rxTail, len, &xHigherPriorityTaskWoken);
		dropped += newTail;
		dropped -= xStreamBufferSendFromISR(stream_rx, hwdef->rxBuf, newTail, &xHigherPriorityTaskWoken);
	}
	rxTail = newTail;

	if (dropped) {
//		printf("usart: dropped %lu bytes\n", dropped);
	}

	return xHigherPriorityTaskWoken;
}

void USART::irq_usart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_IDLE(hwdef->USARTx) && LL_USART_IsEnabledIT_IDLE(hwdef->USARTx)) {
		LL_USART_ClearFlag_IDLE(hwdef->USARTx);
		xHigherPriorityTaskWoken |= rx_push();
	}
	if (LL_USART_IsActiveFlag_RXNE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		uint8_t b = LL_USART_ReceiveData8(hwdef->USARTx);
		if (!xStreamBufferSendFromISR(stream_rx, &b, 1, &xHigherPriorityTaskWoken)) {
//			puts("usart: dropped");
		}
	}
	if (LL_USART_IsActiveFlag_ORE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		LL_USART_ClearFlag_ORE(hwdef->USARTx);
//		puts("usart: overrun");
	}
	if (LL_USART_IsActiveFlag_TXE(hwdef->USARTx) && LL_USART_IsEnabledIT_TXE(hwdef->USARTx)) {
		LL_USART_TransmitData8(hwdef->USARTx, *(txbuf++));
		if (--txndtr == 0) {
			LL_USART_DisableIT_TXE(hwdef->USARTx);
			LL_USART_ClearFlag_TC(hwdef->USARTx);
			LL_USART_EnableIT_TC(hwdef->USARTx);
		}
	}
	if (LL_USART_IsActiveFlag_TC(hwdef->USARTx) && LL_USART_IsEnabledIT_TC(hwdef->USARTx)) {
		LL_USART_DisableIT_TC(hwdef->USARTx);
		LL_USART_DisableDirectionTx(hwdef->USARTx);
		if (xTaskNotifyFromISR(tx_task, 0, eNoAction, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART::irq_dma_rx() {
	assert(hwdef->rxDMA.DMAx);
	uint32_t flags = hwdef->rxDMA.irq_handler();
	if (flags & (DMA_LISR_HTIF0 | DMA_LISR_TCIF0)) {
		rx_push();
	}
	else {
		Error_Handler();
	}
}

void USART::irq_dma_tx() {
	assert(hwdef->txDMA.DMAx);
	uint32_t flags = hwdef->txDMA.irq_handler();
	if (flags & (DMA_LISR_TCIF0)) {
		LL_USART_EnableIT_TC(hwdef->USARTx);
	}
	else {
		Error_Handler();
	}
}


static uint8_t usart3_rxbuf[64];
static const USART_Def usart3_def = {
	MX_USART3_UART_Init,
	USART3,
	usart3_rxbuf,
	sizeof(usart3_rxbuf),
	util::LL_DMA_STREAM(DMA1, 1), //rx
	util::LL_DMA_STREAM(DMA1, 4), //tx
};
USART usart3(&usart3_def);
extern "C" void USART3_IRQHandler(void) { usart3.irq_usart(); }
extern "C" void DMA1_Stream1_IRQHandler(void) { usart3.irq_dma_rx(); }
extern "C" void DMA1_Stream4_IRQHandler(void) { usart3.irq_dma_tx(); }


static uint8_t uart4_rxbuf[64];
static const USART_Def uart4_def = {
	MX_UART4_Init,
	UART4,
	uart4_rxbuf,
	sizeof(uart4_rxbuf),
	util::LL_DMA_STREAM(DMA1, 2), //rx
};
USART uart4(&uart4_def);
extern "C" void UART4_IRQHandler(void) { uart4.irq_usart(); }
extern "C" void DMA1_Stream2_IRQHandler(void) { uart4.irq_dma_rx(); }


static uint8_t uart5_rxbuf[64];
static const USART_Def uart5_def = {
	MX_UART5_Init,
	UART5,
	uart5_rxbuf,
	sizeof(uart5_rxbuf),
	util::LL_DMA_STREAM(DMA1, 0), //rx
};
USART uart5(&uart5_def);
extern "C" void UART5_IRQHandler(void) { uart5.irq_usart(); }
extern "C" void DMA1_Stream0_IRQHandler(void) { uart5.irq_dma_rx(); }

}
}
