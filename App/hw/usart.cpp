#include "usart.hpp"
#include <assert.h>
#include <stdio.h>
#include <config.hpp>
#include <util.hpp>

namespace hw {
namespace usart {

void USART::Setup() {
	hwdef->MX_USARTx_Init();
	flags = xEventGroupCreate();
	assert(flags);
	if (hwdef->rxBuf) {
		if (hwdef->rxDMA.DMAx) {
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
	if (flags) {
		vGrantAccessToEventGroup(task, flags);
	}
}

void USART::receive(uint8_t *buf, size_t len) {
	while (len > 0) {
		while (rxHead == rxTail) {
			xEventGroupWaitBits(flags, FLAG_RX_AVAILABLE, pdTRUE, pdTRUE, portMAX_DELAY);
		}
		uint32_t newTail = rxTail;
		*(buf++) = hwdef->rxBuf[rxTail++];
		if (newTail > hwdef->rxBufSize) {
			newTail = 0;
		}
		rxTail = newTail;
	}
}

void USART::send(const uint8_t *buf, size_t len) {
	if (len <= 0) {
		return;
	}
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

	xEventGroupWaitBits(flags, FLAG_TX_COMPLETE, pdTRUE, pdTRUE, portMAX_DELAY);
}

void USART::setBaud(uint32_t baud) {
	LL_USART_DisableDirectionRx(hwdef->USARTx);
	if (hwdef->rxDMA.DMAx) {
		LL_USART_DisableDMAReq_RX(hwdef->USARTx);
	}
	if (hwdef->txDMA.DMAx) {
		LL_USART_DisableDMAReq_TX(hwdef->USARTx);
	}
	LL_USART_Disable(hwdef->USARTx);

	LL_USART_SetBaudRate(hwdef->USARTx, hwdef->periphclk, LL_USART_GetOverSampling(hwdef->USARTx), baud);

	LL_USART_Enable(hwdef->USARTx);
	if (hwdef->txDMA.DMAx) {
		LL_USART_EnableDMAReq_TX(hwdef->USARTx);
	}
	if (hwdef->rxDMA.DMAx) {
		LL_USART_EnableDMAReq_RX(hwdef->USARTx);
	}
	LL_USART_EnableDirectionRx(hwdef->USARTx);
}

BaseType_t USART::rx_push() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint32_t newHead = hwdef->rxBufSize - LL_DMA_GetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Stream);
	if (newHead != rxHead && xEventGroupSetBitsFromISR(flags, FLAG_RX_AVAILABLE, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
	rxHead = newHead;

	return xHigherPriorityTaskWoken;
}

void USART::irq_usart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_IDLE(hwdef->USARTx) && LL_USART_IsEnabledIT_IDLE(hwdef->USARTx)) {
		LL_USART_ClearFlag_IDLE(hwdef->USARTx);
		xHigherPriorityTaskWoken |= rx_push();
	}
	if (LL_USART_IsActiveFlag_RXNE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		uint32_t newHead = rxHead;
		hwdef->rxBuf[newHead++] = LL_USART_ReceiveData8(hwdef->USARTx);
		if (newHead >= hwdef->rxBufSize) {
			newHead = 0;
		}
		rxHead = newHead;
		if (xEventGroupSetBitsFromISR(flags, FLAG_RX_AVAILABLE, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
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

		if (xEventGroupSetBitsFromISR(flags, FLAG_TX_COMPLETE, &xHigherPriorityTaskWoken) != pdPASS) {
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

static uint8_t usart3_rxbuf[64] ALIGN_CACHE;
static constexpr USART_Def usart3_def = {
	MX_USART3_UART_Init,
	util::ioCast<USART_TypeDef>(USART3_BASE),
	config::clocks::hclk,
	usart3_rxbuf,
	sizeof(usart3_rxbuf),
	util::LL_DMA_STREAM(util::ioCast<DMA_TypeDef>(DMA1_BASE), 1), //rx
	util::LL_DMA_STREAM(util::ioCast<DMA_TypeDef>(DMA1_BASE), 4), //tx
};
USART usart3(&usart3_def);
extern "C" void USART3_IRQHandler(void) { usart3.irq_usart(); }
extern "C" void DMA1_Stream1_IRQHandler(void) { usart3.irq_dma_rx(); }
extern "C" void DMA1_Stream4_IRQHandler(void) { usart3.irq_dma_tx(); }


static uint8_t uart4_rxbuf[64] ALIGN_CACHE;
static constexpr USART_Def uart4_def = {
	MX_UART4_Init,
	util::ioCast<USART_TypeDef>(UART4_BASE),
	config::clocks::pclk1,
	uart4_rxbuf,
	sizeof(uart4_rxbuf),
	util::LL_DMA_STREAM(util::ioCast<DMA_TypeDef>(DMA1_BASE), 2), //rx
};
USART uart4(&uart4_def);
extern "C" void UART4_IRQHandler(void) { uart4.irq_usart(); }
extern "C" void DMA1_Stream2_IRQHandler(void) { uart4.irq_dma_rx(); }


static uint8_t uart5_rxbuf[config::gps_rxbuf_size] ALIGN_CACHE;
static constexpr USART_Def uart5_def = {
	MX_UART5_Init,
	util::ioCast<USART_TypeDef>(UART5_BASE),
	config::clocks::pclk1,
	uart5_rxbuf,
	sizeof(uart5_rxbuf),
	util::LL_DMA_STREAM(util::ioCast<DMA_TypeDef>(DMA1_BASE), 0), //rx
};
USART uart5(&uart5_def);
extern "C" void UART5_IRQHandler(void) { uart5.irq_usart(); }
extern "C" void DMA1_Stream0_IRQHandler(void) { uart5.irq_dma_rx(); }

}
}
