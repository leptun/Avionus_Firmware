#pragma once
#include <inttypes.h>
#include <main.h>
#include <util.hpp>
#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>

namespace modules {
namespace usart {

struct USART_Def {
	void (*MX_USARTx_Init)(void);
	USART_TypeDef *USARTx;
	uint8_t *rxBuf;
	uint16_t rxBufSize;
	util::LL_DMA_STREAM rxDMA;
	util::LL_DMA_STREAM txDMA;
};

class USART {
	const USART_Def *hwdef;
	StreamBufferHandle_t stream_rx;
	uint32_t rxTail;
	const uint8_t *txbuf;
	size_t txndtr;
	TaskHandle_t tx_task;

public:
	USART(const USART_Def *hwdef) : hwdef(hwdef) { }
	void Setup(size_t rxStreamSize);
	void GrantAccess(TaskHandle_t task);

	void send(const uint8_t *buf, size_t len);

private:
	BaseType_t rx_push();
public:
	void irq_usart();
	void irq_dma_rx();
	void irq_dma_tx();
};

extern USART usart3;
extern USART uart4;
extern USART uart5;

}
}
