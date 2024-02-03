#pragma once
#include <inttypes.h>
#include <main.h>
#include <util.hpp>
#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>

namespace modules {
namespace usart {

struct USART_Def {
	void (*MX_USARTx_Init)(void);
	USART_TypeDef *USARTx;
	uint32_t periphclk;
	uint8_t *rxBuf;
	uint16_t rxBufSize;
	util::LL_DMA_STREAM rxDMA;
	util::LL_DMA_STREAM txDMA;
};

class USART {
	enum EventFlags {
		FLAG_RX_AVAILABLE = 0x01,
		FLAG_TX_COMPLETE = 0x02,
	};

	const USART_Def *hwdef;
	EventGroupHandle_t flags;
	uint32_t rxHead;
	uint32_t rxTail;
	const uint8_t *txbuf;
	size_t txndtr;

public:
	USART(const USART_Def *hwdef) : hwdef(hwdef) { }
	void Setup();
	void GrantAccess(TaskHandle_t task);

	void receive(uint8_t *buf, size_t len);
	void send(const uint8_t *buf, size_t len);
	void setBaud(uint32_t baud);

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
