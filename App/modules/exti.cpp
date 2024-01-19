#include "exti.hpp"

namespace modules {
namespace exti {


EventGroupHandle_t exti_flags;

static void (* const extiHandlers[])(void) = {
	Error_Handler,
	Error_Handler,
	Error_Handler,
	Error_Handler,
	Error_Handler,
	Error_Handler,
	Error_Handler,
	Error_Handler,
	exti23_handler,
	exti22_handler,
	exti21_handler,
	exti20_handler,
	exti19_handler,
	exti18_handler,
	exti17_handler,
	exti16_handler,
	exti15_handler,
	exti14_handler,
	exti13_handler,
	exti12_handler,
	exti11_handler,
	exti10_handler,
	exti9_handler,
	exti8_handler,
	exti7_handler,
	exti6_handler,
	exti5_handler,
	exti4_handler,
	exti3_handler,
	exti2_handler,
	exti1_handler,
	exti0_handler,
};

void Setup(void) {
	exti_flags = xEventGroupCreate();
	if (!exti_flags) {
		Error_Handler();
	}
}

static void IrqHandler(void) {
	BaseType_t xHigherPriorityTaskWoken, xResult = pdFAIL;

	uint32_t flags = LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_ALL) & READ_BIT(EXTI->IMR, LL_EXTI_LINE_ALL);
	LL_EXTI_ClearFlag_0_31(flags);

	if (exti_flags) {
		xResult = xEventGroupSetBitsFromISR(exti_flags, flags, &xHigherPriorityTaskWoken);
	}

	while (flags) {
		uint32_t hidx = __CLZ(flags);
		extiHandlers[hidx]();
		flags &= ~(1 << (31 - hidx));
	}

	if(xResult != pdFAIL) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

extern "C" void EXTI0_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI1_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI2_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI3_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI4_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI9_5_IRQHandler(void) { IrqHandler(); }
extern "C" void EXTI15_10_IRQHandler(void) { IrqHandler(); }
extern "C" void PVD_IRQHandler(void) { IrqHandler(); }
extern "C" void RTC_Alarm_IRQHandler(void) { IrqHandler(); }
extern "C" void OTG_FS_WKUP_IRQHandler(void) { IrqHandler(); }
//exti19 reserved
extern "C" void OTG_HS_WKUP_IRQHandler(void) { IrqHandler(); }
extern "C" void TAMP_STAMP_IRQHandler(void) { IrqHandler(); }
extern "C" void RTC_WKUP_IRQHandler(void) { IrqHandler(); }
extern "C" void LPTIM1_IRQHandler(void) { IrqHandler(); }

__weak void exti0_handler(void) { }
__weak void exti1_handler(void) { }
__weak void exti2_handler(void) { }
__weak void exti3_handler(void) { }
__weak void exti4_handler(void) { }
__weak void exti5_handler(void) { }
__weak void exti6_handler(void) { }
__weak void exti7_handler(void) { }
__weak void exti8_handler(void) { }
__weak void exti9_handler(void) { }
__weak void exti10_handler(void) { }
__weak void exti11_handler(void) { }
__weak void exti12_handler(void) { }
__weak void exti13_handler(void) { }
__weak void exti14_handler(void) { }
__weak void exti15_handler(void) { }
__weak void exti16_handler(void) { }
__weak void exti17_handler(void) { }
__weak void exti18_handler(void) { }
__weak void exti19_handler(void) { }
__weak void exti20_handler(void) { }
__weak void exti21_handler(void) { }
__weak void exti22_handler(void) { }
__weak void exti23_handler(void) { }

}
}
