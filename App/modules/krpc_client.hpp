#pragma once
#include <inttypes.h>
#include "module.hpp"
#include <defs.hpp>
#include <krpc_cnano.h>

namespace modules {
namespace krpc_client {

class KrpcClient : public Module {
public:
	void Init() override;
	bool Update() override;
	void Cycle() override;

	void NotifyCommRx();
	void NotifyCommTx();
	void NotifyCommLineState();

	krpc_error_t krpc_open();
	krpc_error_t krpc_close();
	krpc_error_t krpc_read(uint8_t *buf, size_t count);
	krpc_error_t krpc_write(const uint8_t *buf, size_t count);
private:
	enum FlagDef0 {
		FLAG0_CYCLE = 0x000001,
		FLAG0_COMM_LINE_STATE = 0x000002,
	};

	enum FlagDef1 {
		FLAG1_COMM_RX = 0x000001,
		FLAG1_COMM_TX = 0x000002,
		FLAG1_COMM_LINE_STATE = 0x000004,
	};

	enum class InternalStates {
		Init = 0,
		Connected,
		Flight,
		Processing,
	} state;

	enum class BulkState {
		DISABLED = 0,
		RECEIVE = 1,
		SEND = 2,
	} bulkState;

	TaskHandle_t px_krpc_client_TaskHandle;
	defs::Flight plane_flight;
	defs::Control plane_control;

	static void task_krpc_client_Main_StaticWrapper(void *pvParameters);
	void task_krpc_client_Main();
	static const TaskParameters_t krpc_clientTaskDefinition;
};

}
}
