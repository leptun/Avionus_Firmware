#include "module_manager.hpp"
#include <modules/gps.hpp>
#include <modules/krpc_client.hpp>
#include <modules/servo.hpp>
#include <AppMain.hpp>

namespace modules {

servo::Servo sv;

Module * moduleList[] = {
	&sv
};

void Update() {
	for (Module *mod : moduleList) {
		mod->Update();
	}
}

void Cycle() {
	for (Module *mod : moduleList) {
		mod->Cycle();
	}
}

void NotifyWork() {
	xTaskNotifyIndexed(AppMain::pxTaskHandle, 0, AppMain::FLAG_MODULES, eSetBits);
}

}
