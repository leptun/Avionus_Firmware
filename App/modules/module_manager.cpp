#include "module_manager.hpp"
#include "module.hpp"
#include "modules.hpp"
#include <AppMain.hpp>

namespace modules {

static Module *const moduleList[] = {
//	&sv,
	&krpc,
};

void Init() {
	for (Module *mod : moduleList) {
		mod->Init();
	}
}

void Update() {
	bool moreWork;
	do {
		moreWork = false;
		for (Module *mod : moduleList) {
			moreWork |= mod->Update();
		}
	} while (moreWork);
}

void Cycle() {
	for (Module *mod : moduleList) {
		mod->Cycle();
	}
}

void NotifyWork() {
	if (xTaskNotifyIndexed(AppMain::pxTaskHandle, 0, AppMain::FLAG_MODULES, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

}
