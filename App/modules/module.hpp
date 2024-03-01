#pragma once
#include <inttypes.h>
#include <FreeRTOS.h>
#include <task.h>

namespace modules {

class Module {
protected:
	enum class State {
		noInit = 0,
		init,
		powerup,
		initializing,
		ready,
		busy,
		fault,
	} state;

	// The handle of the task that manages this module
	TaskHandle_t task;
public:
	// Initialization to be called only once
	virtual void Init() {
		task = xTaskGetCurrentTaskHandle();
		state = State::init;
	}

	// work on pending action during cycle update.
	// @ret true if work is still to be done
	virtual bool Update() { return false; }

	// begin a new cycle
	virtual void Cycle() {}

private:
	// check if all dependencies are met
	virtual bool DependenciesMet() { return true; }

	// used to power on or off the module components
	virtual void SetPower(bool val) {}

	// check if all necessary power rails are enabled
	virtual bool PowerOK() { return true; }
};

}
