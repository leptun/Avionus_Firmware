#pragma once
#include <inttypes.h>

namespace modules {

class Module {
	enum class State {
		startup = 0,
		powerup,
		init,
		ready,
		cycling,
		fault,
	} state;
public:
	// work on pending action during cycle update.
	// @ret true if work is still to be done
	virtual bool Update() = 0;

	// begin a new cycle
	virtual void Cycle() = 0;

private:
	// module initialization code
	virtual void Setup() = 0;

	// check if all dependencies are met
	virtual bool DependenciesMet() = 0;

	// check if all necessary power rails are enabled
	virtual bool PowerOK() = 0;
};

void Update();
void Cycle();
void NotifyWork();

}
