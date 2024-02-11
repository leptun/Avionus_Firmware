#pragma once
#include <inttypes.h>
#include "module_manager.hpp"

namespace modules {
namespace servo {

class Servo : public Module {
public:
	bool Update() override { return false; }
	void Cycle() override {}
	bool DependenciesMet() override { return true; }
	bool PowerOK() override { return true; }
private:
	void Setup() override;
	int SetPosition(uint32_t servo, uint32_t pos_us);
};

}
}
