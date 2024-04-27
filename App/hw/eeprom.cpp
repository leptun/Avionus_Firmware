#include <hw/eeprom.hpp>
#include <main.h>

namespace hw {
namespace eeprom {

void Setup() {
	LL_PWR_EnableBkUpRegulator();
}

EEPROM data __attribute__((section(".bkpsram")));

}
}
