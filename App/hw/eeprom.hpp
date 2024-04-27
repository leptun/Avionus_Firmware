#pragma once
#include <dump.hpp>

namespace hw {
namespace eeprom {

void Setup();

struct EEPROM {
	crash_dump::info_t dump_header;
};

extern EEPROM data;

}
}
