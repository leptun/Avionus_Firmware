#include "gps.hpp"
#include <config.hpp>
#include <ubx.h>

namespace modules {
namespace gps {

#define com (*config::gps_usart)

bfs::Ubx gnss;


void Setup() {
	com.Setup();
}

}
}
