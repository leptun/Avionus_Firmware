#include "gps.hpp"
#include <config.hpp>
#include <ubx.h>

namespace modules {
namespace gps {

#define com (*config::gps_usart)

bfs::Ubx gnss;


void Setup() {
	com.Setup();
	HAL_GPIO_WritePin(GPS_NRESET_GPIO_Port, GPS_NRESET_Pin, GPIO_PIN_SET);
}

}
}
