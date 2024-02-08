#include "gps.hpp"
#include <config.hpp>

namespace modules {
namespace gps {

#define com (*config::gps_usart)


void Setup() {
	com.Setup();
	HAL_GPIO_WritePin(GPS_NRESET_GPIO_Port, GPS_NRESET_Pin, GPIO_PIN_SET);
}

}
}
