#pragma once
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// initialized the quad spi flash and switches it into memory mapped mode
void Avionus_QSPILoader_init();

// used by the idle task to deselect the quad spi flash when the cpu enters tickless idle mode and no AHB master is using the flash
void Avionus_QSPILoader_idleTaskEnterLowPower();

// used by tasks that potentially use a dma stream reading from the memory mapped area
void Avionus_QSPILoader_announceDmaStreamBegin();
void Avionus_QSPILoader_announceDmaStreamEnd();

#ifdef __cplusplus
}
#endif
