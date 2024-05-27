#include <inttypes.h>
#include <main.h>
#include <config.hpp>
#include <timing.hpp>

#define cli() __disable_irq()
#define sei() __enable_irq()

#define F_CPU config::system_clock_frequency
#define micros ticks_us
