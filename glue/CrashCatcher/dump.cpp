#include <string.h>
#include "dump.hpp"
#include <fatfs.h>
#include "utility_extensions.hpp"
#include <quadspi.h>
#include <main.h>
#include <regions.h>
#include <hw/eeprom.hpp>
#include "FreeRTOS.h"
#include "task.h"
//#include "safe_state.h"
//#include <wdt.hpp>
#include <algorithm>
extern "C" {
#include "CrashCatcher.h"
}

namespace crash_dump {

/// While dumping, this stores size of already dumped data
static uint32_t dump_size;
static bool dump_breakpoint_paused = false;
static bool wdg_reset_safeguard = false; ///< Safeguard to prevent multiple refresh of watchdog in dump

/// Just random value, used to check that dump is probably valid in flash
inline constexpr uint32_t CRASH_DUMP_MAGIC_NR = 0x3DC53F;

/// Position of dump data
inline constexpr uint32_t dump_data_addr = QSPI_BASE;
/// Max size of dump (header + data)
inline constexpr uint32_t dump_max_size = Regions::__xflash_crash_dump_region_size__;
/// Max size of dump data
inline constexpr uint32_t dump_max_data_size = dump_max_size;

enum {
    // dumped ram area (256kb)
    RAM_ADDR = 0x20000000,
    RAM_SIZE = 0x00040000,

    // dumped itcmram area (16kb except the first word)
    ITCMRAM_ADDR = RAMITCM_BASE + 4,
	ITCMRAM_SIZE = 0x4000 - 4,

    SCB_ADDR = (uintptr_t)SCB_BASE,
    SCB_SIZE = sizeof(SCB_Type),

};

bool dump_is_exported() {
    return !any(hw::eeprom::data.dump_header.dump_flags & DumpFlags::EXPORTED);
}

bool dump_is_valid() {
    return (bool)(
    		(hw::eeprom::data.dump_header.crash_dump_magic_nr == CRASH_DUMP_MAGIC_NR)
			&& hw::eeprom::data.dump_header.dump_size > 0
			&& hw::eeprom::data.dump_header.dump_size <= dump_max_data_size
	);
}

size_t dump_get_size() {
    if (!dump_is_valid()) {
        return 0;
    }
    return hw::eeprom::data.dump_header.dump_size;
}

static void dump_set_flag(const DumpFlags flag) {
	hw::eeprom::data.dump_header.dump_flags = hw::eeprom::data.dump_header.dump_flags & (~flag);
}

void dump_set_exported() {
    dump_set_flag(DumpFlags::EXPORTED);
}

void dump_reset() {
    static_assert((dump_data_addr + dump_max_size) % Regions::XFLASH_SECTOR_SIZE == 0, "More than reserved area is erased.");
    CSP_QSPI_EraseSector(dump_data_addr, dump_data_addr + dump_max_size - 1);
}

bool save_dump_to_sd(const char *fn, FIL *fil) {
	uint8_t retSD = 0;
	f_unlink(fn);
	retSD |= f_open(fil, fn, FA_WRITE | FA_CREATE_NEW);
	if (retSD != FR_OK) {
		return false;
	}
	UINT bw = 0;
	retSD |= f_write(fil, (const uint32_t *)dump_data_addr, hw::eeprom::data.dump_header.dump_size, &bw);
	retSD |= f_close(fil);

	return !retSD && bw == hw::eeprom::data.dump_header.dump_size;
}

static void dump_failed() {
    // nothing left to do here, when dump fails just restart
	NVIC_SystemReset();
}

static const CrashCatcherMemoryRegion regions[] = {
    { crash_dump::SCB_ADDR, crash_dump::SCB_ADDR + crash_dump::SCB_SIZE, CRASH_CATCHER_WORD },
    { crash_dump::RAM_ADDR, crash_dump::RAM_ADDR + crash_dump::RAM_SIZE, CRASH_CATCHER_BYTE },
    { crash_dump::ITCMRAM_ADDR, crash_dump::ITCMRAM_ADDR + crash_dump::ITCMRAM_SIZE, CRASH_CATCHER_BYTE },
    { 0xFFFFFFFF, 0, CRASH_CATCHER_BYTE },
};

void before_dump() {
    // avoid triggering before_dump multiple times (it can be called before BSOD and then again in hardfault handler)
    if (!dump_breakpoint_paused) {
        dump_breakpoint_paused = true;
//        buddy_disable_heaters(); // put HW to safe state
//#ifdef DEBUG
//        if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
//            // if case debugger is attached, issue breakpoint instead of crash dump.
//            // If you still want to do crash dump, resume the processor
//            CRASH_CATCHER_BREAKPOINT();
//        }
//#endif
    }

    // this function is called before WDR or when preparing to dump on flash. Flash erase takes
    // 300ms typically. But according to dataheet, it can take multiple seconds, and we might need
    // to initialize it first. Refresh the watchdog once to give us an additioanl 4s to dump it.
    if (!crash_dump::wdg_reset_safeguard) {
//        wdt_iwdg_refresh();

        // disable the callback itself to avoid WDR recursion and get the last full effective timeout
//        wdt_iwdg_warning_cb = nullptr;
        crash_dump::wdg_reset_safeguard = true;
    }
}

void trigger_crash_dump() {
    before_dump();

    // trigger hardfault, hardfault will dump the processor state
    CRASH_CATCHER_INVALID_INSTRUCTION();

    // just to make function no-return
    while (1) {
    }
}

}

const CrashCatcherMemoryRegion *CrashCatcher_GetMemoryRegions(void) {
    return crash_dump::regions;
}

void CrashCatcher_DumpStart([[maybe_unused]] const CrashCatcherInfo *pInfo) {
	return;
    __disable_irq();
//    vTaskEndScheduler();

    crash_dump::before_dump();

    __HAL_RCC_QSPI_FORCE_RESET();  //completely reset peripheral
    __HAL_RCC_QSPI_RELEASE_RESET();

    if (CSP_QUADSPI_Init()) {
        crash_dump::dump_failed();
    }

    if (crash_dump::dump_is_valid() && !crash_dump::dump_is_exported()) {
        // do not overwrite dump that is already valid & wasn't displayed to user yet
        crash_dump::dump_failed();
    }

    crash_dump::dump_reset();

    crash_dump::dump_size = 0;
}

void CrashCatcher_DumpMemory(const void *pvMemory, CrashCatcherElementSizes element_size, size_t elementCount) {
	return;
    if (element_size == CRASH_CATCHER_BYTE) {
        if (crash_dump::dump_size + elementCount > crash_dump::dump_max_data_size) {
            crash_dump::dump_failed();
        }

        if (CSP_QSPI_WriteMemory((uint8_t *)pvMemory, crash_dump::dump_data_addr + crash_dump::dump_size, elementCount) != HAL_OK) {
        	crash_dump::dump_failed();
        }
        crash_dump::dump_size += elementCount;
    } else if (element_size == CRASH_CATCHER_WORD) {
        if (crash_dump::dump_size + elementCount * sizeof(uint32_t) > crash_dump::dump_max_data_size) {
            crash_dump::dump_failed();
        }

        const uint32_t *ptr = reinterpret_cast<const uint32_t *>(pvMemory);
        while (elementCount) {
            uint32_t word = *ptr++;
            if (CSP_QSPI_WriteMemory((uint8_t *)ptr, crash_dump::dump_data_addr + crash_dump::dump_size, sizeof(word)) != HAL_OK) {
            	crash_dump::dump_failed();
            }
            crash_dump::dump_size += sizeof(word);
            elementCount--;
        }
    } else {
        crash_dump::dump_failed();
    }
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void) {
	return CRASH_CATCHER_TRY_AGAIN;
    // if we got up to here with success, program dump header
    crash_dump::info_t dump_info = {
        .crash_dump_magic_nr = crash_dump::CRASH_DUMP_MAGIC_NR,
        .dump_flags = crash_dump::DumpFlags::DEFAULT,
        .dump_size = crash_dump::dump_size,
    };

    hw::eeprom::data.dump_header = dump_info;

    // All done, now restart and display BSOD
    NVIC_SystemReset();

    // need to return something, but it should never get here.
    return CRASH_CATCHER_TRY_AGAIN;
}
