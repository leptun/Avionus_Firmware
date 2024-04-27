#pragma once

extern "C" {
#include "CrashCatcher.h"
}
#include <utility_extensions.hpp>
#include <fatfs.h>

namespace crash_dump {

/// Dump types and flags (when set to zero, flag is active)
enum class DumpFlags : uint8_t {
    EXPORTED = 0x01, ///< dump not exported to usb flash or send via internet
    DEFAULT = 0xFF, ///< Initial value when crash dump is created
};

typedef struct {
    /// Magic number, that indicates that crash dump is valid
    uint32_t crash_dump_magic_nr;
    DumpFlags dump_flags;
    uint32_t dump_size;
} info_t;

inline DumpFlags operator|(const DumpFlags a, const DumpFlags b) {
    return DumpFlags(ftrstd::to_underlying(a) | ftrstd::to_underlying(b));
}

inline DumpFlags operator&(const DumpFlags a, const DumpFlags b) {
    return DumpFlags(ftrstd::to_underlying(a) & ftrstd::to_underlying(b));
}

inline DumpFlags operator~(const DumpFlags a) {
    return DumpFlags(~ftrstd::to_underlying(a));
}

inline bool any(const DumpFlags a) {
    return ftrstd::to_underlying(a);
}

/**
 * @brief Check if dump is valid
 */
bool dump_is_valid();

/**
 * @brief Check if dump was saved to
 * @return true for saved
 */
bool dump_is_exported();

/**
 * @brief Return size of dump data
 */
size_t dump_get_size();

/**
 * @brief Erase dump.
 */
void dump_reset();

/**
 * @brief Set dump as displayed.
 */
void dump_set_displayed();

/**
 * @brief Set dump as exported
 */
void dump_set_exported();

/**
 * @brief Store dump to SD.
 * @param fn Filename to store dump to
 * @return true on success
 */
bool save_dump_to_sd(const char *fn, FIL *fil);

/**
 * Function that should be called before error messages & dumps are issued.
 * It will trigger breakpoint if necessary and put printer to safe state
 */
void before_dump();

[[noreturn]] void trigger_crash_dump();
} // namespace crash_dump
