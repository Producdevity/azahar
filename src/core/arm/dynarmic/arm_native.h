// Copyright Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <chrono>

#include <memory>
#include <vector>
#include "common/common_types.h"
#include "core/arm/dynarmic/ctr_abi.h"

namespace Core {

class ARM_Dynarmic;

/// Runs ahead-of-time recompiled guest code (ctr-recomp modules) for the
/// current process, falling back to the JIT outside the module's coverage.
class NativeRunner {
public:
    explicit NativeRunner(ARM_Dynarmic& parent);
    ~NativeRunner();

    /// Returns false when no module covers the current process.
    bool Run();
    void RequestExit() {
        state.exit = 1;
    }
    bool InNative() const {
        return in_native;
    }

private:
    bool Attach();
    CtrFunction Lookup(u32 addr_thumb) const;
    CtrFunction LookupSlow(u32 addr_thumb) const;
    CtrFunction LookupSorted(u32 addr_thumb) const;
    void EnterState();
    void LeaveState();
    void SetHostFpcr();

    ARM_Dynarmic& parent;
    CtrState state{};
    CtrCallbacks callbacks{};
    const CtrModule* module = nullptr;
    static constexpr u32 SLOT_SHARED = ~0u;
    // Per halfword of .text: entry index + 1, 0 for none, SLOT_SHARED when ARM and Thumb entries share it.
    std::vector<u32> entry_slots;
    // Direct-mapped memo of Lookup; the slot tables above miss the data cache on every call.
    struct LookupCacheEntry {
        u32 addr;
        CtrFunction fn;
    };
    static constexpr std::size_t LOOKUP_CACHE_SIZE = 2048;
    mutable std::array<LookupCacheEntry, LOOKUP_CACHE_SIZE> lookup_cache{};
    void* handle = nullptr;
    u64 attached_program_id = ~0ULL;
    bool in_native = false;
    s64 ticks_at_entry = 0;
    u64 host_fpcr = 0;
    u32 fpcr_mode = 0;
    static constexpr u32 NO_STALL = ~0u;
    // A native entry that returned without moving pc, cpsr or ticks; dynarmic runs it next.
    u32 stalled_pc = NO_STALL;
    std::chrono::steady_clock::time_point next_stats_log{};
    void LogStats();

public:
    struct Stats {
        u64 native_calls = 0;
        u64 fallback_slices = 0;
        u64 svcs = 0;
        u64 stalls = 0;
        s64 native_ticks = 0;
        s64 fallback_ticks = 0;
    } stats;
};

} // namespace Core
