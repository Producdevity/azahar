// Copyright Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>
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
    void EnterState();
    void LeaveState();

    ARM_Dynarmic& parent;
    CtrState state{};
    CtrCallbacks callbacks{};
    const CtrModule* module = nullptr;
    void* handle = nullptr;
    u64 attached_program_id = ~0ULL;
    bool in_native = false;
    s64 ticks_at_entry = 0;
    u64 host_fpcr = 0;

public:
    struct Stats {
        u64 native_calls = 0;
        u64 fallback_slices = 0;
        u64 svcs = 0;
    } stats;
};

} // namespace Core
