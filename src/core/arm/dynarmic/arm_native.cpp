// Copyright Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cryptopp/sha.h>
#include <dlfcn.h>
#include <dynarmic/interface/A32/a32.h>
#include "common/logging/log.h"
#include "core/arm/dynarmic/arm_dynarmic.h"
#include "core/arm/dynarmic/arm_exclusive_monitor.h"
#include "core/arm/dynarmic/arm_native.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/hle/kernel/process.h"
#include "core/memory.h"

namespace Core {

namespace {

ARM_Dynarmic& Parent(CtrState* s) {
    return *static_cast<ARM_Dynarmic*>(s->user);
}

#if defined(__aarch64__)
u64 ReadFpcr() {
    u64 v;
    asm volatile("mrs %0, fpcr" : "=r"(v));
    return v;
}
void WriteFpcr(u64 v) {
    asm volatile("msr fpcr, %0" : : "r"(v));
}
#else
u64 ReadFpcr() {
    return 0;
}
void WriteFpcr(u64) {}
#endif

// FPSCR AHP/DN/FZ/RMode (bits 26..22) sit where AArch64 FPCR keeps them.
constexpr u64 FPCR_MODE_MASK = 0x07C00000;

} // namespace

NativeRunner::NativeRunner(ARM_Dynarmic& parent_) : parent(parent_) {
    state.user = &parent;
    state.cb = &callbacks;

    callbacks.read8 = [](CtrState* s, u32 a) { return Parent(s).memory.Read8(a); };
    callbacks.read16 = [](CtrState* s, u32 a) { return Parent(s).memory.Read16(a); };
    callbacks.read32 = [](CtrState* s, u32 a) { return Parent(s).memory.Read32(a); };
    callbacks.read64 = [](CtrState* s, u32 a) { return Parent(s).memory.Read64(a); };
    callbacks.write8 = [](CtrState* s, u32 a, u8 v) { Parent(s).memory.Write8(a, v); };
    callbacks.write16 = [](CtrState* s, u32 a, u16 v) { Parent(s).memory.Write16(a, v); };
    callbacks.write32 = [](CtrState* s, u32 a, u32 v) { Parent(s).memory.Write32(a, v); };
    callbacks.write64 = [](CtrState* s, u32 a, u64 v) { Parent(s).memory.Write64(a, v); };

#define CTR_EXCLUSIVE(BITS, T)                                                                     \
    callbacks.exclusive_read##BITS = [](CtrState* s, u32 a) -> T {                                 \
        auto& p = Parent(s);                                                                       \
        return p.exclusive_monitor.monitor.ReadAndMark<T>(p.GetID(), a, [&] {                      \
            return p.memory.Read##BITS(a);                                                         \
        });                                                                                        \
    };                                                                                             \
    callbacks.exclusive_write##BITS = [](CtrState* s, u32 a, T v) -> u32 {                         \
        auto& p = Parent(s);                                                                       \
        const bool ok = p.exclusive_monitor.monitor.DoExclusiveOperation<T>(                       \
            p.GetID(), a, [&](T expected) { return p.memory.WriteExclusive##BITS(a, v, expected); }); \
        return ok ? 0 : 1;                                                                         \
    };
    CTR_EXCLUSIVE(8, u8)
    CTR_EXCLUSIVE(16, u16)
    CTR_EXCLUSIVE(32, u32)
    CTR_EXCLUSIVE(64, u64)
#undef CTR_EXCLUSIVE
    callbacks.clear_exclusive = [](CtrState* s) {
        auto& p = Parent(s);
        p.exclusive_monitor.monitor.ClearProcessor(p.GetID());
    };

    callbacks.svc = [](CtrState* s, u32 imm) {
        auto& p = Parent(s);
        auto& runner = *p.native;
        runner.stats.svcs++;
        runner.LeaveState();
        p.NativeSVC(imm);
        runner.EnterState();
    };
    callbacks.exception = [](CtrState* s, u32 pc, u32 kind) {
        auto& p = Parent(s);
        auto& runner = *p.native;
        runner.LeaveState();
        p.NativeException(pc, kind);
        runner.EnterState();
        s->exit = 1;
    };
    callbacks.cp15_read = [](CtrState* s, u32 opc1, u32 crn, u32 crm, u32 opc2) -> u32 {
        LOG_ERROR(Core_ARM11, "native: unhandled cp15 read {} c{} c{} {}", opc1, crn, crm, opc2);
        return 0;
    };
    callbacks.cp15_write = [](CtrState*, u32, u32, u32, u32, u32) {};
}

NativeRunner::~NativeRunner() {
    if (handle) {
        dlclose(handle);
    }
}

bool NativeRunner::Attach() {
    auto process = parent.system.Kernel().GetCurrentProcess();
    if (!process || !process->codeset) {
        return false;
    }
    const u64 program_id = process->codeset->program_id;
    if (program_id == attached_program_id) {
        return module != nullptr;
    }
    attached_program_id = program_id;
    module = nullptr;

    const char* path = std::getenv("AZAHAR_NATIVE_MODULE");
    if (!path || !*path) {
        return false;
    }
    if (!handle) {
        handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            LOG_ERROR(Core_ARM11, "native: {}", dlerror());
            return false;
        }
    }
    auto get = reinterpret_cast<const CtrModule* (*)()>(dlsym(handle, "ctr_module"));
    const CtrModule* m = get ? get() : nullptr;
    if (!m || m->abi_version != CTR_ABI_VERSION || m->title_id != program_id) {
        LOG_WARNING(Core_ARM11, "native: module does not match program {:016x}", program_id);
        return false;
    }
    std::vector<u8> text(m->text_size);
    parent.memory.ReadBlock(*process, m->text_base, text.data(), text.size());
    u8 digest[CryptoPP::SHA256::DIGESTSIZE];
    CryptoPP::SHA256().CalculateDigest(digest, text.data(), text.size());
    if (std::memcmp(digest, m->text_sha256, sizeof(digest)) != 0) {
        LOG_WARNING(Core_ARM11, "native: .text of {:016x} differs from the module's", program_id);
        return false;
    }
    module = m;
    LOG_INFO(Core_ARM11, "native: {} functions, {} entries for {:016x}", m->function_count,
             m->entry_count, program_id);
    return true;
}

CtrFunction NativeRunner::Lookup(u32 addr_thumb) const {
    const CtrEntry* begin = module->entries;
    const CtrEntry* end = begin + module->entry_count;
    const CtrEntry* it = std::lower_bound(
        begin, end, addr_thumb, [](const CtrEntry& e, u32 a) { return e.addr < a; });
    if (it == end || it->addr != addr_thumb) {
        return nullptr;
    }
    return module->functions[it->func];
}

void NativeRunner::EnterState() {
    auto* jit = parent.jit;
    std::memcpy(state.r, jit->Regs().data(), sizeof(state.r));
    std::memcpy(state.ext, jit->ExtRegs().data(), sizeof(state.ext));
    state.cpsr = jit->Cpsr();
    state.fpscr = jit->Fpscr();
    state.tpidrurw = parent.cp15_state.cp15_thread_uprw;
    state.tpidruro = parent.cp15_state.cp15_thread_uro;
    state.page_table = parent.current_page_table->GetPointerArray().data();
    const s64 downcount = parent.GetTimer().GetDowncount();
    state.ticks = downcount;
    ticks_at_entry = downcount;
    WriteFpcr((host_fpcr & ~FPCR_MODE_MASK) | (state.fpscr & FPCR_MODE_MASK));
    in_native = true;
}

void NativeRunner::LeaveState() {
    in_native = false;
    WriteFpcr(host_fpcr);
    auto* jit = parent.jit;
    std::memcpy(jit->Regs().data(), state.r, sizeof(state.r));
    std::memcpy(jit->ExtRegs().data(), state.ext, sizeof(state.ext));
    jit->SetCpsr(state.cpsr);
    jit->SetFpscr(state.fpscr);
    parent.cp15_state.cp15_thread_uprw = state.tpidrurw;
    if (ticks_at_entry > state.ticks) {
        parent.GetTimer().AddTicks(static_cast<u64>(ticks_at_entry - state.ticks));
    }
    ticks_at_entry = state.ticks;
}

bool NativeRunner::Run() {
    if (!Attach()) {
        return false;
    }
    host_fpcr = ReadFpcr();
    while (parent.GetTimer().GetDowncount() > 0) {
        auto* jit = parent.jit;
        const u32 pc = jit->Regs()[15];
        const bool thumb = (jit->Cpsr() & (1u << 5)) != 0;
        if (CtrFunction fn = Lookup(pc | (thumb ? 1u : 0u))) {
            stats.native_calls++;
            state.exit = 0;
            EnterState();
            fn(&state);
            LeaveState();
            if (state.exit || parent.halt_requested) {
                break;
            }
            continue;
        }
        stats.fallback_slices++;
        parent.native_slice = true;
        jit->Run();
        parent.native_slice = false;
        if (parent.halt_requested) {
            break;
        }
    }
    return true;
}

} // namespace Core
