// Copyright Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <vector>
#include <cryptopp/sha.h>
#include <dlfcn.h>
#include <link.h>
#include <sys/mman.h>
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
u32 TakeFpsr() {
    u64 v;
    asm volatile("mrs %0, fpsr\n\tmsr fpsr, xzr" : "=r"(v));
    return static_cast<u32>(v);
}
#else
u64 ReadFpcr() {
    return 0;
}
void WriteFpcr(u64) {}
u32 TakeFpsr() {
    return 0;
}
#endif

// FPSCR AHP/DN/FZ/RMode (bits 26..22) sit where AArch64 FPCR keeps them.
constexpr u64 FPCR_MODE_MASK = 0x07C00000;
// FPSCR cumulative exception bits IDC/IXC/UFC/OFC/DZC/IOC, at the same positions in FPSR.
constexpr u32 FPSR_CUMULATIVE_MASK = 0x9F;

// Diagnostic: slow-path memory accesses per guest page, enabled by AZAHAR_NATIVE_SLOWSTATS.
std::vector<u32> slow_pages;
inline void CountSlow(u32 a) {
    if (!slow_pages.empty()) {
        slow_pages[a >> 12]++;
    }
}

} // namespace

NativeRunner::NativeRunner(ARM_Dynarmic& parent_) : parent(parent_) {
    if (std::getenv("AZAHAR_NATIVE_SLOWSTATS")) {
        slow_pages.assign(1u << 20, 0);
    }
    state.user = &parent;
    state.cb = &callbacks;

    callbacks.read8 = [](CtrState* s, u32 a) { CountSlow(a); return Parent(s).memory.Read8(a); };
    callbacks.read16 = [](CtrState* s, u32 a) { CountSlow(a); return Parent(s).memory.Read16(a); };
    callbacks.read32 = [](CtrState* s, u32 a) { CountSlow(a); return Parent(s).memory.Read32(a); };
    callbacks.read64 = [](CtrState* s, u32 a) { CountSlow(a); return Parent(s).memory.Read64(a); };
    callbacks.write8 = [](CtrState* s, u32 a, u8 v) { CountSlow(a); Parent(s).memory.Write8(a, v); };
    callbacks.write16 = [](CtrState* s, u32 a, u16 v) { CountSlow(a); Parent(s).memory.Write16(a, v); };
    callbacks.write32 = [](CtrState* s, u32 a, u32 v) { CountSlow(a); Parent(s).memory.Write32(a, v); };
    callbacks.write64 = [](CtrState* s, u32 a, u64 v) { CountSlow(a); Parent(s).memory.Write64(a, v); };

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
        using Dynarmic::A32::Exception;
        switch (static_cast<Exception>(kind)) {
        case Exception::SendEvent:
        case Exception::SendEventLocal:
        case Exception::WaitForInterrupt:
        case Exception::WaitForEvent:
        case Exception::Yield:
        case Exception::PreloadData:
        case Exception::PreloadDataWithIntentToWrite:
        case Exception::PreloadInstruction:
            return;
        default:
            break;
        }
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
    callbacks.lookup = [](CtrState* s, u32 addr_thumb) {
        return Parent(s).native->Lookup(addr_thumb);
    };
}

NativeRunner::~NativeRunner() {
    if (module) {
        LOG_INFO(Core_ARM11, "native: {} calls, {} fallback slices, {} svcs, {} stalls",
                 stats.native_calls, stats.fallback_slices, stats.svcs, stats.stalls);
    }
    if (handle) {
        dlclose(handle);
    }
}

namespace {

struct TextRange {
    uintptr_t probe;
    uintptr_t start = 0;
    uintptr_t end = 0;
};

// The generated module is ~75-100 MB of code with a flat profile. The kernel has no read-only THP
// for files, so the executable segment is copied into anonymous memory at the same address and
// marked for 2 MiB pages (about 1% faster in an MK7 race). This runs before any module code
// executes.
void MoveTextToHugePages(const void* in_module) {
    if (const char* e = std::getenv("AZAHAR_NATIVE_HUGE"); e && *e == '0') {
        return;
    }
    TextRange r{reinterpret_cast<uintptr_t>(in_module)};
    dl_iterate_phdr(
        [](dl_phdr_info* info, size_t, void* data) {
            auto* r = static_cast<TextRange*>(data);
            for (int i = 0; i < info->dlpi_phnum; i++) {
                const auto& ph = info->dlpi_phdr[i];
                if (ph.p_type != PT_LOAD || !(ph.p_flags & PF_X)) {
                    continue;
                }
                const uintptr_t start = info->dlpi_addr + ph.p_vaddr;
                const uintptr_t end = start + ph.p_memsz;
                if (r->probe >= start && r->probe < end) {
                    r->start = start;
                    r->end = end;
                    return 1;
                }
            }
            return 0;
        },
        &r);
    constexpr uintptr_t kHuge = 2u << 20;
    const uintptr_t lo = (r.start + kHuge - 1) & ~(kHuge - 1);
    const uintptr_t hi = r.end & ~(kHuge - 1);
    if (r.start == 0 || hi <= lo) {
        return;
    }
    const size_t size = hi - lo;
    void* copy = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (copy == MAP_FAILED) {
        return;
    }
    std::memcpy(copy, reinterpret_cast<void*>(lo), size);
    void* dst = mmap(reinterpret_cast<void*>(lo), size, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (dst == MAP_FAILED) {
        LOG_CRITICAL(Core_ARM11, "native: remapping module text failed");
        std::abort();
    }
    madvise(dst, size, MADV_HUGEPAGE);
    std::memcpy(dst, copy, size);
    munmap(copy, size);
#ifdef MADV_COLLAPSE
    madvise(dst, size, MADV_COLLAPSE);
#else
    madvise(dst, size, 25);
#endif
    if (mprotect(dst, size, PROT_READ | PROT_EXEC) != 0) {
        LOG_CRITICAL(Core_ARM11, "native: protecting module text failed");
        std::abort();
    }
    __builtin___clear_cache(static_cast<char*>(dst), static_cast<char*>(dst) + size);
    LOG_INFO(Core_ARM11, "native: {} MiB of module text on huge pages", size >> 20);
}

} // namespace

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
        if (void* sym = dlsym(handle, "ctr_module")) {
            MoveTextToHugePages(sym);
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
    entry_slots.assign(m->text_size / 2 + 1, 0);
    lookup_cache.fill({~0u, nullptr});
    for (u32 i = 0; i < m->entry_count; i++) {
        const u32 off = (m->entries[i].addr & ~1u) - m->text_base;
        if (off / 2 >= entry_slots.size()) {
            continue;
        }
        u32& slot = entry_slots[off / 2];
        slot = slot == 0 ? i + 1 : SLOT_SHARED;
    }
    LOG_INFO(Core_ARM11, "native: {} functions, {} entries for {:016x}", m->function_count,
             m->entry_count, program_id);
    return true;
}

CtrFunction NativeRunner::Lookup(u32 addr_thumb) const {
    LookupCacheEntry& c = lookup_cache[(addr_thumb >> 1) & (LOOKUP_CACHE_SIZE - 1)];
    if (c.addr == addr_thumb) {
        return c.fn;
    }
    CtrFunction fn = LookupSlow(addr_thumb);
    c = {addr_thumb, fn};
    return fn;
}

CtrFunction NativeRunner::LookupSlow(u32 addr_thumb) const {
    const u32 off = (addr_thumb & ~1u) - module->text_base;
    if (off / 2 >= entry_slots.size()) {
        return LookupSorted(addr_thumb);
    }
    const u32 slot = entry_slots[off / 2];
    if (slot == 0) {
        return nullptr;
    }
    if (slot == SLOT_SHARED) {
        return LookupSorted(addr_thumb);
    }
    const CtrEntry& e = module->entries[slot - 1];
    return e.addr == addr_thumb ? module->functions[e.func] : nullptr;
}

CtrFunction NativeRunner::LookupSorted(u32 addr_thumb) const {
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
    SetHostFpcr();
    TakeFpsr();
    in_native = true;
}

void NativeRunner::SetHostFpcr() {
    fpcr_mode = state.fpscr & FPCR_MODE_MASK;
    WriteFpcr((host_fpcr & ~FPCR_MODE_MASK) | fpcr_mode);
}

void NativeRunner::LeaveState() {
    in_native = false;
    state.fpscr |= TakeFpsr() & FPSR_CUMULATIVE_MASK;
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

void NativeRunner::LogStats() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_stats_log) {
        return;
    }
    next_stats_log = now + std::chrono::seconds(5);
    const s64 total = stats.native_ticks + stats.fallback_ticks;
    LOG_INFO(Core_ARM11,
             "native: {} calls, {} fallback slices, {} svcs, {} stalls, {:.1f}% of ticks native",
             stats.native_calls, stats.fallback_slices, stats.svcs, stats.stalls,
             total > 0 ? 100.0 * static_cast<double>(stats.native_ticks) / total : 0.0);
    if (!slow_pages.empty()) {
        std::vector<std::pair<u32, u32>> top;
        for (u32 i = 0; i < slow_pages.size(); ++i) {
            if (slow_pages[i]) {
                top.emplace_back(slow_pages[i], i);
            }
        }
        std::sort(top.begin(), top.end(), std::greater<>());
        auto& pt = *parent.current_page_table;
        for (std::size_t i = 0; i < top.size() && i < 10; ++i) {
            const u32 page = top[i].second;
            LOG_INFO(Core_ARM11, "native slow page {:05x}000: {} accesses, type {}, ptr {}", page,
                     top[i].first, static_cast<int>(pt.attributes[page]),
                     fmt::ptr(pt.GetPointerArray()[page]));
        }
        std::fill(slow_pages.begin(), slow_pages.end(), 0);
    }
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
        CtrFunction fn = stalled_pc == pc ? nullptr : Lookup(pc | (thumb ? 1u : 0u));
        stalled_pc = NO_STALL;
        if (fn) {
            const s64 downcount = parent.GetTimer().GetDowncount();
            EnterState();
            // Chain native entries in CtrState; dynarmic sees the state only when leaving.
            do {
                stats.native_calls++;
                state.exit = 0;
                const u32 entry_pc = state.r[15];
                const u32 entry_cpsr = state.cpsr;
                const s64 entry_ticks = state.ticks;
                fn(&state);
                if (state.exit || parent.halt_requested || state.ticks <= 0) {
                    break;
                }
                if (state.r[15] == entry_pc && state.cpsr == entry_cpsr &&
                    state.ticks == entry_ticks) {
                    if (stats.stalls++ < 8) {
                        LOG_WARNING(Core_ARM11,
                                    "native: entry {:08x} made no progress, cpsr {:08x} fpscr {:08x}",
                                    entry_pc, entry_cpsr, state.fpscr);
                    }
                    stalled_pc = entry_pc;
                    break;
                }
                if ((state.fpscr ^ fpcr_mode) & FPCR_MODE_MASK) {
                    SetHostFpcr();
                }
                fn = Lookup(state.r[15] | ((state.cpsr >> 5) & 1u));
            } while (fn);
            LeaveState();
            stats.native_ticks += downcount - parent.GetTimer().GetDowncount();
            if (state.exit || parent.halt_requested) {
                break;
            }
            continue;
        }
        stats.fallback_slices++;
        const s64 before = parent.GetTimer().GetDowncount();
        parent.native_slice = true;
        jit->Run();
        parent.native_slice = false;
        stats.fallback_ticks += before - parent.GetTimer().GetDowncount();
        LogStats();
        if (parent.halt_requested) {
            break;
        }
    }
    return true;
}

} // namespace Core
