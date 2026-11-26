// Copyright 2023-2026 Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string>
#include <fmt/format.h>
#include "common/archives.h"
#include "common/hacks/hack_manager.h"
#include "common/microprofile.h"
#include "common/settings.h"
#include "common/thread.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/hle/service/gsp/gsp_gpu.h"
#include "core/hle/service/plgldr/plgldr.h"
#include "core/loader/loader.h"
#include "video_core/debug_utils/debug_utils.h"
#include "video_core/gpu.h"
#include "video_core/gpu_debugger.h"
#include "video_core/gpu_impl.h"
#include "video_core/pica/pica_core.h"
#include "video_core/pica/regs_lcd.h"
#include "video_core/renderer_base.h"
#include "video_core/renderer_software/sw_blitter.h"
#include "video_core/right_eye_disabler.h"
#include "video_core/video_core.h"

namespace VideoCore {

constexpr VAddr VADDR_LCD = 0x1ED02000;
constexpr VAddr VADDR_GPU = 0x1EF00000;

class DelayGenerator {
private:
    DelayGenerator() = default;

    // Average transfer speed based on measurements taken from real
    // hardware. 4 different modes have been taken into consideration:
    // RAM -> RAM, RAM -> VRAM, VRAM -> RAM and VRAM -> VRAM.
    // Furthermore, measurements are split into DMA transfers and tex
    // copies. For simplicity, we will assume fills are as fast as
    // texture copies.

    static constexpr double mibps_to_ns_per_byte(double mib_per_sec) {
        return 1'000'000'000.0 / (mib_per_sec * 1024.0 * 1024.0);
    }

    static constexpr std::array<std::array<double, 4>, 2> speed_mibps = {
        {{
             190.0, // DMA RAMTORAM
             310.0, // DMA RAMTOVRAM
             380.0, // DMA VRAMTORAM
             380.0, // DMA VRAMTOVRAM
         },
         {
             450.0,  // TEX RAMTORAM
             3100.0, // TEX RAMTOVRAM
             5400.0, // TEX VRAMTORAM
             5400.0, // TEX VRAMTOVRAM
         }}};

public:
    enum class CopyMode {
        RAMTORAM,
        RAMTOVRAM,
        VRAMTORAM,
        VRAMTOVRAM,
    };

    static CopyMode GetCopyMode(bool input_vram, bool output_vram) {
        if (!input_vram && !output_vram) {
            return CopyMode::RAMTORAM;
        } else if (!input_vram && output_vram) {
            return CopyMode::RAMTOVRAM;
        } else if (input_vram && !output_vram) {
            return CopyMode::VRAMTORAM;
        } else {
            return CopyMode::VRAMTOVRAM;
        }
    }

    static u64 CalculateDelayNanoseconds(CopyMode mode, bool is_textre, size_t size) {
        double base_ns_per_byte =
            mibps_to_ns_per_byte(speed_mibps[is_textre][static_cast<u32>(mode)]);

        return static_cast<u64>(size * base_ns_per_byte);
    }
};

namespace {
constexpr std::size_t GPU_QUEUE_MAX_ITEMS = 1024;
constexpr u64 GPU_MAX_SWAPS_IN_FLIGHT = 1;
constexpr std::size_t GPU_MAX_DEFERRED_INVALIDATES = 256;
constexpr u32 SKIP_STALE_PAGES =
    (Memory::FCRAM_N3DS_PADDR_END - Memory::VRAM_PADDR) >> Memory::CITRA_PAGE_BITS;
constexpr auto GPU_WAIT_LOG_FIRST = std::chrono::milliseconds(100);
constexpr auto GPU_WAIT_LOG_REPEAT = std::chrono::milliseconds(1000);
constexpr u64 GPU_STATS_EVERY_FRAMES = 600;

constexpr std::array<const char*, GpuThreadState::NumReasons> WAIT_REASON_NAMES = {
    "memory", "readreg", "idle", "dma", "other", "queue", "frame"};

int GpuThreadMode() {
    const char* value = std::getenv("AZAHAR_GPU_THREAD");
    if (value == nullptr || value[0] < '1' || value[0] > '3') {
        return 0;
    }
    return value[0] - '0';
}

u64 ThreadCpuNs() {
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<u64>(ts.tv_sec) * 1'000'000'000ull + static_cast<u64>(ts.tv_nsec);
}

template <typename Pred>
void WaitLogged(std::unique_lock<std::mutex>& lock, GpuThreadState& state, u32 reason,
                Pred pred) {
    state.waits[reason]++;
    if (pred()) {
        return;
    }
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    Clock::duration next_log = GPU_WAIT_LOG_FIRST;
    while (!state.done_cv.wait_for(lock, std::chrono::milliseconds(20), pred)) {
        const auto waited = Clock::now() - start;
        if (waited >= next_log) {
            LOG_WARNING(HW_GPU, "GPU thread: emu thread waiting {} ms for {} (queue={} running={})",
                        std::chrono::duration_cast<std::chrono::milliseconds>(waited).count(),
                        WAIT_REASON_NAMES[reason], state.queue.size(), state.current);
            next_log = waited + GPU_WAIT_LOG_REPEAT;
        }
    }
    state.blocked[reason]++;
    state.blocked_ms[reason] +=
        std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
} // Anonymous namespace

MICROPROFILE_DEFINE(GPU_DisplayTransfer, "GPU", "DisplayTransfer", MP_RGB(100, 100, 255));
MICROPROFILE_DEFINE(GPU_CmdlistProcessing, "GPU", "Cmdlist Processing", MP_RGB(100, 255, 100));

GPU::GPU(Core::System& system, Frontend::EmuWindow& emu_window,
         Frontend::EmuWindow* secondary_window)
    : right_eye_disabler{std::make_unique<RightEyeDisabler>(*this)},
      impl{std::make_unique<Impl>(system, emu_window, secondary_window)} {
    impl->vblank_event = impl->timing.RegisterEvent(
        "GPU::VBlankCallback",
        [this](uintptr_t user_data, s64 cycles_late) { VBlankCallback(user_data, cycles_late); });
    impl->timing.ScheduleEvent(FRAME_TICKS, impl->vblank_event);

    // Bind the rasterizer to the PICA GPU
    impl->pica.BindRasterizer(impl->rasterizer);

    if (GpuThreadMode() != 0) {
        if (Settings::GetWorkingGraphicsAPI() == Settings::GraphicsAPI::Vulkan) {
            StartGpuThread();
        } else {
            LOG_WARNING(HW_GPU, "AZAHAR_GPU_THREAD ignored: only the Vulkan renderer is supported");
        }
    }
}

GPU::~GPU() {
    StopGpuThread();
}

bool GPU::IsThreaded() const {
    return impl->gpu_thread != nullptr;
}

void GPU::StartGpuThread() {
    impl->gpu_thread = std::make_unique<GpuThreadState>();
    auto& state = *impl->gpu_thread;
    state.sync_each_slice = GpuThreadMode() == 2;
    state.sync_on_idle = GpuThreadMode() != 3;
    const char* skip_env = std::getenv("AZAHAR_SKIP_UNSHOWN");
    state.skip_enabled = skip_env != nullptr && skip_env[0] == '1';
    if (const char* stress_env = std::getenv("AZAHAR_GPU_STRESS_US")) {
        state.stress_us = static_cast<u32>(std::strtoul(stress_env, nullptr, 10));
    }
    if (state.skip_enabled) {
        state.skip_stale = std::make_unique<std::atomic<u8>[]>(SKIP_STALE_PAGES);
        for (u32 i = 0; i < SKIP_STALE_PAGES; ++i) {
            state.skip_stale[i].store(0, std::memory_order_relaxed);
        }
    }
    impl->renderer->SetDeferredFrameEnd(true);
    impl->memory.SetGpuDirtyTracking(true);
    impl->memory.SetGpuThreadHooks(
        [this](PAddr start, u32 size, Memory::FlushMode mode) {
            return OnCpuAccess(start, size, mode);
        },
        [this](PAddr start, u32 size, bool cached) {
            return DeferRasterizerMark(start, size, cached);
        });
    state.thread = std::thread([this] { GpuThreadLoop(); });
    state.id = state.thread.get_id();
    LOG_WARNING(HW_GPU,
                "GPU thread started (mode {}, queue limit {}, swaps in flight {}, skip unshown {})",
                GpuThreadMode(), GPU_QUEUE_MAX_ITEMS, GPU_MAX_SWAPS_IN_FLIGHT, state.skip_enabled);
}

void GPU::StopGpuThread() {
    if (!impl->gpu_thread) {
        return;
    }
    auto& state = *impl->gpu_thread;
    FlushCpuInvalidates();
    {
        std::scoped_lock lock{state.mutex};
        state.stop = true;
    }
    state.work_cv.notify_one();
    state.thread.join();

    // Every mark must reach the page tables exactly once, or a later unmark hits UNREACHABLE.
    ApplyPendingMarks();
    impl->memory.SetGpuThreadHooks({}, {});
    impl->memory.SetGpuDirtyTracking(false);
    if (impl->renderer) {
        impl->renderer->SetDeferredFrameEnd(false);
    }
    const std::size_t dropped_irqs = state.irqs.size();
    impl->gpu_thread.reset();
    LOG_WARNING(HW_GPU, "GPU thread stopped (undelivered interrupts dropped: {})", dropped_irqs);
}

void GPU::GpuThreadLoop() {
    Common::SetCurrentThreadName("GPU");
    MicroProfileOnThreadCreate("GPU");
    auto& state = *impl->gpu_thread;
    std::unique_lock lock{state.mutex};
    while (true) {
        state.work_cv.wait(lock, [&] { return state.stop || !state.queue.empty(); });
        if (state.queue.empty()) {
            break;
        }
        GpuThreadState::Item item = std::move(state.queue.front());
        state.queue.pop_front();
        state.busy = true;
        state.current = item.name;
        lock.unlock();
        state.done_cv.notify_all();

        state.job_skipping = item.skip_candidate &&
                             item.frame > state.skip_cancel_upto.load(std::memory_order_acquire) &&
                             !state.skip_disabled.load(std::memory_order_relaxed);
        state.job_skipped_any = false;
        impl->pica.skip_draws = state.job_skipping;

        if (state.stress_us != 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(std::rand() % state.stress_us));
        }
        const auto start = std::chrono::steady_clock::now();
        item.fn();
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= GPU_WAIT_LOG_FIRST) {
            LOG_WARNING(HW_GPU, "GPU thread: {} took {} ms", item.name,
                        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
        }
        item.fn = nullptr;

        if (state.job_skipping) {
            impl->pica.skip_draws = false;
            for (const auto& [addr, size] : impl->pica.skipped_targets) {
                MarkSkipStale(addr, size);
            }
            if (!impl->pica.skipped_targets.empty()) {
                state.job_skipped_any = true;
                impl->pica.skipped_targets.clear();
            }
            if (state.job_skipped_any) {
                state.skipped_jobs.fetch_add(1, std::memory_order_relaxed);
            }
            state.job_skipping = false;
        }

        // Interrupts and cache marks become visible only once the whole job is done.
        if (!state.job_irqs.empty() || !state.job_marks.empty()) {
            std::scoped_lock completion_lock{state.completion_mutex};
            state.irqs.insert(state.irqs.end(), state.job_irqs.begin(), state.job_irqs.end());
            state.marks.insert(state.marks.end(), state.job_marks.begin(),
                               state.job_marks.end());
            state.has_completions.store(true, std::memory_order_release);
        }
        state.job_irqs.clear();
        state.job_marks.clear();
        state.completed_seq.store(item.seq, std::memory_order_release);

        lock.lock();
        state.busy = false;
        state.current = "idle";
        lock.unlock();
        state.done_cv.notify_all();
        lock.lock();
    }
#if MICROPROFILE_ENABLED
    MicroProfileOnThreadExit();
#endif
}

u64 GPU::PushGpuWork(const char* name, std::function<void()> fn) {
    auto& state = *impl->gpu_thread;
    if (std::this_thread::get_id() == state.id) {
        fn();
        return 0;
    }
    FlushCpuInvalidates();
    return Enqueue(name, std::move(fn));
}

u64 GPU::Enqueue(const char* name, std::function<void()> fn) {
    auto& state = *impl->gpu_thread;
    const u64 seq = ++state.pushed_seq;
    {
        std::unique_lock lock{state.mutex};
        WaitLogged(lock, state, GpuThreadState::QueueFull,
                   [&] { return state.queue.size() < GPU_QUEUE_MAX_ITEMS; });
        state.queue.push_back({name, seq, state.frame_id, state.frame_skip, std::move(fn)});
    }
    state.work_cv.notify_one();
    return seq;
}

void GPU::FlushCpuInvalidates() {
    auto& state = *impl->gpu_thread;
    if (state.cpu_invalidates.empty()) {
        return;
    }
    std::vector<std::pair<PAddr, u32>> ranges;
    ranges.swap(state.cpu_invalidates);
    Enqueue("CPU write invalidate", [this, ranges = std::move(ranges)] {
        for (const auto& [addr, size] : ranges) {
            impl->rasterizer->InvalidateRegion(addr, size);
        }
    });
}

bool GPU::MayHaveGpuWrites(PAddr start, u32 size) {
    auto& state = *impl->gpu_thread;
    const u64 completed = state.completed_seq.load(std::memory_order_acquire);
    while (!state.pending_writes.empty() && state.pending_writes.front().seq <= completed) {
        state.pending_writes.pop_front();
    }
    const PAddr end = start + size;
    for (const auto& write : state.pending_writes) {
        if (start < write.end && write.start < end) {
            return true;
        }
    }
    return impl->memory.AnyGpuDirtyPage(start, size);
}

bool GPU::OnCpuAccess(PAddr start, u32 size, Memory::FlushMode mode) {
    auto& state = *impl->gpu_thread;
    if (std::this_thread::get_id() == state.id) {
        return false;
    }
    const bool reads = mode != Memory::FlushMode::Invalidate;
    if ((state.skip_enabled && reads && AnySkipStale(start, size)) ||
        mode == Memory::FlushMode::FlushAndInvalidate || MayHaveGpuWrites(start, size)) {
        WaitIdle(GpuThreadState::Memory);
        if (state.skip_enabled && reads && AnySkipStale(start, size)) {
            // The CPU reads output of work that was already skipped; it cannot be recovered.
            state.skip_violations.fetch_add(1, std::memory_order_relaxed);
            if (!state.skip_disabled.exchange(true)) {
                LOG_WARNING(HW_GPU,
                            "Skip unshown frames: CPU read of skipped output at {:#010X}+{:#X}, "
                            "skipping disabled",
                            start, size);
            }
        }
        return false;
    }
    // No completed or queued GPU write covers these pages, so memory is current: a read needs
    // nothing, and a write only has to drop cached copies before later GPU work uses them.
    if (mode == Memory::FlushMode::Flush) {
        state.cpu_reads_skipped++;
        return true;
    }
    state.cpu_writes_deferred++;
    if (state.cpu_invalidates.empty() || state.cpu_invalidates.back() != std::pair{start, size}) {
        state.cpu_invalidates.emplace_back(start, size);
    }
    if (state.cpu_invalidates.size() >= GPU_MAX_DEFERRED_INVALIDATES) {
        FlushCpuInvalidates();
    }
    return true;
}

void GPU::NotePendingWrites(const Service::GSP::Command& command, u64 seq) {
    using Service::GSP::CommandId;
    auto& state = *impl->gpu_thread;
    const auto add = [&](PAddr start, u64 size) {
        if (size != 0) {
            const u64 end = std::min<u64>(u64{start} + size, 0xFFFFFFFF);
            state.pending_writes.push_back({start, static_cast<PAddr>(end), seq});
        }
    };
    switch (command.id) {
    case CommandId::MemoryFill: {
        const auto& params = command.memory_fill;
        const std::array<std::pair<u32, u32>, 2> fills{{{params.start1, params.end1},
                                                        {params.start2, params.end2}}};
        for (const auto& [start, end] : fills) {
            if (start == 0) {
                continue;
            }
            const PAddr phys_start = VirtualToPhysicalAddress(start);
            const PAddr phys_end = VirtualToPhysicalAddress(end);
            add(phys_start, phys_end > phys_start ? phys_end - phys_start + 16 : 16);
        }
        break;
    }
    case CommandId::DisplayTransfer: {
        const auto& params = command.display_transfer;
        const u64 width = params.out_buffer_size & 0xFFFF;
        const u64 height = params.out_buffer_size >> 16;
        // 4 bytes per pixel covers every output format.
        add(VirtualToPhysicalAddress(params.out_buffer_address), width * height * 4);
        break;
    }
    case CommandId::TextureCopy: {
        const auto& params = command.texture_copy;
        const u64 width = u64{params.out_width_gap & 0xFFFF} * 16;
        const u64 gap = u64{params.out_width_gap >> 16} * 16;
        const u64 span =
            width == 0 ? params.size : (params.size + width - 1) / width * (width + gap);
        add(VirtualToPhysicalAddress(params.out_buffer_address), span);
        break;
    }
    default:
        break;
    }
}

void GPU::WaitIdle() {
    WaitIdle(GpuThreadState::Other);
}

void GPU::WaitIdle(u32 reason) {
    if (!impl->gpu_thread) {
        return;
    }
    auto& state = *impl->gpu_thread;
    if (std::this_thread::get_id() == state.id) {
        return;
    }
    if (reason != GpuThreadState::Idle) {
        // The emulation thread needs GPU results now, so no queued work may be skipped.
        CancelSkip();
    }
    FlushCpuInvalidates();
    {
        std::unique_lock lock{state.mutex};
        WaitLogged(lock, state, reason, [&] { return state.queue.empty() && !state.busy; });
    }
    ApplyPendingMarks();
}

void GPU::CancelSkip() {
    auto& state = *impl->gpu_thread;
    if (!state.skip_enabled) {
        return;
    }
    if (state.frame_skip) {
        state.frame_skip = false;
        state.skip_cancels++;
    }
    state.skip_cancel_upto.store(state.frame_id, std::memory_order_release);
}

bool GPU::SkipGpuOutput(PAddr in_start, u64 in_size, PAddr out_start, u64 out_size) {
    if (!impl->gpu_thread || !impl->gpu_thread->job_skipping ||
        std::this_thread::get_id() != impl->gpu_thread->id) {
        return false;
    }
    // Only work that consumes rendered output is skipped; uploads from CPU-written memory run.
    if (in_start < Memory::VRAM_PADDR || in_size == 0 ||
        in_start + in_size > Memory::FCRAM_N3DS_PADDR_END) {
        return false;
    }
    // Output in FCRAM may be read back by the CPU, so only VRAM destinations are skipped.
    if (out_start < Memory::VRAM_PADDR || out_size == 0 ||
        u64{out_start} + out_size > Memory::VRAM_PADDR_END) {
        return false;
    }
    if (!impl->memory.AnyGpuDirtyPage(in_start, static_cast<u32>(in_size)) &&
        !AnySkipStale(in_start, static_cast<u32>(in_size))) {
        return false;
    }
    MarkSkipStale(out_start, out_size);
    impl->gpu_thread->job_skipped_any = true;
    return true;
}

void GPU::MarkSkipStale(PAddr start, u64 size) {
    auto& state = *impl->gpu_thread;
    const u64 begin = std::max<u64>(start, Memory::VRAM_PADDR);
    const u64 end = std::min<u64>(u64{start} + size, Memory::FCRAM_N3DS_PADDR_END);
    for (u64 page = begin & ~u64{Memory::CITRA_PAGE_MASK}; page < end;
         page += Memory::CITRA_PAGE_SIZE) {
        const u32 index = static_cast<u32>((page - Memory::VRAM_PADDR) >> Memory::CITRA_PAGE_BITS);
        if (state.skip_stale[index].exchange(1, std::memory_order_release) == 0) {
            state.stale_pages.push_back(index);
        }
    }
}

bool GPU::AnySkipStale(PAddr start, u32 size) const {
    const auto& state = *impl->gpu_thread;
    if (!state.skip_stale || size == 0) {
        return false;
    }
    const u64 begin = std::max<u64>(start, Memory::VRAM_PADDR);
    const u64 end = std::min<u64>(u64{start} + size, Memory::FCRAM_N3DS_PADDR_END);
    for (u64 page = begin & ~u64{Memory::CITRA_PAGE_MASK}; page < end;
         page += Memory::CITRA_PAGE_SIZE) {
        const u32 index = static_cast<u32>((page - Memory::VRAM_PADDR) >> Memory::CITRA_PAGE_BITS);
        if (state.skip_stale[index].load(std::memory_order_acquire) != 0) {
            return true;
        }
    }
    return false;
}

void GPU::ClearSkipStale() {
    auto& state = *impl->gpu_thread;
    for (const u32 index : state.stale_pages) {
        state.skip_stale[index].store(0, std::memory_order_release);
    }
    state.stale_pages.clear();
}

void GPU::RouteInterrupt(Service::GSP::InterruptId id, u64 delay_ns) {
    if (impl->gpu_thread && std::this_thread::get_id() == impl->gpu_thread->id) {
        impl->gpu_thread->job_irqs.emplace_back(id, delay_ns);
        return;
    }
    // Keep GPU-thread interrupts ahead of the ones raised later on the emulation thread.
    DeliverCompletions();
    if (impl->emu_signal_interrupt) {
        impl->emu_signal_interrupt(id, delay_ns);
    }
}

bool GPU::DeferRasterizerMark(PAddr start, u32 size, bool cached) {
    auto& state = *impl->gpu_thread;
    if (std::this_thread::get_id() != state.id) {
        // The caller applies its own mark right after; earlier GPU-thread marks go first.
        ApplyPendingMarks();
        return false;
    }
    state.job_marks.push_back({start, size, cached});
    return true;
}

void GPU::ApplyPendingMarks() {
    auto& state = *impl->gpu_thread;
    if (!state.has_completions.load(std::memory_order_acquire)) {
        return;
    }
    std::vector<GpuThreadState::Mark> marks;
    {
        std::scoped_lock lock{state.completion_mutex};
        marks.swap(state.marks);
        state.has_completions.store(!state.irqs.empty(), std::memory_order_relaxed);
    }
    for (const auto& mark : marks) {
        impl->memory.ApplyRasterizerMark(mark.start, mark.size, mark.cached);
    }
}

bool GPU::DeliverCompletions() {
    if (!impl->gpu_thread) {
        return false;
    }
    auto& state = *impl->gpu_thread;
    if (!state.has_completions.load(std::memory_order_acquire) ||
        std::this_thread::get_id() == state.id) {
        return false;
    }
    std::vector<GpuThreadState::Mark> marks;
    std::vector<std::pair<Service::GSP::InterruptId, u64>> irqs;
    {
        std::scoped_lock lock{state.completion_mutex};
        marks.swap(state.marks);
        irqs.swap(state.irqs);
        state.has_completions.store(false, std::memory_order_relaxed);
    }
    // Marks first: the guest may touch the region as soon as it sees the interrupt.
    for (const auto& mark : marks) {
        impl->memory.ApplyRasterizerMark(mark.start, mark.size, mark.cached);
    }
    if (impl->emu_signal_interrupt) {
        for (const auto& [id, delay_ns] : irqs) {
            impl->emu_signal_interrupt(id, delay_ns);
        }
    }
    return !irqs.empty();
}

void GPU::OnSliceStart() {
    if (!impl->gpu_thread) {
        return;
    }
    FlushCpuInvalidates();
    if (impl->gpu_thread->sync_each_slice) {
        WaitIdle(GpuThreadState::Idle);
    }
    DeliverCompletions();
}

bool GPU::SyncForIdle() {
    if (!impl->gpu_thread) {
        return false;
    }
    if (impl->gpu_thread->sync_on_idle) {
        WaitIdle(GpuThreadState::Idle);
    }
    return DeliverCompletions();
}

PAddr GPU::VirtualToPhysicalAddress(VAddr addr) {
    if (addr == 0) {
        return 0;
    }

    if (addr >= Memory::VRAM_VADDR && addr <= Memory::VRAM_VADDR_END) {
        return addr - Memory::VRAM_VADDR + Memory::VRAM_PADDR;
    }
    if (addr >= Memory::LINEAR_HEAP_VADDR && addr <= Memory::LINEAR_HEAP_VADDR_END) {
        return addr - Memory::LINEAR_HEAP_VADDR + Memory::FCRAM_PADDR;
    }
    if (addr >= Memory::NEW_LINEAR_HEAP_VADDR && addr <= Memory::NEW_LINEAR_HEAP_VADDR_END) {
        return addr - Memory::NEW_LINEAR_HEAP_VADDR + Memory::FCRAM_PADDR;
    }
    PAddr plg_fb_addr;
    if (addr >= Memory::PLUGIN_3GX_FB_VADDR && addr <= Memory::PLUGIN_3GX_FB_VADDR_END &&
        (plg_fb_addr = impl->system.Memory().Plugin3GXFramebufferAddress())) {
        return addr - Memory::PLUGIN_3GX_FB_VADDR + plg_fb_addr;
    }

    LOG_ERROR(HW_Memory, "Unknown virtual address @ 0x{:08X}", addr);
    return addr;
}

void GPU::SetInterruptHandler(Service::GSP::InterruptHandler handler) {
    if (!impl->gpu_thread) {
        impl->signal_interrupt = handler;
        impl->pica.SetInterruptHandler(handler);
        return;
    }
    WaitIdle();
    DeliverCompletions();
    impl->emu_signal_interrupt = std::move(handler);
    Service::GSP::InterruptHandler routed = [this](Service::GSP::InterruptId id, u64 delay_ns) {
        RouteInterrupt(id, delay_ns);
    };
    impl->signal_interrupt = routed;
    impl->pica.SetInterruptHandler(routed);
}

void GPU::FlushRegion(PAddr addr, u32 size) {
    WaitIdle();
    impl->rasterizer->FlushRegion(addr, size);
}

void GPU::InvalidateRegion(PAddr addr, u32 size) {
    WaitIdle();
    impl->rasterizer->InvalidateRegion(addr, size);
}

void GPU::ClearAll(bool flush) {
    WaitIdle();
    impl->rasterizer->ClearAll(flush);
}

void GPU::Execute(const Service::GSP::Command& command) {
    if (!impl->gpu_thread) {
        ExecuteImpl(command);
        return;
    }
    if (command.id == Service::GSP::CommandId::RequestDma) {
        // DMA copies guest memory and signals on this thread, so the GPU must be done first.
        WaitIdle(GpuThreadState::Dma);
        ExecuteImpl(command);
        return;
    }
    const u64 seq = PushGpuWork("GSP command", [this, command] { ExecuteImpl(command); });
    NotePendingWrites(command, seq);
}

void GPU::ExecuteImpl(const Service::GSP::Command& command) {
    using Service::GSP::CommandId;
    auto& regs = impl->pica.regs;

    switch (command.id) {
    case CommandId::RequestDma: {
        impl->system.Memory().RasterizerFlushVirtualRegion(
            command.dma_request.source_address, command.dma_request.size, Memory::FlushMode::Flush);
        impl->system.Memory().RasterizerFlushVirtualRegion(command.dma_request.dest_address,
                                                           command.dma_request.size,
                                                           Memory::FlushMode::Invalidate);

        // TODO(Subv): These memory accesses should not go through the application's memory mapping.
        // They should go through the GSP module's memory mapping.
        const auto process = impl->system.Kernel().GetCurrentProcess();
        impl->memory.CopyBlock(*process, command.dma_request.dest_address,
                               command.dma_request.source_address, command.dma_request.size);

        auto is_vram = [&](u32 addr) {
            return addr >= Memory::VRAM_VADDR && addr <= Memory::VRAM_VADDR_END;
        };

        u64 delay = DelayGenerator::CalculateDelayNanoseconds(
            DelayGenerator::GetCopyMode(is_vram(command.dma_request.source_address),
                                        is_vram(command.dma_request.dest_address)),
            false, command.dma_request.size);

        impl->signal_interrupt(Service::GSP::InterruptId::DMA, delay);
        break;
    }
    case CommandId::SubmitCmdList: {
        auto& params = command.submit_gpu_cmdlist;
        auto& cmdbuffer = regs.internal.pipeline.command_buffer;

        // Write to the command buffer GPU registers
        cmdbuffer.addr[0].Assign(VirtualToPhysicalAddress(params.address) >> 3);
        cmdbuffer.size[0].Assign(params.size >> 3);
        cmdbuffer.trigger[0] = 1;

        // Trigger processing of the command list
        SubmitCmdList(0);
        break;
    }
    case CommandId::MemoryFill: {
        auto& params = command.memory_fill;
        auto& memfill = regs.memory_fill_config;

        // Write to the memory fill GPU registers.
        // If both buffers are set GSP dispatches PSC0 only.
        const bool has_both_bufs = params.start1 != 0 && params.start2 != 0;
        if (params.start1 != 0) {
            memfill[0].address_start = VirtualToPhysicalAddress(params.start1) >> 3;
            memfill[0].address_end = VirtualToPhysicalAddress(params.end1) >> 3;
            memfill[0].value_32bit = params.value1;
            memfill[0].control = params.control1;
            MemoryFill(0, has_both_bufs ? std::numeric_limits<u32>::max() : 0);
        }
        if (params.start2 != 0) {
            memfill[1].address_start = VirtualToPhysicalAddress(params.start2) >> 3;
            memfill[1].address_end = VirtualToPhysicalAddress(params.end2) >> 3;
            memfill[1].value_32bit = params.value2;
            memfill[1].control = params.control2;
            MemoryFill(1, has_both_bufs ? 0 : 1);
        }
        break;
    }
    case CommandId::DisplayTransfer: {
        auto& params = command.display_transfer;
        auto& display_transfer = regs.display_transfer_config;

        // Write to the transfer engine GPU registers.
        display_transfer.input_address = VirtualToPhysicalAddress(params.in_buffer_address) >> 3;
        display_transfer.output_address = VirtualToPhysicalAddress(params.out_buffer_address) >> 3;
        display_transfer.input_size = params.in_buffer_size;
        display_transfer.output_size = params.out_buffer_size;
        display_transfer.flags = params.flags;
        display_transfer.trigger.Assign(1);

        // Trigger the display transfer.
        MemoryTransfer();
        break;
    }
    case CommandId::TextureCopy: {
        auto& params = command.texture_copy;
        auto& texture_copy = regs.display_transfer_config;

        // Write to the transfer engine GPU registers.
        texture_copy.input_address = VirtualToPhysicalAddress(params.in_buffer_address) >> 3;
        texture_copy.output_address = VirtualToPhysicalAddress(params.out_buffer_address) >> 3;
        texture_copy.texture_copy.size = params.size;
        texture_copy.texture_copy.input_size = params.in_width_gap;
        texture_copy.texture_copy.output_size = params.out_width_gap;
        texture_copy.flags = params.flags;
        texture_copy.trigger.Assign(1);

        // Trigger the texture copy.
        MemoryTransfer();
        break;
    }
    case CommandId::CacheFlush: {
        // Rasterizer flushing handled elsewhere in CPU read/write and other GPU handlers
        // Use command.cache_flush.regions to implement this handler
        break;
    }
    default:
        LOG_ERROR(HW_GPU, "Unknown command {:#08X}", command.id.Value());
    }

    // Notify debugger that a GSP command was processed.
    if (impl->debug_context) {
        impl->debug_context->OnEvent(Pica::DebugContext::Event::GSPCommandProcessed, &command);
    }
}

void GPU::SetBufferSwap(u32 screen_id, const Service::GSP::FrameBufferInfo& info) {
    if (impl->gpu_thread) {
        PushGpuWork("SetBufferSwap",
                    [this, screen_id, info] { SetBufferSwapImpl(screen_id, info); });
    } else {
        SetBufferSwapImpl(screen_id, info);
    }

    if (screen_id == 0) {
        if (impl->gpu_thread) {
            impl->gpu_thread->top_swap_this_frame = true;
        }
        MicroProfileFlip();
        impl->system.perf_stats->EndGameFrame();
    }
}

void GPU::SetBufferSwapImpl(u32 screen_id, const Service::GSP::FrameBufferInfo& info) {
    const PAddr phys_address_left = VirtualToPhysicalAddress(info.address_left);
    const PAddr phys_address_right = VirtualToPhysicalAddress(info.address_right);

    // Update framebuffer properties.
    auto& framebuffer = impl->pica.regs.framebuffer_config[screen_id];
    if (info.active_fb == 0) {
        framebuffer.address_left1 = phys_address_left;
        framebuffer.address_right1 = phys_address_right;
    } else {
        framebuffer.address_left2 = phys_address_left;
        framebuffer.address_right2 = phys_address_right;
    }

    framebuffer.stride = info.stride;
    framebuffer.format = info.format;
    framebuffer.active_fb = info.shown_fb;

    // Notify debugger about the buffer swap.
    if (impl->debug_context) {
        impl->debug_context->OnEvent(Pica::DebugContext::Event::BufferSwapped, nullptr);
    }

    if (screen_id == 0) {
        right_eye_disabler->ReportEndFrame();
    }
}

void GPU::SetColorFill(const Pica::ColorFill& fill) {
    const auto apply = [this, fill] {
        impl->pica.regs_lcd.color_fill_top = fill;
        impl->pica.regs_lcd.color_fill_bottom = fill;
    };
    if (impl->gpu_thread) {
        PushGpuWork("SetColorFill", apply);
    } else {
        apply();
    }
}

u32 GPU::ReadReg(VAddr addr) {
    WaitIdle(GpuThreadState::ReadReg);
    switch (addr & 0xFFFFF000) {
    case VADDR_LCD: {
        const u32 offset = addr - VADDR_LCD;
        const u32 index = offset / sizeof(u32);
        ASSERT(addr % sizeof(u32) == 0);
        ASSERT(index < Pica::RegsLcd::NumIds());
        return impl->pica.regs_lcd[index];
    }
    case VADDR_GPU:
    case VADDR_GPU + 0x1000: {
        const u32 offset = addr - VADDR_GPU;
        const u32 index = offset / sizeof(u32);
        ASSERT(addr % sizeof(u32) == 0);
        ASSERT(index < Pica::PicaCore::Regs::NUM_REGS);
        return impl->pica.regs.reg_array[index];
    }
    default:
        UNREACHABLE_MSG("Read from unknown GPU address {:#08X}", addr);
    }
}

void GPU::WriteReg(VAddr addr, u32 data) {
    if (impl->gpu_thread) {
        const u64 seq = PushGpuWork("WriteReg", [this, addr, data] { WriteRegImpl(addr, data); });
        const u32 index = (addr - VADDR_GPU) / sizeof(u32);
        const bool gpu_reg = (addr & 0xFFFFF000) == VADDR_GPU ||
                             (addr & 0xFFFFF000) == VADDR_GPU + 0x1000;
        if (gpu_reg && (index == GPU_REG_INDEX(memory_fill_config[0].trigger) ||
                        index == GPU_REG_INDEX(memory_fill_config[1].trigger) ||
                        index == GPU_REG_INDEX(display_transfer_config.trigger))) {
            // The output range lives in GPU registers; treat all memory as written until done.
            impl->gpu_thread->pending_writes.push_back({0, 0xFFFFFFFF, seq});
        }
        return;
    }
    WriteRegImpl(addr, data);
}

void GPU::WriteRegImpl(VAddr addr, u32 data) {
    switch (addr & 0xFFFFF000) {
    case VADDR_LCD: {
        const u32 offset = addr - VADDR_LCD;
        const u32 index = offset / sizeof(u32);
        ASSERT(addr % sizeof(u32) == 0);
        ASSERT(index < Pica::RegsLcd::NumIds());
        impl->pica.regs_lcd[index] = data;
        break;
    }
    case VADDR_GPU:
    case VADDR_GPU + 0x1000: {
        const u32 offset = addr - VADDR_GPU;
        const u32 index = offset / sizeof(u32);

        ASSERT(addr % sizeof(u32) == 0);
        ASSERT(index < Pica::PicaCore::Regs::NUM_REGS);
        impl->pica.regs.reg_array[index] = data;

        // Handle registers that trigger GPU actions
        switch (index) {
        case GPU_REG_INDEX(memory_fill_config[0].trigger):
            MemoryFill(0, 0);
            break;
        case GPU_REG_INDEX(memory_fill_config[1].trigger):
            MemoryFill(1, 1);
            break;
        case GPU_REG_INDEX(display_transfer_config.trigger):
            MemoryTransfer();
            break;
        case GPU_REG_INDEX(internal.pipeline.command_buffer.trigger[0]):
            SubmitCmdList(0);
            break;
        case GPU_REG_INDEX(internal.pipeline.command_buffer.trigger[1]):
            SubmitCmdList(1);
            break;
        default:
            break;
        }
        break;
    }
    default:
        UNREACHABLE_MSG("Write to unknown GPU address {:#08X}", addr);
    }
}

VideoCore::RendererBase& GPU::Renderer() {
    return *impl->renderer;
}

Pica::PicaCore& GPU::PicaCore() {
    return impl->pica;
}

const Pica::PicaCore& GPU::PicaCore() const {
    return impl->pica;
}

GraphicsDebugger& GPU::Debugger() {
    return impl->gpu_debugger;
}

void GPU::ApplyPerProgramSettings(u64 program_ID) {
    WaitIdle();
    auto hack = Common::Hacks::hack_manager.GetHack(
        Common::Hacks::HackType::ACCURATE_MULTIPLICATION, program_ID);
    bool use_accurate_mul = Settings::values.shaders_accurate_mul.GetValue();
    if (hack) {
        switch (hack->mode) {
        case Common::Hacks::HackAllowMode::DISALLOW:
            use_accurate_mul = false;
            break;
        case Common::Hacks::HackAllowMode::FORCE:
            use_accurate_mul = true;
            break;
        case Common::Hacks::HackAllowMode::ALLOW:
        default:
            break;
        }
    }
    impl->rasterizer->SetAccurateMul(use_accurate_mul);
}

void GPU::SubmitCmdList(u32 index) {
    // Check if a command list was triggered.
    auto& config = impl->pica.regs.internal.pipeline.command_buffer;
    if (!config.trigger[index]) {
        return;
    }

    MICROPROFILE_SCOPE(GPU_CmdlistProcessing);

    // Forward command list processing to the PICA core.
    const PAddr addr = config.GetPhysicalAddress(index);
    const u32 size = config.GetSize(index);
    impl->pica.ProcessCmdList(addr, size,
                              !right_eye_disabler->ShouldAllowCmdQueueTrigger(addr, size));
    config.trigger[index] = 0;
}

void GPU::MemoryFill(u32 index, u32 intr_index) {
    // Check if a memory fill was triggered.
    auto& config = impl->pica.regs.memory_fill_config[index];
    if (!config.trigger) {
        return;
    }

    // Perform memory fill.
    const u64 fill_size = config.GetEndAddress() - config.GetStartAddress();
    if (!SkipGpuOutput(config.GetStartAddress(), fill_size, config.GetStartAddress(),
                       fill_size) &&
        !impl->rasterizer->AccelerateFill(config)) {
        impl->sw_blitter->MemoryFill(config);
    }

    // Treat fill as texture transfer from VRAM
    u64 delay = DelayGenerator::CalculateDelayNanoseconds(
        DelayGenerator::GetCopyMode(true, config.IsVRAM()), true,
        config.GetEndAddress() - config.GetStartAddress());

    // It seems that it won't signal interrupt if "address_start" is zero.
    // TODO: hwtest this
    if (config.GetStartAddress() != 0) {
        if (intr_index == 0) {
            impl->signal_interrupt(Service::GSP::InterruptId::PSC0, delay);
        } else if (intr_index == 1) {
            impl->signal_interrupt(Service::GSP::InterruptId::PSC1, delay);
        }
    }

    // Reset "trigger" flag and set the "finish" flag
    // This was confirmed to happen on hardware even if "address_start" is zero.
    config.trigger.Assign(0);
    config.finished.Assign(1);
}

void GPU::MemoryTransfer() {
    // Check if a transfer was triggered.
    auto& config = impl->pica.regs.display_transfer_config;
    if (!config.trigger.Value()) {
        return;
    }

    MICROPROFILE_SCOPE(GPU_DisplayTransfer);

    // Notify debugger about the display transfer.
    if (impl->debug_context) {
        impl->debug_context->OnEvent(Pica::DebugContext::Event::IncomingDisplayTransfer, nullptr);
    }

    u64 delay{};
    // Perform memory transfer
    if (config.is_texture_copy) {
        const u64 copy_size = config.texture_copy.size;
        const u64 out_width = u64{config.texture_copy.output_width} * 16;
        const u64 out_gap = u64{config.texture_copy.output_gap} * 16;
        const u64 out_span =
            out_width == 0 ? copy_size
                           : (copy_size + out_width - 1) / out_width * (out_width + out_gap);
        if (!SkipGpuOutput(config.GetPhysicalInputAddress(), copy_size,
                           config.GetPhysicalOutputAddress(), out_span) &&
            !impl->rasterizer->AccelerateTextureCopy(config)) {
            impl->sw_blitter->TextureCopy(config);
        }
        delay = DelayGenerator::CalculateDelayNanoseconds(
            DelayGenerator::GetCopyMode(config.IsInputVRAM(), config.IsOutputVRAM()), true,
            config.texture_copy.size);
    } else {
        if (right_eye_disabler->ShouldAllowDisplayTransfer(config.GetPhysicalInputAddress(),
                                                           config.input_height)) {
            const u64 in_size = u64{config.input_width} * config.input_height *
                                BytesPerPixel(config.input_format);
            const u64 out_size = u64{config.output_width} * config.output_height *
                                 BytesPerPixel(config.output_format);
            if (!SkipGpuOutput(config.GetPhysicalInputAddress(), in_size,
                               config.GetPhysicalOutputAddress(), out_size) &&
                !impl->rasterizer->AccelerateDisplayTransfer(config)) {
                impl->sw_blitter->DisplayTransfer(config);
            }
        }
        delay = DelayGenerator::CalculateDelayNanoseconds(
            DelayGenerator::GetCopyMode(config.IsInputVRAM(), config.IsOutputVRAM()), true,
            config.input_width * config.input_height * BytesPerPixel(config.input_format));
    }

    // Complete transfer.
    config.trigger.Assign(0);
    impl->signal_interrupt(Service::GSP::InterruptId::PPF, delay);
}

void GPU::VBlankCallback(std::uintptr_t user_data, s64 cycles_late) {
    // Signal to GSP that GPU interrupt has occurred
    impl->signal_interrupt(Service::GSP::InterruptId::PDC0, 0);
    impl->signal_interrupt(Service::GSP::InterruptId::PDC1, 0);

    if (impl->gpu_thread) {
        auto& state = *impl->gpu_thread;
        state.swaps_pushed++;
        PushGpuWork("SwapBuffers", [this] {
            auto& gpu_state = *impl->gpu_thread;
            if (gpu_state.job_skipping) {
                impl->renderer->SkipPresent();
                gpu_state.skipped_frames.fetch_add(1, std::memory_order_relaxed);
                gpu_state.job_skipped_any = true;
            } else {
                impl->renderer->SwapBuffers();
                if (gpu_state.skip_stale) {
                    ClearSkipStale();
                }
            }
            impl->renderer->gpu_thread_cpu_ns.store(ThreadCpuNs(), std::memory_order_relaxed);
            std::scoped_lock lock{impl->gpu_thread->mutex};
            impl->gpu_thread->swaps_done++;
        });
        {
            // Skip the next frame only after two frames that each swapped the top screen,
            // and never two frames in a row.
            const bool this_skipped =
                state.frame_skip &&
                state.frame_id > state.skip_cancel_upto.load(std::memory_order_relaxed);
            const bool next_skip = state.skip_enabled &&
                                   !state.skip_disabled.load(std::memory_order_relaxed) &&
                                   !this_skipped && state.top_swap_this_frame &&
                                   state.top_swap_prev_frame;
            state.top_swap_prev_frame = state.top_swap_this_frame;
            state.top_swap_this_frame = false;
            state.frame_id++;
            state.frame_skip = next_skip;
        }
        {
            std::unique_lock lock{state.mutex};
            WaitLogged(lock, state, GpuThreadState::Frame, [&] {
                return state.swaps_pushed - state.swaps_done <= GPU_MAX_SWAPS_IN_FLIGHT;
            });
        }
        if (state.swaps_pushed % GPU_STATS_EVERY_FRAMES == 0) {
            std::string line;
            for (u32 i = 0; i < GpuThreadState::NumReasons; ++i) {
                line += fmt::format(" {}={}/{}/{:.0f}ms", WAIT_REASON_NAMES[i], state.waits[i],
                                    state.blocked[i], state.blocked_ms[i]);
            }
            LOG_WARNING(HW_GPU,
                        "GPU thread stats (calls/blocked/ms) frame={}:{} cpu_read_skip={} "
                        "cpu_write_defer={} skipped_frames/jobs={}/{} skip_cancel={} "
                        "skip_violation={}",
                        state.swaps_pushed, line, state.cpu_reads_skipped,
                        state.cpu_writes_deferred, state.skipped_frames.load(),
                        state.skipped_jobs.load(), state.skip_cancels,
                        state.skip_violations.load());
        }
        impl->renderer->FinishFrame();
    } else {
        // Present renderered frame.
        impl->renderer->SwapBuffers();
    }

    // Reschedule recurrent event
    impl->timing.ScheduleEvent(FRAME_TICKS - cycles_late, impl->vblank_event);
}

void GPU::RecreateRenderer(Frontend::EmuWindow& emu_window, Frontend::EmuWindow* secondary_window) {
    WaitIdle();
    // Reset the renderer (this will destroy OpenGL resources)
    impl->renderer.reset();

    // Create a new renderer
    impl->renderer =
        VideoCore::CreateRenderer(emu_window, secondary_window, impl->pica, impl->system);
    impl->rasterizer = impl->renderer->Rasterizer();
    if (impl->memory.IsGpuDirtyTracking()) {
        impl->memory.SetGpuDirtyPages(0, 0xFFFFFFFF, false);
    }

    // Rebind the rasterizer to the PICA GPU
    impl->pica.BindRasterizer(impl->rasterizer);

    // Update the sw_blitter with the new rasterizer
    impl->sw_blitter = std::make_unique<SwRenderer::SwBlitter>(impl->memory, impl->rasterizer);

    // Re-apply per-game configuration and reload disk shader cache
    u64 program_id{};
    impl->system.GetAppLoader().ReadProgramId(program_id);
    ApplyPerProgramSettings(program_id);
    if (Settings::values.use_disk_shader_cache) {
        impl->renderer->Rasterizer()->LoadDefaultDiskResources(false, nullptr);
    }

    // Mark ALL GPU registers as dirty so current state gets uploaded to new renderer
    impl->pica.dirty_regs.SetAllDirty();

    // Also mark shader setups as dirty so uniforms get re-uploaded and
    // stale pointers to the old rasterizer's JIT cache are cleared.
    impl->pica.vs_setup.uniforms_dirty = true;
    impl->pica.vs_setup.cached_shader = nullptr;
    impl->pica.gs_setup.uniforms_dirty = true;
    impl->pica.gs_setup.cached_shader = nullptr;

    // Mark all cached LUT/table state in pica as dirty
    impl->pica.lighting.lut_dirty = impl->pica.lighting.LutAllDirty;
    impl->pica.fog.lut_dirty = true;
    impl->pica.proctex.table_dirty = impl->pica.proctex.TableAllDirty;
}

void GPU::ReleaseRenderer() {
    WaitIdle();
    // Just reset the renderer to release OpenGL resources
    // Don't null out rasterizer pointer as it will become dangling
    impl->renderer.reset();
    impl->sw_blitter.reset();
    LOG_INFO(HW_GPU, "Renderer released for context destroy");
}

template <class Archive>
void GPU::serialize(Archive& ar, const u32 file_version) {
    WaitIdle();
    ar & impl->pica;
}

SERIALIZE_IMPL(GPU)

} // namespace VideoCore
