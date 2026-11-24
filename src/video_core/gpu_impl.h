// Copyright 2023 Citra Emulator Project
// Copyright 2024 Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include "common/archives.h"
#include "common/microprofile.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/hle/service/gsp/gsp_gpu.h"
#include "core/hle/service/plgldr/plgldr.h"
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

struct GpuThreadState {
    enum WaitReason : u32 { Memory, ReadReg, Idle, Dma, Other, QueueFull, Frame, NumReasons };

    struct Item {
        const char* name;
        u64 seq;
        // Emulated frame (vblank interval) the job was pushed in, and whether that frame was
        // chosen to be left unshown.
        u64 frame;
        bool skip_candidate;
        std::function<void()> fn;
    };

    struct Mark {
        PAddr start;
        u32 size;
        bool cached;
    };

    // Memory a queued fill or transfer will write; [start, end) with the job's sequence number.
    struct PendingWrite {
        PAddr start;
        PAddr end;
        u64 seq;
    };

    std::thread thread;
    std::thread::id id;
    // AZAHAR_GPU_THREAD=2: finish GPU work at every slice start, so interrupts land at
    // deterministic points in emulated time (for movie playback).
    bool sync_each_slice{};
    // AZAHAR_GPU_THREAD=3: never drain when all cores idle (emulated time may skip ahead of
    // pending GPU interrupts; for measuring the overlap ceiling only).
    bool sync_on_idle{true};

    // Guarded by mutex.
    std::mutex mutex;
    std::condition_variable work_cv;
    std::condition_variable done_cv;
    std::deque<Item> queue;
    bool busy{};
    bool stop{};
    const char* current{"idle"};
    u64 swaps_done{};

    // Emulation thread only.
    u64 swaps_pushed{};
    u64 pushed_seq{};
    std::deque<PendingWrite> pending_writes;
    std::vector<std::pair<PAddr, u32>> cpu_invalidates;
    u64 cpu_reads_skipped{};
    u64 cpu_writes_deferred{};
    std::array<u64, NumReasons> waits{};
    std::array<u64, NumReasons> blocked{};
    std::array<double, NumReasons> blocked_ms{};

    // AZAHAR_SKIP_UNSHOWN=1, emulation thread only.
    bool skip_enabled{};
    u64 frame_id{1};
    bool frame_skip{};
    bool top_swap_this_frame{};
    bool top_swap_prev_frame{};
    u64 skip_cancels{};

    // Jobs of frames up to this one run in full even if their frame was chosen for skipping.
    std::atomic<u64> skip_cancel_upto{0};
    std::atomic<bool> skip_disabled{false};
    std::atomic<u64> skipped_frames{0};
    std::atomic<u64> skipped_jobs{0};
    std::atomic<u64> skip_violations{0};
    // Pages a skipped job would have written, from VRAM to the end of FCRAM. Set on the GPU
    // thread, read on the emulation thread, cleared when the next shown frame is presented.
    std::unique_ptr<std::atomic<u8>[]> skip_stale;

    // GPU thread only: skip state of the running job and the stale pages to clear later.
    bool job_skipping{};
    bool job_skipped_any{};
    std::vector<u32> stale_pages;

    // GPU thread only: what the running job produced, published when the job ends.
    std::vector<std::pair<Service::GSP::InterruptId, u64>> job_irqs;
    std::vector<Mark> job_marks;

    // Results produced on the GPU thread, consumed on the emulation thread.
    std::mutex completion_mutex;
    std::vector<std::pair<Service::GSP::InterruptId, u64>> irqs;
    std::vector<Mark> marks;
    std::atomic<bool> has_completions{false};
    std::atomic<u64> completed_seq{0};
};

struct GPU::Impl {
    Core::Timing& timing;
    Core::System& system;
    Memory::MemorySystem& memory;
    std::shared_ptr<Pica::DebugContext> debug_context;
    Pica::PicaCore pica;
    GraphicsDebugger gpu_debugger;
    std::unique_ptr<RendererBase> renderer;
    RasterizerInterface* rasterizer;
    std::unique_ptr<SwRenderer::SwBlitter> sw_blitter;
    Core::TimingEventType* vblank_event;
    Service::GSP::InterruptHandler signal_interrupt;
    Service::GSP::InterruptHandler emu_signal_interrupt;
    std::unique_ptr<GpuThreadState> gpu_thread;

    explicit Impl(Core::System& system, Frontend::EmuWindow& emu_window,
                  Frontend::EmuWindow* secondary_window)
        : timing{system.CoreTiming()}, system{system}, memory{system.Memory()},
          debug_context{Pica::g_debug_context}, pica{memory, debug_context},
          renderer{VideoCore::CreateRenderer(emu_window, secondary_window, pica, system)},
          rasterizer{renderer->Rasterizer()},
          sw_blitter{std::make_unique<SwRenderer::SwBlitter>(memory, rasterizer)} {}
    ~Impl() = default;
};
} // namespace VideoCore