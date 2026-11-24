// Copyright 2015-2026 Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <chrono>
#include <cstdlib>
#include <ctime>
#include "common/settings.h"
#include "core/core.h"
#include "core/frontend/emu_window.h"
#include "core/movie.h"
#include "core/tracer/recorder.h"
#include "video_core/debug_utils/debug_utils.h"
#include "video_core/renderer_base.h"

namespace VideoCore {

RendererBase::RendererBase(Core::System& system_, Frontend::EmuWindow& window,
                           Frontend::EmuWindow* secondary_window_)
    : system{system_}, render_window{window}, secondary_window{secondary_window_} {
    if (std::getenv("AZAHAR_BENCH")) {
        Settings::values.deterministic_async_operations = true;
    }
}

RendererBase::~RendererBase() = default;

u32 RendererBase::GetResolutionScaleFactor() {
    const auto graphics_api = Settings::GetWorkingGraphicsAPI();
    if (graphics_api == Settings::GraphicsAPI::Software) {
        // Software renderer always render at native resolution
        return 1;
    }

    const u32 scale_factor = Settings::values.resolution_factor.GetValue();
    return scale_factor != 0 ? scale_factor
                             : render_window.GetFramebufferLayout().GetScalingRatio();
}

void RendererBase::UpdateCurrentFramebufferLayout(bool is_portrait_mode) {
    const auto update_layout = [is_portrait_mode](Frontend::EmuWindow& window) {
        const Layout::FramebufferLayout& layout = window.GetFramebufferLayout();
        window.UpdateCurrentFramebufferLayout(layout.width, layout.height, is_portrait_mode);
    };
    update_layout(render_window);
    if (secondary_window != nullptr) {
        update_layout(*secondary_window);
    }
}

void RendererBase::EndFrame() {
    current_frame++;
    if (!defer_frame_end) {
        FinishFrame();
    }
}

void RendererBase::FinishFrame() {
    finished_frames++;

    static const bool bench = std::getenv("AZAHAR_BENCH") != nullptr;
    if (bench && finished_frames % 300 == 0) {
        static auto last_wall = std::chrono::steady_clock::now();
        static timespec last_cpu{};
        static u64 last_gpu_ns = 0;
        timespec cpu{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu);
        const auto wall = std::chrono::steady_clock::now();
        const double wall_ms = std::chrono::duration<double, std::milli>(wall - last_wall).count();
        const double cpu_ms = (cpu.tv_sec - last_cpu.tv_sec) * 1e3 + (cpu.tv_nsec - last_cpu.tv_nsec) / 1e6;
        const u64 gpu_ns = gpu_thread_cpu_ns.load(std::memory_order_relaxed);
        LOG_WARNING(Render, "BENCH frame={} wall_ms={:.1f} emu_cpu_ms={:.1f} gpu_cpu_ms={:.1f}",
                    finished_frames, wall_ms, cpu_ms, (gpu_ns - last_gpu_ns) / 1e6);
        last_wall = wall;
        last_cpu = cpu;
        last_gpu_ns = gpu_ns;
    }
    static const char* save_every = std::getenv("AZAHAR_BENCH_SAVE_EVERY");
    if (save_every && finished_frames % std::strtoull(save_every, nullptr, 10) == 0 &&
        system.Movie().GetPlayMode() == Core::Movie::PlayMode::Recording) {
        system.Movie().SaveMovie();
        LOG_WARNING(Render, "BENCH movie saved at frame {}", finished_frames);
    }

    system.perf_stats->EndSystemFrame();

    render_window.PollEvents();

    system.frame_limiter.DoFrameLimiting(system.CoreTiming().GetGlobalTimeUs());
    system.perf_stats->BeginSystemFrame();
}

bool RendererBase::IsScreenshotPending() const {
    return settings.screenshot_requested;
}

void RendererBase::RequestScreenshot(void* data, std::function<void(bool)> callback,
                                     const Layout::FramebufferLayout& layout) {
    if (settings.screenshot_requested) {
        LOG_ERROR(Render, "A screenshot is already requested or in progress, ignoring the request");
        return;
    }
    settings.screenshot_bits = data;
    settings.screenshot_complete_callback = callback;
    settings.screenshot_framebuffer_layout = layout;
    settings.screenshot_requested = true;
}
} // namespace VideoCore
