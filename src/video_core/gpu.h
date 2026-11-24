// Copyright 2023-2026 Citra Emulator Project / Azahar Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <functional>
#include <memory>
#include <boost/serialization/access.hpp>

#include "core/hle/service/gsp/gsp_interrupt.h"

namespace Service::GSP {
struct Command;
struct FrameBufferInfo;
} // namespace Service::GSP

namespace Core {
class System;
}

namespace Pica {
class DebugContext;
class PicaCore;
struct RegsLcd;
union ColorFill;
} // namespace Pica

namespace Frontend {
class EmuWindow;
}

namespace Memory {
enum class FlushMode;
}

namespace VideoCore {

/// Measured on hardware to be 2240568 timer cycles or 4481136 ARM11 cycles
constexpr u64 FRAME_TICKS = 4481136ull;

class GraphicsDebugger;
class RendererBase;
class RightEyeDisabler;

/**
 * The GPU class is the high level interface to the video_core for core services.
 */
class GPU {
public:
    explicit GPU(Core::System& system, Frontend::EmuWindow& emu_window,
                 Frontend::EmuWindow* secondary_window);
    ~GPU();

    /// Sets the function to call for signalling GSP interrupts.
    void SetInterruptHandler(Service::GSP::InterruptHandler handler);

    /// Notify rasterizer that any caches of the specified region should be flushed to Switch memory
    void FlushRegion(PAddr addr, u32 size);

    /// Notify rasterizer that any caches of the specified region should be invalidated
    void InvalidateRegion(PAddr addr, u32 size);

    /// Flushes and invalidates all memory in the rasterizer cache and removes any leftover state.
    void ClearAll(bool flush);

    /// Executes the provided GSP command.
    void Execute(const Service::GSP::Command& command);

    /// Updates GPU display framebuffer configuration using the specified parameters.
    void SetBufferSwap(u32 screen_id, const Service::GSP::FrameBufferInfo& info);

    /// Sets the LCD color fill configuration for the top and bottom screens.
    void SetColorFill(const Pica::ColorFill& fill);

    /// Reads a word from the GPU virtual address.
    u32 ReadReg(VAddr addr);

    /// Writes the provided value to the GPU virtual address.
    void WriteReg(VAddr addr, u32 data);

    /// Returns a mutable reference to the renderer.
    [[nodiscard]] VideoCore::RendererBase& Renderer();

    /// Returns a mutable reference to the PICA GPU.
    [[nodiscard]] Pica::PicaCore& PicaCore();

    /// Returns an immutable reference to the PICA GPU.
    [[nodiscard]] const Pica::PicaCore& PicaCore() const;

    /// Returns a mutable reference to the GSP command debugger.
    [[nodiscard]] GraphicsDebugger& Debugger();

    RightEyeDisabler& GetRightEyeDisabler() {
        return *right_eye_disabler;
    }

    void ApplyPerProgramSettings(u64 program_ID);

    /// Recreates the renderer (for GL context reset in libretro)
    void RecreateRenderer(Frontend::EmuWindow& emu_window, Frontend::EmuWindow* secondary_window);

    /// Releases the renderer (for GL context destroy in libretro)
    void ReleaseRenderer();

    /// Returns true when GPU work runs on a dedicated thread (AZAHAR_GPU_THREAD=1, Vulkan).
    [[nodiscard]] bool IsThreaded() const;

    /// Blocks until the GPU thread has finished all queued work, then applies its
    /// cached-region changes. No-op without a GPU thread or when called on it.
    void WaitIdle();

    /// Applies cached-region changes and delivers interrupts produced by the GPU thread.
    /// Must run on the emulation thread. Returns true if any interrupt was delivered.
    bool DeliverCompletions();

    /// Called when every CPU core is idle: drains the GPU thread so interrupts the guest waits
    /// for arrive before emulated time skips ahead. Returns true if an interrupt was delivered.
    bool SyncForIdle();

    /// Called by the emulation thread at the start of every run loop slice.
    void OnSliceStart();

private:
    void ExecuteImpl(const Service::GSP::Command& command);
    void WriteRegImpl(VAddr addr, u32 data);
    void SetBufferSwapImpl(u32 screen_id, const Service::GSP::FrameBufferInfo& info);

    void StartGpuThread();
    void StopGpuThread();
    void GpuThreadLoop();
    u64 PushGpuWork(const char* name, std::function<void()> fn);
    u64 Enqueue(const char* name, std::function<void()> fn);
    void WaitIdle(u32 reason);
    bool OnCpuAccess(PAddr start, u32 size, Memory::FlushMode mode);
    bool MayHaveGpuWrites(PAddr start, u32 size);
    void FlushCpuInvalidates();
    void NotePendingWrites(const Service::GSP::Command& command, u64 seq);
    void RouteInterrupt(Service::GSP::InterruptId id, u64 delay_ns);
    bool DeferRasterizerMark(PAddr start, u32 size, bool cached);
    void ApplyPendingMarks();
    void CancelSkip();
    bool SkipGpuOutput(PAddr in_start, u64 in_size, PAddr out_start, u64 out_size);
    void MarkSkipStale(PAddr start, u64 size);
    bool AnySkipStale(PAddr start, u32 size) const;
    void ClearSkipStale();

    void SubmitCmdList(u32 index);

    // Interrupt index must be 0 or 1 to signal the relative PSC interrupt.
    void MemoryFill(u32 index, u32 intr_index);

    void MemoryTransfer();

    void VBlankCallback(uintptr_t user_data, s64 cycles_late);

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const u32 file_version);

    std::unique_ptr<RightEyeDisabler> right_eye_disabler;

private:
    friend class RightEyeDisabler;
    struct Impl;
    std::unique_ptr<Impl> impl;

    PAddr VirtualToPhysicalAddress(VAddr addr);
};

} // namespace VideoCore
