// 对拍 harness 专用 stub：DWT 虚拟计数器。harness 每推进一拍调用
// dwt_advance_tick()（480 MHz 下 1 ms = 480000 周期），使 DWT_GetDeltaT
// 恒等于 1 ms，与 Helios 1 kHz 定时语义一致。
#pragma once

#include <cstdint>

inline uint32_t g_dwt_virtual_cycles = 0;
inline uint32_t g_dwt_ticks_per_ms = 480000;
inline uint32_t g_dwt_last_cycles = 0;

struct HarnessDwt {
    uint32_t CYCCNT = 0;
};
inline HarnessDwt g_harness_dwt;
#define DWT (&g_harness_dwt)

inline void DWT_Init(uint32_t ticks_per_ms) {
    g_dwt_ticks_per_ms = ticks_per_ms;
}

inline void dwt_advance_tick() {
    g_dwt_virtual_cycles += g_dwt_ticks_per_ms;
    g_harness_dwt.CYCCNT = g_dwt_virtual_cycles;
}

inline float DWT_GetDeltaT(uint32_t* last) {
    const uint32_t now = g_dwt_virtual_cycles;
    const uint32_t dt_cycles = now - *last;
    *last = now;
    return static_cast<float>(dt_cycles) / static_cast<float>(g_dwt_ticks_per_ms)
         * 0.001f * 1000.0f / 1000.0f;
}
