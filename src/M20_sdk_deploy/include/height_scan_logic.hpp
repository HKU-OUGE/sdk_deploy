// src/M20_sdk_deploy/include/height_scan_logic.hpp
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "types/common_types.h"   // types::Vec3f, types::Mat3f (re-exported below)
#include "utils/basic_function.hpp"  // RpyToRm

namespace height_scan {

// Re-export the SDK type aliases so code in this namespace can write
// `Vec3f` / `Mat3f` directly (they live under ::types in common_types.h).
using Vec3f = ::types::Vec3f;
using Mat3f = ::types::Mat3f;


// === Grid geometry (matches RayCaster training cfg) ===
constexpr int   GRID_NX        = 17;        // forward-axis cells
constexpr int   GRID_NY        = 11;        // lateral-axis cells
constexpr int   GRID_N         = GRID_NX * GRID_NY;  // 187
constexpr float GRID_RESOLUTION = 0.1f;     // metres per cell
constexpr float GRID_HALF_X    = 0.8f;      // x ∈ [-0.8, +0.8]
constexpr float GRID_HALF_Y    = 0.5f;      // y ∈ [-0.5, +0.5]
constexpr float MAX_POINT_DIST = 1.2f;      // reject points outside ~grid radius

// === Observation transform (matches mdp.height_scan with offset=0.5) ===
constexpr float OBS_OFFSET     = 0.5f;
constexpr float OBS_CLIP       = 1.0f;

// === Ring buffer / fallback ===
constexpr int   BUFFER_LEN     = 10;
constexpr float DEFAULT_FILL   = OBS_CLIP;  // = +1.0: "terrain far below / no hit" (safe default: implies clear floor, not obstacle clearance)

// Flat index convention: row-major, x outer, y inner.
inline int FlatIdx(int ix, int iy) { return ix * GRID_NY + iy; }

// Rotation that maps body-frame vectors to horizon-aligned frame
// (gravity-aligned, shares the body's yaw). Convention matches SDK RpyToRm.
inline Mat3f BodyToHorizon(float roll, float pitch) {
    return RpyToRm(Vec3f(roll, pitch, 0.0f));
}

}  // namespace height_scan
