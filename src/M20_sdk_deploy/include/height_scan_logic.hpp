// src/M20_sdk_deploy/include/height_scan_logic.hpp
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
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

// Project each body-frame point into the gravity-aligned (horizon) frame,
// bin into the 2-D grid, keep the max z_h per cell, then convert to the
// clipped observation value: obs = clip(-z_h - OBS_OFFSET, -OBS_CLIP, +OBS_CLIP).
inline void ProjectAndBin(const std::vector<Vec3f>& points_body,
                          float roll, float pitch,
                          std::array<float, GRID_N>& obs_out,
                          std::array<bool,  GRID_N>& valid_out) {
    // Initialise: invalid + "no obstacle" default
    valid_out.fill(false);
    obs_out.fill(DEFAULT_FILL);

    // Per-cell running max(z_h)
    std::array<float, GRID_N> max_zh;
    max_zh.fill(-std::numeric_limits<float>::infinity());

    const Mat3f R = BodyToHorizon(roll, pitch);

    const float max_dist_sq = MAX_POINT_DIST * MAX_POINT_DIST;

    for (const auto& p_b : points_body) {
        if (!std::isfinite(p_b.x()) || !std::isfinite(p_b.y()) || !std::isfinite(p_b.z()))
            continue;
        // Cheap early reject in body frame
        const float r2 = p_b.x()*p_b.x() + p_b.y()*p_b.y() + p_b.z()*p_b.z();
        if (r2 > max_dist_sq) continue;

        const Vec3f p_h = R * p_b;
        const int ix = static_cast<int>(std::lround((p_h.x() + GRID_HALF_X) / GRID_RESOLUTION));
        const int iy = static_cast<int>(std::lround((p_h.y() + GRID_HALF_Y) / GRID_RESOLUTION));
        if (ix < 0 || ix >= GRID_NX || iy < 0 || iy >= GRID_NY) continue;
        const int idx = FlatIdx(ix, iy);
        if (p_h.z() > max_zh[idx]) max_zh[idx] = p_h.z();
    }

    for (int i = 0; i < GRID_N; ++i) {
        if (std::isfinite(max_zh[i]) && max_zh[i] > -std::numeric_limits<float>::infinity()) {
            obs_out[i] = std::clamp(-max_zh[i] - OBS_OFFSET, -OBS_CLIP, OBS_CLIP);
            valid_out[i] = true;
        }
    }
}

}  // namespace height_scan
