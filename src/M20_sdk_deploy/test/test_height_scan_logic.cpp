// src/M20_sdk_deploy/test/test_height_scan_logic.cpp
// Standalone unit tests for height_scan_logic.hpp.
// No gtest dependency — plain main + assert.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "height_scan_logic.hpp"

using height_scan::Vec3f;
using height_scan::Mat3f;

static bool ApproxEq(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) < eps;
}

static void TestBodyToHorizon_Identity() {
    Mat3f R = height_scan::BodyToHorizon(0.0f, 0.0f);
    Vec3f p(1.0f, 2.0f, 3.0f);
    Vec3f q = R * p;
    assert(ApproxEq(q.x(), 1.0f));
    assert(ApproxEq(q.y(), 2.0f));
    assert(ApproxEq(q.z(), 3.0f));
    std::puts("[OK] TestBodyToHorizon_Identity");
}

static void TestBodyToHorizon_Pitch90() {
    // Per the SDK convention (RpyToRm = Rz·Ry·Rx with right-hand rule),
    // pitch=+pi/2 maps body-x to horizon (0, 0, -1).
    Mat3f R = height_scan::BodyToHorizon(0.0f, static_cast<float>(M_PI_2));
    Vec3f p(1.0f, 0.0f, 0.0f);
    Vec3f q = R * p;
    assert(ApproxEq(q.x(), 0.0f));
    assert(ApproxEq(q.y(), 0.0f));
    assert(ApproxEq(q.z(), -1.0f));
    std::puts("[OK] TestBodyToHorizon_Pitch90");
}

static void TestProjectAndBin_BoxAtFront() {
    // A 0.2 m tall box top surface at (x=0.5, y=0.0, z=-0.4) in body frame
    // (i.e., body is 0.5 m above the box top). Synthesise 100 points
    // tightly clustered on the box top.
    std::vector<height_scan::Vec3f> points;
    for (int i = 0; i < 100; ++i) {
        float dx = (i % 10) * 0.005f - 0.025f;  // ±2.5 cm spread, well inside one cell
        float dy = (i / 10) * 0.005f - 0.025f;
        points.emplace_back(0.5f + dx, 0.0f + dy, -0.4f);
    }

    std::array<float, height_scan::GRID_N> obs{};
    std::array<bool, height_scan::GRID_N> valid{};
    height_scan::ProjectAndBin(points, /*roll=*/0.0f, /*pitch=*/0.0f, obs, valid);

    // Cell at (ix = round((0.5 + 0.8)/0.1) = 13, iy = round((0.0+0.5)/0.1) = 5)
    int ix = 13, iy = 5;
    int idx = height_scan::FlatIdx(ix, iy);
    assert(valid[idx]);
    // obs = clip(-z_h - offset, -clip, +clip) = clip(-(-0.4) - 0.5, -1, +1) = -0.1
    assert(ApproxEq(obs[idx], -0.1f, 1e-3f));
    std::puts("[OK] TestProjectAndBin_BoxAtFront");
}

static void TestProjectAndBin_IndexCorners() {
    // One point at each corner of the grid, at z=-0.5 in body frame.
    // After projection: ix=0, iy=0 → flat 0; ix=16, iy=10 → flat 186.
    std::vector<height_scan::Vec3f> points = {
        {-0.8f, -0.5f, -0.5f},  // ix=0, iy=0
        {+0.8f, +0.5f, -0.5f},  // ix=16, iy=10
    };
    std::array<float, height_scan::GRID_N> obs{};
    std::array<bool, height_scan::GRID_N> valid{};
    height_scan::ProjectAndBin(points, 0.0f, 0.0f, obs, valid);
    assert(valid[0]);
    assert(valid[186]);
    // obs = clip(-(-0.5) - 0.5, ...) = 0.0
    assert(ApproxEq(obs[0], 0.0f, 1e-3f));
    assert(ApproxEq(obs[186], 0.0f, 1e-3f));
    std::puts("[OK] TestProjectAndBin_IndexCorners");
}

static void TestProjectAndBin_PitchInvariance() {
    // Same world geometry, two configurations:
    //   A. body level, point at world (1.0, 0.0, -0.4) — i.e. body-frame (1.0, 0.0, -0.4)
    //   B. body pitched +10°: same world point now expressed in body-frame is rotated.
    // Both should produce the same obs (height_scan is gravity-aligned).
    const float pitch = 10.0f * static_cast<float>(M_PI) / 180.0f;
    height_scan::Vec3f p_world(0.6f, 0.0f, -0.4f);

    // A: roll=0, pitch=0 → body frame == horizon frame; body-frame point = p_world.
    std::vector<height_scan::Vec3f> pts_a = { p_world };

    // B: body is rotated by +pitch around Y in world; therefore the body-frame coords
    // of the same world point are R_body_world * p_world = R_horizon_body^T * p_world.
    // Since R_horizon_body = BodyToHorizon(0, pitch), R_body_world ≈ that transposed.
    height_scan::Mat3f R_h_b = height_scan::BodyToHorizon(0.0f, pitch);
    height_scan::Vec3f p_body_b = R_h_b.transpose() * p_world;
    std::vector<height_scan::Vec3f> pts_b = { p_body_b };

    std::array<float, height_scan::GRID_N> obs_a{}, obs_b{};
    std::array<bool, height_scan::GRID_N> valid_a{}, valid_b{};
    height_scan::ProjectAndBin(pts_a, 0.0f, 0.0f, obs_a, valid_a);
    height_scan::ProjectAndBin(pts_b, 0.0f, pitch, obs_b, valid_b);

    int ix = static_cast<int>(std::lround((p_world.x() + height_scan::GRID_HALF_X)
                                          / height_scan::GRID_RESOLUTION));
    int iy = static_cast<int>(std::lround((p_world.y() + height_scan::GRID_HALF_Y)
                                          / height_scan::GRID_RESOLUTION));
    int idx = height_scan::FlatIdx(ix, iy);

    assert(valid_a[idx] && valid_b[idx]);
    assert(ApproxEq(obs_a[idx], obs_b[idx], 5e-3f));
    std::puts("[OK] TestProjectAndBin_PitchInvariance");
}

int main() {
    TestBodyToHorizon_Identity();
    TestBodyToHorizon_Pitch90();
    TestProjectAndBin_BoxAtFront();
    TestProjectAndBin_IndexCorners();
    TestProjectAndBin_PitchInvariance();
    std::puts("ALL TESTS PASSED");
    return 0;
}
