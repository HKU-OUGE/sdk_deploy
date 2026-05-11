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

int main() {
    TestBodyToHorizon_Identity();
    TestBodyToHorizon_Pitch90();
    std::puts("ALL TESTS PASSED");
    return 0;
}
