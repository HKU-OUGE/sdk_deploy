# Height Scan Node — Onboard Elevation Observation for MoE Teacher

**Date:** 2026-05-11
**Author:** TANG Tianyang (with Claude as co-designer)
**Status:** Spec — pending review
**Scope:** sdk_deploy (new C++ node) + rl_training (re-enable `height_scan` obs)

---

## 1. Goal

Add an additional `height_scan` observation channel to the M20 MoE Teacher policy:

- Deploy side: write an onboard ROS 2 C++ node `height_scan_node` that consumes the merged LiDAR pointcloud + IMU and publishes a 187-dim height-scan vector that matches the training observation semantics.
- Training side: re-enable the `height_scan` observation in `moe_teacher_env_cfg.py` (currently disabled) and retrain.

The deploy node must run on the target machine's RK3588 CPU only (no GPU, no laptop dependency).

## 2. Background and motivation

The MoE Teacher cfg previously had `height_scan` enabled but it was removed (commit `e19a350 "Delete elevation"`; see also notes in `moe_teacher_env_cfg.py:477,731`). Code comments state:

> height_scan 已禁用 —— 真机 elevation map sim2real gap 太大，改用半球 LIDAR 代替

The abandoned path computed elevation on a laptop (via ANYbotics `elevation_mapping`) and pushed a `grid_map_msgs/GridMap` to the robot over TCP (`udp_bridge_sdk_node.py` + `map_receiver_node.cpp`). This pipeline contributed several sim2real gaps:

- Heavy ANYbotics filter chain (Kalman update, inpaint, min-filter, smooth) — semantically very different from the training side, which uses a clean `RayCaster` projection.
- TCP/network jitter and unbounded latency.
- Resolution / FOV mismatch: deploy yaml used 80×80 at 5cm; training RayCaster was 17×11 at 10cm.

This redesign attacks the sim2real gap by **making the deploy-side source as close to the training projection as possible**, on the robot directly, with no laptop in the loop.

## 3. Out of scope

- Moving the LiDAR raw stream from the laptop bridge directly into RK3588 (orthogonal infra change).
- Training-side augmentation re-tuning (the existing `height_scan_sim2real` augmentation is kept as the starting point; tuning is deferred until empirical sim2real gap is measured with the new deploy node).
- Replacing the existing `multi_layer_scan` (forward/backward hemispherical LiDAR) obs. The height_scan is **additive**.
- Onboard SLAM / odometry.

## 4. Architecture

### 4.1 Topology

```
/IMU_DATA  (drdds::msg::ImuData) ─────────────┐  (latest roll, pitch)
                                              ▼
/LIDAR_POINT_CLOUD_MERGED ─────►  [height_scan_node (C++)]
   (PointCloud2, frame=base_link,                    │
    body-tilted, contains RP)                        ├─ R_b2h = RpyToRm({r,p,0})ᵀ  (yaw stripped)
                                                     ├─ p_h = R_b2h · p_b
                                                     ├─ bin (x,y) into 17×11 grid
                                                     ├─ per-cell max(z_h) (≈ nearest hit above terrain)
                                                     ├─ obs_i = clip(-z_h - 0.5, -1.0, +1.0)
                                                     ├─ per-cell ring buffer fills missing cells
                                                     └─ publish /scan/height_features_array
                                                        (std_msgs/Float32MultiArray, 187 floats)
                                                                 │
                                                                 ▼
                                                         [rl_deploy]
                                                         (concatenates into policy obs)
```

### 4.2 Equivalence with training-side `mdp.height_scan`

Training observation per cell:

```
obs_train = base_z_world - hit_z_world - offset
```

Because RayCaster uses `ray_alignment="yaw"`, the grid is yaw-aligned (rotates with robot yaw but stays horizontal). `base_z_world` is the world-frame height of the base; `hit_z_world` is the world-frame height of the nearest ray hit.

Deploy observation per cell. To match `ray_alignment="yaw"`, we strip the body's roll and pitch by rotating the pointcloud into a "horizon" frame that shares the body's yaw but is gravity-aligned.

Construction (reusing the existing helper `RpyToRm(rpy)` from the deploy codebase, see `m20_policy_runner.hpp:414`):

```
R_h2b = RpyToRm({roll, pitch, 0})    # body relative to horizon (yaw zeroed out)
R_b2h = R_h2bᵀ                        # horizon relative to body
p_h   = R_b2h · p_b                   # point in horizon-aligned base frame
z_h   = p_h[2]                         # vertical component, relative to base origin
obs_deploy = clip(-z_h - offset, -clip_range, +clip_range)
```

These are equivalent to the training observation because `base_z_world - hit_z_world` equals the negative vertical distance from base to hit in any horizon-aligned (gravity-perpendicular) frame anchored at the base — which is exactly `-z_h`. Using the existing `RpyToRm` helper guarantees the rotation convention matches everything else in the deploy code (e.g., `projected_gravity` computation).

### 4.3 Why not use tf2

Investigation showed:

- `udp_bridge_sdk_node.py:91-97` does publish tf2 (odom → base_link), but only when the desktop-side sim2sim bridge is running.
- The robot's native SDK does not publish tf2.
- The existing `m20_policy_runner.hpp` already extracts attitude via `ri_ptr_->GetImuRpy()` from `drdds::msg::ImuData` — this is the canonical onboard attitude source.

`height_scan_node` therefore subscribes to `/IMU_DATA` directly. No tf2 dependency on the runtime path.

## 5. Components

### 5.1 New: `height_scan_node` (C++, ROS 2)

**Location:** `src/M20_sdk_deploy/src/height_scan_node.cpp`
**Class:** `HeightScanNode : public rclcpp::Node`
**CMake target:** new `add_executable(height_scan_node ...)` entry in `CMakeLists.txt`
**Dependencies:** `rclcpp`, `sensor_msgs`, `std_msgs`, `drdds`

#### Subscribers

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/LIDAR_POINT_CLOUD_MERGED` | `sensor_msgs::msg::PointCloud2` | `rclcpp::SensorDataQoS()` | Source pointcloud, frame=base_link |
| `/IMU_DATA` | `drdds::msg::ImuData` | `rclcpp::SensorDataQoS()` | Roll/pitch for horizon alignment |

#### Publishers

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/scan/height_features_array` | `std_msgs::msg::Float32MultiArray` | `KeepLast(10)` reliable | 187 floats, row-major |

#### Internal state

- `std::atomic<float> latest_roll_, latest_pitch_;` — written by IMU callback, read by pointcloud callback.
- `std::array<std::vector<float>, BUFFER_LEN> grid_history_;` — ring buffer of past per-cell observations.
- `int head_;` — ring buffer write index.
- Diagnostics counters (recv_count, pub_count, total_time_ms_).

#### Pointcloud callback (per frame, ~20 Hz)

1. Snapshot `latest_roll_`, `latest_pitch_`. If IMU never received, log warn and return.
2. Build `R_b2h = RpyToRm({roll, pitch, 0.0})ᵀ` using the existing helper from the deploy codebase (3×3 matrix, computed once per frame). Yaw is intentionally zeroed so the resulting frame is horizon-aligned but shares the body's yaw.
3. Allocate `std::vector<float> cells(187, -INF)` to track per-cell max z_h.
4. Iterate over PointCloud2 fields x/y/z via `PointCloud2ConstIterator`:
   - Skip non-finite, skip points with `r > MAX_DIST` (= 1.2 m to cover 1.6 × 1.0 diagonal margin).
   - `p_h = R_b2h * p_b`
   - Compute `ix = round((p_h.x + 0.8) / 0.1)`, `iy = round((p_h.y + 0.5) / 0.1)`.
   - Skip if `ix < 0 || ix >= 17 || iy < 0 || iy >= 11`.
   - `idx = ix * 11 + iy`; update `cells[idx] = max(cells[idx], p_h.z)`.
5. Convert to observation:
   - For each cell `i`:
     - If `cells[i] != -INF`: `obs[i] = clip(-cells[i] - 0.5, -1.0, +1.0)`.
     - Else (missing): look up most recent valid value in the ring buffer for this cell. If none, fall back to `clip_max = +1.0` (= "very far below base", safe for policy because it implies "no obstacle / pit").
6. Push current observation onto ring buffer; advance `head_`.
7. Publish `Float32MultiArray`.
8. Record process time in diagnostics.

#### IMU callback

Cheap: extract roll/pitch (already in degrees per `dds_interface.hpp:266`), convert to radians, store atomically. No further processing.

#### Diagnostics timer (1 Hz)

Log received Hz, published Hz, average process time, and the fraction of cells served from ring buffer in the last second. Crucial for debugging.

### 5.2 Build integration

Append to `CMakeLists.txt` (after existing `lidar_relay` block):

```cmake
add_executable(height_scan_node src/height_scan_node.cpp)
ament_target_dependencies(height_scan_node rclcpp sensor_msgs std_msgs drdds)
install(TARGETS height_scan_node DESTINATION lib/${PROJECT_NAME})
```

Append to `launch/m20_deploy.launch.py`:

```python
Node(
    package='m20_sdk_deploy',
    executable='height_scan_node',
    name='height_scan_node',
    output='screen',
    parameters=[{
        'grid_size_x': 1.6, 'grid_size_y': 1.0,
        'resolution': 0.1,
        'obs_offset': 0.5, 'clip_range': 1.0,
        'update_rate_hz': 20.0,
        'buffer_len': 10,
        'pointcloud_topic': '/LIDAR_POINT_CLOUD_MERGED',
        'imu_topic': '/IMU_DATA',
        'output_topic': '/scan/height_features_array',
    }],
),
```

### 5.3 Training-side changes (rl_training repo)

These changes are independent of the deploy node and can be developed in parallel.

1. `source/rl_training/rl_training/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/moe_teacher_env_cfg.py`:
   - Lines 467, 477-478, 528, 560: revert `height_scan = None` to:
     ```python
     height_scan = ObsTerm(
         func=mdp.height_scan_sim2real,
         params={
             "sensor_cfg": SceneEntityCfg("height_scanner"),
             "offset": 0.5,
             "mask_prob": 0.15,       # keep current default; tune later if real is cleaner
             "min_latency": 1,
             "max_latency": 3,
             "smooth_kernel_size": 3,
             "max_drift_pixels": 2,
             "grid_length": 17,
             "min_noise_amp": 0.1,
         },
         noise=Unoise(n_min=-0.05, n_max=0.05),
         clip=(-1.0, 1.0),
     )
     ```
     These are the function's defaults. The intent for v1 is to **not change augmentation** — we are betting on deploy-side improvements to close the gap. If empirical sim2real eval (after v1 lands) shows the deploy node is much cleaner than the augmentation assumes, lower `mask_prob` / `max_latency` / `max_drift_pixels` in a follow-up.
   - Lines 731-732: restore `self.scene.height_scanner` (re-enable the RayCaster sensor).
   - Lines 763, 1054: remove the `= None` overrides in pretraincfg / noisy_elevation.

2. `scripts/reinforcement_learning/rsl_rl/play_moe.py`:
   - Add `height_scan` to ONNX export input names and shapes (shape `(1, 187)` after squeeze).
   - Verify policy `forward()` accepts the new input.

3. `run_policy/m20_policy_runner.hpp`:
   - Subscribe to `/scan/height_features_array`.
   - Concatenate 187 floats into the observation tensor at the position determined by the (newly retrained) policy.
   - Mirror the validation pattern used by `forward_scan` / `backward_scan` consumers.

## 6. Configuration parameters

All exposed as ROS 2 node parameters with defaults:

| Param | Default | Range | Notes |
|---|---|---|---|
| `grid_size_x` (m) | 1.6 | (0.5, 5.0) | Total forward+back span |
| `grid_size_y` (m) | 1.0 | (0.5, 3.0) | Total lateral span |
| `resolution` (m) | 0.1 | (0.02, 0.5) | Cell size |
| `obs_offset` (m) | 0.5 | — | Subtracted before clip; matches training |
| `clip_range` (m) | 1.0 | (0.1, 3.0) | Clamp to ±clip_range |
| `update_rate_hz` | 20.0 | (5, 100) | Output rate; pointcloud cb gated by this |
| `buffer_len` | 10 | (1, 50) | Ring buffer depth |
| `max_point_dist` (m) | 1.2 | — | Early reject points outside grid disk |
| `min_imu_age_s` | 0.1 | — | Stale-IMU guard; warn + zero output |
| `pointcloud_topic` | `/LIDAR_POINT_CLOUD_MERGED` | — | |
| `imu_topic` | `/IMU_DATA` | — | |
| `output_topic` | `/scan/height_features_array` | — | |

## 7. Error handling and robustness

1. **No IMU received** → emit warn at 1 Hz, do not publish output. Policy runner side: treat absence of `/scan/height_features_array` for > 200 ms as fault; zero-fill obs and log.
2. **Stale IMU** (age > `min_imu_age_s`) → same behavior as above.
3. **Empty / degenerate pointcloud** → still publish the observation, all cells served from ring buffer; if ring buffer also empty (first few frames), fill with `clip_range` (= "no obstacle").
4. **Malformed pointcloud (missing x/y/z fields)** → error log, skip frame, do not crash.
5. **Update rate overrun** (process time > 1 / update_rate_hz) → log warn, do not back-pressure (best-effort).
6. **All cells empty for > 1 s** → log error (sensor likely failed); still publish ring-buffer-filled output so policy keeps a defined obs.

The node must never abort under runtime conditions. Diagnostics counters expose health to the operator.

## 8. Testing strategy

### 8.1 Unit-level (offline, on dev machine)

1. **Geometry test**: synthesise a known pointcloud (e.g., a 0.2 m tall box at +0.5 m forward), set IMU to zero, run one callback, assert the corresponding cells have the correct obs value within 1 cm.
2. **Attitude correction test**: rotate the same pointcloud by 10° pitch (simulating the body tilting forward), set IMU roll/pitch accordingly, assert the output cells are within 1 cm of the zero-tilt case.
3. **Index mapping test**: corner cells (ix=0/16, iy=0/10) populated with marker points; assert flat indices land at the expected positions (0, 10, 176, 186).

### 8.2 Integration (offline bag replay)

1. Record `/LIDAR_POINT_CLOUD_MERGED` + `/IMU_DATA` from a real run on stair / ramp / flat terrain.
2. Replay through `height_scan_node`; visualise the 187-dim output as a heatmap; sanity-check stair edge alignment with known geometry.
3. Compare against a Python reference implementation (slower but easier to inspect) — element-wise max error < 1 cm.

### 8.3 Sim2sim alignment (training side)

In Isaac Sim, with a single env on a known terrain:
1. Get the RayCaster output via `mdp.height_scan` (no augmentation).
2. Capture the same env's mock pointcloud (raw raycast hits) and route it through a Python port of `height_scan_node`.
3. Cell-by-cell diff: max error should be < 2 cm; any larger means the projection / binning / offset is wrong.

### 8.4 On-robot smoke test (after deploy)

1. Power on, stand still on flat ground. Verify all 187 cells output ≈ `clip(-0 - 0.5, ...) = -0.5` (within noise).
2. Place a 10 cm box at +0.5 m forward. Verify forward cells in that location report ≈ `-0.5 - (-0.10) = -0.40` after clipping.
3. Pitch robot ±10° (manually or on tilted surface). Verify obs remains stable (attitude correction working).

## 9. Performance budget

Target on RK3588 single core:

- Pointcloud callback: < 5 ms per frame
- IMU callback: < 0.05 ms
- 20 Hz total CPU: < 10% of one core

Validation: built-in diagnostics print `avg_process_time_ms` once per second.

## 10. Risks and open questions

| Risk / Question | Mitigation |
|---|---|
| Sim2real gap still too large after this redesign | Empirical: collect data, compare to `mdp.height_scan_sim2real` distribution; tune training-side augmentation if needed (deferred to a follow-up). |
| IMU rate slower than expected | Spec defines `min_imu_age_s = 0.1` warn; if SDK IMU is slow, consider extrapolating with last omega. Out of scope for v1. |
| Pointcloud frame is *not* `base_link` after some upstream change | Spec requires `frame_id = "base_link"`. The node logs warn if `msg->header.frame_id != "base_link"` and proceeds anyway (fail-open). |
| `RpyToRm` convention mismatch | Use the same convention as `m20_policy_runner.hpp:414` (`base_rot_mat.inverse() * gravity`). Borrow that exact helper to avoid drift. |

## 11. References

- `src/M20_sdk_deploy/src/lidar_to_scan.cpp` — code-pattern template
- `src/M20_sdk_deploy/interface/robot/hardware/dds_interface.hpp:130,266` — IMU subscriber pattern
- `source/rl_training/rl_training/tasks/manager_based/locomotion/velocity/velocity_env_cfg.py:75-90` — RayCaster training spec
- `source/rl_training/rl_training/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/moe_teacher_env_cfg.py:188-330` — `height_scan_sim2real` augmentation (kept on training side)
- Fankhauser & Hutter, "Universal Grid Map Library", 2016 — for reference, *not* used here (filter chain causes sim2real divergence)
