#!/usr/bin/env python3
"""新 unified policy 感知节点: 发布完整 691 维 noisy_elevation。

布局 (与 ckpt 2026-05-18 / model_27400 的 exported/sim2real_layout 严格对齐):
    [ height_scan(187) | forward_scan(252) | backward_scan(252) ]  = 691
顺序写死, runner (M20SensorPolicyRunner, use_full_perception_) 检测到 onnx
proprio_and_env=748 时订阅 /perception/noisy_elevation_array 并原样拷入 env 块。

forward/backward_scan 几何 = 训练 multi_pitch_arc_pattern + multi_layer_scan:
    boresight=+X, ray=(cosP·cosA, cosP·sinA, sinP)
    pitch  = linspace(-25, 25, 12) 度 (仰角, 12 档); azimuth = linspace(-45, 45, 21) 度
    flatten: pitch-major / azimuth-minor, k = pitch_idx*21 + az_idx
    归一化: clip(depth/2.5, 0, 1); depth<0.3 → 1.0 (盲区=no-hit)

height_scan(187): 17×11 yaw 网格 @0.1m (x∈[-0.8,0.8], y∈[-0.5,0.5]),
    flatten x-minor/y-major (idx = y_idx*17 + x_idx, 对齐 onnx reshape[11,17])。
    训练定义是 clamp(base_z - terrain_z - 0.5, -1, 1)。
    sim /height_map 发布 terrain_z_base = terrain_world_z - base_z, 因此旧模式使用 -terrain_z_base - 0.5。
    M20 官方 /height_map 已经过 height_map_nav 的 robot_height/base_gravity_frame 处理, 不再额外减 0.5;
    真机官方模式用中心地面高度作零点, 保持高台/障碍为负、坑/下台阶为正。

话题 (参数化):
    lidar_topic   默认 /LIDAR_SIM_RAW   (sim mujoco; 真机改 /CLOUD_REGISTERED_BODY ~10Hz, base 系点云)
    height_topic  默认 /height_map      (真机 height_map_nav, 需 /HEIGHT_MAP_ENABLE 开; sim 发同名)
    /LIO_ODOM     仅订阅打印 z 供校验 (height_scan 不依赖它)
输出: /perception/noisy_elevation_array  (Float32MultiArray, 691)
"""
import math
import threading
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

try:
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None

try:
    from nav_msgs.msg import Odometry
    _HAS_ODOM = True
except Exception:
    _HAS_ODOM = False

try:
    from drdds.msg import JointsData
    _HAS_JOINTS = True
except Exception:
    _HAS_JOINTS = False

# ---- forward/backward scan 几何 (multi_pitch_arc) ----
NUM_PITCH = 12
NUM_AZ = 21
NUM_PER_DIR = NUM_PITCH * NUM_AZ          # 252
PITCH_MIN_DEG, PITCH_MAX_DEG = -25.0, 25.0
AZ_MIN_DEG, AZ_MAX_DEG = -45.0, 45.0
PITCH_STEP_DEG = (PITCH_MAX_DEG - PITCH_MIN_DEG) / (NUM_PITCH - 1)   # 50/11
AZ_STEP_DEG = (AZ_MAX_DEG - AZ_MIN_DEG) / (NUM_AZ - 1)              # 90/20
MAX_DIST = 2.5
MIN_DIST = 0.3

# ---- height_scan 几何 (grid_pattern res=0.1 size=[1.6,1.0], ordering 'xy') ----
H_NX = 17            # x: -0.8..0.8 (length 1.6)
H_NY = 11            # y: -0.5..0.5 (width 1.0)
HEIGHT_DIM = H_NX * H_NY   # 187
HEIGHT_OFFSET = 0.5
# 187 个目标点 (base 系), flatten x-minor/y-major: idx = y_idx*17 + x_idx
_HX = (-0.8 + 0.1 * np.arange(H_NX)).astype(np.float32)   # 17
_HY = (-0.5 + 0.1 * np.arange(H_NY)).astype(np.float32)   # 11
_TX = np.tile(_HX, H_NY)                                  # 187, x 变化最快
_TY = np.repeat(_HY, H_NX)                                # 187

NOISY_ELEV_DIM = HEIGHT_DIM + NUM_PER_DIR * 2   # 187 + 252 + 252 = 691

FWD_OFFSET = np.array([0.32028, 0.0, -0.013], dtype=np.float32)
BWD_OFFSET = np.array([-0.32028, 0.0, -0.013], dtype=np.float32)

# M20Interface raw DDS joint -> control joint conversion.
_POS_OFFSET_DEG = np.array(
    [-25, -131, 160, 0, 25, -131, 160, 0, -25, 131, -160, 0, 25, 131, -160, 0],
    dtype=np.float32,
)
_JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
_POS_OFFSET = np.deg2rad(_POS_OFFSET_DEG).astype(np.float32)
_BASE_X = np.array([0.31, 0.31, -0.31, -0.31], dtype=np.float32)
_BASE_Y = np.array([0.17, -0.17, 0.17, -0.17], dtype=np.float32)
_HIPY_IDX = [1, 5, 9, 13]
_KNEE_IDX = [2, 6, 10, 14]


def _bin_one_direction(pts, boresight_sign):
    """点云 -> 252 维归一化扫描 (新 multi_pitch_arc 约定)。

    boresight_sign = +1 前向(+X); -1 后向(绕 Z 180° 后 boresight=+X)。
    返回长度 252 的归一化深度 (已含 d/2.5 + 盲区<0.3->1.0)。
    """
    norm = np.ones(NUM_PER_DIR, dtype=np.float32)   # 默认无 hit -> 归一化 1.0
    if pts.size == 0:
        return norm

    rel = pts - (FWD_OFFSET if boresight_sign > 0 else BWD_OFFSET)
    if boresight_sign > 0:
        xs, ys, zs = rel[:, 0], rel[:, 1], rel[:, 2]
    else:
        xs, ys, zs = -rel[:, 0], -rel[:, 1], rel[:, 2]   # 180° about +Z

    m = xs > 1e-6                                          # 前半球
    if not np.any(m):
        return norm
    xs, ys, zs = xs[m], ys[m], zs[m]

    r = np.sqrt(xs * xs + ys * ys + zs * zs)
    valid = r > 1e-6
    if not np.any(valid):
        return norm
    xs, ys, zs, r = xs[valid], ys[valid], zs[valid], r[valid]

    pitch_deg = np.degrees(np.arcsin(np.clip(zs / r, -1.0, 1.0)))   # 仰角
    az_deg = np.degrees(np.arctan2(ys, xs))                         # 方位 (绕 +X)

    in_fov = (np.abs(pitch_deg) <= PITCH_MAX_DEG + 1e-3) & (np.abs(az_deg) <= AZ_MAX_DEG + 1e-3)
    if not np.any(in_fov):
        return norm
    pitch_deg, az_deg, r = pitch_deg[in_fov], az_deg[in_fov], r[in_fov]

    p_idx = np.clip(np.rint((pitch_deg - PITCH_MIN_DEG) / PITCH_STEP_DEG).astype(np.int32), 0, NUM_PITCH - 1)
    a_idx = np.clip(np.rint((az_deg - AZ_MIN_DEG) / AZ_STEP_DEG).astype(np.int32), 0, NUM_AZ - 1)
    flat = p_idx * NUM_AZ + a_idx   # pitch-major / azimuth-minor

    depth = np.full(NUM_PER_DIR, MAX_DIST, dtype=np.float32)
    np.minimum.at(depth, flat, r.astype(np.float32))

    norm = np.clip(depth / MAX_DIST, 0.0, 1.0).astype(np.float32)   # multi_layer_scan 归一化
    norm[depth < MIN_DIST] = 1.0                                    # 盲区 = no-hit
    return norm


def _cloud_to_xyz(msg):
    if point_cloud2 is not None:
        raw = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
        if not raw:
            return np.empty((0, 3), dtype=np.float32)
        a = np.array(raw)
        if a.dtype.names is not None:
            return np.column_stack((a['x'], a['y'], a['z'])).astype(np.float32)
        return a.astype(np.float32)

    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != 7 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)

    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    width = int(msg.width)
    height = int(msg.height)
    if point_step <= 0 or width <= 0 or height <= 0:
        return np.empty((0, 3), dtype=np.float32)

    dtype = ">f4" if msg.is_bigendian else "<f4"
    data = memoryview(msg.data)
    count = width * height

    def read_field(name):
        offset = int(fields[name].offset)
        if row_step == point_step * width:
            return np.ndarray(
                shape=(count,),
                dtype=dtype,
                buffer=data,
                offset=offset,
                strides=(point_step,),
            ).astype(np.float32, copy=True)
        out = np.empty(count, dtype=np.float32)
        dst = 0
        for row in range(height):
            row_offset = row * row_step + offset
            vals = np.ndarray(
                shape=(width,),
                dtype=dtype,
                buffer=data,
                offset=row_offset,
                strides=(point_step,),
            )
            out[dst:dst + width] = vals
            dst += width
        return out

    return np.column_stack((read_field("x"), read_field("y"), read_field("z"))).astype(np.float32, copy=False)


def _quat_to_rot(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
    ], dtype=np.float32)


def _quat_to_yaw(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NoisyElevationNode(Node):
    def __init__(self):
        super().__init__('noisy_elevation_node')
        self.declare_parameter('lidar_topic', '/LIDAR_SIM_RAW')
        # 真机: /height_map (height_map_nav, 81×81@0.1m base_link, 需 /HEIGHT_MAP_ENABLE command:1 开启)
        # sim:  发同名 /height_map 即可 (Phase4)。注意 /HEIGHT_POINTS 是死占位, 勿用
        self.declare_parameter('height_topic', '/height_map')
        self.declare_parameter('zero_height_scan', False)
        self.declare_parameter('fk_height_scan', False)
        self.declare_parameter('terrain_cache_scan', False)
        self.declare_parameter('height_map_mode', 'terrain_z_base')
        self.declare_parameter('height_invalid_mode', 'zero')
        self.declare_parameter('height_center_kernel', 5)
        self.declare_parameter('cache_resolution', 0.1)
        self.declare_parameter('cache_ttl_sec', 8.0)
        self.declare_parameter('cache_radius_m', 3.0)
        self.declare_parameter('cache_min_points', 3)
        lt = self.get_parameter('lidar_topic').value
        ht = self.get_parameter('height_topic').value
        self.zero_height_scan = bool(self.get_parameter('zero_height_scan').value)
        self.fk_height_scan = bool(self.get_parameter('fk_height_scan').value)
        self.terrain_cache_scan = bool(self.get_parameter('terrain_cache_scan').value)
        self.height_map_mode = str(self.get_parameter('height_map_mode').value).strip().lower()
        self.height_invalid_mode = str(self.get_parameter('height_invalid_mode').value).strip().lower()
        self.height_center_kernel = int(self.get_parameter('height_center_kernel').value)
        if self.height_map_mode not in ('terrain_z_base', 'official_relative', 'official_centered'):
            self.get_logger().warn(f"unknown height_map_mode={self.height_map_mode}, using terrain_z_base")
            self.height_map_mode = 'terrain_z_base'
        if self.height_invalid_mode not in ('zero', 'previous'):
            self.get_logger().warn(f"unknown height_invalid_mode={self.height_invalid_mode}, using zero")
            self.height_invalid_mode = 'zero'
        if self.height_center_kernel < 1:
            self.height_center_kernel = 1
        if self.height_center_kernel % 2 == 0:
            self.height_center_kernel += 1
        self.cache_res = float(self.get_parameter('cache_resolution').value)
        self.cache_ttl = float(self.get_parameter('cache_ttl_sec').value)
        self.cache_radius = float(self.get_parameter('cache_radius_m').value)
        self.cache_min_points = int(self.get_parameter('cache_min_points').value)

        self.sub = self.create_subscription(PointCloud2, lt, self.pc_callback, 10)
        self.height_sub = self.create_subscription(PointCloud2, ht, self.height_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/perception/noisy_elevation_array', 10)

        self.height_lock = threading.Lock()
        self.height_xyz = None          # (N,3) base 系高程网格
        self.height_recv = 0
        self.joint_lock = threading.Lock()
        self.last_joint_q = None
        self.joint_recv = 0
        self.imu_rot = np.eye(3, dtype=np.float32)
        self.imu_recv = 0
        self.pose_lock = threading.Lock()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_pose_recv = 0
        self.cache_lock = threading.Lock()
        self.terrain_cache = {}  # (ix, iy) -> [height_world, confidence, timestamp]
        self.cache_update_cells = 0
        self.cache_hits = 0
        self.cache_misses = 0
        self.last_base_height = None
        self.last_height_scan = None
        self.height_invalid_count = 0
        self.height_invalid_total = 0

        # /LIO_ODOM 仅用于记录/校验 z (height_scan 不依赖绝对 z)
        self.last_odom_z = None
        self.odom_z_min = None
        self.odom_z_max = None
        if _HAS_ODOM:
            self.odom_sub = self.create_subscription(Odometry, '/LIO_ODOM', self.odom_callback, 10)
        if _HAS_JOINTS:
            self.joint_sub = self.create_subscription(JointsData, '/JOINTS_DATA', self.joint_callback, 10)
            self.imu_sub = self.create_subscription(Imu, '/IMU', self.imu_callback, 10)

        self.recv_count = 0
        self.pub_count = 0
        self.process_times = []
        self.create_timer(1.0, self.log_diagnostics)
        self.get_logger().info(
            f"✅ noisy_elevation: lidar={lt} height={ht} | "
            f"height({HEIGHT_DIM}) + fwd({NUM_PER_DIR}) + bwd({NUM_PER_DIR}) = {NOISY_ELEV_DIM} | "
            f"zero_height_scan={self.zero_height_scan} fk_height_scan={self.fk_height_scan} "
            f"terrain_cache_scan={self.terrain_cache_scan} height_map_mode={self.height_map_mode} "
            f"height_invalid_mode={self.height_invalid_mode}")

    def odom_callback(self, msg):
        z = float(msg.pose.pose.position.z)
        self.last_odom_z = z
        self.odom_z_min = z if self.odom_z_min is None else min(self.odom_z_min, z)
        self.odom_z_max = z if self.odom_z_max is None else max(self.odom_z_max, z)
        with self.pose_lock:
            self.odom_x = float(msg.pose.pose.position.x)
            self.odom_y = float(msg.pose.pose.position.y)
            self.odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)
            self.odom_pose_recv += 1

    def height_callback(self, msg):
        a = _cloud_to_xyz(msg)
        with self.height_lock:
            self.height_xyz = a
            self.height_recv += 1

    def imu_callback(self, msg):
        self.imu_rot = _quat_to_rot(msg.orientation)
        self.imu_recv += 1

    def joint_callback(self, msg):
        raw = np.array([j.position for j in msg.data.joints_data], dtype=np.float32)
        if raw.size != 16:
            return
        q = raw * _JOINT_DIR + _POS_OFFSET
        with self.joint_lock:
            self.last_joint_q = q
            self.joint_recv += 1

    def compute_fk_base_height(self):
        with self.joint_lock:
            q = None if self.last_joint_q is None else self.last_joint_q.copy()
        if q is None:
            return None

        heights = []
        R = self.imu_rot
        for i in range(4):
            q_hipy = float(q[_HIPY_IDX[i]])
            q_knee = float(q[_KNEE_IDX[i]])
            z_drop = 0.25 * math.cos(q_hipy) + 0.25 * math.cos(q_hipy + q_knee) + 0.086
            x_shift = 0.25 * math.sin(q_hipy) + 0.25 * math.sin(q_hipy + q_knee)
            foot_local = np.array([_BASE_X[i] + x_shift, _BASE_Y[i], -z_drop], dtype=np.float32)
            foot_world = R @ foot_local
            heights.append(-float(foot_world[2]))
        base_height = float(np.mean(heights))
        self.last_base_height = base_height
        return base_height

    def compute_fk_height_scan(self):
        base_height = self.compute_fk_base_height()
        if base_height is None:
            return np.zeros(HEIGHT_DIM, dtype=np.float32)
        height_value = np.clip(base_height - HEIGHT_OFFSET, -1.0, 1.0)
        return np.full(HEIGHT_DIM, height_value, dtype=np.float32)

    def _pose2d(self):
        with self.pose_lock:
            return self.odom_x, self.odom_y, self.odom_yaw

    def update_terrain_cache(self, pts, base_height):
        if pts.size == 0 or base_height is None:
            return

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        # Keep terrain-sized returns in a local region. This rejects many walls/high obstacles
        # while keeping steps, platforms and ramps that are observable as horizontal patches.
        local = (
            finite
            & (x > -1.2) & (x < 2.5)
            & (np.abs(y) < 1.6)
            & (z > -base_height - 0.35)
            & (z < 0.45)
        )
        if not np.any(local):
            return

        x, y, z = x[local], y[local], z[local]
        odom_x, odom_y, yaw = self._pose2d()
        c, s = math.cos(yaw), math.sin(yaw)
        wx = odom_x + c * x - s * y
        wy = odom_y + s * x + c * y
        wh = z + base_height

        buckets = defaultdict(list)
        inv_res = 1.0 / max(self.cache_res, 1e-6)
        for xi, yi, hi in zip(wx, wy, wh):
            if not math.isfinite(float(hi)) or hi < -0.35 or hi > 0.75:
                continue
            key = (int(round(float(xi) * inv_res)), int(round(float(yi) * inv_res)))
            buckets[key].append(float(hi))

        candidates = {}
        for key, vals in buckets.items():
            if len(vals) < self.cache_min_points:
                continue
            vals.sort()
            # Low percentile favors walkable lower surfaces over vertical clutter in the same cell.
            h = vals[int(0.2 * (len(vals) - 1))]
            spread = vals[-1] - vals[0]
            if spread > 0.45:
                continue
            candidates[key] = h

        accepted = {}
        for key, h in candidates.items():
            if h <= 0.07:
                accepted[key] = h
                continue
            ix, iy = key
            x_support = False
            y_support = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nh = candidates.get((ix + dx, iy + dy))
                    if nh is None or abs(nh - h) > 0.12:
                        continue
                    if dx != 0:
                        x_support = True
                    if dy != 0:
                        y_support = True
            if x_support and y_support:
                accepted[key] = h

        now = time.time()
        with self.cache_lock:
            for key, h in accepted.items():
                old = self.terrain_cache.get(key)
                if old is None:
                    self.terrain_cache[key] = [h, 1.0, now]
                else:
                    old_h, conf, _ = old
                    # Faster correction for real height changes, slower smoothing for small jitter.
                    alpha = 0.65 if abs(h - old_h) < 0.08 else 0.25
                    self.terrain_cache[key] = [alpha * old_h + (1.0 - alpha) * h, min(conf + 1.0, 10.0), now]

            max_cell_dist = int(math.ceil(self.cache_radius * inv_res))
            cx = int(round(odom_x * inv_res))
            cy = int(round(odom_y * inv_res))
            stale = [
                key for key, (_, _, ts) in self.terrain_cache.items()
                if now - ts > self.cache_ttl
                or abs(key[0] - cx) > max_cell_dist
                or abs(key[1] - cy) > max_cell_dist
            ]
            for key in stale:
                self.terrain_cache.pop(key, None)
            self.cache_update_cells = len(accepted)

    def compute_cached_height_scan(self, pts):
        base_height = self.compute_fk_base_height()
        if base_height is None:
            return np.zeros(HEIGHT_DIM, dtype=np.float32)
        self.update_terrain_cache(pts, base_height)

        odom_x, odom_y, yaw = self._pose2d()
        c, s = math.cos(yaw), math.sin(yaw)
        now = time.time()
        inv_res = 1.0 / max(self.cache_res, 1e-6)
        out = np.empty(HEIGHT_DIM, dtype=np.float32)
        hits = 0
        misses = 0

        with self.cache_lock:
            cache = dict(self.terrain_cache)

        for i, (qx, qy) in enumerate(zip(_TX, _TY)):
            wx = odom_x + c * float(qx) - s * float(qy)
            wy = odom_y + s * float(qx) + c * float(qy)
            ix = int(round(wx * inv_res))
            iy = int(round(wy * inv_res))
            best_h = None
            best_score = 1e9
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    entry = cache.get((ix + dx, iy + dy))
                    if entry is None:
                        continue
                    h, conf, ts = entry
                    age = now - ts
                    if age > self.cache_ttl:
                        continue
                    score = dx * dx + dy * dy + 0.05 * age - 0.01 * conf
                    if score < best_score:
                        best_score = score
                        best_h = h
            if best_h is None:
                # Unknown/occluded cells fall back to the FK flat-ground baseline.
                best_h = 0.0
                misses += 1
            else:
                hits += 1
            out[i] = np.clip(base_height - best_h - HEIGHT_OFFSET, -1.0, 1.0)

        self.cache_hits = hits
        self.cache_misses = misses
        return out

    def _official_center_ref(self, terrain_grid):
        k = min(self.height_center_kernel, H_NX, H_NY)
        if k % 2 == 0:
            k -= 1
        if k < 1:
            k = 1
        cy, cx = H_NY // 2, H_NX // 2
        half = k // 2
        patch = terrain_grid[cy - half: cy + half + 1, cx - half: cx + half + 1]
        finite_patch = patch[np.isfinite(patch)]
        if finite_patch.size:
            return float(np.median(finite_patch))
        finite_all = terrain_grid[np.isfinite(terrain_grid)]
        return float(np.median(finite_all)) if finite_all.size else 0.0

    def compute_height_scan(self):
        """Convert /height_map samples to the 187-dim training height_scan convention."""
        with self.height_lock:
            g = self.height_xyz
        if g is None or len(g) == 0:
            self.height_invalid_count = HEIGHT_DIM
            self.height_invalid_total = HEIGHT_DIM
            if self.height_invalid_mode == 'previous' and self.last_height_scan is not None:
                return self.last_height_scan.copy()
            return np.zeros(HEIGHT_DIM, dtype=np.float32)        # 网格未到: 安全默认(平地名义)
        xs, ys, zs = g[:, 0], g[:, 1], g[:, 2]
        finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        if not np.any(finite):
            self.height_invalid_count = HEIGHT_DIM
            self.height_invalid_total = HEIGHT_DIM
            if self.height_invalid_mode == 'previous' and self.last_height_scan is not None:
                return self.last_height_scan.copy()
            return np.zeros(HEIGHT_DIM, dtype=np.float32)
        xs, ys, zs = xs[finite], ys[finite], zs[finite]

        # 规则网格: 估分辨率, 建二维索引, 对 187 目标做最近格采样 (向量化)
        ux = np.unique(xs)
        res = float(np.median(np.diff(ux))) if ux.size > 1 else 0.1
        if res <= 1e-6:
            res = 0.1
        x0, y0 = float(xs.min()), float(ys.min())
        gx = np.round((xs - x0) / res).astype(np.int32)
        gy = np.round((ys - y0) / res).astype(np.int32)
        W = int(gx.max()) + 1
        H = int(gy.max()) + 1
        Zmap = np.full(W * H, np.nan, dtype=np.float32)
        Zmap[gy * W + gx] = zs

        tgx = np.clip(np.round((_TX - x0) / res).astype(np.int32), 0, W - 1)
        tgy = np.clip(np.round((_TY - y0) / res).astype(np.int32), 0, H - 1)
        terrain = Zmap[tgy * W + tgx]                  # 187 samples in source height-map semantics
        if self.height_map_mode == 'terrain_z_base':
            hs = -terrain - HEIGHT_OFFSET              # MuJoCo sim: base_z - terrain_z - 0.5
        elif self.height_map_mode == 'official_relative':
            hs = -terrain                              # Official M20 map: no extra 0.5 offset
        else:
            terrain_grid = terrain.reshape(H_NY, H_NX)
            ref = self._official_center_ref(terrain_grid)
            hs = -(terrain - ref)                      # local support plane -> 0
        invalid = ~np.isfinite(hs)
        self.height_invalid_count = int(np.count_nonzero(invalid))
        self.height_invalid_total = int(hs.size)
        if np.any(invalid):
            if self.height_invalid_mode == 'previous' and self.last_height_scan is not None:
                hs = hs.copy()
                hs[invalid] = self.last_height_scan[invalid]
            else:
                hs = np.where(np.isfinite(hs), hs, 0.0)  # initial/no-cache fallback: nominal flat
        out = np.clip(hs, -1.0, 1.0).astype(np.float32)
        self.last_height_scan = out
        return out

    def pc_callback(self, msg):
        self.recv_count += 1
        t0 = time.time()

        points = _cloud_to_xyz(msg)
        if len(points) == 0:
            return

        if self.zero_height_scan:
            height = np.zeros(HEIGHT_DIM, dtype=np.float32)     # 临时排查: 隔离 height_map 漂移
        elif self.terrain_cache_scan:
            height = self.compute_cached_height_scan(points)    # LiDAR rolling grid + FK/IMU fallback
        elif self.fk_height_scan:
            height = self.compute_fk_height_scan()              # 临时真机: FK+IMU 稳定高度
        else:
            height = self.compute_height_scan()                 # 187
        fwd = _bin_one_direction(points, +1)                    # 252
        bwd = _bin_one_direction(points, -1)                    # 252

        out = Float32MultiArray()
        out.data = height.tolist() + fwd.tolist() + bwd.tolist()
        if len(out.data) != NOISY_ELEV_DIM:
            self.get_logger().error(f"dim {len(out.data)} != {NOISY_ELEV_DIM}, 丢弃")
            return
        self.pub.publish(out)

        self.pub_count += 1
        self.process_times.append((time.time() - t0) * 1000.0)

    def log_diagnostics(self):
        if self.recv_count > 0:
            avg = sum(self.process_times) / len(self.process_times) if self.process_times else 0.0
            hinfo = "height:无网格" if self.height_xyz is None else f"height:{self.height_recv}帧"
            hinfo += f" invalid:{self.height_invalid_count}/{self.height_invalid_total}"
            if self.fk_height_scan or self.terrain_cache_scan:
                hinfo += f" fk_joint:{self.joint_recv} imu:{self.imu_recv}"
            if self.terrain_cache_scan:
                hinfo += (f" cache:{len(self.terrain_cache)} upd:{self.cache_update_cells} "
                          f"hit:{self.cache_hits} miss:{self.cache_misses} "
                          f"base_h:{self.last_base_height if self.last_base_height is not None else float('nan'):.3f}")
            zinfo = ""
            if self.last_odom_z is not None:
                zinfo = (f" | LIO_ODOM z last={self.last_odom_z:.4f} "
                         f"min={self.odom_z_min:.4f} max={self.odom_z_max:.4f}")
            self.get_logger().info(
                f"📊 [noisy_elev] recv {self.recv_count}Hz pub {self.pub_count}Hz "
                f"avg {avg:.2f}ms | {hinfo}{zinfo}")
        else:
            self.get_logger().warn("⚠️ [noisy_elev] waiting lidar cloud ...")
        self.recv_count = 0
        self.pub_count = 0
        self.process_times.clear()


def main(args=None):
    rclpy.init(args=args)
    node = NoisyElevationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
