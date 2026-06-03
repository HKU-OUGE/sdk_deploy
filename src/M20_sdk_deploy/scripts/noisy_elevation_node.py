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
    值 = clamp(base_z - terrain_z - 0.5, -1, 1)。terrain 取自 base 系高程网格
    /height_map (height_map_nav, 81×81@0.1m base_link), base 系下 terrain_z_base = -(base距地高),
    故 height_scan = -terrain_z_base - 0.5。**不依赖 odom 绝对 z** (对 LIO z 漂移/恒0 免疫)。
    注意: 真机 /height_map 需先 ros2 service call /HEIGHT_MAP_ENABLE drdds/srv/StdSrvInt32 "{command: 1}";
    /HEIGHT_POINTS 是死占位(恒 -0.4)勿用。已在真机 /height_map 上验证 187 维输出合理(0空洞)。

话题 (参数化):
    lidar_topic   默认 /LIDAR_SIM_RAW   (sim mujoco; 真机改 /CLOUD_REGISTERED_BODY ~10Hz, base 系点云)
    height_topic  默认 /height_map      (真机 height_map_nav, 需 /HEIGHT_MAP_ENABLE 开; sim 发同名)
    /LIO_ODOM     仅订阅打印 z 供校验 (height_scan 不依赖它)
输出: /perception/noisy_elevation_array  (Float32MultiArray, 691)
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray

try:
    from nav_msgs.msg import Odometry
    _HAS_ODOM = True
except Exception:
    _HAS_ODOM = False

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
    raw = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
    if not raw:
        return np.empty((0, 3), dtype=np.float32)
    a = np.array(raw)
    if a.dtype.names is not None:
        a = np.column_stack((a['x'], a['y'], a['z'])).astype(np.float32)
    else:
        a = a.astype(np.float32)
    return a


class NoisyElevationNode(Node):
    def __init__(self):
        super().__init__('noisy_elevation_node')
        self.declare_parameter('lidar_topic', '/LIDAR_SIM_RAW')
        # 真机: /height_map (height_map_nav, 81×81@0.1m base_link, 需 /HEIGHT_MAP_ENABLE command:1 开启)
        # sim:  发同名 /height_map 即可 (Phase4)。注意 /HEIGHT_POINTS 是死占位, 勿用
        self.declare_parameter('height_topic', '/height_map')
        lt = self.get_parameter('lidar_topic').value
        ht = self.get_parameter('height_topic').value

        self.sub = self.create_subscription(PointCloud2, lt, self.pc_callback, 10)
        self.height_sub = self.create_subscription(PointCloud2, ht, self.height_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/perception/noisy_elevation_array', 10)

        self.height_lock = threading.Lock()
        self.height_xyz = None          # (N,3) base 系高程网格
        self.height_recv = 0

        # /LIO_ODOM 仅用于记录/校验 z (height_scan 不依赖绝对 z)
        self.last_odom_z = None
        self.odom_z_min = None
        self.odom_z_max = None
        if _HAS_ODOM:
            self.odom_sub = self.create_subscription(Odometry, '/LIO_ODOM', self.odom_callback, 10)

        self.recv_count = 0
        self.pub_count = 0
        self.process_times = []
        self.create_timer(1.0, self.log_diagnostics)
        self.get_logger().info(
            f"✅ noisy_elevation: lidar={lt} height={ht} | "
            f"height({HEIGHT_DIM}) + fwd({NUM_PER_DIR}) + bwd({NUM_PER_DIR}) = {NOISY_ELEV_DIM}")

    def odom_callback(self, msg):
        z = float(msg.pose.pose.position.z)
        self.last_odom_z = z
        self.odom_z_min = z if self.odom_z_min is None else min(self.odom_z_min, z)
        self.odom_z_max = z if self.odom_z_max is None else max(self.odom_z_max, z)

    def height_callback(self, msg):
        a = _cloud_to_xyz(msg)
        with self.height_lock:
            self.height_xyz = a
            self.height_recv += 1

    def compute_height_scan(self):
        """187 维 height_scan = clamp(-terrain_z_base - 0.5, -1, 1), 取自 base 系高程网格。"""
        with self.height_lock:
            g = self.height_xyz
        if g is None or len(g) == 0:
            return np.zeros(HEIGHT_DIM, dtype=np.float32)        # 网格未到: 安全默认(平地名义)
        xs, ys, zs = g[:, 0], g[:, 1], g[:, 2]
        finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        if not np.any(finite):
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
        terrain = Zmap[tgy * W + tgx]                  # 187 terrain_z_base
        hs = -terrain - HEIGHT_OFFSET                  # = base_z - terrain_z - 0.5
        hs = np.where(np.isfinite(hs), hs, 0.0)        # 空洞 -> 0 (名义平地)
        return np.clip(hs, -1.0, 1.0).astype(np.float32)

    def pc_callback(self, msg):
        self.recv_count += 1
        t0 = time.time()

        points = _cloud_to_xyz(msg)
        if len(points) == 0:
            return

        height = self.compute_height_scan()                     # 187
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
