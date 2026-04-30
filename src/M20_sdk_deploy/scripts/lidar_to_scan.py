#!/usr/bin/env python3
"""半球 LiDAR → policy obs (与 sim/rl_training/sensors/patterns.py 一致)

输入: base_link 系下的 PointCloud2 (Robosense Airy)
输出: Float32MultiArray (992 维) = 前向 16×31 (496) + 后向 16×31 (496)
布局: row-major (polar 外, azimuth 内)
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


NUM_POLAR = 16
NUM_AZ = 31
NUM_PER_DIR = NUM_POLAR * NUM_AZ          # 496
NUM_TOTAL = NUM_PER_DIR * 2                # 992
MAX_DIST = 2.5
MIN_DIST = 0.3
HALF_PI = math.pi / 2
TWO_PI = 2 * math.pi


class LidarToScanArrayNode(Node):
    def __init__(self):
        super().__init__('lidar_to_scan_array_node')
        self.subscription = self.create_subscription(
            PointCloud2, '/LIDAR_SIM_RAW', self.pc_callback, 10)
        self.array_pub = self.create_publisher(
            Float32MultiArray, '/scan/multi_layer_features_array', 10)

        self.fwd_offset = np.array([0.32028, 0.0, -0.013], dtype=np.float32)
        self.bwd_offset = np.array([-0.32028, 0.0, -0.013], dtype=np.float32)

        self.recv_count = 0
        self.pub_count = 0
        self.process_times = []
        self.diag_timer = self.create_timer(1.0, self.log_diagnostics)

    @staticmethod
    def _bin_one_direction(pts, boresight_sign):
        """boresight_sign = +1: 前向 (boresight=+X); -1: 后向 (boresight=-X, 配合 Y 取反)"""
        bins = np.full(NUM_PER_DIR, MAX_DIST, dtype=np.float32)
        if pts.size == 0:
            return bins

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        if boresight_sign > 0:
            mask = x > 0.0
            x_b = x
            y_b = y
        else:
            mask = x < 0.0
            x_b = -x
            y_b = -y                               # 180°Z 旋转 → Y 取反

        if not np.any(mask):
            return bins
        x_b = x_b[mask]; y_b = y_b[mask]; z_b = z[mask]

        r = np.sqrt(x_b * x_b + y_b * y_b + z_b * z_b)
        valid = (r >= MIN_DIST) & (r <= MAX_DIST)
        if not np.any(valid):
            return bins
        x_b = x_b[valid]; y_b = y_b[valid]; z_b = z_b[valid]; r = r[valid]

        perp = np.sqrt(y_b * y_b + z_b * z_b)
        polar = np.arctan2(perp, x_b)              # [0, π/2]
        az = np.arctan2(y_b, z_b)                  # [-π, π]

        p_idx = np.clip(np.rint(polar * (NUM_POLAR - 1) / HALF_PI).astype(np.int32),
                        0, NUM_POLAR - 1)
        a_idx = np.clip(np.rint((az + math.pi) * (NUM_AZ - 1) / TWO_PI).astype(np.int32),
                        0, NUM_AZ - 1)
        flat = p_idx * NUM_AZ + a_idx

        np.minimum.at(bins, flat, r)
        return bins

    def pc_callback(self, msg):
        self.recv_count += 1
        start_time = time.time()

        pt_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pt_list = list(pt_data)
        if not pt_list:
            return

        tmp_arr = np.array(pt_list)
        if tmp_arr.dtype.names is not None:
            points = np.column_stack((tmp_arr['x'], tmp_arr['y'], tmp_arr['z'])).astype(np.float32)
        else:
            points = tmp_arr.astype(np.float32)

        if len(points) == 0:
            return

        rel_fwd = points - self.fwd_offset
        rel_bwd = points - self.bwd_offset

        fwd_bins = self._bin_one_direction(rel_fwd, +1)
        bwd_bins = self._bin_one_direction(rel_bwd, -1)

        # 兜底盲区 (与 sim multi_layer_scan 一致：< 0.3m → 2.5m)
        fwd_bins[fwd_bins < MIN_DIST] = MAX_DIST
        bwd_bins[bwd_bins < MIN_DIST] = MAX_DIST

        array_msg = Float32MultiArray()
        array_msg.data = fwd_bins.tolist() + bwd_bins.tolist()
        self.array_pub.publish(array_msg)

        self.pub_count += 1
        self.process_times.append((time.time() - start_time) * 1000.0)

    def log_diagnostics(self):
        if self.recv_count > 0:
            avg_time = sum(self.process_times) / len(self.process_times) if self.process_times else 0.0
            color = "\033[92m" if self.recv_count >= 8 else "\033[93m"
            reset = "\033[0m"
            self.get_logger().info(
                f"📊 [Scan Node] {color}Recv: {self.recv_count} Hz | Pub: {self.pub_count} Hz{reset} | "
                f"Avg Process Time: {avg_time:.2f} ms"
            )
        else:
            self.get_logger().warn("⚠️ [Scan Node] Waiting for PointCloud2 data on /LIDAR_SIM_RAW ...")
        self.recv_count = 0
        self.pub_count = 0
        self.process_times.clear()


def main(args=None):
    rclpy.init(args=args)
    node = LidarToScanArrayNode()
    node.get_logger().info(
        f"✅ Lidar to Scan Array Node started. Hemispherical {NUM_POLAR}×{NUM_AZ} per dir, "
        f"output dim = {NUM_TOTAL}.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
