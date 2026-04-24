#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from sensor_msgs_py import point_cloud2
import numpy as np
import time

class LidarToScanArrayNode(Node):
    def __init__(self):
        super().__init__('lidar_to_scan_array_node')
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2, '/LIDAR_SIM_RAW', self.pc_callback, 10)
            
        # 发布Float32 数组 (252 个数值)
        self.array_pub = self.create_publisher(
            Float32MultiArray, '/scan/multi_layer_features_array', 10)

        # 预先缓存常量
        self.target_angles = np.array([-25.0, -15.0, -5.0, 5.0, 15.0, 25.0], dtype=np.float32)
        self.fwd_offset = np.array([0.32028, 0.0, -0.013], dtype=np.float32)
        self.bwd_offset = np.array([-0.32028, 0.0, -0.013], dtype=np.float32)
        self.angle_tol = 4.0
        self.dy_step = 0.05
        self.y_min = -0.5
        self.num_rays = 21

        # ================= 诊断统计变量 =================
        self.recv_count = 0
        self.pub_count = 0
        self.process_times = []  # 用于统计单次处理的耗时
        
        # 创建一个 1Hz 的定时器，每秒打印一次诊断信息
        self.diag_timer = self.create_timer(1.0, self.log_diagnostics)

    def pc_callback(self, msg):
        self.recv_count += 1  # 记录接收次数
        start_time = time.time()

        # 提取点云数据，兼容 ROS 2 Jazzy 结构化数组
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

        # 准备 252 维容器 (初始化为最大距离 5.0)
        fwd_bins = np.full((6, self.num_rays), 5.0, dtype=np.float32)
        bwd_bins = np.full((6, self.num_rays), 5.0, dtype=np.float32)

        # ================= 前向处理 =================
        rel_fwd = points - self.fwd_offset
        mask_fwd = (rel_fwd[:, 0] > 0.0) & (rel_fwd[:, 1] >= -0.525) & (rel_fwd[:, 1] <= 0.525)
        pts_fwd = rel_fwd[mask_fwd]
        
        if len(pts_fwd) > 0:
            r_fwd = np.linalg.norm(pts_fwd, axis=1)
            pitch_fwd = np.degrees(np.arctan2(-pts_fwd[:, 2], pts_fwd[:, 0]))
            bin_y_fwd = np.round((pts_fwd[:, 1] - self.y_min) / self.dy_step).astype(int)

            valid_idx_mask = (bin_y_fwd >= 0) & (bin_y_fwd < self.num_rays)
            r_fwd, pitch_fwd, bin_y_fwd = r_fwd[valid_idx_mask], pitch_fwd[valid_idx_mask], bin_y_fwd[valid_idx_mask]

            diff_fwd = np.abs(pitch_fwd[:, None] - self.target_angles[None, :])
            match_mask_fwd = diff_fwd <= self.angle_tol

            for i in range(6):
                idx = match_mask_fwd[:, i]
                if np.any(idx):
                    np.minimum.at(fwd_bins[i], bin_y_fwd[idx], r_fwd[idx])

        # ================= 后向处理 =================
        rel_bwd = points - self.bwd_offset
        mask_bwd = (rel_bwd[:, 0] < 0.0) & (rel_bwd[:, 1] >= -0.525) & (rel_bwd[:, 1] <= 0.525)
        pts_bwd = rel_bwd[mask_bwd]
        
        if len(pts_bwd) > 0:
            r_bwd = np.linalg.norm(pts_bwd, axis=1)
            pitch_bwd = np.degrees(np.arctan2(-pts_bwd[:, 2], -pts_bwd[:, 0]))
            bin_y_bwd = np.round((-pts_bwd[:, 1] - self.y_min) / self.dy_step).astype(int)

            valid_idx_mask = (bin_y_bwd >= 0) & (bin_y_bwd < self.num_rays)
            r_bwd, pitch_bwd, bin_y_bwd = r_bwd[valid_idx_mask], pitch_bwd[valid_idx_mask], bin_y_bwd[valid_idx_mask]

            diff_bwd = np.abs(pitch_bwd[:, None] - self.target_angles[None, :])
            match_mask_bwd = diff_bwd <= self.angle_tol

            for i in range(6):
                idx = match_mask_bwd[:, i]
                if np.any(idx):
                    np.minimum.at(bwd_bins[i], bin_y_bwd[idx], r_bwd[idx])

        # ================= 极近盲区处理 =================
        fwd_bins[fwd_bins < 0.3] = -1.0
        bwd_bins[bwd_bins < 0.3] = -1.0

        # ================= 发布轻量数组 =================
        array_msg = Float32MultiArray()
        array_msg.data = fwd_bins.flatten().tolist() + bwd_bins.flatten().tolist()
        self.array_pub.publish(array_msg)
        
        self.pub_count += 1  # 记录发布次数
        
        # 计算单次处理耗时 (毫秒)
        process_time_ms = (time.time() - start_time) * 1000.0
        self.process_times.append(process_time_ms)

    def log_diagnostics(self):
        """每秒钟打印一次监控日志"""
        if self.recv_count > 0:
            avg_time = sum(self.process_times) / len(self.process_times) if self.process_times else 0.0
            
            # 使用简单的色彩区分正常/异常状态
            color = "\033[92m" if self.recv_count >= 8 else "\033[93m"  # 正常雷达至少10Hz，给点容差用8Hz
            reset = "\033[0m"
            
            self.get_logger().info(
                f"📊 [Scan Node] {color}Recv: {self.recv_count} Hz | Pub: {self.pub_count} Hz{reset} | "
                f"Avg Process Time: {avg_time:.2f} ms"
            )
        else:
            self.get_logger().warn("⚠️ [Scan Node] Waiting for PointCloud2 data on /LIDAR_SIM_RAW ...")
            
        # 清零计数器
        self.recv_count = 0
        self.pub_count = 0
        self.process_times.clear()

def main(args=None):
    rclpy.init(args=args)
    node = LidarToScanArrayNode()
    node.get_logger().info("✅ Lidar to Scan Array Node started with Diagnostics Monitoring.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
