#!/usr/bin/env python3
# M20_sdk_deploy/scripts/lidar_to_scan.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import math
from std_msgs.msg import Header

class LidarToScanNode(Node):
    def __init__(self):
        super().__init__('lidar_to_scan_node')
        self.subscription = self.create_subscription(PointCloud2, '/LIDAR_POINT_CLOUD_MERGED', self.pc_callback, 10)
        self.merged_pc_pub = self.create_publisher(PointCloud2, '/scan/multi_layer_features', 10)

        # 配置参数 (与 IsaacLab 完全对齐)
        self.z_heights = [-0.20, -0.12, -0.04, 0.04, 0.12, 0.20]
        self.z_tolerance = 0.02
        self.num_rays = 21
        self.max_range = 3.0

    def process_layer(self, points, sensor_pos, is_forward):
        # 1. 计算相对坐标
        rel_points = points - sensor_pos
        
        # 2. 定义 Y 轴平行的“走廊” (Bins)
        y_min, y_max = -0.5, 0.5
        dy = 1.0 / (self.num_rays - 1)  # 0.05m
        
        # 3. 过滤出前方/后方点
        if is_forward:
            mask = (rel_points[:, 0] > 0) & (rel_points[:, 1] >= y_min - dy/2) & (rel_points[:, 1] <= y_max + dy/2)
        else:
            mask = (rel_points[:, 0] < 0) & (rel_points[:, 1] >= y_min - dy/2) & (rel_points[:, 1] <= y_max + dy/2)

        valid_points = rel_points[mask]
        bins = np.full(self.num_rays, self.max_range)
        
        if len(valid_points) > 0:
            # 4. 基于 Y 坐标分 Bin（不再使用极坐标 theta）
            y_coords = valid_points[:, 1]
            if not is_forward:
                y_coords = -y_coords
                
            bin_indices = np.round((y_coords - y_min) / dy).astype(int)
            idx_mask = (bin_indices >= 0) & (bin_indices < self.num_rays)
            bin_indices = bin_indices[idx_mask]
            
            # 计算到 sensor_pos 的欧式距离
            r_dists = np.linalg.norm(valid_points[idx_mask, :2], axis=1)
            
            # ========================================================
            # 5. 剔除原点错误点和自我遮挡点 (物理盲区 r < 0.15m)
            # 这行代码将彻底消灭那两个无论如何都存在的“孤立点”！
            # ========================================================
            valid_r_mask = r_dists > 0.2
            r_dists = r_dists[valid_r_mask]
            bin_indices = bin_indices[valid_r_mask]
            
            # 6. 找出每个 Y 走廊里的最小距离
            if len(r_dists) > 0:
                np.minimum.at(bins, bin_indices, r_dists)

        return bins

    def pc_callback(self, msg):
        pt_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(pt_data) == 0:
            return

        points = np.empty((len(pt_data), 3), dtype=np.float32)
        points[:, 0], points[:, 1], points[:, 2] = pt_data['x'], pt_data['y'], pt_data['z']

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        merged_xyz = []

        for z in self.z_heights:
            # 切片
            z_mask = (points[:, 2] >= (z - self.z_tolerance)) & (points[:, 2] <= (z + self.z_tolerance))
            layer_points = points[z_mask]
            
            # --- 前向处理 ---
            fwd_bins = self.process_layer(layer_points, np.array([0.3, 0.0, z]), is_forward=True)
            for j, r in enumerate(fwd_bins):
                if r < self.max_range:
                    y = -0.5 + j * 0.05
                    x_val = math.sqrt(max(0, r**2 - y**2))
                    merged_xyz.append([0.3 + x_val, y, z])

            # --- 后向处理 ---
            bwd_bins = self.process_layer(layer_points, np.array([-0.3, 0.0, z]), is_forward=False)
            for j, r in enumerate(bwd_bins):
                if r < self.max_range:
                    y = -0.5 + j * 0.05
                    x_val = math.sqrt(max(0, r**2 - y**2))
                    merged_xyz.append([-0.3 - x_val, -y, z])

        if merged_xyz:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, merged_xyz)
            self.merged_pc_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()