#!/usr/bin/env python3
"""TCP Bridge (SDK 端 / Node B): 跨机器接收 lidar 点云 + base_link TF.

A→B (本节点接收): pose (28 bytes: 3 trans + 4 quat) + N×12 bytes point cloud
B→A: 已无内容下发 (elevation map 在 ElevationAE 移除后不再使用)
"""
import socket
import struct
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

def recvall(sock, n):
    """辅助函数：确保从TCP流中完整读取 n 个字节"""
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return bytes(data)

class TcpBridgeBNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_b_node')

        # --- 请替换为电脑 A 的真实 IP ---
        self.A_IP = "127.0.0.1" 
        self.A_PORT = 5002

        self.sock = None
        self.is_connected = False

        self.lidar_pub = self.create_publisher(PointCloud2, '/LIDAR_POINT_CLOUD_MERGED', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ElevationAE 已移除：不再订阅 /elevation_mapping_node/elevation_map_raw
        self.get_logger().info("Node B (Client) started. Trying to connect to Node A...")

        self.recv_thread = threading.Thread(target=self.tcp_connect_and_receive_loop, daemon=True)
        self.recv_thread.start()

    def tcp_connect_and_receive_loop(self):
        while True:
            # 1. 连接/重连逻辑
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.sock.connect((self.A_IP, self.A_PORT))
                self.is_connected = True
                self.get_logger().info(f"✅ Successfully connected to Node A ({self.A_IP}:{self.A_PORT})")
            except Exception as e:
                self.get_logger().warn(f"Waiting for Node A... ({e})", throttle_duration_sec=2.0)
                time.sleep(2)
                continue

            # 2. 持续接收雷达点云与 TF 数据的循环
            while self.is_connected:
                try:
                    # 读取4字节包长
                    raw_msglen = recvall(self.sock, 4)
                    if not raw_msglen:
                        break # 连接断开
                    msglen = struct.unpack('<I', raw_msglen)[0]

                    # 读取真实载荷
                    data = recvall(self.sock, msglen)
                    if not data:
                        break # 连接断开

                    if len(data) <= 28:
                        continue

                    # 拆解数据
                    tx, ty, tz, qx, qy, qz, qw = struct.unpack('<7f', data[:28])
                    payload = data[28:]
                    if len(payload) % 12 != 0:
                        continue

                    points_array = np.frombuffer(payload, dtype=np.float32).reshape(-1, 3)
                    current_time = self.get_clock().now().to_msg()

                    # 恢复本地 TF 树
                    t = TransformStamped()
                    t.header.stamp = current_time
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tx, ty, tz
                    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
                    self.tf_broadcaster.sendTransform(t)

                    # 发布本地雷达云
                    header = Header()
                    header.stamp = current_time
                    header.frame_id = 'base_link'
                    pc2_msg = pc2.create_cloud_xyz32(header, points_array.tolist())
                    self.lidar_pub.publish(pc2_msg)

                except Exception as e:
                    self.get_logger().error(f"TCP Recv Error: {e}")
                    break
            
            # 如果跳出循环，说明连接断开，准备重连
            self.is_connected = False
            self.sock.close()
            self.get_logger().warn("❌ Connection lost. Reconnecting in 2 seconds...")
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = TcpBridgeBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()