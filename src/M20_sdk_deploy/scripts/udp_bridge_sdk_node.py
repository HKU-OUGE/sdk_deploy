#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
import tf2_ros
import socket
import numpy as np
import threading
import time
import struct

class UdpBridgeANode(Node):
    def __init__(self):
        super().__init__('udp_bridge_a_node')

        # --- 请替换为电脑 B 的真实 IP ---
        self.B_IP_PORT = ("192.168.8.103", 5001)
        self.B_IP_PORT = ("127.0.0.1", 5001)
        self.LOCAL_PORT = 5002

        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(("0.0.0.0", self.LOCAL_PORT))

        # 监听本地仿真器发布的 TF 树
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/LIDAR_SIM_RAW', self.lidar_callback, 10)

        # 专属隔离话题，供 C++ 和仿真器订阅
        self.map_pub = self.create_publisher(
            GridMap, '/m20_deploy/elevation_map_udp', 10)

        self.recv_thread = threading.Thread(target=self.udp_receive_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info("Node A started. Streaming Lidar & TF via UDP...")

    def lidar_callback(self, msg: PointCloud2):
        raw_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if raw_points is None or len(raw_points) == 0:
            return

        if isinstance(raw_points, np.ndarray):
            points = np.column_stack((raw_points['x'], raw_points['y'], raw_points['z'])).astype(np.float32)
        else:
            points = np.array(list(raw_points), dtype=np.float32)
            if len(points.shape) == 1:
                return

        # 1. 尝试获取最新的 TF 变换 (odom -> base_link)
        try:
            # 查找最新时间的 TF
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
        except Exception as e:
            # 如果刚启动还没拿到 TF，发一个全 0 默认姿态过去保底
            tx, ty, tz, qx, qy, qz, qw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0

        # 2. 将 TF 7个参数打包成 28 字节的二进制头部 (Little-Endian Float32)
        tf_header = struct.pack('<7f', tx, ty, tz, qx, qy, qz, qw)

        CHUNK_SIZE = 2000
        np.random.shuffle(points)

        for i in range(0, points.shape[0], CHUNK_SIZE):
            chunk_points = points[i : i+CHUNK_SIZE]
            try:
                # 3. 将 28 字节的 TF 头部和点云数据拼接在一起发送
                payload = tf_header + chunk_points.tobytes()
                self.sock_send.sendto(payload, self.B_IP_PORT)
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"UDP Send Error: {e}")

    def udp_receive_loop(self):
        while True:
            try:
                data, addr = self.sock_recv.recvfrom(65535)
                if len(data) > 1000:
                    msg = deserialize_message(data, GridMap)
                    self.map_pub.publish(msg)
                    self.get_logger().info(f"✅ Received GridMap from B", throttle_duration_sec=1.0)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UdpBridgeANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()