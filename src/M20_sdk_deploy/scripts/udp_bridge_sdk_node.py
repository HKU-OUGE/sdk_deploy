#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import socket
import numpy as np
import threading
import time
from grid_map_msgs.msg import GridMap
from rclpy.serialization import deserialize_message

class UdpBridgeANode(Node):
    def __init__(self):
        super().__init__('udp_bridge_a_node')

        self.B_IP_PORT = ("127.0.0.1", 5001)
        self.LOCAL_PORT = 5002

        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(("0.0.0.0", self.LOCAL_PORT))

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/LIDAR_SIM_RAW', self.lidar_callback, 10)
        # 新增：发布还原后的高程图给 C++ 控制器
        self.map_pub = self.create_publisher(
            GridMap,
            '/m20_deploy/elevation_map_udp',
            10)
        self.recv_thread = threading.Thread(target=self.udp_receive_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info("Node A started. Streaming Lidar chunks and waiting for Map...")

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

        # 将一帧大点云切分为每次约 2000 点的小块 (24KB)，保证绝对不超 UDP 限制
        CHUNK_SIZE = 2000

        # (可选) 随机打乱点云：即使发生轻微丢包，地图密度也能保持均匀
        np.random.shuffle(points)

        chunks_sent = 0
        for i in range(0, points.shape[0], CHUNK_SIZE):
            chunk_points = points[i : i+CHUNK_SIZE]
            try:
                self.sock_send.sendto(chunk_points.tobytes(), self.B_IP_PORT)
                chunks_sent += 1
                # 给予操作系统底层缓冲微小的喘息时间，防止瞬间突发丢包
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"UDP Send Error: {e}")

        # self.get_logger().info(f"Sent 1 frame as {chunks_sent} chunks.")

    def udp_receive_loop(self):
        while True:
            try:
                data, addr = self.sock_recv.recvfrom(65535)

                # 保护：只有足够大的包才是序列化后的 GridMap (通常 25KB~55KB)
                if len(data) > 1000:
                    # 反序列化还原出原汁原味的 GridMap 消息
                    msg = deserialize_message(data, GridMap)
                    self.map_pub.publish(msg)
                    self.get_logger().info(f"✅ Received and published full GridMap to C++ Controller", throttle_duration_sec=1.0)
            except Exception as e:
                # 忽略残破的数据包
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UdpBridgeANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()