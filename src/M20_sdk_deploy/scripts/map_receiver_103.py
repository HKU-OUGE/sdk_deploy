#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
import socket
import struct
import threading
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class M20MapReceiver(Node):
    def __init__(self):
        super().__init__('m20_map_receiver')
        
        # 开放端口等待笔记本连接
        self.port = 9971 
        
        # GridMap 数据量较大，建议使用 KEEP_LAST depth=1 防止堆积
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # 接收端尽量保证可靠性
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 将接收到的地图发布到 M20 本地
        self.pub = self.create_publisher(GridMap, '/laptop_elevation_map', qos)
        
        self.recv_count = 0
        
        # 启动后台 TCP 监听线程
        threading.Thread(target=self.tcp_server_worker, daemon=True).start()
        
        # 定时器：每秒打印一次状态
        self.create_timer(1.0, self.debug_timer_cb)
        self.get_logger().info(f"✅ M20 地图接收端就绪 | 正在监听端口 {self.port} 等待笔记本连接...")

    def tcp_server_worker(self):
        server_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_fd.bind(('0.0.0.0', self.port))
        server_fd.listen(1)
        
        while rclpy.ok():
            conn, addr = server_fd.accept()
            self.get_logger().info(f"✨ 笔记本 ({addr[0]}) 已连接，开始接收地图数据！")
            
            while rclpy.ok():
                try:
                    # 1. 读取 4 字节的长度头
                    len_bytes = conn.recv(4)
                    if not len_bytes or len(len_bytes) < 4:
                        break
                    msg_len = struct.unpack('<I', len_bytes)[0]
                    
                    # 2. 循环读取直到完整接收整帧地图 (防 TCP 截断)
                    data = bytearray()
                    while len(data) < msg_len:
                        packet = conn.recv(msg_len - len(data))
                        if not packet:
                            break
                        data.extend(packet)
                        
                    # 3. 反序列化并发布到本地
                    if len(data) == msg_len:
                        msg = deserialize_message(bytes(data), GridMap)
                        self.pub.publish(msg)
                        self.recv_count += 1
                        
                except Exception as e:
                    self.get_logger().error(f"❌ 接收出错或连接重置: {e}")
                    break
                    
            conn.close()
            self.get_logger().warn("⚠️ 连接已断开，等待重新连接...")

    def debug_timer_cb(self):
        if self.recv_count > 0:
            self.get_logger().info(f"🗺️ [地图接收状态] 过去一秒收到并发布地图: {self.recv_count} 帧")
        self.recv_count = 0

def main():
    rclpy.init()
    node = M20MapReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()