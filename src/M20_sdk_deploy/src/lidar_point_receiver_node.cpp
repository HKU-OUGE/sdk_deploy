#include <memory>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/serialization.hpp"

using namespace std::chrono_literals;

class LidarReceiveNode : public rclcpp::Node {
public:
    LidarReceiveNode() : Node("lidar_point_receive_node") {
        // 使用 SensorDataQoS，严格匹配 m20_sensor_policy_runner.hpp 中的订阅者 QoS
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/LIDAR_SIM_RAW", qos);
        
        recv_count_ = 0;
        total_bytes_received_ = 0;

        // 启动后台 TCP 接收线程，监听 9998 端口
        tcp_thread_ = std::thread(&LidarReceiveNode::tcp_worker, this, 9998);
        
        monitor_timer_ = this->create_wall_timer(1s, std::bind(&LidarReceiveNode::log_diagnostics, this));
        RCLCPP_INFO(this->get_logger(), "🚀 [103主机] RL雷达接收端已启动，监听端口 9998");
    }

    ~LidarReceiveNode() {
        if (tcp_thread_.joinable()) tcp_thread_.join();
    }

private:
    void tcp_worker(int port) {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) return;

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) return;
        if (listen(server_fd, 1) < 0) return;

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

        while (rclcpp::ok()) {
            int new_socket = accept(server_fd, nullptr, nullptr);
            if (new_socket < 0) continue;
            
            // 降低接收延迟
            int nodelay = 1;
            setsockopt(new_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

            RCLCPP_INFO(this->get_logger(), "✨ 106 主机已连接，开始接收 Policy 专属点云！");

            while (rclcpp::ok()) {
                uint32_t frame_len = 0;
                // 读取长度头
                if (recv(new_socket, &frame_len, 4, MSG_WAITALL) <= 0) break;

                // 防爆内存保护 (单帧 > 20MB 则异常)
                if (frame_len > 20 * 1024 * 1024) break;

                // 读取点云数据
                std::vector<uint8_t> buffer(frame_len);
                if (recv(new_socket, buffer.data(), frame_len, MSG_WAITALL) <= 0) break;

                recv_count_++;
                total_bytes_received_ += (frame_len + 4);

                // 反序列化并发布
                try {
                    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    rclcpp::SerializedMessage serialized_msg(frame_len);
                    memcpy(serialized_msg.get_rcl_serialized_message().buffer, buffer.data(), frame_len);
                    serialized_msg.get_rcl_serialized_message().buffer_length = frame_len;

                    serializer.deserialize_message(&serialized_msg, msg.get());
                    
                    // 强制时间同步，对齐 103 本地时间戳，消除分布式时间差对 RL policy 观测延迟的影响
                    msg->header.stamp = this->now();
                    msg->header.frame_id = "lidar_link";

                    lidar_pub_->publish(*msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "序列化错误: %s", e.what());
                }
            }
            close(new_socket);
            RCLCPP_WARN(this->get_logger(), "⚠️ 与 106 断开，等待重连...");
        }
        close(server_fd);
    }

    void log_diagnostics() {
        double bw = static_cast<double>(total_bytes_received_.load()) / (1024.0 * 1024.0);
        if (recv_count_.load() > 0) {
            RCLCPP_INFO(this->get_logger(), 
                "📊 [RL感知] 接收频率: %d Hz | 带宽: %.2f MB/s",
                recv_count_.load(), bw);
        }
        recv_count_ = 0;
        total_bytes_received_ = 0;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    std::thread tcp_thread_;
    std::atomic<int> recv_count_;
    std::atomic<size_t> total_bytes_received_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarReceiveNode>());
    rclcpp::shutdown();
    return 0;
}