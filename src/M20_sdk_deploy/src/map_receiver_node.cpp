#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>  // <--- 新增这一行，提供 close() 函数
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/serialization.hpp"

using namespace std::chrono_literals;

class MapReceiverNode : public rclcpp::Node {
public:
    MapReceiverNode() : Node("map_receiver_node") {
        // 声明并获取参数
        this->declare_parameter<int>("port", 9971);
        this->declare_parameter<std::string>("topic_name", "/elevation_map_remote");

        int port = this->get_parameter("port").as_int();
        std::string topic_name = this->get_parameter("topic_name").as_string();

        // 创建发布者，使用 Reliable QoS 保证地图不丢失，保留最新 1 帧
        map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            topic_name, rclcpp::QoS(1).reliable().keep_last(1));
        
        recv_count_ = 0;
        total_bytes_received_ = 0;

        // 启动后台 TCP 接收线程
        tcp_thread_ = std::thread(&MapReceiverNode::tcp_worker, this, port);
        
        // 监控定时器，每秒打印一次网络状态
        monitor_timer_ = this->create_wall_timer(1s, std::bind(&MapReceiverNode::log_diagnostics, this));

        RCLCPP_INFO(this->get_logger(), "🚀 C++ 地图接收端已就绪：正在监听端口 %d", port);
    }

    ~MapReceiverNode() {
        if (tcp_thread_.joinable()) {
            tcp_thread_.join();
        }
    }

private:
    void tcp_worker(int port) {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket 创建失败");
            return;
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "端口 %d 绑定失败，可能被占用", port);
            return;
        }

        if (listen(server_fd, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen 失败");
            return;
        }

        rclcpp::Serialization<grid_map_msgs::msg::GridMap> serializer;

        while (rclcpp::ok()) {
            int new_socket = accept(server_fd, nullptr, nullptr);
            if (new_socket < 0) continue;
            
            RCLCPP_INFO(this->get_logger(), "✨ 笔记本已连接，开始接收高程图数据！");

            while (rclcpp::ok()) {
                uint32_t frame_len = 0;
                // 1. 读取长度头 (使用 MSG_WAITALL 阻塞直到读满)
                int bytes_read = recv(new_socket, &frame_len, 4, MSG_WAITALL);
                if (bytes_read <= 0) break;

                // 防御性编程：限制单帧最大为 50MB，防止粘包引发内存越界
                if (frame_len > 50 * 1024 * 1024) {
                    RCLCPP_WARN(this->get_logger(), "接收到的数据包过大 (%u bytes)，触发保护机制断开连接", frame_len);
                    break;
                }

                // 2. 读取序列化的负载数据
                std::vector<uint8_t> buffer(frame_len);
                bytes_read = recv(new_socket, buffer.data(), frame_len, MSG_WAITALL);
                if (bytes_read <= 0) break;

                // 统计数据
                recv_count_++;
                total_bytes_received_ += (frame_len + 4);

                // 3. 反序列化并发布
                try {
                    auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
                    rclcpp::SerializedMessage serialized_msg(frame_len);
                    memcpy(serialized_msg.get_rcl_serialized_message().buffer, buffer.data(), frame_len);
                    serialized_msg.get_rcl_serialized_message().buffer_length = frame_len;

                    serializer.deserialize_message(&serialized_msg, msg.get());
                    map_pub_->publish(*msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "❌ 地图反序列化失败: %s", e.what());
                }
            }
            close(new_socket);
            RCLCPP_WARN(this->get_logger(), "⚠️ 连接已断开，重新进入等待状态...");
        }
        close(server_fd);
    }

    void log_diagnostics() {
        double bw = static_cast<double>(total_bytes_received_.load()) / (1024.0 * 1024.0);
        if (recv_count_.load() > 0) {
            RCLCPP_INFO(this->get_logger(), 
                "🗺️ [接收状态] 地图接收频率: %d Hz | 实时网络带宽: %.2f MB/s",
                recv_count_.load(), bw);
        }
        
        recv_count_ = 0;
        total_bytes_received_ = 0;
    }

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
    std::thread tcp_thread_;
    std::atomic<int> recv_count_;
    std::atomic<size_t> total_bytes_received_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapReceiverNode>());
    rclcpp::shutdown();
    return 0;
}