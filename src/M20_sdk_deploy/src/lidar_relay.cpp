#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarRelay : public rclcpp::Node {
public:
    LidarRelay() : Node("lidar_relay_node") {
        // 严格对齐雷达底层的 QoS 设置：可靠传输 (Reliable) + 挥发性 (Volatile)
        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();
        qos_profile.durability_volatile();

        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/LIDAR/POINTS_RELAY", qos_profile);

        // 创建订阅者（注意这里使用了 UniquePtr，这是防止内存溢出的核心！）
        // 收到数据后，所有权直接转移，不发生深拷贝
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIDAR/POINTS", qos_profile,
            [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
                publisher_->publish(std::move(msg));
            });
            
        RCLCPP_INFO(this->get_logger(), "C++ Lidar Relay Node started successfully! Zero-copy forwarding enabled.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRelay>());
    rclcpp::shutdown();
    return 0;
}