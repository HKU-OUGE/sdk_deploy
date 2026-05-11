// src/M20_sdk_deploy/src/height_scan_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "drdds/msg/imu_data.hpp"

#include "height_scan_logic.hpp"

class HeightScanNode : public rclcpp::Node {
public:
    HeightScanNode() : Node("height_scan_node") {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scan/height_features_array", 10);

        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIDAR_POINT_CLOUD_MERGED", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                (void)msg;  // TODO: implement in Task 5
            });

        imu_sub_ = this->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA", rclcpp::SensorDataQoS(),
            [this](drdds::msg::ImuData::SharedPtr msg) {
                (void)msg;  // TODO: implement in Task 6
            });

        RCLCPP_INFO(this->get_logger(),
            "[height_scan] ready; output dim=%d, grid=%dx%d res=%.2fm",
            height_scan::GRID_N, height_scan::GRID_NX, height_scan::GRID_NY,
            height_scan::GRID_RESOLUTION);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightScanNode>());
    rclcpp::shutdown();
    return 0;
}
