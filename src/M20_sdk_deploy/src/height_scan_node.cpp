// src/M20_sdk_deploy/src/height_scan_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "drdds/msg/imu_data.hpp"

#include <atomic>
#include <chrono>
#include <vector>

#include "height_scan_logic.hpp"

using namespace std::chrono_literals;

class HeightScanNode : public rclcpp::Node {
public:
    HeightScanNode() : Node("height_scan_node") {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scan/height_features_array", 10);

        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIDAR_POINT_CLOUD_MERGED", rclcpp::SensorDataQoS(),
            std::bind(&HeightScanNode::OnPointCloud, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA", rclcpp::SensorDataQoS(),
            std::bind(&HeightScanNode::OnImu, this, std::placeholders::_1));

        diag_timer_ = this->create_wall_timer(1s,
            std::bind(&HeightScanNode::LogDiagnostics, this));

        RCLCPP_INFO(this->get_logger(),
            "[height_scan] ready; output dim=%d, grid=%dx%d res=%.2fm",
            height_scan::GRID_N, height_scan::GRID_NX, height_scan::GRID_NY,
            height_scan::GRID_RESOLUTION);
    }

private:
    void OnImu(drdds::msg::ImuData::SharedPtr msg) {
        const float deg2rad = static_cast<float>(M_PI / 180.0);
        latest_roll_.store(static_cast<float>(msg->data.roll) * deg2rad);
        latest_pitch_.store(static_cast<float>(msg->data.pitch) * deg2rad);
        imu_received_.store(true);
    }

    void OnPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!imu_received_.load()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[height_scan] waiting for IMU before processing pointcloud");
            return;
        }

        // Update-rate gating: target update_rate_hz_; skip frames arriving too fast.
        const auto now = std::chrono::steady_clock::now();
        const auto target_dt = std::chrono::duration<double>(1.0 / update_rate_hz_);
        if (last_publish_time_.time_since_epoch().count() > 0 &&
            (now - last_publish_time_) < std::chrono::duration_cast<std::chrono::steady_clock::duration>(target_dt)) {
            return;
        }
        last_publish_time_ = now;

        const auto t0 = std::chrono::steady_clock::now();

        // Pull all finite points into a flat vector (cheap; ~10k points typical).
        std::vector<height_scan::Vec3f> pts;
        pts.reserve(msg->width * msg->height);
        try {
            sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
            for (; ix != ix.end(); ++ix, ++iy, ++iz) {
                pts.emplace_back(*ix, *iy, *iz);
            }
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[height_scan] malformed PointCloud2 (missing x/y/z field?): %s — frame dropped",
                e.what());
            return;
        }

        std::array<float, height_scan::GRID_N> obs_now{};
        std::array<bool,  height_scan::GRID_N> valid_now{};
        height_scan::ProjectAndBin(pts,
                                   latest_roll_.load(), latest_pitch_.load(),
                                   obs_now, valid_now);

        std::array<float, height_scan::GRID_N> obs_out{};
        height_scan::ApplyHistory(history_, obs_now, valid_now, obs_out);

        std_msgs::msg::Float32MultiArray out_msg;
        out_msg.data.assign(obs_out.begin(), obs_out.end());
        pub_->publish(out_msg);

        const auto t1 = std::chrono::steady_clock::now();
        const double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        total_time_ms_ += dt_ms;
        ++process_count_;
    }

    void LogDiagnostics() {
        if (process_count_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "[height_scan] processed %d/s, avg %.2f ms",
                process_count_, total_time_ms_ / process_count_);
        } else if (!imu_received_.load()) {
            RCLCPP_WARN(this->get_logger(), "[height_scan] no IMU yet");
        } else {
            RCLCPP_WARN(this->get_logger(), "[height_scan] no pointcloud yet");
        }
        process_count_ = 0;
        total_time_ms_ = 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::atomic<float> latest_roll_{0.0f};
    std::atomic<float> latest_pitch_{0.0f};
    std::atomic<bool>  imu_received_{false};

    height_scan::HistoryBuffer history_;
    int process_count_ = 0;
    double total_time_ms_ = 0.0;

    double update_rate_hz_ = 20.0;
    std::chrono::steady_clock::time_point last_publish_time_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightScanNode>());
    rclcpp::shutdown();
    return 0;
}
