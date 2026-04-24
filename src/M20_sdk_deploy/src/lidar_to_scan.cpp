#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>

class LidarToScanArrayNode : public rclcpp::Node {
public:
    LidarToScanArrayNode() : Node("lidar_to_scan_array_node") {
        
        // 1. 声明并获取话题参数，默认值为仿真话题
        this->declare_parameter<std::string>("lidar_topic", "/LIDAR_SIM_RAW");
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();

        // 2. 使用获取到的话题名进行订阅
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, rclcpp::SensorDataQoS(),
            std::bind(&LidarToScanArrayNode::pc_callback, this, std::placeholders::_1));
            
        // 发布轻量数组
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scan/multi_layer_features_array", 10);

        // 诊断定时器
        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LidarToScanArrayNode::log_diagnostics, this));

        RCLCPP_INFO(this->get_logger(), "✅ C++ Lidar to Scan Node started. Subscribing to: %s", lidar_topic.c_str());
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        recv_count_++;
        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<float> fwd_bins(126, 5.0f);
        std::vector<float> bwd_bins(126, 5.0f);

        // 零拷贝迭代器：直接读取二进制内存
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float px = *iter_x;
            float py = *iter_y;
            float pz = *iter_z;

            // 跳过无效点
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

            // ================= 前向处理 =================
            float fx = px - fwd_offset_[0];
            float fy = py - fwd_offset_[1];
            float fz = pz - fwd_offset_[2];

            if (fx > 0.0f && fy >= -0.525f && fy <= 0.525f) {
                float r = std::sqrt(fx*fx + fy*fy + fz*fz);
                float pitch = std::atan2(-fz, fx) * 180.0f / M_PI;
                int bin_y = std::round((fy - y_min_) / dy_step_);

                if (bin_y >= 0 && bin_y < num_rays_) {
                    for (int i = 0; i < 6; ++i) {
                        if (std::abs(pitch - target_angles_[i]) <= angle_tol_) {
                            int idx = i * num_rays_ + bin_y;
                            if (r < fwd_bins[idx]) fwd_bins[idx] = r;
                        }
                    }
                }
            }

            // ================= 后向处理 =================
            float bx = px - bwd_offset_[0];
            float by = py - bwd_offset_[1];
            float bz = pz - bwd_offset_[2];

            if (bx < 0.0f && by >= -0.525f && by <= 0.525f) {
                float r = std::sqrt(bx*bx + by*by + bz*bz);
                float pitch = std::atan2(-bz, -bx) * 180.0f / M_PI; // 后向 X 为负
                int bin_y = std::round((-by - y_min_) / dy_step_);  // Y 轴镜像映射

                if (bin_y >= 0 && bin_y < num_rays_) {
                    for (int i = 0; i < 6; ++i) {
                        if (std::abs(pitch - target_angles_[i]) <= angle_tol_) {
                            int idx = i * num_rays_ + bin_y;
                            if (r < bwd_bins[idx]) bwd_bins[idx] = r;
                        }
                    }
                }
            }
        }

        // ================= 极近盲区处理 =================
        for (auto& val : fwd_bins) { if (val < 0.3f) val = -1.0f; }
        for (auto& val : bwd_bins) { if (val < 0.3f) val = -1.0f; }

        // ================= 发布数据 =================
        std_msgs::msg::Float32MultiArray array_msg;
        array_msg.data.reserve(252);
        array_msg.data.insert(array_msg.data.end(), fwd_bins.begin(), fwd_bins.end());
        array_msg.data.insert(array_msg.data.end(), bwd_bins.begin(), bwd_bins.end());
        pub_->publish(array_msg);

        pub_count_++;
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        total_time_ms_ += elapsed_ms;
    }

    void log_diagnostics() {
        if (recv_count_ > 0) {
            double avg_time = total_time_ms_ / recv_count_;
            RCLCPP_INFO(this->get_logger(), 
                "📊 [Scan C++] Recv: %d Hz | Pub: %d Hz | Avg Process Time: %.3f ms",
                recv_count_, pub_count_, avg_time);
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ [Scan C++] Waiting for PointCloud2...");
        }
        recv_count_ = 0;
        pub_count_ = 0;
        total_time_ms_ = 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // 缓存参数
    const float target_angles_[6] = {-25.0f, -15.0f, -5.0f, 5.0f, 15.0f, 25.0f};
    const float fwd_offset_[3] = {0.32028f, 0.0f, -0.013f};
    const float bwd_offset_[3] = {-0.32028f, 0.0f, -0.013f};
    const float angle_tol_ = 4.0f;
    const float dy_step_ = 0.05f;
    const float y_min_ = -0.5f;
    const int num_rays_ = 21;

    int recv_count_ = 0;
    int pub_count_ = 0;
    double total_time_ms_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToScanArrayNode>());
    rclcpp::shutdown();
    return 0;
}