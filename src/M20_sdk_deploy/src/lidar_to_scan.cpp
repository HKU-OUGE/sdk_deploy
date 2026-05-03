// 半球 LiDAR → policy obs 转换 (Sim 端 HemisphericalLidarPatternCfg 对齐)
//
// 输入：base_link 系下的 PointCloud2 (Robosense Airy)
// 输出：Float32MultiArray (992 维)
//   [0   .. 495]  前向半球 16 polar × 31 azimuth (row-major: polar 外, azimuth 内)
//   [496 .. 991]  后向半球，同样 16 × 31 layout
//
// 几何 (与 sim/rl_training/sensors/patterns.py 完全一致):
//   - polar  ∈ [0, POLAR_MAX_RAD = 80°]:  从 boresight 出发的极角
//        - 真 Airy Lissajous 扫描密度在 polar > 84° 急剧下降，导致 sim2real
//          gap (real polar=90° no-hit 86% vs sim 59%)。把 sim 与 deploy
//          都收紧到 80°，确保两端密度匹配。polar > 80° 的真机点直接丢弃。
//   - azimuth ∈ [-π, π]:    绕 boresight 一圈，从 +Z 起算，正向 +Y
//   - 前 boresight = robot +X，后 boresight = robot -X (对应 sim 后 sensor 的 180°Z 旋转)
//
// Bin 规则 (nearest neighbor, 匹配 sim 的 linspace 采样点):
//   polar_idx   = round(polar   * (NUM_POLAR-1)   / POLAR_MAX_RAD)  ∈ [0, 15]
//   azimuth_idx = round((azimuth + π) * (NUM_AZ-1) / (2π))           ∈ [0, 30]
//   flat_idx    = polar_idx * NUM_AZ + azimuth_idx
//
// 每 bin 取最小深度；< 0.3m 视为盲区 → 填 2.5m (no-hit max_distance, 与 sim 一致)。

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
        this->declare_parameter<std::string>("lidar_topic", "/LIDAR_SIM_RAW");
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, rclcpp::SensorDataQoS(),
            std::bind(&LidarToScanArrayNode::pc_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scan/multi_layer_features_array", 10);

        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LidarToScanArrayNode::log_diagnostics, this));

        RCLCPP_INFO(this->get_logger(),
            "✅ [Scan C++] Hemispherical 16×31 binning. Subscribing to: %s. Output dim: %d.",
            lidar_topic.c_str(), NUM_TOTAL);
    }

private:
    static constexpr int NUM_POLAR   = 16;
    static constexpr int NUM_AZ      = 31;
    static constexpr int NUM_PER_DIR = NUM_POLAR * NUM_AZ;   // 496
    static constexpr int NUM_TOTAL   = NUM_PER_DIR * 2;       // 992

    static constexpr float MAX_DIST       = 2.5f;
    static constexpr float MIN_DIST       = 0.3f;
    // 真 Airy 在 polar > 80° 数据极不可靠，sim 与 deploy 同步限制到 80°
    static constexpr float POLAR_MAX_DEG  = 80.0f;
    static constexpr float POLAR_MAX_RAD  = POLAR_MAX_DEG * static_cast<float>(M_PI) / 180.0f;
    static constexpr float TWO_PI         = static_cast<float>(2.0 * M_PI);

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        recv_count_++;
        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<float> fwd_bins(NUM_PER_DIR, MAX_DIST);
        std::vector<float> bwd_bins(NUM_PER_DIR, MAX_DIST);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float px = *iter_x;
            const float py = *iter_y;
            const float pz = *iter_z;
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

            // ===== 前向半球 (boresight = +X) =====
            {
                const float fx = px - fwd_offset_[0];
                const float fy = py - fwd_offset_[1];
                const float fz = pz - fwd_offset_[2];
                if (fx > 0.0f) {
                    const float r = std::sqrt(fx*fx + fy*fy + fz*fz);
                    if (r >= MIN_DIST && r <= MAX_DIST) {
                        const float perp  = std::sqrt(fy*fy + fz*fz);
                        const float polar = std::atan2(perp, fx);          // [0, π/2]
                        if (polar <= POLAR_MAX_RAD) {
                            const float az = std::atan2(fy, fz);            // [-π, π]
                            const int p_idx = clampi(
                                static_cast<int>(std::lround(polar * (NUM_POLAR - 1) / POLAR_MAX_RAD)),
                                0, NUM_POLAR - 1);
                            const int a_idx = clampi(
                                static_cast<int>(std::lround((az + static_cast<float>(M_PI)) * (NUM_AZ - 1) / TWO_PI)),
                                0, NUM_AZ - 1);
                            const int idx = p_idx * NUM_AZ + a_idx;
                            if (r < fwd_bins[idx]) fwd_bins[idx] = r;
                        }
                    }
                }
            }

            // ===== 后向半球 (boresight = -X)，对应 sim 后 sensor 的 180°Z 旋转 =====
            {
                const float bx = px - bwd_offset_[0];
                const float by = py - bwd_offset_[1];
                const float bz = pz - bwd_offset_[2];
                if (bx < 0.0f) {
                    const float r = std::sqrt(bx*bx + by*by + bz*bz);
                    if (r >= MIN_DIST && r <= MAX_DIST) {
                        const float perp  = std::sqrt(by*by + bz*bz);
                        const float polar = std::atan2(perp, -bx);          // 从 -X 算
                        if (polar <= POLAR_MAX_RAD) {
                            const float az = std::atan2(-by, bz);            // Y 取反 (180°Z mirror)
                            const int p_idx = clampi(
                                static_cast<int>(std::lround(polar * (NUM_POLAR - 1) / POLAR_MAX_RAD)),
                                0, NUM_POLAR - 1);
                            const int a_idx = clampi(
                                static_cast<int>(std::lround((az + static_cast<float>(M_PI)) * (NUM_AZ - 1) / TWO_PI)),
                                0, NUM_AZ - 1);
                            const int idx = p_idx * NUM_AZ + a_idx;
                            if (r < bwd_bins[idx]) bwd_bins[idx] = r;
                        }
                    }
                }
            }
        }

        // 盲区处理：< 0.3m 视为 no-hit (与 sim multi_layer_scan 一致)
        for (auto& v : fwd_bins) { if (v < MIN_DIST) v = MAX_DIST; }
        for (auto& v : bwd_bins) { if (v < MIN_DIST) v = MAX_DIST; }

        std_msgs::msg::Float32MultiArray array_msg;
        array_msg.data.reserve(NUM_TOTAL);
        array_msg.data.insert(array_msg.data.end(), fwd_bins.begin(), fwd_bins.end());
        array_msg.data.insert(array_msg.data.end(), bwd_bins.begin(), bwd_bins.end());
        pub_->publish(array_msg);

        pub_count_++;
        auto end_time = std::chrono::high_resolution_clock::now();
        total_time_ms_ += std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }

    static inline int clampi(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
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

    const float fwd_offset_[3] = {0.32028f, 0.0f, -0.013f};
    const float bwd_offset_[3] = {-0.32028f, 0.0f, -0.013f};

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
