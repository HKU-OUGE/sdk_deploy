#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class NoisyElevationScanOnlyNode : public rclcpp::Node {
public:
    NoisyElevationScanOnlyNode() : Node("noisy_elevation_scan_only_node") {
        declare_parameter<std::string>("lidar_topic", "/CLOUD_REGISTERED_BODY");
        declare_parameter<std::string>("output_topic", "/perception/noisy_elevation_scan_only_array");
        declare_parameter<double>("pitch_min_deg", -60.0);
        declare_parameter<double>("pitch_max_deg", 10.0);
        declare_parameter<double>("az_min_deg", -45.0);
        declare_parameter<double>("az_max_deg", 45.0);

        lidar_topic_ = get_parameter("lidar_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        pitch_min_deg_ = static_cast<float>(get_parameter("pitch_min_deg").as_double());
        pitch_max_deg_ = static_cast<float>(get_parameter("pitch_max_deg").as_double());
        az_min_deg_ = static_cast<float>(get_parameter("az_min_deg").as_double());
        az_max_deg_ = static_cast<float>(get_parameter("az_max_deg").as_double());
        if (pitch_max_deg_ <= pitch_min_deg_) {
            throw std::runtime_error("pitch_max_deg must be greater than pitch_min_deg");
        }
        if (az_max_deg_ <= az_min_deg_) {
            throw std::runtime_error("az_max_deg must be greater than az_min_deg");
        }
        pitch_step_deg_ = (pitch_max_deg_ - pitch_min_deg_) / static_cast<float>(NUM_PITCH - 1);
        az_step_deg_ = (az_max_deg_ - az_min_deg_) / static_cast<float>(NUM_AZ - 1);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&NoisyElevationScanOnlyNode::pointCloudCallback, this, std::placeholders::_1));
        pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, 10);
        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NoisyElevationScanOnlyNode::logDiagnostics, this));

        RCLCPP_INFO(
            get_logger(),
            "scan-only noisy_elevation: lidar=%s output=%s dim=%d pitch=[%.1f, %.1f] az=[%.1f, %.1f]",
            lidar_topic_.c_str(), output_topic_.c_str(), NOISY_ELEV_DIM,
            pitch_min_deg_, pitch_max_deg_, az_min_deg_, az_max_deg_);
    }

private:
    static constexpr int NUM_PITCH = 12;
    static constexpr int NUM_AZ = 21;
    static constexpr int NUM_PER_DIR = NUM_PITCH * NUM_AZ;
    static constexpr int HEIGHT_DIM = 187;
    static constexpr int NOISY_ELEV_DIM = HEIGHT_DIM + NUM_PER_DIR * 2;
    static constexpr float MAX_DIST = 2.5f;
    static constexpr float MIN_DIST = 0.3f;
    static constexpr float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        recv_count_++;
        const auto start = std::chrono::high_resolution_clock::now();

        std::vector<float> fwd(NUM_PER_DIR, MAX_DIST);
        std::vector<float> bwd(NUM_PER_DIR, MAX_DIST);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float px = *iter_x;
            const float py = *iter_y;
            const float pz = *iter_z;
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

            binPoint(px - FWD_OFFSET_X, py, pz - OFFSET_Z, +1, fwd);
            binPoint(px - BWD_OFFSET_X, py, pz - OFFSET_Z, -1, bwd);
        }

        std_msgs::msg::Float32MultiArray out;
        out.data.reserve(NOISY_ELEV_DIM);
        out.data.insert(out.data.end(), HEIGHT_DIM, 0.0f);
        appendNormalized(fwd, out.data);
        appendNormalized(bwd, out.data);
        pub_->publish(out);

        last_fwd_min_ = minDepth(fwd);
        last_bwd_min_ = minDepth(bwd);
        last_fwd_hits_ = countHits(fwd);
        last_bwd_hits_ = countHits(bwd);

        pub_count_++;
        const auto end = std::chrono::high_resolution_clock::now();
        total_time_ms_ += std::chrono::duration<double, std::milli>(end - start).count();
    }

    void binPoint(float x, float y, float z, int boresight_sign, std::vector<float>& bins) const {
        float bx = x;
        float by = y;
        float bz = z;
        if (boresight_sign < 0) {
            bx = -x;
            by = -y;
        }
        if (bx <= 1e-6f) return;

        const float r = std::sqrt(bx * bx + by * by + bz * bz);
        if (r <= 1e-6f || r > MAX_DIST) return;

        const float pitch = std::asin(std::clamp(bz / r, -1.0f, 1.0f)) * RAD_TO_DEG;
        const float az = std::atan2(by, bx) * RAD_TO_DEG;
        if (pitch < pitch_min_deg_ || pitch > pitch_max_deg_ || az < az_min_deg_ || az > az_max_deg_) return;

        const int p_idx = clampi(
            static_cast<int>(std::lround((pitch - pitch_min_deg_) / pitch_step_deg_)),
            0, NUM_PITCH - 1);
        const int a_idx = clampi(
            static_cast<int>(std::lround((az - az_min_deg_) / az_step_deg_)),
            0, NUM_AZ - 1);
        const int idx = p_idx * NUM_AZ + a_idx;
        if (r < bins[idx]) bins[idx] = r;
    }

    static void appendNormalized(const std::vector<float>& depth, std::vector<float>& out) {
        for (float d : depth) {
            if (d < MIN_DIST) {
                out.push_back(1.0f);
            } else {
                out.push_back(std::clamp(d / MAX_DIST, 0.0f, 1.0f));
            }
        }
    }

    static int clampi(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    void logDiagnostics() {
        if (recv_count_ > 0) {
            const double avg = total_time_ms_ / recv_count_;
            RCLCPP_INFO(
                get_logger(),
                "[scan-only noisy_elev] recv=%dHz pub=%dHz avg=%.3fms fwd_min=%.2fm bwd_min=%.2fm hits=%d/%d topic=%s",
                recv_count_, pub_count_, avg, last_fwd_min_, last_bwd_min_, last_fwd_hits_, last_bwd_hits_,
                output_topic_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "[scan-only noisy_elev] waiting for %s", lidar_topic_.c_str());
        }
        recv_count_ = 0;
        pub_count_ = 0;
        total_time_ms_ = 0.0;
    }

    static constexpr float FWD_OFFSET_X = 0.32028f;
    static constexpr float BWD_OFFSET_X = -0.32028f;
    static constexpr float OFFSET_Z = -0.013f;

    static float minDepth(const std::vector<float>& depth) {
        float v = MAX_DIST;
        for (float d : depth) v = std::min(v, d);
        return v;
    }

    static int countHits(const std::vector<float>& depth) {
        int n = 0;
        for (float d : depth) {
            if (d < MAX_DIST) ++n;
        }
        return n;
    }

    std::string lidar_topic_;
    std::string output_topic_;
    float pitch_min_deg_ = -60.0f;
    float pitch_max_deg_ = 10.0f;
    float az_min_deg_ = -45.0f;
    float az_max_deg_ = 45.0f;
    float pitch_step_deg_ = 1.0f;
    float az_step_deg_ = 1.0f;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    int recv_count_ = 0;
    int pub_count_ = 0;
    double total_time_ms_ = 0.0;
    float last_fwd_min_ = MAX_DIST;
    float last_bwd_min_ = MAX_DIST;
    int last_fwd_hits_ = 0;
    int last_bwd_hits_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyElevationScanOnlyNode>());
    rclcpp::shutdown();
    return 0;
}
