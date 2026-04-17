/**
 * @file m20_policy_runner.hpp
 * @brief m20_policy_runner (Scheme B: ONNX with Explicit Hidden State)
 * + [新增] 影子感知 Debug 模式 (后台采集并预处理高程图与雷达点云，导出至 CSV)
 * @author Bo (Percy) Peng / DeepRobotics
 * @version 1.4 (Sim2Sim / Sim2Real Compatible + Data Logger)
 * @date 2025-02-17
 * @copyright Copyright (c) 2025 DeepRobotics
 */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"
#include <ctime>
#include <cmath>
#include <utility>
#include <vector>
#include <array>
#include <cstring>
#include <algorithm>
#include <fstream>   // 新增：用于文件写入
#include <iomanip>
#include <onnxruntime_cxx_api.h>

// === 新增：用于后台收集感知数据的依赖 ===
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <thread>
#include <deque>

class M20PolicyRunner : public PolicyRunnerBase {
private:
    // === 物理参数 ===
    VecXf kp_, kd_;
    VecXf dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f max_cmd_vel_, gravity_direction = Vec3f(0., 0., -1.);
    VecXf dof_pos_default_;
    timespec system_time;

    const int motor_num = 16;
    const int observation_dim = 57; // Blind Policy 只有 57 维
    const int action_dim = 16;
    float agent_timestep = 0.02;
    float current_time;
    bool is_fallen = true;

    // === 中间变量 ===
    VecXf joint_pos_rl = VecXf(action_dim);
    VecXf joint_vel_rl = VecXf(action_dim);
    const std::string policy_path_;

    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;
    VecXf imu_w_eigen, base_acc_eigen, motor_p_eigen, motor_v_eigen,
          current_action_eigen, last_action_eigen, current_observation_, projected_gravity,
          tmp_action_eigen;

    RobotAction robot_action;

    // === 关节映射 ===
    std::vector<std::string> robot_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};

    std::vector<std::string> policy_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
        "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
    };

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5};

    std::vector<int> robot2policy_idx, policy2robot_idx;

    // === ONNX Runtime & Hidden State ===
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::Env env_;
    Ort::MemoryInfo memory_info{nullptr};

    const char* input_names_[2] = {"obs", "hidden_state"};
    const char* output_names_[2] = {"action", "next_hidden"};

    const std::array<int64_t, 2> input_observationShape = {1, observation_dim};
    const int hidden_dim = 256;
    const std::array<int64_t, 3> hidden_state_shape = {1, 1, hidden_dim};
    std::vector<float> hidden_state_data_;

    VecXf command;
    float time_step = 0.;
    int stop_count = 1000;

    // ====================================================================
    // === 新增：影子模式 (Shadow Mode) 感知预处理组件 ===
    // ====================================================================
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::thread ros_spin_thread_;

    std::mutex map_mutex_;
    grid_map_msgs::msg::GridMap latest_map_;
    bool map_received_ = false;

    std::mutex scan_mutex_;
    std::vector<float> fwd_scan_bins_;
    std::vector<float> bwd_scan_bins_;
    bool scan_received_ = false;

    // 新增：数据记录器相关变量
    std::ofstream data_log_file_;
    const std::string log_file_name_ = "shadow_perception_log.csv";

public:
    M20PolicyRunner(const std::string &policy_name, const std::string &policy_path) :
            PolicyRunnerBase(policy_name), policy_path_(policy_path),
            env_(ORT_LOGGING_LEVEL_WARNING, "M20PolicyRunner"),
            session_options_{},
            session_{nullptr},
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {

        // 1. 初始化默认姿态
        dof_default_eigen_policy.setZero(action_dim);
        dof_default_eigen_robot.setZero(action_dim);
        dof_default_eigen_policy << 0.0, -0.6,  1.0,
                                    0.0, -0.6,  1.0,
                                    0.0,  0.6, -1.0,
                                    0.0,  0.6, -1.0,
                                    0.0, 0.0, 0.0, 0.0;
        dof_default_eigen_robot << 0.0, -0.6,  1.0, 0.0,
                                   0.0, -0.6,  1.0, 0.0,
                                   0.0,  0.6, -1.0, 0.0,
                                   0.0,  0.6, -1.0, 0.0;

        // 2. 配置 ONNX Runtime
        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "Model file not found: " << policy_path_ << std::endl;
            throw std::runtime_error("Model file missing");
        }

        // 3. 加载模型
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        std::cout << "[M20PolicyRunner] Loaded ONNX model from: " << policy_path_ << std::endl;

        // 4. 初始化 Hidden State
        hidden_state_data_.resize(hidden_dim, 0.0f);

        // 5. 初始化 PID 和 映射
        kp_ = Vec4f(80, 80, 80, 0.).replicate(4, 1);
        kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);

        robot2policy_idx = generate_permutation(robot_order, policy_order);
        policy2robot_idx = generate_permutation(policy_order, robot_order);

        robot_action.kp = kp_;
        robot_action.kd = kd_;
        robot_action.tau_ff = VecXf::Zero(motor_num);
        robot_action.goal_joint_pos = VecXf::Zero(motor_num);
        robot_action.goal_joint_vel = VecXf::Zero(motor_num);

        current_observation_.setZero(observation_dim);
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        current_action_eigen.setZero(action_dim);

        // ====================================================================
        // === 新增：初始化数据记录器 CSV 文件 ===
        // ====================================================================
        data_log_file_.open(log_file_name_);
        if (data_log_file_.is_open()) {
            // 写入表头
            data_log_file_ << "step,time,robot_x,robot_y,robot_z,robot_yaw,valid_h_count,min_fwd,min_bwd";
            for(int i = 0; i < 187; ++i) data_log_file_ << ",h_" << i;
            for(int i = 0; i < 126; ++i) data_log_file_ << ",fwd_" << i;
            for(int i = 0; i < 126; ++i) data_log_file_ << ",bwd_" << i;
            data_log_file_ << "\n";
            std::cout << "✅ [Shadow Logger] Data will be saved to: " << log_file_name_ << std::endl;
        } else {
            std::cerr << "❌ [Shadow Logger] Failed to open file: " << log_file_name_ << std::endl;
        }

        // ====================================================================
        // === 初始化影子感知节点与接收队列 ===
        // ====================================================================
        fwd_scan_bins_.resize(126, 5.0f);
        bwd_scan_bins_.resize(126, 5.0f);

        ros_node_ = rclcpp::Node::make_shared("m20_shadow_perception_node");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅高程图
        grid_map_sub_ = ros_node_->create_subscription<grid_map_msgs::msg::GridMap>(
        "/elevation_map_remote", rclcpp::SensorDataQoS(),
            [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                latest_map_ = *msg;
                map_received_ = true;
            });

        // 订阅雷达 (直接复用感知 Policy 里的完整预处理逻辑)
        lidar_sub_ = ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_SIM_RAW", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::vector<float> local_fwd_bins(126, 5.0f);
                std::vector<float> local_bwd_bins(126, 5.0f);

                int x_offset_idx = -1, y_offset_idx = -1, z_offset_idx = -1;
                for (const auto& field : msg->fields) {
                    if (field.name == "x") x_offset_idx = field.offset;
                    if (field.name == "y") y_offset_idx = field.offset;
                    if (field.name == "z") z_offset_idx = field.offset;
                }
                if (x_offset_idx == -1 || y_offset_idx == -1 || z_offset_idx == -1) return;

                const float target_angles[6] = {-25.0f, -15.0f, -5.0f, 5.0f, 15.0f, 25.0f};
                const float angle_tol = 4.0f;
                const float dy_step = 0.05f;
                const float y_min = -0.5f;
                const int num_rays = 21;

                const float fwd_x_offset = 0.32028f;
                const float bwd_x_offset = -0.32028f;
                const float z_offset = -0.013f;

                for (size_t i = 0; i < msg->data.size(); i += msg->point_step) {
                    float x, y, z;
                    memcpy(&x, &msg->data[i + x_offset_idx], sizeof(float));
                    memcpy(&y, &msg->data[i + y_offset_idx], sizeof(float));
                    memcpy(&z, &msg->data[i + z_offset_idx], sizeof(float));

                    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

                    // 1. 前向 Lidar
                    float fwd_dx = x - fwd_x_offset;
                    float fwd_dy = y;
                    float fwd_dz = z - z_offset;

                    if (fwd_dx > 0.0f && fwd_dy >= y_min - dy_step/2.0f && fwd_dy <= -y_min + dy_step/2.0f) {
                        float angle_deg = std::atan2(-fwd_dz, fwd_dx) * 180.0f / M_PI;
                        for (int l = 0; l < 6; ++l) {
                            if (std::abs(angle_deg - target_angles[l]) <= angle_tol) {
                                int bin = std::round((fwd_dy - y_min) / dy_step);
                                if (bin >= 0 && bin < num_rays) {
                                    float r = std::sqrt(fwd_dx*fwd_dx + fwd_dy*fwd_dy + fwd_dz*fwd_dz);
                                    if (r < local_fwd_bins[l * num_rays + bin]) {
                                        local_fwd_bins[l * num_rays + bin] = r;
                                    }
                                }
                            }
                        }
                    }

                    // 2. 后向 Lidar
                    float bwd_dx = x - bwd_x_offset;
                    float bwd_dy = y;
                    float bwd_dz = z - z_offset;

                    if (bwd_dx < 0.0f && bwd_dy >= y_min - dy_step/2.0f && bwd_dy <= -y_min + dy_step/2.0f) {
                        float angle_deg = std::atan2(-bwd_dz, -bwd_dx) * 180.0f / M_PI;
                        for (int l = 0; l < 6; ++l) {
                            if (std::abs(angle_deg - target_angles[l]) <= angle_tol) {
                                int bin = std::round((-bwd_dy - y_min) / dy_step);
                                if (bin >= 0 && bin < num_rays) {
                                    float r = std::sqrt(bwd_dx*bwd_dx + bwd_dy*bwd_dy + bwd_dz*bwd_dz);
                                    if (r < local_bwd_bins[l * num_rays + bin]) {
                                        local_bwd_bins[l * num_rays + bin] = r;
                                    }
                                }
                            }
                        }
                    }
                }

                std::lock_guard<std::mutex> lock(scan_mutex_);
                fwd_scan_bins_ = local_fwd_bins;
                bwd_scan_bins_ = local_bwd_bins;
                scan_received_ = true;
            });

        // 启动后台 ROS Spinner
        ros_spin_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });
    }

    ~M20PolicyRunner() override {
        rclcpp::shutdown();
        if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

        if (data_log_file_.is_open()) {
            data_log_file_.close();
        }
    }

    std::vector<int> generate_permutation(const std::vector<std::string>& from, const std::vector<std::string>& to, int default_index = 0) {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < from.size(); ++i) idx_map[from[i]] = i;
        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) perm.push_back(it->second);
            else perm.push_back(default_index);
        }
        return perm;
    }

    void OnEnter(const RobotBasicState &rbs) override {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);

        std::fill(hidden_state_data_.begin(), hidden_state_data_.end(), 0.0f);
        std::cout << "[M20PolicyRunner] Reset hidden state." << std::endl;
    }

    VecXf Onnx_infer(VecXf current_observation){
        std::vector<Ort::Value> input_tensors;
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, current_observation.data(), current_observation.size(), input_observationShape.data(), input_observationShape.size()));
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, hidden_state_data_.data(), hidden_state_data_.size(), hidden_state_shape.data(), hidden_state_shape.size()));

        auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, input_tensors.data(), 2, output_names_, 2);

        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, action_dim);

        float* next_hidden_data = outputs[1].GetTensorMutableData<float>();
        std::memcpy(hidden_state_data_.data(), next_hidden_data, hidden_dim * sizeof(float));

        return VecXf(action_map);
    }

    // ====================================================================
    // === 新增：影子数据收集函数 (计算、记录文件，但不喂给网络) ===
    // ====================================================================
    void CollectShadowPerceptionData() {
        std::vector<float> curr_heights(187, 0.0f);
        float robot_x = 0, robot_y = 0, robot_z = 0.45f, robot_yaw = 0;

        // 1. TF 获取
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            robot_x = t.transform.translation.x; robot_y = t.transform.translation.y; robot_z = t.transform.translation.z;
            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
            robot_yaw = yaw;
        } catch (const tf2::TransformException & ex) { }

        // 2. 地图采样
        grid_map_msgs::msg::GridMap local_map_msg;
        bool has_map = false;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (map_received_) {
                local_map_msg = latest_map_;
                has_map = true;
            }
        }

        grid_map::GridMap local_grid_map;
        bool map_converted = false;
        if (has_map) {
            map_converted = grid_map::GridMapRosConverter::fromMessage(local_map_msg, local_grid_map);
        }

        int env_idx = 0;
        int valid_h_count = 0; // 统计有效点数量
        for (float dy = -0.5f; dy <= 0.5f + 1e-5; dy += 0.1f) {
            for (float dx = -0.8f; dx <= 0.8f + 1e-5; dx += 0.1f) {
                if (map_converted && local_grid_map.exists("elevation")) {
                    float query_x = robot_x + (dx * std::cos(robot_yaw) - dy * std::sin(robot_yaw));
                    float query_y = robot_y + (dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw));
                    grid_map::Position pos(query_x, query_y);
                    if (local_grid_map.isInside(pos)) {
                        float h = local_grid_map.atPosition("elevation", pos);
                        if (!std::isnan(h)) {
                            curr_heights[env_idx] = std::clamp(robot_z - h - 0.5f, -1.0f, 1.0f);
                            valid_h_count++;
                        }
                    }
                }
                env_idx++;
            }
        }

        // 3. 雷达状态
        std::vector<float> curr_fwd_bins(126, 5.0f);
        std::vector<float> curr_bwd_bins(126, 5.0f);
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (scan_received_) {
                curr_fwd_bins = fwd_scan_bins_;
                curr_bwd_bins = bwd_scan_bins_;
            }
        }

        float min_fwd = 5.0f, min_bwd = 5.0f;
        for(float d : curr_fwd_bins) min_fwd = std::min(min_fwd, d);
        for(float d : curr_bwd_bins) min_bwd = std::min(min_bwd, d);

        // 4. 将提取好的数据写入 CSV 文件
        if (data_log_file_.is_open()) {
            data_log_file_ << run_cnt_ << "," << getCurrentTime() << ","
                           << robot_x << "," << robot_y << "," << robot_z << "," << robot_yaw << ","
                           << valid_h_count << "," << min_fwd << "," << min_bwd;

            // 依次追加高度图和雷达数组
            for (int i = 0; i < 187; ++i) data_log_file_ << "," << curr_heights[i];
            for (int i = 0; i < 126; ++i) data_log_file_ << "," << curr_fwd_bins[i];
            for (int i = 0; i < 126; ++i) data_log_file_ << "," << curr_bwd_bins[i];

            data_log_file_ << "\n"; // 换行，利用 C++ ofstream 自身缓冲机制，避免阻塞
        }
    }

    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {

        Vec3f base_omgea = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        for (int i = 0; i < action_dim; ++i){
            joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
            joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
        }

        joint_pos_rl.segment(12, 4).setZero();
        joint_pos_rl -= dof_default_eigen_policy;

        current_observation_ << base_omgea,
                              projected_gravity,
                              command,
                              joint_pos_rl,
                              joint_vel_rl,
                              last_action_eigen;

        current_action_eigen = Onnx_infer(current_observation_);
        last_action_eigen = current_action_eigen;

        for (int i = 0; i < action_dim; ++i){
            tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);
            tmp_action_eigen(i) *= action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;

        for (int i = 0; i < 4; ++i){
            robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
            robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
        }

        // ====================================================================
        // === 触发影子模式：记录所有数据到本地 csv，不影响盲步控制 ===
        // ====================================================================
        CollectShadowPerceptionData();

        ++run_cnt_;
        ++time_step;
        return robot_action;
    }

    void setDefaultJointPos(const VecXf& pos){
        dof_pos_default_.setZero(motor_num); 
        for(int i=0;i<motor_num;++i) {
            dof_pos_default_(i) = pos(i);
        }
    }

    double getCurrentTime() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    }
};