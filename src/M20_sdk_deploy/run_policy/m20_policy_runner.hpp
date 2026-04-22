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
#include <std_msgs/msg/float32_multi_array.hpp>
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
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr scan_array_sub_;
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
    std::string log_file_name_;

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

        // 5. 加载速度配置
        LoadVelocityConfig(policy_path_);

        // 6. 初始化 PID 和 映射
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
        // === 新增：初始化数据记录器 CSV 文件 (带时间戳) ===
        // ====================================================================
        std::time_t t = std::time(nullptr);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));
        log_file_name_ = "shadow_perception_log_" + std::string(time_str) + ".csv";

        data_log_file_.open(log_file_name_);
        if (data_log_file_.is_open()) {
            // 写入表头 (新增 hole_ratio_pct)
            data_log_file_ << "step,time,robot_x,robot_y,robot_z,robot_yaw,valid_h_count,hole_ratio_pct,min_fwd,min_bwd";
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

        // 订阅雷达
        scan_array_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/scan/multi_layer_features_array", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                // 安全检查：必须是前向 126 + 后向 126 = 252 个特征
                if (msg->data.size() != 252) return;
                
                std::lock_guard<std::mutex> lock(scan_mutex_);
                // 直接内存拷贝前 126 个给前向，后 126 个给后向
                std::copy(msg->data.begin(), msg->data.begin() + 126, fwd_scan_bins_.begin());
                std::copy(msg->data.begin() + 126, msg->data.end(), bwd_scan_bins_.begin());
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
        current_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);

        std::fill(hidden_state_data_.begin(), hidden_state_data_.end(), 0.0f);
        dbg_ = DebugState{};
        dashboard_lines_ = 0;
        std::cout << "[M20PolicyRunner] Reset hidden state & FSM buffers." << std::endl;
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
    // === 影子数据收集函数 (计算、记录文件，但不喂给网络) ===
    // ====================================================================
    void CollectShadowPerceptionData(const RobotBasicState &ro) {
        // ==========================================================
        // === 智能位姿获取：优先 TF，失败则降级使用地图中心 ===
        // ==========================================================
        std::vector<float> curr_heights(187, 0.0f);
        float robot_x = 0, robot_y = 0, robot_z = 0.45f;
        float robot_yaw = ro.base_rpy(2);
        bool tf_valid = false;

        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            robot_x = t.transform.translation.x; 
            robot_y = t.transform.translation.y; 
            robot_z = t.transform.translation.z;
            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
            robot_yaw = yaw;
            tf_valid = true;
        } catch (const tf2::TransformException & ex) { 
            if (run_cnt_ % 50 == 0) {
                std::cerr << "⚠️ [Shadow TF Warning] 无 TF，将启用降级方案" << std::endl;
            }
        }

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
            
            // 【核心】：如果 TF 失败（真机环境），启用“四足正向运动学 + 地图联合采样”
            if (!tf_valid) {
                grid_map::Position map_center = local_grid_map.getPosition();
                robot_x = map_center.x();
                robot_y = map_center.y();

                // ==========================================================
                // === 高精度运动学高度补偿 (4-Point Leg Odometry) ===
                // ==========================================================
                float estimated_z_sum = 0.0f;
                int valid_feet_count = 0;

                // 足端相对于基座中心的近似初始水平偏移
                float base_x_offsets[4] = {0.31f, 0.31f, -0.31f, -0.31f};
                float base_y_offsets[4] = {0.17f, -0.17f, 0.17f, -0.17f};
                int hipy_indices[4] = {1, 5, 9, 13};
                int knee_indices[4] = {2, 6, 10, 14};

                for (int i = 0; i < 4; ++i) {
                    float q_hipy = ro.joint_pos(hipy_indices[i]);
                    float q_knee = ro.joint_pos(knee_indices[i]);
                    
                    // 1. 修复1：使用 0.072f 补偿轮毂半径和基座中心 Z 偏移
                    float z_drop = 0.25f * std::cos(q_hipy) + 0.25f * std::cos(q_hipy + q_knee) + 0.086f;
                    
                    // 2. 修复2：关节角度已自带正确方向，绝对不能对后腿的 X 偏移取反！
                    float x_shift = 0.25f * std::sin(q_hipy) + 0.25f * std::sin(q_hipy + q_knee);
                    
                    Eigen::Vector3f foot_local(base_x_offsets[i] + x_shift, base_y_offsets[i], -z_drop);
                    Eigen::Vector3f foot_world_offset = ro.base_rot_mat * foot_local;
                    
                    // 3. 查询该脚踩着的真实地面高度
                    grid_map::Position foot_pos(robot_x + foot_world_offset.x(), robot_y + foot_world_offset.y());
                    if (local_grid_map.isInside(foot_pos)) {
                        float foot_ground_h = local_grid_map.atPosition("elevation", foot_pos);
                        if (!std::isnan(foot_ground_h)) {
                            // 4. Z_base = 脚底地面高度 - 脚相对于Base在世界Z轴的偏移
                            estimated_z_sum += (foot_ground_h - foot_world_offset.z());
                            valid_feet_count++;
                        }
                    }
                }

                if (valid_feet_count > 0) {
                    robot_z = estimated_z_sum / valid_feet_count;
                } else {
                    if (local_grid_map.exists("elevation") && local_grid_map.isInside(map_center)) {
                        float center_ground_h = local_grid_map.atPosition("elevation", map_center);
                        if (!std::isnan(center_ground_h)) robot_z = center_ground_h + 0.45f;
                    }
                }
                // ==========================================================
            }


            if (run_cnt_ % 50 == 0) {
                dbg_.robot_x = robot_x;
                dbg_.robot_y = robot_y;
                dbg_.robot_z = robot_z;
                dbg_.robot_yaw = robot_yaw;
                dbg_.tf_valid = tf_valid;
                dbg_.has_map = true;
            }
        }


        // ==========================================================
        // === 运动学 Z 轴补偿对比测试 (仅在有 TF 时打印验证) ===
        // ==========================================================
        if (map_converted && run_cnt_ % 10 == 0 && tf_valid) {
            float estimated_z_sum = 0.0f;
            int valid_feet_count = 0;

            float base_x_offsets[4] = {0.31f, 0.31f, -0.31f, -0.31f};
            float base_y_offsets[4] = {0.17f, -0.17f, 0.17f, -0.17f};
            int hipy_indices[4] = {1, 5, 9, 13};
            int knee_indices[4] = {2, 6, 10, 14};

            for (int i = 0; i < 4; ++i) {
                float q_hipy = ro.joint_pos(hipy_indices[i]);
                float q_knee = ro.joint_pos(knee_indices[i]);
                float z_drop = 0.25f * std::cos(q_hipy) + 0.25f * std::cos(q_hipy + q_knee) + 0.086f;
                float x_shift = 0.25f * std::sin(q_hipy) + 0.25f * std::sin(q_hipy + q_knee);
                Eigen::Vector3f foot_local(base_x_offsets[i] + x_shift, base_y_offsets[i], -z_drop);
                Eigen::Vector3f foot_world_offset = ro.base_rot_mat * foot_local;
                
                // 注意这里使用的也是 map_center 来模拟没 TF 时的场景
                grid_map::Position map_center = local_grid_map.getPosition();
                grid_map::Position foot_pos(map_center.x() + foot_world_offset.x(), map_center.y() + foot_world_offset.y());
                
                if (local_grid_map.isInside(foot_pos)) {
                    float foot_ground_h = local_grid_map.atPosition("elevation", foot_pos);
                    if (!std::isnan(foot_ground_h)) {
                        estimated_z_sum += (foot_ground_h - foot_world_offset.z());
                        valid_feet_count++;
                    }
                }
            }

            if (valid_feet_count > 0) {
                float est_z = estimated_z_sum / valid_feet_count;
                float z_error = robot_z - est_z;
                dbg_.fk_z = est_z;
                dbg_.z_error = z_error;
                dbg_.has_fk = true;
            }
        }
        // ==========================================================

        int env_idx = 0;
        int valid_h_count = 0; // === 新增：统计有效高度点 ===
        // 这里才是 187 个点的采样循环，里面不要放打印！
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
                            valid_h_count++; // 累加有效点
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
            float hole_ratio_pct = (187.0f - valid_h_count) / 187.0f * 100.0f;

            if (run_cnt_ % 50 == 0) {
                dbg_.hole_ratio_pct = hole_ratio_pct;
                dbg_.valid_h_count = valid_h_count;
                dbg_.min_fwd = min_fwd;
                dbg_.min_bwd = min_bwd;
            }
            data_log_file_ << run_cnt_ << "," << getCurrentTime() << ","
                           << robot_x << "," << robot_y << "," << robot_z << "," << robot_yaw << ","
                           << valid_h_count << "," << hole_ratio_pct << "," << min_fwd << "," << min_bwd;

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
        Vec3f command = ClampCommand(Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale));

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
        CollectShadowPerceptionData(ro);

        dbg_.cmd_vx = command(0);
        dbg_.cmd_vy = command(1);
        dbg_.cmd_wz = command(2);
        if (run_cnt_ % 50 == 0) {
            PrintDebugDashboard("Blind", run_cnt_);
        }

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