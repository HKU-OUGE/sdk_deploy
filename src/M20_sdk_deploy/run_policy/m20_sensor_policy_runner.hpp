#pragma once

#include "policy_runner_base.hpp"
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <grid_map_ros/grid_map_ros.hpp>

class M20SensorPolicyRunner : public PolicyRunnerBase {
private:
    VecXf kp_, kd_, dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f gravity_direction = Vec3f(0., 0., -1.);
    const int motor_num = 16, action_dim = 16;
    float omega_scale_ = 0.25, dof_vel_scale_ = 0.05;

    // 网络维度定义 (严格对齐导出日志)
    const int proprio_dim_ = 57;
    const int elevation_dim_ = 187;
    const int proprio_env_dim_ = 244;  // 57 + 187
    const int history_len_ = 15;
    const int estimator_dim_ = 855;    // 57 * 15
    const int hidden_dim_ = 256;       // 假设 GRU 隐藏层为 256

    // ONNX 输入输出张量
    std::vector<float> proprio_env_data_;
    std::vector<float> estimator_history_data_;
    std::vector<float> hidden_state_data_;

    // 输入历史队列
    std::deque<std::vector<float>> history_buffer_;
    bool is_first_step_ = true;
    // 输入张量 Shape
    std::array<int64_t, 2> shape_proprio_env_ = {1, proprio_env_dim_};
    std::array<int64_t, 2> shape_estimator_   = {1, estimator_dim_};
    std::array<int64_t, 3> shape_h0_          = {1, 1, hidden_dim_}; // PyTorch GRU 默认要求 3D shape

    VecXf joint_pos_rl = VecXf(action_dim), joint_vel_rl = VecXf(action_dim);
    VecXf current_action_eigen, last_action_eigen, tmp_action_eigen;
    RobotAction robot_action;

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

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25, 5, 0.125, 0.25, 0.25, 5, 0.125, 0.25, 0.25, 5, 0.125, 0.25, 0.25, 5};
    std::vector<int> robot2policy_idx, policy2robot_idx;

    // ONNX 核心
    Ort::SessionOptions session_options_;
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info{nullptr};

    // const char* input_names_[3] = {"proprio_and_env", "estimator_history", "h0"};
    const char* input_names_[2] = {"proprio_and_env", "h0"};
    const char* output_names_[2] = {"action", "next_h0"};

    // ROS 2 高程图相关
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::thread ros_spin_thread_;
    std::mutex map_mutex_;
    grid_map_msgs::msg::GridMap latest_map_;
    bool map_received_ = false;

public:
    M20SensorPolicyRunner(const std::string &policy_name, const std::string &policy_path) :
            PolicyRunnerBase(policy_name), env_(ORT_LOGGING_LEVEL_WARNING, "M20SensorPolicyRunner"),
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {

        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, policy_path.c_str(), session_options_);

        std::cout << "[M20SensorPolicyRunner] Loaded Unified MoE Policy successfully." << std::endl;

        // 初始化数据容器
        proprio_env_data_.resize(proprio_env_dim_, 0.0f);
        estimator_history_data_.resize(estimator_dim_, 0.0f);
        hidden_state_data_.resize(hidden_dim_, 0.0f);

        for (int i = 0; i < history_len_; ++i) {
            history_buffer_.push_back(std::vector<float>(proprio_dim_, 0.0f));
        }

        // 初始化增益与位姿
        kp_ = Vec4f(80, 80, 80, 0.).replicate(4, 1);
        kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
        robot_action.kp = kp_; robot_action.kd = kd_;
        robot_action.tau_ff = VecXf::Zero(motor_num);
        robot_action.goal_joint_pos = VecXf::Zero(motor_num);
        robot_action.goal_joint_vel = VecXf::Zero(motor_num);

        robot2policy_idx.resize(16); policy2robot_idx.resize(16);
        for(int i=0; i<16; i++) {
            robot2policy_idx[i] = std::distance(policy_order.begin(), std::find(policy_order.begin(), policy_order.end(), robot_order[i]));
            policy2robot_idx[i] = std::distance(robot_order.begin(), std::find(robot_order.begin(), robot_order.end(), policy_order[i]));
        }
        dof_default_eigen_policy = VecXf::Zero(16); dof_default_eigen_robot = VecXf::Zero(16);
        dof_default_eigen_policy << 0,-0.6,1, 0,-0.6,1, 0,0.6,-1, 0,0.6,-1, 0,0,0,0;
        dof_default_eigen_robot << 0,-0.6,1,0, 0,-0.6,1,0, 0,0.6,-1,0, 0,0.6,-1,0;
        last_action_eigen.setZero(16); tmp_action_eigen.setZero(16); current_action_eigen.setZero(16);

        // ================= ROS 2 节点与订阅 =================
        ros_node_ = rclcpp::Node::make_shared("m20_elevation_subscriber");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        grid_map_sub_ = ros_node_->create_subscription<grid_map_msgs::msg::GridMap>(
            "/elevation_mapping_node/elevation_map_raw", rclcpp::SensorDataQoS(),
            [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                latest_map_ = *msg;
                map_received_ = true;
            });

        ros_spin_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });
    }

    ~M20SensorPolicyRunner() {
        rclcpp::shutdown();
        if (ros_spin_thread_.joinable()) ros_spin_thread_.join();
    }

    void OnEnter(const RobotBasicState &rbs) override {
        std::fill(hidden_state_data_.begin(), hidden_state_data_.end(), 0.0f);
        last_action_eigen.setZero(16);
        is_first_step_ = true;
    }


RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        Vec3f base_omega = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        for (int i = 0; i < action_dim; ++i) {
            joint_pos_rl(i) = ro.joint_pos(policy2robot_idx[i]);
            joint_vel_rl(i) = ro.joint_vel(policy2robot_idx[i]) * dof_vel_scale_;
        }
        joint_pos_rl.segment(12, 4).setZero();
        joint_pos_rl -= dof_default_eigen_policy;

        // ====== 1. 组装 57 维的 Proprio 本体感知 ======
        std::vector<float> curr_proprio(proprio_dim_);
        Eigen::Map<Eigen::VectorXf>(curr_proprio.data(), proprio_dim_) << base_omega, projected_gravity, command, joint_pos_rl, joint_vel_rl, last_action_eigen;
        std::copy(curr_proprio.begin(), curr_proprio.end(), proprio_env_data_.begin());

        // ====== 2. 更新并组装 Estimator History (855 维) ======
        if (is_first_step_) {
            for (auto& frame : history_buffer_) {
                std::copy(curr_proprio.begin(), curr_proprio.end(), frame.begin());
            }
            is_first_step_ = false;
        } else {
            history_buffer_.pop_back();
            history_buffer_.push_front(curr_proprio); 
        }

        // 从最老 (t-14) 到最新 (t)，严格对齐 IsaacLab 的 shifts=-1
        int offset = 0;
        for (int i = history_len_ - 1; i >= 0; --i) {
            std::copy(history_buffer_[i].begin(), history_buffer_[i].end(), estimator_history_data_.begin() + offset);
            offset += proprio_dim_;
        }

        // ====== 3. 采样 187 维高程网格 ======
        float robot_x = 0, robot_y = 0, robot_z = 0.45f, robot_yaw = 0;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            robot_x = t.transform.translation.x; robot_y = t.transform.translation.y; robot_z = t.transform.translation.z;
            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
            robot_yaw = yaw;
        } catch (const tf2::TransformException & ex) {
            // 获取不到 TF 时，默认高度保持 0.45f
        }


        float default_relative_height = -1.0f;

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

        int env_idx = proprio_dim_;
        for (float dy = -0.5f; dy <= 0.5f + 1e-5; dy += 0.1f) {
            for (float dx = -0.8f; dx <= 0.8f + 1e-5; dx += 0.1f) {
                if (env_idx < proprio_env_dim_) {
                    float relative_height = default_relative_height; 
                    
                    if (map_converted && local_grid_map.exists("elevation")) {
                        float query_x = robot_x + (dx * std::cos(robot_yaw) - dy * std::sin(robot_yaw));
                        float query_y = robot_y + (dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw));
                        
                        grid_map::Position pos(query_x, query_y);
                        if (local_grid_map.isInside(pos)) {
                            float h = local_grid_map.atPosition("elevation", pos);
                            if (!std::isnan(h)) {
                                // 【修正 2】：严格对齐 IsaacLab 的公式 (h - robot_z + 0.5f)
                                relative_height = std::clamp(h - robot_z + 0.5f, -1.0f, 1.0f);
                            }
                        }
                    }
                    proprio_env_data_[env_idx++] = relative_height;
                }
            }
        }

        // ====== 4. 执行 ONNX 多张量推理 ======
        std::vector<Ort::Value> input_tensors;
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, proprio_env_data_.data(), proprio_env_data_.size(), shape_proprio_env_.data(), 2));
        

        // input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, estimator_history_data_.data(), estimator_history_data_.size(), shape_estimator_.data(), 2));
        
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, hidden_state_data_.data(), hidden_state_data_.size(), shape_h0_.data(), 3));

        // 输入数量从 3 改成 2
        auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names_, input_tensors.data(), 2, output_names_, 2);

        // ====== 5. 提取 Action 和更新 Hidden State ======
        current_action_eigen = Eigen::Map<Eigen::VectorXf>(outputs[0].GetTensorMutableData<float>(), action_dim);
        std::memcpy(hidden_state_data_.data(), outputs[1].GetTensorMutableData<float>(), hidden_dim_ * sizeof(float));

        last_action_eigen = current_action_eigen;

        for (int i = 0; i < action_dim; ++i) {
            tmp_action_eigen(i) = current_action_eigen(robot2policy_idx[i]) * action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;

        for (int i = 0; i < 4; ++i){
            robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
            robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
        }
        return robot_action;
    }
};