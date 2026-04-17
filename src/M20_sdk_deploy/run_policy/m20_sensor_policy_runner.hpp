#pragma once

#include "policy_runner_base.hpp"
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

// === 新增：用于文件写入和时间获取 ===
#include <fstream>
#include <iomanip>
#include <ctime>

class M20SensorPolicyRunner : public PolicyRunnerBase {
private:
    VecXf kp_, kd_, dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f gravity_direction = Vec3f(0., 0., -1.);
    const int motor_num = 16, action_dim = 16;
    float omega_scale_ = 0.25, dof_vel_scale_ = 0.05;

    const int proprio_dim_ = 57;
    const int elevation_dim_ = 187;
    const int scan_dim_ = 252; 
    const int proprio_env_dim_ = 496;  
    
    const int history_len_ = 15;
    const int estimator_dim_ = 855;    
    const int hidden_dim_ = 256;       

    std::vector<float> proprio_env_data_;
    std::vector<float> estimator_history_data_;
    std::vector<float> hidden_state_data_;

    std::deque<std::vector<float>> history_buffer_;
    bool is_first_step_ = true;
    
    std::array<int64_t, 2> shape_proprio_env_ = {1, proprio_env_dim_};
    std::array<int64_t, 2> shape_estimator_   = {1, estimator_dim_};
    std::array<int64_t, 3> shape_h0_          = {1, 1, hidden_dim_}; 

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

    Ort::SessionOptions session_options_;
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info{nullptr};

    const char* input_names_[3] = {"proprio_and_env", "estimator_history", "h0"};
    const char* output_names_[2] = {"action", "next_h0"};

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
    bool is_offline_test_ = false;

    // === 新增：数据记录器相关变量 ===
    std::ofstream data_log_file_;
    std::string log_file_name_;
    timespec system_time;

public:
    M20SensorPolicyRunner(const std::string &policy_name, const std::string &policy_path, bool is_offline_test = false) :
            PolicyRunnerBase(policy_name), env_(ORT_LOGGING_LEVEL_WARNING, "M20SensorPolicyRunner"),
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
            is_offline_test_(is_offline_test) {

        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, policy_path.c_str(), session_options_);

        proprio_env_data_.resize(proprio_env_dim_, 0.0f);
        estimator_history_data_.resize(estimator_dim_, 0.0f);
        hidden_state_data_.resize(hidden_dim_, 0.0f);
        
        fwd_scan_bins_.resize(126, 5.0f); 
        bwd_scan_bins_.resize(126, 5.0f);

        for (int i = 0; i < history_len_; ++i) {
            history_buffer_.push_back(std::vector<float>(proprio_dim_, 0.0f));
        }

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

        if (!is_offline_test_) {
            ros_node_ = rclcpp::Node::make_shared("m20_elevation_subscriber");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            grid_map_sub_ = ros_node_->create_subscription<grid_map_msgs::msg::GridMap>(
            "/elevation_map_remote", rclcpp::SensorDataQoS(),
                [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    latest_map_ = *msg;
                    map_received_ = true;
                });

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
                    const float angle_tol = 4.0f; // 角度容差 (度)
                    const float dy_step = 0.05f;
                    const float y_min = -0.5f;
                    const int num_rays = 21;
                    
                    // 对齐 env_cfg.py 中的位置偏移
                    const float fwd_x_offset = 0.32028f;
                    const float bwd_x_offset = -0.32028f;
                    const float z_offset = -0.013f;

                    for (size_t i = 0; i < msg->data.size(); i += msg->point_step) {
                        float x, y, z;
                        memcpy(&x, &msg->data[i + x_offset_idx], sizeof(float));
                        memcpy(&y, &msg->data[i + y_offset_idx], sizeof(float));
                        memcpy(&z, &msg->data[i + z_offset_idx], sizeof(float));

                        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

                        // 1. 检查是否属于前向 Lidar
                        float fwd_dx = x - fwd_x_offset;
                        float fwd_dy = y;
                        float fwd_dz = z - z_offset;

                        if (fwd_dx > 0.0f && fwd_dy >= y_min - dy_step/2.0f && fwd_dy <= -y_min + dy_step/2.0f) {
                            // 计算真实的俯仰角 (Pitch Angle)
                            float angle_deg = std::atan2(-fwd_dz, fwd_dx) * 180.0f / M_PI;
                            
                            for (int l = 0; l < 6; ++l) {
                                if (std::abs(angle_deg - target_angles[l]) <= angle_tol) {
                                    int bin = std::round((fwd_dy - y_min) / dy_step);
                                    if (bin >= 0 && bin < num_rays) {
                                        // 必须使用 3D 空间真实距离！
                                        float r = std::sqrt(fwd_dx*fwd_dx + fwd_dy*fwd_dy + fwd_dz*fwd_dz);
                                        if (r < local_fwd_bins[l * num_rays + bin]) {
                                            local_fwd_bins[l * num_rays + bin] = r;
                                        }
                                    }
                                }
                            }
                        }

                        // 2. 检查是否属于后向 Lidar
                        float bwd_dx = x - bwd_x_offset;
                        float bwd_dy = y;
                        float bwd_dz = z - z_offset;

                        if (bwd_dx < 0.0f && bwd_dy >= y_min - dy_step/2.0f && bwd_dy <= -y_min + dy_step/2.0f) {
                            // 由于向后看，-dx 才是前向投影
                            float angle_deg = std::atan2(-bwd_dz, -bwd_dx) * 180.0f / M_PI;
                            
                            for (int l = 0; l < 6; ++l) {
                                if (std::abs(angle_deg - target_angles[l]) <= angle_tol) {
                                    // 遵循原有的 (-y) 映射逻辑保证左右一致性
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

            ros_spin_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });

            // === 新增：初始化数据记录器 CSV 文件 (带时间戳) ===
            std::time_t t = std::time(nullptr);
            char time_str[100];
            std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));
            log_file_name_ = "sensor_perception_log_" + std::string(time_str) + ".csv";
            
            data_log_file_.open(log_file_name_);
            if (data_log_file_.is_open()) {
                data_log_file_ << "step,time,robot_x,robot_y,robot_z,robot_yaw,valid_h_count,hole_ratio_pct,min_fwd,min_bwd";
                for(int i = 0; i < 187; ++i) data_log_file_ << ",h_" << i;
                for(int i = 0; i < 126; ++i) data_log_file_ << ",fwd_" << i;
                for(int i = 0; i < 126; ++i) data_log_file_ << ",bwd_" << i;
                data_log_file_ << "\n";
                std::cout << "✅ [Sensor Logger] Data will be saved to: " << log_file_name_ << std::endl;
            } else {
                std::cerr << "❌ [Sensor Logger] Failed to open file: " << log_file_name_ << std::endl;
            }
        }
    }

    ~M20SensorPolicyRunner() {
        if (!is_offline_test_) {
            rclcpp::shutdown();
            if (ros_spin_thread_.joinable()) ros_spin_thread_.join();
            
            // === 新增：关闭文件 ===
            if (data_log_file_.is_open()) {
                data_log_file_.close();
            }
        }
    }

    void OnEnter(const RobotBasicState &rbs) override {
        std::fill(hidden_state_data_.begin(), hidden_state_data_.end(), 0.0f);
        
        last_action_eigen.setZero(action_dim);
        current_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);

        // 触发 processAndInfer 中的历史帧刷新机制
        is_first_step_ = true; 
        run_cnt_ = 0;
    }

    // === 新增：获取系统当前时间 ===
    double getCurrentTime() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    }

    VecXf processAndInfer(
        const std::vector<float>& curr_proprio,
        const std::vector<float>& heights_187,
        const std::vector<float>& fwd_scan_126,
        const std::vector<float>& bwd_scan_126) 
    {
        std::copy(curr_proprio.begin(), curr_proprio.end(), proprio_env_data_.begin());

        if (is_first_step_) {
            std::vector<float> static_proprio = curr_proprio;
            
            // base_omega 在 curr_proprio 的索引是 0, 1, 2
            for (int i = 0; i < 3; ++i) static_proprio[i] = 0.0f; 
            
            // 核心修复2：必须抹除历史 Command 指令，防止摇杆不为0时网络产生卡死幻觉
            for (int i = 6; i < 9; ++i) static_proprio[i] = 0.0f; 

            // joint_vel 在 curr_proprio 的索引是 25 到 40
            for (int i = 25; i < 41; ++i) static_proprio[i] = 0.0f; 
            
            for (auto& frame : history_buffer_) {
                std::copy(static_proprio.begin(), static_proprio.end(), frame.begin());
            }
            is_first_step_ = false;
        } else {
            history_buffer_.pop_front();
            history_buffer_.push_back(curr_proprio);
        }

        std::vector<int> term_dims = {3, 3, 3, 16, 16, 16}; 
        int offset = 0;             
        int feature_start_idx = 0;  

        for (int dim : term_dims) {
            for (int t = 0; t < history_len_; ++t) {
                for (int i = 0; i < dim; ++i) {
                    estimator_history_data_[offset++] = history_buffer_[t][feature_start_idx + i];
                }
            }
            feature_start_idx += dim;
        }

        int env_idx = proprio_dim_;
        std::copy(heights_187.begin(), heights_187.end(), proprio_env_data_.begin() + env_idx);
        env_idx += 187;

        auto map_scan_fn = [](float d) {
            if (d < 0.2f) return -1.0f;
            return std::clamp(d / 5.0f, 0.0f, 1.0f);
        };

        for (int i = 0; i < 126; ++i) proprio_env_data_[env_idx++] = map_scan_fn(fwd_scan_126[i]);
        for (int i = 0; i < 126; ++i) proprio_env_data_[env_idx++] = map_scan_fn(bwd_scan_126[i]);

        std::vector<Ort::Value> input_tensors;
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, proprio_env_data_.data(), proprio_env_data_.size(), shape_proprio_env_.data(), 2));
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, estimator_history_data_.data(), estimator_history_data_.size(), shape_estimator_.data(), 2));
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(memory_info, hidden_state_data_.data(), hidden_state_data_.size(), shape_h0_.data(), 3));

        auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names_, input_tensors.data(), 3, output_names_, 2);

        current_action_eigen = Eigen::Map<Eigen::VectorXf>(outputs[0].GetTensorMutableData<float>(), action_dim);
        std::memcpy(hidden_state_data_.data(), outputs[1].GetTensorMutableData<float>(), hidden_dim_ * sizeof(float));

        last_action_eigen = current_action_eigen;
        return current_action_eigen;
    }

    VecXf testOfflineStep(
        const std::vector<float>& omega, const std::vector<float>& proj_g, const std::vector<float>& cmd,
        const std::vector<float>& jp, const std::vector<float>& jv, const std::vector<float>& last_act,
        const std::vector<float>& raw_heights, const std::vector<float>& raw_fwd, const std::vector<float>& raw_bwd) 
    {
        std::vector<float> curr_proprio(proprio_dim_, 0.0f);
        
        int idx = 0;
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(static_cast<float>(omega[i] * omega_scale_), -100.0f, 100.0f);
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(static_cast<float>(proj_g[i] * 1.0f), -100.0f, 100.0f);
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(static_cast<float>(cmd[i] * 1.0f), -100.0f, 100.0f);
        
        for (int i=0; i<action_dim; ++i) {
            float rel_pos = jp[policy2robot_idx[i]] - dof_default_eigen_policy(i);
            if (i >= 12) rel_pos = 0.0f;
            curr_proprio[idx++] = std::clamp(rel_pos * 1.0f, -100.0f, 100.0f); 
        }

        for (int i=0; i<action_dim; ++i) {
            curr_proprio[idx++] = std::clamp(static_cast<float>(jv[policy2robot_idx[i]] * dof_vel_scale_), -100.0f, 100.0f);
        }
        
        for (int i=0; i<action_dim; ++i) {
            curr_proprio[idx++] = std::clamp(last_act[i], -100.0f, 100.0f); 
        }

        return processAndInfer(curr_proprio, raw_heights, raw_fwd, raw_bwd);
    }

    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        Vec3f base_omega = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        std::vector<float> curr_proprio(proprio_dim_, 0.0f);
        int idx = 0;
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(base_omega(i), -100.0f, 100.0f);
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(projected_gravity(i), -100.0f, 100.0f);
        for (int i=0; i<3; ++i) curr_proprio[idx++] = std::clamp(command(i), -100.0f, 100.0f);

        for (int i = 0; i < action_dim; ++i) {
            float rel_pos = ro.joint_pos(policy2robot_idx[i]) - dof_default_eigen_policy(i);
            if (i >= 12) rel_pos = 0.0f;
            curr_proprio[idx++] = std::clamp(rel_pos * 1.0f, -100.0f, 100.0f);
        }

        for (int i = 0; i < action_dim; ++i) {
            curr_proprio[idx++] = std::clamp(static_cast<float>(ro.joint_vel(policy2robot_idx[i]) * dof_vel_scale_), -100.0f, 100.0f);
        }

        for (int i = 0; i < action_dim; ++i) {
            curr_proprio[idx++] = std::clamp(last_action_eigen[i], -100.0f, 100.0f);
        }

        std::vector<float> curr_heights(187, 0.0f);
        // ==========================================================
        // === 智能位姿获取：优先 TF，失败则降级使用地图中心 ===
        // ==========================================================
        float robot_x = 0, robot_y = 0, robot_z = 0.45f;
        float robot_yaw = ro.base_rpy(2); // 默认使用高频可靠的 IMU 航向角
        bool tf_valid = false;

        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            robot_x = t.transform.translation.x; 
            robot_y = t.transform.translation.y; 
            robot_z = t.transform.translation.z; // 拿到了真实的动态高度！
            
            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
            robot_yaw = yaw; // 如果 TF 正常，以 TF 为准
            tf_valid = true;
        } catch (const tf2::TransformException & ex) { 
            if (run_cnt_ % 50 == 0) {
                std::cerr << "⚠️ [TF Warning] 无 TF，将启用地图中心降级方案: " << ex.what() << std::endl;
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
            
            // 【核心】：如果 TF 失败（真机环境），启用“地图中心 + 运动学姿态估计”
            if (!tf_valid) {
                grid_map::Position map_center = local_grid_map.getPosition();
                robot_x = map_center.x();
                robot_y = map_center.y();

                if (local_grid_map.exists("elevation") && local_grid_map.isInside(map_center)) {
                    float center_ground_h = local_grid_map.atPosition("elevation", map_center);
                    if (!std::isnan(center_ground_h)) {
                        
                        // ==========================================================
                        // === 运动学高度补偿 (Forward Kinematics + IMU) ===
                        // ==========================================================
                        float leg_h_sum = 0.0f;
                        // 提取 4 条腿的 Knee 关节索引: FL(2), FR(6), HL(10), HR(14)
                        int knee_indices[4] = {2, 6, 10, 14}; 
                        for (int i = 0; i < 4; ++i) {
                            // 无论膝关节是正偏还是负偏，cos(x) 都能求出正确的连杆对角线长度
                            leg_h_sum += std::sqrt(0.125f + 0.125f * std::cos(ro.joint_pos(knee_indices[i])));
                        }
                        float avg_leg_h = leg_h_sum / 4.0f;
                        
                        // 加上结构偏置和轮子半径 (经标准站立高度 0.45m 校准得出的固定差值)
                        float base_offset = avg_leg_h + 0.0113f; 
                        
                        // 使用 IMU 的 Pitch 和 Roll 将腿部长度投影到世界坐标系的 Z 轴上
                        float pitch = ro.base_rpy(1);
                        float roll = ro.base_rpy(0);
                        float dynamic_z = base_offset * std::cos(pitch) * std::cos(roll);
                        
                        // 最终动态高度 = 地面高度 + 运动学悬挂高度
                        robot_z = center_ground_h + dynamic_z; 
                        // ==========================================================
                    }
                }
            }

            if (run_cnt_ % 50 == 0) {
                std::cout << "🎯 [Pose] X: " << std::fixed << std::setprecision(2) << robot_x 
                          << " | Y: " << robot_y << " | Z: " << robot_z 
                          << (tf_valid ? " (From TF)" : " (From MapCenter)") << std::endl;
            }
        }

        // ==========================================================
        // === 运动学 Z 轴补偿对比测试 (仅在有 TF 时打印验证) ===
        // ==========================================================
        if (map_converted && run_cnt_ % 10 == 0 && tf_valid) {
            grid_map::Position map_center = local_grid_map.getPosition();
            if (local_grid_map.exists("elevation") && local_grid_map.isInside(map_center)) {
                float center_ground_h = local_grid_map.atPosition("elevation", map_center);
                if (!std::isnan(center_ground_h)) {
                    // 重跑一遍运动学估计
                    float leg_h_sum = 0.0f;
                    int knee_indices[4] = {2, 6, 10, 14}; 
                    for (int i = 0; i < 4; ++i) {
                        leg_h_sum += std::sqrt(0.125f + 0.125f * std::cos(ro.joint_pos(knee_indices[i])));
                    }
                    float dynamic_z = (leg_h_sum / 4.0f + 0.0113f) * std::cos(ro.base_rpy(1)) * std::cos(ro.base_rpy(0));
                    float est_z = center_ground_h + dynamic_z;
                    
                    float z_error = robot_z - est_z;
                    std::cout << "📐 [运动学验证] TF真实Z: " << std::fixed << std::setprecision(3) << robot_z 
                              << " | 运动学估计Z: " << est_z 
                              << " | 误差: " << z_error << " m" << std::endl;
                }
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

        std::vector<float> curr_fwd_bins(126, 5.0f);
        std::vector<float> curr_bwd_bins(126, 5.0f);
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (scan_received_) {
                curr_fwd_bins = fwd_scan_bins_;
                curr_bwd_bins = bwd_scan_bins_;
            }
        }

        // === 新增：将现成的感知数据写入 CSV（不影响推断速度） ===
        if (data_log_file_.is_open() && !is_offline_test_) {
            float min_fwd = 5.0f, min_bwd = 5.0f;
            for(float d : curr_fwd_bins) min_fwd = std::min(min_fwd, d);
            for(float d : curr_bwd_bins) min_bwd = std::min(min_bwd, d);

            // 计算空洞数据百分比 (总采样点数为 187)
            float hole_ratio_pct = (187.0f - valid_h_count) / 187.0f * 100.0f;

            if (run_cnt_ % 50 == 0) {
                std::cout << "[Sensor Policy] 高程图空洞占比: " 
                          << std::fixed << std::setprecision(1) << hole_ratio_pct << "% "
                          << "(有效点: " << valid_h_count << "/187)" << std::endl;
            }
            data_log_file_ << run_cnt_ << "," << getCurrentTime() << ","
                           << robot_x << "," << robot_y << "," << robot_z << "," << robot_yaw << ","
                           << valid_h_count << "," << hole_ratio_pct << "," << min_fwd << "," << min_bwd;

            for (int i = 0; i < 187; ++i) data_log_file_ << "," << curr_heights[i];
            for (int i = 0; i < 126; ++i) data_log_file_ << "," << curr_fwd_bins[i];
            for (int i = 0; i < 126; ++i) data_log_file_ << "," << curr_bwd_bins[i];

            data_log_file_ << "\n";
        }

        // 喂给网络推断
        processAndInfer(curr_proprio, curr_heights, curr_fwd_bins, curr_bwd_bins);

        for (int i = 0; i < action_dim; ++i) {
            tmp_action_eigen(i) = current_action_eigen(robot2policy_idx[i]) * action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;

        int smooth_steps = 30; 
        if (run_cnt_ < smooth_steps) {
            float alpha = static_cast<float>(run_cnt_ + 1) / smooth_steps; 
            
            for (int i = 0; i < action_dim; ++i) {
                if (i % 4 != 3) {
                    float actual_pos = ro.joint_pos(i);
                    tmp_action_eigen(i) = (1.0f - alpha) * actual_pos + alpha * tmp_action_eigen(i);
                }
            }
        }
        run_cnt_++; // 更新步数

        // 装载动作发给底层控制
        for (int i = 0; i < 4; ++i){
            robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
            robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
        }
        return robot_action;
    }
};