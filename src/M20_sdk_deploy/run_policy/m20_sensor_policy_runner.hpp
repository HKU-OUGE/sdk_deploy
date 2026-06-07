#pragma once

#include "policy_runner_base.hpp"
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>

class M20SensorPolicyRunner : public PolicyRunnerBase {
private:
    VecXf kp_, kd_, dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f gravity_direction = Vec3f(0., 0., -1.);
    const int motor_num = 16, action_dim = 16;
    float omega_scale_ = 0.25, dof_vel_scale_ = 0.05;

    // 感知观测维度按加载的 ONNX 输入自适应 (per-state, 实现"只迁 sensor")：
    //   legacy scan policy (platform/crawl/gap/旧unified):
    //       半球 LIDAR 16 polar × 31 azimuth = 496/向, env = 992, 走 fwd/bwd map_scan_fn 旧路径
    //   new unified+elevation policy (sensor, 27400 导出):
    //       env = 691 = height_scan(187) + forward(252) + backward(252)
    //       由独立感知节点发布完整、已归一化的 691 数组, runner 原样拷贝
    static constexpr int NUM_POLAR = 16;
    static constexpr int NUM_AZ = 31;
    static constexpr int SCAN_PER_DIR = NUM_POLAR * NUM_AZ;   // 496  (legacy, per dir)
    static constexpr int LEGACY_ENV_DIM = 992;               // fwd496 + bwd496
    static constexpr int HEIGHT_DIM = 187;                   // new: height_scan 17×11
    static constexpr int ARC_SCAN_PER_DIR = 252;             // new: 12 pitch × 21 az
    static constexpr int GAP_ELEV_DIM = HEIGHT_DIM + ARC_SCAN_PER_DIR;  // gap model: height187 + scan252
    static constexpr int NOISY_ELEV_DIM = 691;              // new: height187 + fwd252 + bwd252
    const int proprio_dim_ = 57;
    // 以下两个在构造函数里按 ONNX 输入 shape 重新设定:
    int scan_dim_ = LEGACY_ENV_DIM;                          // = proprio_env_dim_ - proprio_dim_
    int proprio_env_dim_ = proprio_dim_ + LEGACY_ENV_DIM;    // 1049 (legacy 默认)
    bool use_full_perception_ = false;                       // true 当检测到新 691 布局
    bool use_scan_only_perception_ = false;                  // unified/gap scan-only deployment path
    std::string perception_topic_ = "/perception/noisy_elevation_array";
    
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

    // scan history 已禁用 — 当前训练 (platform/scan/split MoE teacher) 都是
    // scan_history_offsets=[]; ONNX 只有 3 输入 / 2 输出, 不需要 ring buffer.

    const char* input_names_[3] = {"proprio_and_env", "estimator_history", "h0"};
    const char* output_names_[2] = {"action", "next_h0"};

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr scan_array_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr perception_sub_;  // 新模型: 完整 691 noisy_elevation
    std::thread ros_spin_thread_;

    std::mutex scan_mutex_;
    std::vector<float> fwd_scan_bins_;
    std::vector<float> bwd_scan_bins_;
    std::vector<float> perception_full_;   // 新模型: 完整 691 维 noisy_elevation (已归一化, 顺序 height187|fwd252|bwd252)
    bool scan_received_ = false;
    bool is_offline_test_ = false;

    // === 新增：数据记录器相关变量 ===
    std::ofstream data_log_file_;
    std::string log_file_name_;
    timespec system_time;

    static bool pathEndsWithFilename(const std::string& path, const std::string& filename) {
        if (path == filename) return true;
        const std::string suffix = "/" + filename;
        return path.size() >= suffix.size()
            && path.compare(path.size() - suffix.size(), std::string::npos, suffix) == 0;
    }

public:
    M20SensorPolicyRunner(const std::string &policy_name, const std::string &policy_path, bool is_offline_test = false) :
            PolicyRunnerBase(policy_name), env_(ORT_LOGGING_LEVEL_WARNING, "M20SensorPolicyRunner"),
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
            is_offline_test_(is_offline_test) {

        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, policy_path.c_str(), session_options_);

        // === 从 ONNX 输入0 (proprio_and_env) 自适应感知维度 (per-state, 实现"只迁 sensor") ===
        {
            auto ti = session_->GetInputTypeInfo(0);
            auto shp = ti.GetTensorTypeAndShapeInfo().GetShape();   // 期望 [1, N]
            int n = (shp.size() >= 2 && shp[1] > 0) ? static_cast<int>(shp[1]) : proprio_env_dim_;
            proprio_env_dim_ = n;
            scan_dim_ = proprio_env_dim_ - proprio_dim_;
            use_full_perception_ = (scan_dim_ == NOISY_ELEV_DIM || scan_dim_ == GAP_ELEV_DIM);
            const bool is_scan_only_policy =
                pathEndsWithFilename(policy_path, "unified_policy.onnx")
                || pathEndsWithFilename(policy_path, "gap_unified_policy.onnx");
            use_scan_only_perception_ = use_full_perception_ && is_scan_only_policy;
            if (use_scan_only_perception_) {
                perception_topic_ = "/perception/noisy_elevation_scan_only_array";
            }
            shape_proprio_env_ = {1, proprio_env_dim_};
            std::cout << "[" << policy_name_ << "] ONNX proprio_and_env=" << proprio_env_dim_
                      << " (env=" << scan_dim_ << ", "
                      << (scan_dim_ == NOISY_ELEV_DIM ? "NEW noisy_elevation 691"
                          : (scan_dim_ == GAP_ELEV_DIM ? "gap elevation 439" : "legacy scan 992"))
                      << ", topic=" << (use_full_perception_ ? perception_topic_ : "/scan/multi_layer_features_array")
                      << ")" << std::endl;
        }

        LoadVelocityConfig(policy_path);

        proprio_env_data_.resize(proprio_env_dim_, 0.0f);
        estimator_history_data_.resize(estimator_dim_, 0.0f);
        hidden_state_data_.resize(hidden_dim_, 0.0f);

        fwd_scan_bins_.resize(SCAN_PER_DIR, 2.5f);
        bwd_scan_bins_.resize(SCAN_PER_DIR, 2.5f);
        if (use_full_perception_) {
            // 安全默认: height_scan 区(前187)=0(名义平地), scan 区(后504)=1.0(无障碍)
            perception_full_.assign(scan_dim_, 0.0f);
            std::fill(perception_full_.begin() + HEIGHT_DIM, perception_full_.end(), 1.0f);
        }

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
            ros_node_ = rclcpp::Node::make_shared("m20_scan_subscriber");

            if (use_full_perception_) {
                // 新模型: 订阅独立感知节点发布的完整、已归一化 691 维 noisy_elevation。
                // scan-only unified/gap: [height187=0 | fwd252 | bwd252]。
                // 旧 height-map 691: [height187 | fwd252 | bwd252] 原样拷入。
                // 439 gap:     [height187 | fwd252] 取前 439 维，避免误走旧 992 scan。
                perception_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
                    perception_topic_, rclcpp::SensorDataQoS(),
                    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                        const int msg_dim = static_cast<int>(msg->data.size());
                        if (msg_dim != scan_dim_ && !(scan_dim_ == GAP_ELEV_DIM && msg_dim == NOISY_ELEV_DIM)) return;
                        std::lock_guard<std::mutex> lock(scan_mutex_);
                        std::copy(msg->data.begin(), msg->data.begin() + scan_dim_, perception_full_.begin());
                        if (use_scan_only_perception_) {
                            std::fill(perception_full_.begin(), perception_full_.begin() + HEIGHT_DIM, 0.0f);
                        }
                        scan_received_ = true;
                    });
            } else {
                scan_array_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "/scan/multi_layer_features_array", rclcpp::SensorDataQoS(),
                    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                        // 半球 hemispherical: 前 496 + 后 496 = 992
                        if (static_cast<int>(msg->data.size()) != scan_dim_) return;

                        std::lock_guard<std::mutex> lock(scan_mutex_);
                        std::copy(msg->data.begin(), msg->data.begin() + SCAN_PER_DIR, fwd_scan_bins_.begin());
                        std::copy(msg->data.begin() + SCAN_PER_DIR, msg->data.end(), bwd_scan_bins_.begin());
                        scan_received_ = true;
                    });
            }

            ros_spin_thread_ = std::thread([this]() {
                rclcpp::executors::MultiThreadedExecutor executor;
                executor.add_node(ros_node_);
                executor.spin();
            });
            // === 数据记录器 CSV：用 policy_name_ 命名，每个 state 独立文件 ===
            std::time_t t = std::time(nullptr);
            char time_str[100];
            std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));
            log_file_name_ = policy_name_ + "_log_" + std::string(time_str) + ".csv";

            data_log_file_.open(log_file_name_);
            if (data_log_file_.is_open()) {
                if (use_full_perception_) {
                    data_log_file_ << "step,time,min_height,min_scan\n";
                } else {
                    data_log_file_ << "step,time,min_fwd,min_bwd";
                    for(int i = 0; i < SCAN_PER_DIR; ++i) data_log_file_ << ",fwd_" << i;
                    for(int i = 0; i < SCAN_PER_DIR; ++i) data_log_file_ << ",bwd_" << i;
                    data_log_file_ << "\n";
                }
                std::cout << "✅ [" << policy_name_ << " Logger] Data will be saved to: " << log_file_name_ << std::endl;
            } else {
                std::cerr << "❌ [" << policy_name_ << " Logger] Failed to open file: " << log_file_name_ << std::endl;
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

        is_first_step_ = true;
        run_cnt_ = 0;
        dbg_ = DebugState{};
        dashboard_lines_ = 0;
    }

    // === 新增：获取系统当前时间 ===
    double getCurrentTime() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    }

    // 公共: 拷贝 proprio 到输入 + 构建 estimator history (15帧, 逐 term 重排 {3,3,3,16,16,16})
    void loadProprioAndEstimator(const std::vector<float>& curr_proprio) {
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
    }

    // 公共: 三输入推断 (proprio_env_data_ 的 env 块必须已由调用方填好), 返回 action 并回填 hidden
    VecXf runOrt() {
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

    // legacy 路径: 半球 scan, runner 内归一化 (d/2.5, <0.3->1.0); 行为与旧版一致
    VecXf processAndInfer(
        const std::vector<float>& curr_proprio,
        const std::vector<float>& fwd_scan,
        const std::vector<float>& bwd_scan)
    {
        loadProprioAndEstimator(curr_proprio);

        int env_idx = proprio_dim_;
        auto map_scan_fn = [](float d) {
            // 与 sim multi_layer_scan 一致：盲区 (d < 0.3) 视为 no-hit → 归一化 1.0
            if (d < 0.3f) return 1.0f;
            return std::clamp(d / 2.5f, 0.0f, 1.0f);
        };
        for (int i = 0; i < SCAN_PER_DIR; ++i) proprio_env_data_[env_idx++] = map_scan_fn(fwd_scan[i]);
        for (int i = 0; i < SCAN_PER_DIR; ++i) proprio_env_data_[env_idx++] = map_scan_fn(bwd_scan[i]);

        return runOrt();
    }

    // 新路径: 完整 691 noisy_elevation (感知节点已归一化), 原样拷入 env 块
    VecXf processAndInferFull(
        const std::vector<float>& curr_proprio,
        const std::vector<float>& env_block)
    {
        loadProprioAndEstimator(curr_proprio);
        std::copy(env_block.begin(), env_block.end(), proprio_env_data_.begin() + proprio_dim_);
        return runOrt();
    }

    VecXf testOfflineStep(
        const std::vector<float>& omega, const std::vector<float>& proj_g, const std::vector<float>& cmd,
        const std::vector<float>& jp, const std::vector<float>& jv, const std::vector<float>& last_act,
        const std::vector<float>& raw_fwd, const std::vector<float>& raw_bwd)
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

        return processAndInfer(curr_proprio, raw_fwd, raw_bwd);
    }

    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        Vec3f base_omega = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = ClampCommand(Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale));

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

        if (use_full_perception_) {
            // === 新模型: 取完整 691 noisy_elevation, 原样喂入 ===
            std::vector<float> env_block;
            {
                std::lock_guard<std::mutex> lock(scan_mutex_);
                env_block = perception_full_;   // 未收到时为 ctor 安全默认 (height=0, scan=1)
            }
            float min_h = 1.0f, min_s = 1.0f;
            for (int i = 0; i < HEIGHT_DIM; ++i) min_h = std::min(min_h, env_block[i]);
            for (int i = HEIGHT_DIM; i < scan_dim_; ++i) min_s = std::min(min_s, env_block[i]);
            if (run_cnt_ % 50 == 0) { dbg_.min_fwd = min_h; dbg_.min_bwd = min_s; }
            if (data_log_file_.is_open() && !is_offline_test_) {
                data_log_file_ << run_cnt_ << "," << getCurrentTime() << "," << min_h << "," << min_s << "\n";
            }

            dbg_.cmd_vx = command(0);
            dbg_.cmd_vy = command(1);
            dbg_.cmd_wz = command(2);
            if (run_cnt_ % 50 == 0) {
                const char* label = (policy_name_.find("crawl") != std::string::npos) ? "Crawl" : "Sensor";
                PrintDebugDashboard(label, run_cnt_);
            }
            processAndInferFull(curr_proprio, env_block);
        } else {
            // === legacy: 半球 scan 拷贝 (前/后各 SCAN_PER_DIR 维) ===
            std::vector<float> curr_fwd_bins(SCAN_PER_DIR, 2.5f);
            std::vector<float> curr_bwd_bins(SCAN_PER_DIR, 2.5f);
            {
                std::lock_guard<std::mutex> lock(scan_mutex_);
                if (scan_received_) {
                    curr_fwd_bins = fwd_scan_bins_;
                    curr_bwd_bins = bwd_scan_bins_;
                }
            }
            if (!scan_received_ && run_cnt_ % 200 == 0) {
                std::cerr << "⚠️ [" << policy_name_
                          << "] no /scan/multi_layer_features_array received; "
                          << "legacy scan policy is using all-2.5m no-hit defaults."
                          << std::endl;
            }

            // === CSV 日志：仅记录 scan (elevation 已移除) ===
            if (data_log_file_.is_open() && !is_offline_test_) {
                float min_fwd = 2.5f, min_bwd = 2.5f;
                for (float d : curr_fwd_bins) min_fwd = std::min(min_fwd, d);
                for (float d : curr_bwd_bins) min_bwd = std::min(min_bwd, d);

                if (run_cnt_ % 50 == 0) {
                    dbg_.min_fwd = min_fwd;
                    dbg_.min_bwd = min_bwd;
                }
                data_log_file_ << run_cnt_ << "," << getCurrentTime() << ","
                               << min_fwd << "," << min_bwd;
                for (int i = 0; i < SCAN_PER_DIR; ++i) data_log_file_ << "," << curr_fwd_bins[i];
                for (int i = 0; i < SCAN_PER_DIR; ++i) data_log_file_ << "," << curr_bwd_bins[i];
                data_log_file_ << "\n";
            }

            dbg_.cmd_vx = command(0);
            dbg_.cmd_vy = command(1);
            dbg_.cmd_wz = command(2);
            if (run_cnt_ % 50 == 0) {
                const char* label = (policy_name_.find("crawl") != std::string::npos) ? "Crawl" : "Sensor";
                PrintDebugDashboard(label, run_cnt_);
            }

            // 喂给网络推断
            processAndInfer(curr_proprio, curr_fwd_bins, curr_bwd_bins);
        }

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
