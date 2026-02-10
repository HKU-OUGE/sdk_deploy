/**
 * @file m20_policy_runner.hpp
 * @brief m20_policy_runner (Scheme B: ONNX with Explicit Hidden State)
 * @author Bo (Percy) Peng / DeepRobotics
 * @version 1.2 (Sim2Sim / Sim2Real Compatible)
 * @date 2025-02-17
 * * @copyright Copyright (c) 2025 DeepRobotics
 * */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"
#include <ctime>
#include <cmath>
#include <utility>
#include <vector>
#include <array>
#include <cstring> // For std::memcpy
#include <algorithm> // For std::fill
#include <onnxruntime_cxx_api.h>

class M20PolicyRunner : public PolicyRunnerBase {
private:
    // === 物理参数 ===
    VecXf kp_, kd_;
    VecXf dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f max_cmd_vel_, gravity_direction = Vec3f(0., 0., -1.);
    VecXf dof_pos_default_;
    timespec system_time;

    const int motor_num = 16;
    const int observation_dim = 57; // 3+3+3+16+16+16
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

    // === ONNX Runtime & Hidden State (方案 B) ===
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::Env env_;
    Ort::MemoryInfo memory_info{nullptr};

    // [关键修改] 必须与 play_moe.py 导出时的 input_names 和 output_names 严格一致
    // input_names=["obs", "hidden_state"]
    const char* input_names_[2] = {"obs", "hidden_state"}; 
    // output_names=["action", "next_hidden"]
    const char* output_names_[2] = {"action", "next_hidden"};

    const std::array<int64_t, 2> input_observationShape = {1, observation_dim};
    
    // GRU 参数: Batch=1, Seq=1, Hidden=256
    const int hidden_dim = 256;
    const std::array<int64_t, 3> hidden_state_shape = {1, 1, hidden_dim};
    std::vector<float> hidden_state_data_;

    VecXf command;
    float time_step = 0.;
    int stop_count = 1000;

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
        session_options_.SetIntraOpNumThreads(1); // 限制单线程，防止 Sim2Sim/Sim2Real 抢占资源
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        
        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "Model file not found: " << policy_path_ << std::endl;
            throw std::runtime_error("Model file missing");
        }

        // 3. 加载模型
        try {
            session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
            std::cout << "[M20PolicyRunner] Loaded ONNX model from: " << policy_path_ << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[M20PolicyRunner] Failed to load model: " << e.what() << std::endl;
            throw;
        }

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
    }

    ~M20PolicyRunner() override = default;

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from, 
        const std::vector<std::string>& to, 
        int default_index = 0) 
    {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < from.size(); ++i) {
            idx_map[from[i]] = i;
        }

        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) {
                perm.push_back(it->second);
            } else {
                perm.push_back(default_index);
            }
        }
        return perm;
    }

    void DisplayPolicyInfo(){}

    void OnEnter(const RobotBasicState &rbs) {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);

        // [关键] 重置 Hidden State 为全 0
        std::fill(hidden_state_data_.begin(), hidden_state_data_.end(), 0.0f);
        std::cout << "[M20PolicyRunner] Reset hidden state." << std::endl;
    }

    // === Scheme B: 推理函数 (带状态) ===
    VecXf Onnx_infer(VecXf current_observation){
        std::vector<Ort::Value> input_tensors;
        
        // 1. 输入 Tensor: obs (1, 57)
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
            memory_info,
            current_observation.data(),
            current_observation.size(),
            input_observationShape.data(), 
            input_observationShape.size()
        ));

        // 2. 输入 Tensor: hidden_state (1, 1, 256)
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
            memory_info,
            hidden_state_data_.data(),
            hidden_state_data_.size(),
            hidden_state_shape.data(),
            hidden_state_shape.size()
        ));

        // 3. 执行推理
        auto outputs = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_,         // {"obs", "hidden_state"}
            input_tensors.data(), // data
            2,                    // 输入数量
            output_names_,        // {"action", "next_hidden"}
            2                     // 输出数量
        );

        // 4. 获取 Action 输出
        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, action_dim);

        // 5. 获取并更新 Hidden State 输出
        float* next_hidden_data = outputs[1].GetTensorMutableData<float>();
        // 将新的隐藏状态拷贝回成员变量，供下一帧使用
        std::memcpy(hidden_state_data_.data(), next_hidden_data, hidden_dim * sizeof(float));

        return VecXf(action_map); 
    }

    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {

        Vec3f base_omgea = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        // 映射关节数据
        for (int i = 0; i < action_dim; ++i){
            joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
            joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
        }
        
        // 轮子位置归零 (Blind Student Policy 通常不依赖轮子的绝对位置，因为它们是连续旋转的)
        joint_pos_rl.segment(12, 4).setZero();

        joint_pos_rl -= dof_default_eigen_policy;

        // 组装观测向量 (57维)
        // Order must match play_moe.py / env_cfg.yaml expectations
        current_observation_ << base_omgea, 
                              projected_gravity, 
                              command, 
                              joint_pos_rl, 
                              joint_vel_rl, 
                              last_action_eigen;

        // 执行 ONNX 推理 (包含状态更新)
        current_action_eigen = Onnx_infer(current_observation_);
        last_action_eigen = current_action_eigen;

        // 映射回机器人关节
        for (int i = 0; i < action_dim; ++i){
            tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);
            tmp_action_eigen(i) *= action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;
        
        for (int i = 0; i < 4; ++i){
            robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
            robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
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