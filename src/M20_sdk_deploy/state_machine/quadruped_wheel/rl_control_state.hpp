/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state for quadruped-wheel robot
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * * @copyright Copyright (c) 2025  DeepRobotics
 * */
#pragma once
#include "state_base.h"
#include "policy_runner_base.hpp"
#include "m20_policy_runner.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "json.hpp"
#include "basic_function.hpp"
#include <algorithm>    // For std::clamp
#include <fstream>      // For std::ifstream
#include <filesystem>   // For std::filesystem
#include <cmath>        // For std::abs

namespace qw {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_;
        int state_run_cnt_;

        std::shared_ptr<PolicyRunnerBase> policy_ptr_;
        std::shared_ptr<M20PolicyRunner> m20_policy_;

        std::thread run_policy_thread_;
        bool start_flag_ = true;

        float policy_cost_time_ = 1;

        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

        // --- 安全保护参数配置 ---
        Eigen::Vector3f leg_vel_limit_ = Eigen::Vector3f(22.4, 22.4, 22.4);
        Eigen::Vector3f leg_torque_limit_ = Eigen::Vector3f(76.4, 76.4, 76.4);
        float wheel_vel_limit_ = 79.3;
        float wheel_torque_limit_ = 21.6;

        // 新增：电机持续异常计数器（防抖用）
        int motor_unsafe_cnt_ = 0;

        void LoadSafetyConfig() {
            namespace fs = std::filesystem;
            fs::path base = fs::path(__FILE__).parent_path();
            fs::path config_path = base / ".." / "parameters" / "safety_params.json";
            std::ifstream f(config_path.string());
            if (f.is_open()) {
                try {
                    nlohmann::json j;
                    f >> j;
                    if (j.contains("leg_vel_limit")) {
                        leg_vel_limit_ << j["leg_vel_limit"][0], j["leg_vel_limit"][1], j["leg_vel_limit"][2];
                    }
                    if (j.contains("leg_torque_limit")) {
                        leg_torque_limit_ << j["leg_torque_limit"][0], j["leg_torque_limit"][1], j["leg_torque_limit"][2];
                    }
                    if (j.contains("wheel_vel_limit")) {
                        wheel_vel_limit_ = j["wheel_vel_limit"];
                    }
                    if (j.contains("wheel_torque_limit")) {
                        wheel_torque_limit_ = j["wheel_torque_limit"];
                    }
                    std::cout << "[Safety] 成功加载安全限制参数: " << config_path.string() << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "[Safety] 解析 JSON 出错: " << e.what() << "，将使用默认保护参数。" << std::endl;
                }
            } else {
                std::cout << "[Safety] 未找到配置文件: " << config_path.string() << "，将使用默认保护参数。" << std::endl;
            }
        }

        void init_rbs_() {
            rbs_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        }

        void UpdateRobotObservation() {
            rbs_.base_rpy = ri_ptr_->GetImuRpy();
            rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
            rbs_.base_omega = ri_ptr_->GetImuOmega();
            rbs_.base_acc = ri_ptr_->GetImuAcc();
            rbs_.joint_pos = ri_ptr_->GetJointPosition();
            rbs_.joint_vel = ri_ptr_->GetJointVelocity();
            rbs_.joint_tau = ri_ptr_->GetJointTorque();

            rbs_.flt_base_acc_mat.row(acc_rot_count) = rbs_.base_acc.transpose();
            acc_rot_count += 1;
            acc_rot_count = acc_rot_count % 20;
        }

        void PolicyRunner() {
            int run_cnt_record = -1;
            while (start_flag_) {

                if (state_run_cnt_ % policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record) {
                    timespec start_timestamp, end_timestamp;
                    clock_gettime(CLOCK_MONOTONIC, &start_timestamp);
                    auto ra = policy_ptr_->getRobotAction(rbs_, *(uc_ptr_->GetUserCommand()));
                    
                    MatXf res = ra.ConvertToMat();

                    // --- 指令硬限幅：确保下发给电机的目标值永远不越界 ---
                    if (res.cols() >= 5) {
                        for (int i = 0; i < res.rows(); ++i) {
                            float v_lim, t_lim;
                            if (i < 12) { 
                                v_lim = leg_vel_limit_[i % 3];
                                t_lim = leg_torque_limit_[i % 3];
                            } else { 
                                v_lim = wheel_vel_limit_;
                                t_lim = wheel_torque_limit_;
                            }
                            res(i, 1) = std::clamp(res(i, 1), -v_lim, v_lim);
                            res(i, 4) = std::clamp(res(i, 4), -t_lim, t_lim);
                        }
                    }

                    ri_ptr_->SetJointCommand(res);
                    run_cnt_record = state_run_cnt_;
                    clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
                    policy_cost_time_ = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3
                                        + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6;

                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

    public:
        RLControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_name, state_name, data_ptr) {
            std::memset(&rbs_, 0, sizeof(rbs_));
            if (robot_name_ == RobotName::M20) {
                namespace fs = std::filesystem;
                fs::path base = fs::path(__FILE__).parent_path();
                auto model_path = fs::canonical(base / ".." / ".." / "policy" / "policy_blind_hard.onnx");
                m20_policy_ = std::make_shared<M20PolicyRunner>("m20_policy", model_path.string());
            }

            policy_ptr_ = m20_policy_;
            if (!policy_ptr_) {
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
            policy_ptr_->DisplayPolicyInfo();
            init_rbs_();
            LoadSafetyConfig();
        }

        ~RLControlState() {}

        virtual void OnEnter() {
            state_run_cnt_ = -1;
            motor_unsafe_cnt_ = 0; // 状态进入时重置计数器
            start_flag_ = true;
            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            policy_ptr_->OnEnter(rbs_);
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        };

        virtual void OnExit() {
            start_flag_ = false;
            run_policy_thread_.join();
            state_run_cnt_ = -1;
        }

        virtual void Run() {
            UpdateRobotObservation();
            state_run_cnt_++;
        }

        virtual bool LoseControlJudge() {
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
            if (MotorUnsafeCheck()) return true; 
            return PostureUnsafeCheck();
        }

        // ================== 加入防抖的异常检查 ==================
        bool MotorUnsafeCheck() {
            int num_joints = rbs_.joint_vel.size();
            bool is_unsafe_this_step = false;
            int err_i = -1; float err_v = 0, err_t = 0;

            for (int i = 0; i < num_joints; ++i) {
                float v_lim, t_lim;
                if (i < 12) {
                    // 对真实反馈放宽至 1.5 倍阈值，容忍着地瞬间的冲量
                    v_lim = leg_vel_limit_[i % 3] * 2.0f; 
                    t_lim = leg_torque_limit_[i % 3] * 2.0f;
                } else {
                    v_lim = wheel_vel_limit_ * 1.5f;
                    t_lim = wheel_torque_limit_ * 1.5f;
                }

                if (std::abs(rbs_.joint_vel(i)) > v_lim || std::abs(rbs_.joint_tau(i)) > t_lim) {
                    is_unsafe_this_step = true;
                    err_i = i;
                    err_v = rbs_.joint_vel(i);
                    err_t = rbs_.joint_tau(i);
                    break; 
                }
            }

            if (is_unsafe_this_step) {
                motor_unsafe_cnt_++;
                // 核心防抖逻辑：连续超限 15 个循环(假如是 1000Hz 则是 15 毫秒)才认为是真失控
                if (motor_unsafe_cnt_ > 30) {
                    std::cout << "[Safety] 电机真实状态持续失控! 将切断控制。异常关节: " << err_i 
                              << " 当前速度: " << err_v 
                              << " 当前力矩: " << err_t << std::endl;
                    return true;
                }
            } else {
                // 冲击瞬间过去，数值恢复正常，计数器清零
                motor_unsafe_cnt_ = 0; 
            }
            return false;
        }

        bool PostureUnsafeCheck() {
            return false;
        }

        virtual StateName GetNextStateName() {
            if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) return StateName::kJointDamping;
            return StateName::kRLControl;
        }
    };
};
