/**
 * @file rl_sensor_control_state.hpp
 * @brief rl policy running state for quadruped-wheel robot (Sensor-based Framework)
 */
#pragma once
#include "state_base.h"
#include "policy_runner_base.hpp"
#include "m20_sensor_policy_runner.hpp" // [修改 1] 引入新的带感知的 Runner 头文件
#include "robot_interface.h"
#include "user_command_interface.h"
#include "json.hpp"
#include "basic_function.hpp"

namespace qw {
    class RLSensorControlState : public StateBase {
    private:
        RobotBasicState rbs_;
        int state_run_cnt_;

        std::shared_ptr<PolicyRunnerBase> policy_ptr_;
        std::shared_ptr<M20SensorPolicyRunner> m20_policy_; // [修改 2] 实例化带感知的 Runner

        std::thread run_policy_thread_;
        bool start_flag_ = true;

        float policy_cost_time_ = 1;

        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

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
        RLSensorControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_name, state_name, data_ptr) {
            // std::memset(&rbs_, 0, sizeof(rbs_));

            if (robot_name_ == RobotName::M20) {
                namespace fs = std::filesystem;
                fs::path base = fs::path(__FILE__).parent_path();

                auto model_path = fs::canonical(base / ".." / ".." / "policy" / "unified_policy.onnx");
                m20_policy_ = std::make_shared<M20SensorPolicyRunner>("m20_sensor_policy", model_path.string());
            }

            policy_ptr_ = m20_policy_;
            if (!policy_ptr_) {
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
            policy_ptr_->DisplayPolicyInfo();
            init_rbs_();
        }

        ~RLSensorControlState() {}

        virtual void OnEnter() {
            state_run_cnt_ = -1;
            start_flag_ = true;

            // 获取机器人当前稳定的站立物理状态
            UpdateRobotObservation();
            
            policy_ptr_->OnEnter(rbs_);

            // ================== 网络预热 (Network Warm-up) ==================
            std::cout << "[RLSensor] 开始网络预热 (Network Warm-up 50 frames)..." << std::endl;
            
            // 构造一个绝对静止的摇杆指令
            UserCommand zero_cmd = *(uc_ptr_->GetUserCommand());
            zero_cmd.forward_vel_scale = 0.0f;
            zero_cmd.side_vel_scale = 0.0f;
            zero_cmd.turnning_vel_scale = 0.0f;

            // 在不发送给电机的情况下，让网络连续空跑 50 次，使得 GRU 隐状态完全收敛
            for (int i = 0; i < 150; ++i) {
                policy_ptr_->getRobotAction(rbs_, zero_cmd);
            }
            std::cout << "[RLSensor] 网络预热完成！隐藏状态已收敛。" << std::endl;
            // =========================================================================

            run_policy_thread_ = std::thread(std::bind(&RLSensorControlState::PolicyRunner, this));
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLSensorControlMode);
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
            return PostureUnsafeCheck();
        }

        bool PostureUnsafeCheck() {
            return false;
        }

        virtual StateName GetNextStateName() {
            if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) return StateName::kJointDamping;

            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::StandingUp)) return StateName::kStandUp;
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLControlMode)) return StateName::kRLControl;

            return StateName::kRLSensorControl;
        }
    };
};