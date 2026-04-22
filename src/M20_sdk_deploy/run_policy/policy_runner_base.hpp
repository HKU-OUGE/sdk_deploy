/**
 * @file policy_runner_base.hpp
 * @brief policy runner base class
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#pragma once

#include "common_types.h"
#include "basic_function.hpp"
#include "onnxruntime_cxx_api.h"
#include "json.hpp"
#include <fstream>
#include <filesystem>

using namespace types;

class PolicyRunnerBase{
public:
    PolicyRunnerBase(std::string policy_name):policy_name_(policy_name){
        vel_delta_const_ << 0.3, 0.2, 0.3;
        cmd_vel_input_.setZero();
    }
    virtual ~PolicyRunnerBase(){}
    /**
     * @brief use to display policy detail
     */
    virtual void DisplayPolicyInfo(){
        
    }

    /**
     * @brief Get the robot action object by run your policy
     * @return RobotAction 
     */
    virtual RobotAction getRobotAction(const RobotBasicState&, const UserCommand&) = 0;

    /**
     * @brief execute function when first entering policy runner
     */
    virtual void OnEnter(const RobotBasicState&) = 0;

    /**
     * @brief Set the decimation
     * @param  d decimation
     */
    virtual void SetDecimation(int d){
        decimation_ = d;
    }

    void LoadVelocityConfig(const std::string& policy_path) {
        namespace fs = std::filesystem;
        fs::path json_path = fs::path(policy_path).replace_extension(".json");
        std::ifstream f(json_path.string());
        if (f.is_open()) {
            try {
                nlohmann::json j;
                f >> j;
                if (j.contains("max_lin_vel_x")) max_lin_vel_x_ = j["max_lin_vel_x"];
                if (j.contains("max_lin_vel_y")) max_lin_vel_y_ = j["max_lin_vel_y"];
                if (j.contains("max_ang_vel_z")) max_ang_vel_z_ = j["max_ang_vel_z"];
                std::cout << "[VelConfig] Loaded from: " << json_path.string()
                          << " -> vx=" << max_lin_vel_x_ << " vy=" << max_lin_vel_y_
                          << " wz=" << max_ang_vel_z_ << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[VelConfig] JSON parse error: " << e.what()
                          << ", using defaults." << std::endl;
            }
        } else {
            std::cout << "[VelConfig] No config found: " << json_path.string()
                      << ", using defaults (vx=" << max_lin_vel_x_
                      << " vy=" << max_lin_vel_y_ << " wz=" << max_ang_vel_z_ << ")" << std::endl;
        }
    }

    Vec3f ClampCommand(const Vec3f& cmd) const {
        return Vec3f(
            std::clamp(cmd(0), -max_lin_vel_x_, max_lin_vel_x_),
            std::clamp(cmd(1), -max_lin_vel_y_, max_lin_vel_y_),
            std::clamp(cmd(2), -max_ang_vel_z_, max_ang_vel_z_)
        );
    }

    const std::string policy_name_;
    int decimation_;
    int run_cnt_;
    Vec3f vel_delta_const_, cmd_vel_input_;
    float max_lin_vel_x_ = 1.5f, max_lin_vel_y_ = 0.5f, max_ang_vel_z_ = 1.5f;
};

