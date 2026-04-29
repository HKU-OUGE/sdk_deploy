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
#include <sstream>
#include <iomanip>

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

protected:
    int dashboard_lines_ = 0;

    struct DebugState {
        float robot_x = 0, robot_y = 0, robot_z = 0, robot_yaw = 0;
        bool tf_valid = false;
        bool has_map = false;
        int valid_h_count = 0;
        float hole_ratio_pct = 0;
        float min_fwd = 2.5f, min_bwd = 2.5f;
        float cmd_vx = 0, cmd_vy = 0, cmd_wz = 0;
        float fk_z = 0, z_error = 0;
        bool has_fk = false;
    } dbg_;

    void PrintDebugDashboard(const char* mode_label, int step) {
        std::ostringstream os;
        os << std::fixed;

        if (dashboard_lines_ > 0) {
            os << "\033[" << dashboard_lines_ << "A\r";
        }

        int lines = 0;

        os << "\033[36m─── M20 RL [\033[1m" << mode_label
           << "\033[0;36m] ── Step " << step
           << " ──────────────────────────────────\033[0m\033[K\n";
        lines++;

        os << "  \033[1mPose\033[0m   X:" << std::setprecision(2) << std::setw(7) << dbg_.robot_x
           << "  Y:" << std::setw(7) << dbg_.robot_y
           << "  Z:" << std::setw(6) << dbg_.robot_z
           << "  Yaw:" << std::setw(6) << dbg_.robot_yaw;
        if (dbg_.has_map)
            os << (dbg_.tf_valid ? "  \033[32m[TF]\033[0m" : "  \033[33m[Map]\033[0m");
        os << "\033[K\n";
        lines++;

        os << "  \033[1mCmd\033[0m    Vx:" << std::setprecision(2) << std::setw(6) << dbg_.cmd_vx
           << "  Vy:" << std::setw(6) << dbg_.cmd_vy
           << "  Wz:" << std::setw(6) << dbg_.cmd_wz
           << "\033[K\n";
        lines++;

        os << "  \033[1mMap\033[0m    ";
        if (dbg_.has_map) {
            float pct = dbg_.hole_ratio_pct;
            const char* color = pct > 50.0f ? "\033[33m" : "\033[0m";
            os << "Holes:" << color << std::setprecision(1) << std::setw(5) << pct << "%\033[0m"
               << "  Valid: " << dbg_.valid_h_count << "/187";
        } else {
            os << "\033[90m-- waiting --\033[0m";
        }
        os << "\033[K\n";
        lines++;

        os << "  \033[1mLidar\033[0m  fwd:" << std::setprecision(2) << std::setw(5) << dbg_.min_fwd
           << "m  bwd:" << std::setw(5) << dbg_.min_bwd << "m\033[K\n";
        lines++;

        os << "  \033[1mFK\033[0m     ";
        if (dbg_.has_fk) {
            const char* color = std::abs(dbg_.z_error) > 0.05f ? "\033[33m" : "\033[32m";
            os << "TF:" << std::setprecision(3) << std::setw(6) << dbg_.robot_z
               << "  Est:" << std::setw(6) << dbg_.fk_z
               << "  Err:" << color << std::showpos << std::setw(7) << dbg_.z_error
               << std::noshowpos << "m\033[0m";
        } else {
            os << "\033[90m-- no TF data --\033[0m";
        }
        os << "\033[K\n";
        lines++;

        os << "\033[36m──────────────────────────────────────────────────────────────\033[0m\033[K\n";
        lines++;

        std::cout << os.str() << std::flush;
        dashboard_lines_ = lines;
    }
};

