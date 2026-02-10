/**
 * @file joy_interface.hpp
 * @brief Direct Joystick Driver for M20 (Bypasses ROS DDS isolation)
 */
#pragma once

#include "user_command_interface.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <chrono>

using namespace interface;
using namespace types;

class JoyInterface : public UserCommandInterface
{
private:
    std::thread poll_thread_;
    std::atomic<bool> running_{false};

    // 缓存手柄状态
    std::vector<int> axis_values_;
    std::vector<int> button_values_;

    // --- 手柄映射 (Logitech F710 XInput 模式) ---
    // 轴索引 (Linux js API 原始索引)
    const int RAW_AXIS_LX = 0; // 左摇杆 X (左右)
    const int RAW_AXIS_LY = 1; // 左摇杆 Y (上下)
    const int RAW_AXIS_RX = 3; // 右摇杆 X (转向)

    // 按键索引
    const int RAW_BTN_A = 0;
    const int RAW_BTN_B = 1;
    const int RAW_BTN_X = 2;
    const int RAW_BTN_Y = 3;

    // 速度参数
    float max_forward_ = 1.0f;
    float max_side_    = 0.5f;
    float max_yaw_     = 1.0f;

    // 辅助函数：将 raw short (-32767~32767) 归一化为 float (-1.0~1.0)
    float normalize_axis(int val) {
        const int deadzone = 4000; // 死区
        if (std::abs(val) < deadzone) return 0.0f;
        return static_cast<float>(val) / 32767.0f;
    }

    double GetCurrentTimeStamp() {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }

    void poll_loop() {
        const char* device_path = "/dev/input/js0";
        int fd = -1;

        // 初始状态缓存 (足够容纳常见手柄)
        axis_values_.resize(16, 0);
        button_values_.resize(16, 0);

        std::cout << "[JoyInterface] Driver thread started. Trying to open " << device_path << "...\n";

        while (running_) {
            // 1. 尝试打开设备
            if (fd < 0) {
                fd = open(device_path, O_RDONLY | O_NONBLOCK);
                if (fd >= 0) {
                    std::cout << "\n[JoyInterface] >>> SUCCESS: Joystick Connected directly! <<<\n";
                    std::cout << "  > Press 'B' for Damping\n";
                    std::cout << "  > Press 'A' for Stand Up\n";
                    std::cout << "  > Press 'X' for RL Control\n";
                } else {
                    // 如果打开失败，每秒重试一次
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            // 2. 读取事件
            struct js_event js;
            while (read(fd, &js, sizeof(js)) > 0) {
                // 处理事件
                switch (js.type & ~JS_EVENT_INIT) {
                    case JS_EVENT_AXIS:
                        if (js.number < axis_values_.size())
                            axis_values_[js.number] = js.value;
                        break;
                    case JS_EVENT_BUTTON:
                        if (js.number < button_values_.size())
                            button_values_[js.number] = js.value;
                        break;
                }
            }

            // 检查设备是否断开 (read 返回 -1 且 errno != EAGAIN)
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // 设备可能断开了
                close(fd);
                fd = -1;
                std::cout << "[JoyInterface] Joystick disconnected. Reconnecting...\n";
                continue;
            }

            // 3. 核心逻辑：将缓存的状态映射到 Robot UserCommand
            usr_cmd_->time_stamp = GetCurrentTimeStamp(); // 持续更新心跳

            // --- 模式切换逻辑 ---
            if (button_values_[RAW_BTN_B]) {
                 usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                 // std::cout << "\r[CMD] Joint Damping    " << std::flush;
            }
            else if (button_values_[RAW_BTN_A]) {
                if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand) {
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                    // std::cout << "\r[CMD] Standing Up      " << std::flush;
                }
            }
            else if (button_values_[RAW_BTN_X]) {
                if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                    // std::cout << "\r[CMD] RL Control       " << std::flush;
                }
            }

            // --- 速度控制逻辑 ---
            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                // 注意：Linux Joystick Y轴通常向下为正，所以前进(向上推)是负值，需要取反
                float fwd = -normalize_axis(axis_values_[RAW_AXIS_LY]);
                float side = -normalize_axis(axis_values_[RAW_AXIS_LX]);
                float yaw = -normalize_axis(axis_values_[RAW_AXIS_RX]);

                usr_cmd_->forward_vel_scale  = fwd * max_forward_;
                usr_cmd_->side_vel_scale     = side * max_side_;
                usr_cmd_->turnning_vel_scale = yaw * max_yaw_;
            } else {
                usr_cmd_->forward_vel_scale = 0;
                usr_cmd_->side_vel_scale = 0;
                usr_cmd_->turnning_vel_scale = 0;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (fd >= 0) close(fd);
    }

public:
    JoyInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
        std::cout << "[JoyInterface] Initialized (Direct /dev/input/js0 mode).\n";
    }

    ~JoyInterface() { Stop(); }

    void Start() override {
        if (running_) return;
        running_ = true;
        poll_thread_ = std::thread(&JoyInterface::poll_loop, this);
    }

    void Stop() override {
        if (!running_) return;
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
    }

    UserCommand* GetUserCommand() override { return usr_cmd_; }
};