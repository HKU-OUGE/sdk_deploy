/**
 * @file joy_interface.h
 * @brief Dual-Mode Joystick Driver for M20 (Supports Local USB & Remote UDP)
 */
#pragma once

#include "user_command_interface.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>

// USB 手柄所需头文件
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

// UDP 通信所需头文件
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

using namespace interface;
using namespace types;

// 定义和 Python 端一致的数据包结构
#pragma pack(push, 1)
struct JoyDataPacket {
    int32_t axes[8];
    int32_t buttons[16];
};
#pragma pack(pop)

class JoyInterface : public UserCommandInterface
{
private:
    std::thread poll_thread_;
    std::atomic<bool> running_{false};

    // 缓存手柄状态
    std::vector<int> axis_values_;
    std::vector<int> button_values_;

    // --- 手柄映射 (Logitech F710 XInput 模式) ---
    const int RAW_AXIS_LX = 0;
    const int RAW_AXIS_LY = 1;
    const int RAW_AXIS_RX = 3;

    const int RAW_AXIS_DPAD_X = 6;
    const int RAW_AXIS_DPAD_Y = 7;

    const int RAW_BTN_A = 0;
    const int RAW_BTN_B = 1;
    const int RAW_BTN_X = 2;
    const int RAW_BTN_Y = 3;

    float max_forward_ = 0.5f;
    float max_side_    = 1.0f;
    float max_yaw_     = 1.0f;

    float normalize_axis(int val) {
        const int deadzone = 4000;
        if (std::abs(val) < deadzone) return 0.0f;
        return static_cast<float>(val) / 32767.0f;
    }

    double GetCurrentTimeStamp() {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }

    void poll_loop() {
        axis_values_.resize(16, 0);
        button_values_.resize(16, 0);

        // ================= [初始化 UDP 通信] =================
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd >= 0) {
            struct sockaddr_in servaddr;
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = INADDR_ANY;
            servaddr.sin_port = htons(9999);

            // 将 UDP Socket 设置为非阻塞模式
            int flags = fcntl(sockfd, F_GETFL, 0);
            fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
            bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        }

        // ================= [初始化 USB 手柄] =================
        const char* device_path = "/dev/input/js0";
        int fd_js = -1;

        double last_recv_time = GetCurrentTimeStamp();

        std::cout << "\n[JoyInterface] >>> SUCCESS: Dual-Mode Initialized! <<<\n";
        std::cout << "  > Mode 1: Auto-detect local USB Joystick at " << device_path << "\n";
        std::cout << "  > Mode 2: Listening for UDP remote data on port 9999\n";

        while (running_) {
            bool received_data = false;

            // --- 1. 尝试读取本地 USB 手柄 (非阻塞) ---
            if (fd_js < 0) {
                fd_js = open(device_path, O_RDONLY | O_NONBLOCK);
            }
            if (fd_js >= 0) {
                struct js_event js;
                while (read(fd_js, &js, sizeof(js)) > 0) {
                    received_data = true;
                    switch (js.type & ~JS_EVENT_INIT) {
                        case JS_EVENT_AXIS:
                            if (js.number < axis_values_.size()) axis_values_[js.number] = js.value;
                            break;
                        case JS_EVENT_BUTTON:
                            if (js.number < button_values_.size()) button_values_[js.number] = js.value;
                            break;
                    }
                }
                // 检查设备是否被拔出
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    close(fd_js); fd_js = -1;
                }
            }

            // --- 2. 尝试读取 UDP 远程数据 (非阻塞) ---
            if (sockfd >= 0) {
                JoyDataPacket packet;
                struct sockaddr_in cliaddr;
                socklen_t len = sizeof(cliaddr);
                // 循环读取清空缓冲区，确保只使用最新的一帧
                while (recvfrom(sockfd, &packet, sizeof(packet), 0, (struct sockaddr *)&cliaddr, &len) == sizeof(JoyDataPacket)) {
                    received_data = true;
                    for(int i = 0; i < 8; i++) axis_values_[i] = packet.axes[i];
                    for(int i = 0; i < 16; i++) button_values_[i] = packet.buttons[i];
                }
            }

            // --- 3. 安全断联保护机制 ---
            if (received_data) {
                last_recv_time = GetCurrentTimeStamp();
            } else if (GetCurrentTimeStamp() - last_recv_time > 500.0) {
                // 如果超过 500ms 无论本地还是远程都没有数据更新，自动松开所有按键和摇杆
                std::fill(axis_values_.begin(), axis_values_.end(), 0);
                std::fill(button_values_.begin(), button_values_.end(), 0);
            }

            // --- 4. 核心逻辑：将缓存的状态映射到 Robot UserCommand ---
            usr_cmd_->time_stamp = GetCurrentTimeStamp();

            if (button_values_[RAW_BTN_B]) {
                 usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            }
            else if (button_values_[RAW_BTN_A]) {
                if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand) {
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                }
            }
            else if (button_values_[RAW_BTN_X]) {
                if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                }
            }
            else if (button_values_[RAW_BTN_Y]) {
                if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::RLSensorControlMode);
                }
            }

            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode ||
                msfb_->GetCurrentState() == RobotMotionState::RLSensorControlMode) {

                float stick_fwd  = -normalize_axis(axis_values_[RAW_AXIS_LY]);
                float stick_side = -normalize_axis(axis_values_[RAW_AXIS_LX]);
                float dpad_fwd   = -normalize_axis(axis_values_[RAW_AXIS_DPAD_Y]);
                float dpad_side  = -normalize_axis(axis_values_[RAW_AXIS_DPAD_X]);

                float fwd = 0.0f;
                float side = 0.0f;

                if (std::abs(dpad_fwd) > 0.1f || std::abs(dpad_side) > 0.1f) {
                    fwd  = dpad_fwd;
                    side = dpad_side;
                } else {
                    fwd  = stick_fwd;
                    side = stick_side;
                }

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

        if(sockfd >= 0) close(sockfd);
        if(fd_js >= 0) close(fd_js);
    }

public:
    JoyInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
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