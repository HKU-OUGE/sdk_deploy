/**
 * @file joy_interface.h
 * @brief Dual-Mode Joystick Driver for M20 (Supports Local USB & Remote TCP)
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

// TCP 通信所需头文件
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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
    std::thread tcp_thread_;
    std::atomic<bool> running_{false};

    // 缓存手柄状态
    std::vector<int> axis_values_;
    std::vector<int> button_values_;

    // TCP 数据缓存与锁
    std::mutex tcp_mutex_;
    JoyDataPacket latest_tcp_packet_;
    double last_tcp_recv_time_ = 0.0;

    // --- 手柄映射 (Logitech F710 / Xbox XInput 模式) ---
    const int RAW_AXIS_LX = 0;
    const int RAW_AXIS_LY = 1;
    const int RAW_AXIS_RX = 3;

    const int RAW_AXIS_DPAD_X = 6;
    const int RAW_AXIS_DPAD_Y = 7;

    const int RAW_BTN_A = 0;
    const int RAW_BTN_B = 1;
    const int RAW_BTN_X = 2;
    const int RAW_BTN_Y = 3;

    float max_forward_ = 1.0f;
    float max_side_    = 0.0f;
    float max_yaw_     = 1.5f;

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

    // ==============================================================
    // 后台 TCP 服务端线程 (负责监听端口，接收定长防粘包数据)
    // ==============================================================
    void tcp_worker() {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(9999);

        bind(server_fd, (struct sockaddr *)&address, sizeof(address));
        listen(server_fd, 1);

        while (running_) {
            // 使用 select 允许定期检查 running_ 状态，以便优雅退出
            struct timeval tv;
            tv.tv_sec = 1; tv.tv_usec = 0;
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd, &readfds);

            int activity = select(server_fd + 1, &readfds, NULL, NULL, &tv);
            if (activity > 0 && FD_ISSET(server_fd, &readfds)) {
                int client_fd = accept(server_fd, nullptr, nullptr);
                if (client_fd < 0) continue;

                // 配置客户端 Socket 选项 (禁用 Nagle 降低延迟，设置接收超时防死锁)
                int nodelay = 1;
                setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
                struct timeval timeout;
                timeout.tv_sec = 1; timeout.tv_usec = 0;
                setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

                std::cout << "\n[JoyInterface] ✨ TCP Joystick Connected from Laptop!\n";

                while (running_) {
                    uint32_t frame_len = 0;
                    // 1. 读取 4 字节的长度头
                    if (recv(client_fd, &frame_len, 4, MSG_WAITALL) <= 0) break;

                    // 2. 长度校验与读取负载
                    if (frame_len == sizeof(JoyDataPacket)) {
                        JoyDataPacket pkt;
                        if (recv(client_fd, &pkt, frame_len, MSG_WAITALL) <= 0) break;

                        // 加锁更新缓存
                        {
                            std::lock_guard<std::mutex> lock(tcp_mutex_);
                            latest_tcp_packet_ = pkt;
                            last_tcp_recv_time_ = GetCurrentTimeStamp();
                        }
                    } else {
                        // 收到不符合期望长度的数据，清理缓冲区以同步流
                        std::vector<uint8_t> dump(frame_len);
                        if (recv(client_fd, dump.data(), frame_len, MSG_WAITALL) <= 0) break;
                        std::cout << "[JoyInterface] ⚠️ Warning: Received unknown packet size: " << frame_len << "\n";
                    }
                }
                close(client_fd);
                std::cout << "\n[JoyInterface] 🔌 TCP Joystick Disconnected. Waiting for reconnect...\n";
            }
        }
        close(server_fd);
    }

    void poll_loop() {
        axis_values_.resize(16, 0);
        button_values_.resize(16, 0);

        // 初始化最新数据包
        memset(&latest_tcp_packet_, 0, sizeof(latest_tcp_packet_));

        // ================= [初始化 USB 手柄] =================
        const char* device_path = "/dev/input/js0";
        int fd_js = -1;
        bool usb_connected = false;

        std::cout << "\n[JoyInterface] >>> SUCCESS: Dual-Mode Initialized! <<<\n";
        std::cout << "  > Mode 1: Auto-detect local USB Joystick at " << device_path << "\n";
        std::cout << "  > Mode 2: TCP Server listening for Laptop remote data on port 9999\n";

        while (running_) {
            // --- 1. 读取本地 USB 手柄 (非阻塞) ---
            if (fd_js < 0) {
                fd_js = open(device_path, O_RDONLY | O_NONBLOCK);
            }
            usb_connected = (fd_js >= 0);

            if (usb_connected) {
                struct js_event js;
                while (read(fd_js, &js, sizeof(js)) > 0) {
                    switch (js.type & ~JS_EVENT_INIT) {
                        case JS_EVENT_AXIS:
                            if (js.number < axis_values_.size()) axis_values_[js.number] = js.value;
                            break;
                        case JS_EVENT_BUTTON:
                            if (js.number < button_values_.size()) button_values_[js.number] = js.value;
                            break;
                    }
                }

                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    close(fd_js);
                    fd_js = -1;
                    usb_connected = false;
                    std::cout << "\n[JoyInterface] WARNING: Local USB Joystick disconnected.\n";
                }
            }

            // --- 2. 融合 TCP 远程数据 ---
            bool tcp_connected = false;
            {
                std::lock_guard<std::mutex> lock(tcp_mutex_);
                // 修复：将 0.5 改为 500.0 (即 0.5 秒超时断连保护)
                if (GetCurrentTimeStamp() - last_tcp_recv_time_ < 500.0) {
                    for(int i = 0; i < 8; i++) axis_values_[i] = latest_tcp_packet_.axes[i];
                    for(int i = 0; i < 16; i++) button_values_[i] = latest_tcp_packet_.buttons[i];
                    tcp_connected = true;
                }
            }

            // --- 3. 断联保护机制 (急停) ---
            // 只有当本地 USB 断开，且远程 TCP 也超时断开的情况下，强制归零（急停）
            if (!usb_connected && !tcp_connected) {
                std::fill(axis_values_.begin(), axis_values_.end(), 0);
                std::fill(button_values_.begin(), button_values_.end(), 0);
            }

            // --- 4. 核心逻辑：状态映射 ---
            usr_cmd_->time_stamp = GetCurrentTimeStamp();

            if (button_values_[RAW_BTN_B]) {
                usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            }
            else if (button_values_[RAW_BTN_A]) {
                usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
            }
            else if (button_values_[RAW_BTN_X]) {
                usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
            }
            else if (button_values_[RAW_BTN_Y]) {
                usr_cmd_->target_mode = uint8_t(RobotMotionState::RLSensorControlMode);
            }

            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode ||
                msfb_->GetCurrentState() == RobotMotionState::RLSensorControlMode) {

                float stick_fwd  = -normalize_axis(axis_values_[RAW_AXIS_LY]);
                float stick_side = -normalize_axis(axis_values_[RAW_AXIS_LX]);
                float dpad_fwd   = -normalize_axis(axis_values_[RAW_AXIS_DPAD_Y]);
                float dpad_side  = -normalize_axis(axis_values_[RAW_AXIS_DPAD_X]);

                float fwd = 0.0f;
                float side = 0.0f;

                if (std::abs(dpad_fwd) > 0.8f || std::abs(dpad_side) > 0.8f) {
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

            // 控制循环周期 5ms (200Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

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

        // 启动主轮询线程和 TCP 监听后台线程
        poll_thread_ = std::thread(&JoyInterface::poll_loop, this);
        tcp_thread_ = std::thread(&JoyInterface::tcp_worker, this);
    }

    void Stop() override {
        if (!running_) return;
        running_ = false;

        // 优雅退出线程
        if (tcp_thread_.joinable()) tcp_thread_.join();
        if (poll_thread_.joinable()) poll_thread_.join();
    }

    UserCommand* GetUserCommand() override { return usr_cmd_; }
};