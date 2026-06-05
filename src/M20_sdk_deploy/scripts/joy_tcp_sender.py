#!/usr/bin/env python3

import socket
import struct
import os
import time
import threading
import argparse

# ================= 配置区 =================
# 【重要】请将这里替换为你 M20 机载电脑在局域网中的实际 IP 地址
M20_IP = os.environ.get("M20_IP", "10.21.31.103")
TCP_PORT = int(os.environ.get("M20_TCP_PORT", "9999"))
JOY_DEVICE = os.environ.get("JOY_DEVICE", "/dev/input/js0")
SEND_FREQ = float(os.environ.get("JOY_SEND_FREQ", "200.0")) # 发送频率 200Hz
# ==========================================

# 全局状态缓存
axes = [0] * 8
buttons = [0] * 16
running = True

def js_reader_thread():
    """专门负责读取底层 JS 驱动事件的线程"""
    print(f"🎮 正在尝试打开本地手柄: {JOY_DEVICE}...")
    try:
        fd = os.open(JOY_DEVICE, os.O_RDONLY)
    except FileNotFoundError:
        print(f"❌ 错误: 找不到 {JOY_DEVICE}。请确保手柄已插好，并处于 XInput 模式。")
        global running
        running = False
        return

    print("✅ 手柄读取线程已启动。")
    try:
        while running:
            # os.read 会阻塞直到有事件发生
            ev_data = os.read(fd, 8)
            if ev_data:
                time_ms, value, ev_type, number = struct.unpack('IhBB', ev_data)
                # 屏蔽掉初始化的掩码 0x80
                ev_type &= ~0x80

                # 0x01 是按键 (Button), 0x02 是摇杆 (Axis)
                if ev_type == 0x01 and number < len(buttons):
                    buttons[number] = value
                elif ev_type == 0x02 and number < len(axes):
                    axes[number] = value
    except Exception as e:
        print(f"⚠️ 手柄读取中断: {e}")
    finally:
        os.close(fd)

def main():
    global running, M20_IP, TCP_PORT, JOY_DEVICE, SEND_FREQ

    parser = argparse.ArgumentParser(description="Send local Linux joystick state to M20 rl_deploy over TCP.")
    parser.add_argument("--ip", default=M20_IP)
    parser.add_argument("--port", type=int, default=TCP_PORT)
    parser.add_argument("--device", default=JOY_DEVICE)
    parser.add_argument("--rate", type=float, default=SEND_FREQ)
    args = parser.parse_args()
    M20_IP = args.ip
    TCP_PORT = args.port
    JOY_DEVICE = args.device
    SEND_FREQ = args.rate

    # 启动手柄读取后台线程
    threading.Thread(target=js_reader_thread, daemon=True).start()

    # 主线程负责维持 TCP 连接并以 50Hz 持续发送状态
    while running:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # 禁用 Nagle 算法，降低延迟

        try:
            print(f"📡 正在尝试连接至 M20 主机 {M20_IP}:{TCP_PORT} ...")
            sock.connect((M20_IP, TCP_PORT))
            print("✨ TCP 连接成功！开始高频传输控制指令 (按 Ctrl+C 退出)。")

            while running:
                # 按照 C++ 端的结构体打包数据 (8个int32 + 16个int32 = 96字节)
                packet = struct.pack('<' + 'i'*8 + 'i'*16, *axes, *buttons)

                # 封装 4 字节的长度头 (防止 TCP 粘包)
                header = struct.pack('<I', len(packet))

                # 发送: 长度头 + 数据负载
                sock.sendall(header + packet)

                # 维持固定发送频率 (50Hz)
                time.sleep(1.0 / SEND_FREQ)

        except ConnectionRefusedError:
            print("⏳ 连接被拒绝，M20 端的程序可能还未启动，1秒后重试...")
            time.sleep(1)
        except (ConnectionResetError, BrokenPipeError):
            print("⚠️ 连接意外断开，正在尝试重连...")
            time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 停止发送。")
            running = False
            break
        except Exception as e:
            if running:
                print(f"❌ 发生网络错误: {e}，1秒后重试...")
                time.sleep(1)
        finally:
            sock.close()

if __name__ == "__main__":
    main()
