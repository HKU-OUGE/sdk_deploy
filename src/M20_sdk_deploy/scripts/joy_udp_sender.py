import socket
import struct
import os
import time

# ================= 配置区 =================
# 【重要】请将这里替换为你 M20 机载电脑在局域网中的实际 IP 地址
M20_IP = "192.168.1.100"
UDP_PORT = 9999
JOY_DEVICE = "/dev/input/js0"
# ==========================================

def main():
    print(f"正在尝试打开本地手柄: {JOY_DEVICE}...")
    try:
        fd = os.open(JOY_DEVICE, os.O_RDONLY)
    except FileNotFoundError:
        print(f"错误: 找不到 {JOY_DEVICE}。请确保手柄已插在笔记本上，并且处于 XInput 模式。")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"连接成功！正在将手柄数据通过 UDP 发送至 {M20_IP}:{UDP_PORT} ...")
    print("按 Ctrl+C 退出。")

    # 状态缓存：8个轴，16个按键
    axes = [0] * 8
    buttons = [0] * 16

    try:
        while True:
            # 读取 Linux 底层的 js_event 结构 (8 bytes)
            # struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; };
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

                # 打包数据发送：采用小端序(<)，发送 8个 int32 和 16个 int32 (共96字节)
                packet = struct.pack('<' + 'i'*8 + 'i'*16, *axes, *buttons)
                sock.sendto(packet, (M20_IP, UDP_PORT))

    except KeyboardInterrupt:
        print("\n停止发送。")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        os.close(fd)
        sock.close()

if __name__ == "__main__":
    main()