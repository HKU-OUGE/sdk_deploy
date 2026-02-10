"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco with Auto-Terrain Loading and Joystick/Keyboard Control
 * @author Bo (Percy) Peng / Modified for Terrain & Control
 * @version 2.2 (Added CLI Args & Full Joystick Mapping)
 * @date 2025-02-17
 *
 * @copyright Copyright (c) 2025 DeepRobotics
"""

import sys
# print("[Debug] Script starting...", file=sys.stderr, flush=True)

import os
import time
import socket
import struct
import threading
import traceback
import argparse # [新增] 用于解析命令行参数
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer
import xml.etree.ElementTree as ET

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd, GamepadData
    # print("[Debug] Imports successful.", file=sys.stderr, flush=True)
except ImportError as e:
    print(f"[Critical Error] Import failed: {e}", file=sys.stderr, flush=True)
    sys.exit(1)

# ... [Keep Definitions] ...
MODEL_NAME = "M20"
CURRENT_DIR = Path(__file__).resolve().parent

# Define the paths
ROBOT_XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "model" / "M20" / "mjcf" / "M20.xml"
TERRAIN_XML_PATH = CURRENT_DIR / "m20_terrain.xml"

ROBOT_XML_PATH = str(ROBOT_XML_PATH.resolve())
TERRAIN_XML_PATH = str(TERRAIN_XML_PATH.resolve())

USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50

# Calibaration parameters (from V1.0)
JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
POS_OFFSET_DEG = np.array([-25, 229, 160, 0, 25, -131, -200, 0, -25, -229, -160, 0, 25, 131, 200, 0], dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

JOINT_INIT = {
    "M20": np.array([-0.438, -1.16, 2.76, 0,
                     0.438, -1.16, 2.76, 0,
                     -0.438, 1.16, -2.76, 0,
                     0.438, 1.16, -2.76, 0], dtype=np.float32),
}

def merge_xmls(robot_path, terrain_path):
    """
    Helper function to merge Terrain XML assets and geoms into Robot XML in-memory.
    """
    print(f"[Sim] Loading Robot: {robot_path}")
    if not os.path.exists(terrain_path):
        print(f"[Sim] Warning: Terrain file not found at {terrain_path}. Loading robot only.")
        with open(robot_path, 'r') as f:
            return f.read()

    print(f"[Sim] Merging Terrain: {terrain_path}")
    
    try:
        # 1. Parse both XMLs
        robot_tree = ET.parse(robot_path)
        robot_root = robot_tree.getroot()
        
        terrain_tree = ET.parse(terrain_path)
        terrain_root = terrain_tree.getroot()
        
        # 2. Find key sections
        robot_worldbody = robot_root.find("worldbody")
        terrain_worldbody = terrain_root.find("worldbody")
        
        robot_asset = robot_root.find("asset")
        terrain_asset = terrain_root.find("asset")
        
        # 3. Merge Assets (Materials/Textures)
        if terrain_asset is not None:
            if robot_asset is None:
                robot_asset = ET.SubElement(robot_root, "asset")
            for item in terrain_asset:
                robot_asset.append(item)
                
        # 4. Merge Worldbody Geoms (The actual terrain)
        if terrain_worldbody is not None and robot_worldbody is not None:
            for item in terrain_worldbody:
                # 过滤掉 light/floor 避免冲突
                if item.tag == "light" and robot_worldbody.find("light") is not None:
                    continue
                if item.get("name") == "floor" and robot_worldbody.find(".//geom[@name='floor']") is not None:
                    continue
                robot_worldbody.append(item)

        return ET.tostring(robot_root, encoding='unicode')

    except Exception as e:
        print(f"[Sim] Error merging XMLs: {e}")
        with open(robot_path, 'r') as f:
            return f.read()

class JoystickInterface:
    def __init__(self, node):
        self.node = node
        self.active = False
        # Use SensorData QoS (Best Effort) for Joystick to ensure reception by C++ nodes
        self.pub = node.create_publisher(GamepadData, '/GAMEPAD_DATA', qos_profile_sensor_data)
        self.js = None
        self.last_print_time = 0
        
        try:
            import pygame
            self.pygame = pygame
            if not self.pygame.get_init(): self.pygame.init()
            if not self.pygame.joystick.get_init(): self.pygame.joystick.init()
            if self.pygame.joystick.get_count() > 0:
                self.js = self.pygame.joystick.Joystick(0)
                self.js.init()
                self.active = True
                self.node.get_logger().info(f"[Joystick] Connected: {self.js.get_name()}")
                self.node.get_logger().info("[Joystick] Controls Mapped: Left Stick (Move), Right Stick (Turn), A(Damping), B(Stand), X(Control)")
            else:
                self.node.get_logger().warn("[Joystick] No joystick found.")
        except ImportError:
            self.node.get_logger().warn("[Joystick] 'pygame' not installed.")
        except Exception as e:
            self.node.get_logger().error(f"[Joystick] Init error: {e}")

    def update(self):
        if not self.active or self.js is None: return
        try:
            self.pygame.event.pump()
            msg = GamepadData()
            msg.header = MetaType()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 0
            
            # Default values
            msg.left_axis_x = 0.0; msg.left_axis_y = 0.0
            msg.right_axis_x = 0.0; msg.right_axis_y = 0.0
            msg.buttons = 0

            num_axes = self.js.get_numaxes()
            
            # 轴映射逻辑 (Xbox Standard)
            # Axis 0: Left Stick X (Left/Right) -> Side velocity (A/D)
            # Axis 1: Left Stick Y (Up/Down)    -> Forward velocity (W/S)
            # Axis 3: Right Stick X (Left/Right)-> Turning velocity (Q/E)
            
            # Pygame: Left = -1, Right = 1; Up = -1, Down = 1
            # Robot: Forward = +Vel_x, Left = +Vel_y, Turn Left = +Omega_z
            
            if num_axes > 0: msg.left_axis_x = -float(self.js.get_axis(0)) # Robot Left (+y) is Stick Left (-x) -> Invert
            if num_axes > 1: msg.left_axis_y = -float(self.js.get_axis(1)) # Robot Forward (+x) is Stick Up (-y) -> Invert
            
            if num_axes >= 4:
                # Right Stick X for turning
                turn_axis_idx = 3 if num_axes >= 5 else 2 
                msg.right_axis_x = -float(self.js.get_axis(turn_axis_idx)) # Turn Left (+w) is Stick Left (-x) -> Invert
                
                # Right Stick Y (Optional, maybe pitch?)
                # msg.right_axis_y = ...

            # 按钮映射逻辑 (Mapping Keyboard Logic)
            # Mode: R (Damping) -> Button A (0)
            #       Z (Stand)   -> Button B (1)
            #       C (Control) -> Button X (2)
            
            buttons_mask = 0
            num_buttons = self.js.get_numbuttons()
            
            # Xbox Button Map: A=0, B=1, X=2, Y=3, LB=4, RB=5...
            if num_buttons > 0 and self.js.get_button(0): buttons_mask |= (1 << 0) # A -> Damping (Mode 0?)
            if num_buttons > 1 and self.js.get_button(1): buttons_mask |= (1 << 1) # B -> Stand (Mode 1?)
            if num_buttons > 2 and self.js.get_button(2): buttons_mask |= (1 << 2) # X -> Control (Mode 2?)
            
            # Keep original raw button mapping too if needed, but for now we just fill the mask
            # If your C++ node expects specific bits for modes, ensure these match!
            # Assuming C++ maps: 
            #   Bit 0 -> Damping? 
            #   Bit 1 -> Stand?
            #   Bit 2 -> Control?
            # Or does C++ read axes directly for velocity? Yes usually.
            
            # Fill the rest of the buttons just in case
            for i in range(min(16, num_buttons)):
                if self.js.get_button(i): buttons_mask |= (1 << i)
            
            msg.buttons = buttons_mask
            
            self.pub.publish(msg)
            
            # [Debug] Print inputs if significant
            if (abs(msg.left_axis_y) > 0.1 or abs(msg.left_axis_x) > 0.1 or abs(msg.right_axis_x) > 0.1) and (time.time() - self.last_print_time > 1.0):
                # print(f"[Joystick] Publishing: Fwd={msg.left_axis_y:.2f}, Side={msg.left_axis_x:.2f}, Turn={msg.right_axis_x:.2f}, Btn={msg.buttons}", file=sys.stderr)
                self.last_print_time = time.time()
                
        except Exception as e:
            self.node.get_logger().error(f"[Joystick] Runtime error: {e}")
            self.active = False

class MuJoCoSimulationNode(Node):
    def __init__(self, model_key: str = MODEL_NAME, use_joystick: bool = False):
        super().__init__('mujoco_simulation')
        # print("[Debug] Node initializing...", file=sys.stderr, flush=True)
        xml_string = merge_xmls(ROBOT_XML_PATH, TERRAIN_XML_PATH)
        robot_dir = Path(ROBOT_XML_PATH).parent
        merged_xml_path = robot_dir / "merged_scene_autogen.xml"
        try:
            with open(merged_xml_path, 'w') as f: f.write(xml_string)
            self.model = mujoco.MjModel.from_xml_path(str(merged_xml_path))
        except Exception as e:
            self.get_logger().error(f"Failed to load merged XML: {e}")
            self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)
        self.actuator_ids = [a for a in range(self.model.nu)]
        self.dof_num = len(self.actuator_ids)
        self._set_initial_pose(model_key)
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)
        self.timestamp = 0.0
        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")
        
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.cmd_sub = self.create_subscription(JointsDataCmd, '/JOINTS_CMD', self._cmd_callback, 50)
        
        # Joystick
        self.use_joystick = use_joystick
        if self.use_joystick:
            self.joystick = JoystickInterface(self)
        else:
            self.joystick = None
            self.get_logger().info("[Sim] Joystick disabled by default (use --joystick to enable)")
        
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        # print("[Debug] Node init complete.", file=sys.stderr, flush=True)

    def _set_initial_pose(self, key: str):
        if key in JOINT_INIT:
            qpos0 = self.data.qpos.copy()
            qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
            qpos0[:3] = np.array([0, 0, 0.45])
            qpos0[3:7] = np.array([1, 0, 0, 0])
            self.data.qpos[:] = qpos0
            mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsDataCmd):
        if hasattr(msg.data, 'joints_data'): data_list = msg.data.joints_data
        elif hasattr(msg.data, 'joints_cmd'): data_list = msg.data.joints_cmd
        else: return
        if len(data_list) != self.dof_num: return
        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = data_list[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque 
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR

    def _apply_joint_torque(self):
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (self.kp_cmd * (self.pos_cmd - q) + self.kd_cmd * (self.vel_cmd - dq) + self.tau_ff)
        self.data.ctrl[:] = self.input_tq.flatten()

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        t0 = 2.0 * (w * x + y * z); t1 = 1.0 - 2.0 * (x * x + y * y); roll = np.arctan2(t0, t1)
        t2 = 2.0 * (w * y - z * x); t2 = np.clip(t2, -1.0, 1.0); pitch = np.arcsin(t2)
        t3 = 2.0 * (w * z + x * y); t4 = 1.0 - 2.0 * (y * y + z * z); yaw = np.arctan2(t3, t4)
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_robot_state(self, step: int):
        q_world = self.data.sensordata[:4]; rpy_rad = self.quaternion_to_euler(q_world)
        rpy_deg = [angle * (180.0 / np.pi) for angle in rpy_rad]
        body_acc = self.data.sensordata[4:7]; angvel_b = self.data.sensordata[7:10]
        imu_msg = ImuData(); imu_msg.header = MetaType(); imu_msg.header.frame_id = 0
        stamp = Time(); sec = int(self.timestamp); nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec; stamp.nanosec = nanosec; imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue(); imu_msg.data.roll = float(rpy_deg[0]); imu_msg.data.pitch = float(rpy_deg[1]); imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0]); imu_msg.data.omega_y = float(angvel_b[1]); imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0]); imu_msg.data.acc_y = float(body_acc[1]); imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)
        q = self.data.qpos[7:7 + self.dof_num]; dq = self.data.qvel[6:6 + self.dof_num]; tau = self.input_tq.flatten()
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR; pub_vel = dq * JOINT_DIR; pub_tau = tau * JOINT_DIR 
        joints_msg = JointsData(); joints_msg.header = MetaType(); joints_msg.header.frame_id = 0; joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue(); joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]; joint.name = [32]*4; joint.status_word = 1
            joint.position = float(pub_pos[i]); joint.torque = float(pub_tau[i]); joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0; joint.driver_temp = 40.0
        self.joints_pub.publish(joints_msg)

    def start(self):
        step = 0; last_time = time.time()
        print("[Sim] Starting simulation loop...", file=sys.stderr, flush=True)
        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time(); step += 1
                self._apply_joint_torque()
                mujoco.mj_step(self.model, self.data)
                self.timestamp = step * DT
                if step % 5 == 0: self._publish_robot_state(step)
                
                # [新增] 条件更新手柄
                if self.use_joystick and step % 10 == 0: 
                    self.joystick.update()
                    
                if self.viewer and step % RENDER_INTERVAL == 0: self.viewer.sync()
            rclpy.spin_once(self, timeout_sec=0.0)

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    
    # [新增] 命令行参数解析
    parser = argparse.ArgumentParser(description="MuJoCo Simulation for M20")
    parser.add_argument("--joystick", action="store_true", help="Enable joystick/gamepad control")
    parser.add_argument("--keyboard", action="store_true", help="Enable keyboard control (Default)")
    
    # 因为 rclpy 也会解析参数，所以我们需要过滤掉 ROS 相关的参数
    # 或者先初始化 rclpy，再解析我们自己的参数
    
    # 策略：先用 argparse 解析已知参数，剩下的留给 rclpy
    args, unknown = parser.parse_known_args()
    
    # 默认是 keyboard，如果指定了 joystick 则覆盖
    use_joystick = args.joystick
    
    try:
        rclpy.init(args=unknown)
        sim_node = MuJoCoSimulationNode(use_joystick=use_joystick)
        sim_node.start()
    except KeyboardInterrupt: print("[Sim] Stopping...", file=sys.stderr)
    except Exception: traceback.print_exc()
    finally:
        if 'sim_node' in locals():
            if sim_node.viewer: sim_node.viewer.close()
            sim_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()