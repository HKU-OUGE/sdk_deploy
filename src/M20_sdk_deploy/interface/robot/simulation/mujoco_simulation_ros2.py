"""
 * @file mujoco_simulation_ros2.py
 * @brief simulation in mujoco with Real Mid360 LiDAR Simulation (Aligned with Isaac Sim Pose)
 * @author Bo (Percy) Peng / Modified for Sim2Sim True PointCloud
 * @version 2.6
 * @copyright Copyright (c) 2025 DeepRobotics
"""

import sys
import os
import time
import traceback
import argparse
from pathlib import Path
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
    
    # 引入 ROS 2 点云数据类型和 MuJoCo-LiDAR
    from sensor_msgs.msg import PointCloud2, PointField
    from mujoco_lidar import MjLidarWrapper, scan_gen
except ImportError as e:
    print(f"[Critical Error] Import failed: {e}", file=sys.stderr, flush=True)
    sys.exit(1)

MODEL_NAME = "M20"
CURRENT_DIR = Path(__file__).resolve().parent
ROBOT_XML_PATH = str((CURRENT_DIR / ".." / ".." / ".." / "model" / "M20" / "mjcf" / "M20.xml").resolve())
TERRAIN_XML_PATH = str((CURRENT_DIR / "m20_terrain.xml").resolve())

USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50

# Calibaration parameters
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
    if not os.path.exists(terrain_path):
        with open(robot_path, 'r') as f: return f.read()
    try:
        robot_tree = ET.parse(robot_path)
        terrain_tree = ET.parse(terrain_path)
        robot_root = robot_tree.getroot()
        terrain_root = terrain_tree.getroot()
        robot_worldbody = robot_root.find("worldbody")
        terrain_worldbody = terrain_root.find("worldbody")
        robot_asset = robot_root.find("asset")
        terrain_asset = terrain_root.find("asset")
        
        if terrain_asset is not None:
            if robot_asset is None: robot_asset = ET.SubElement(robot_root, "asset")
            for item in terrain_asset: robot_asset.append(item)
                
        if terrain_worldbody is not None and robot_worldbody is not None:
            for item in terrain_worldbody:
                if item.tag == "light" and robot_worldbody.find("light") is not None: continue
                if item.get("name") == "floor" and robot_worldbody.find(".//geom[@name='floor']") is not None: continue
                robot_worldbody.append(item)
        return ET.tostring(robot_root, encoding='unicode')
    except Exception as e:
        with open(robot_path, 'r') as f: return f.read()

class JoystickInterface:
    def __init__(self, node):
        self.node = node
        self.active = False
        self.pub = node.create_publisher(GamepadData, '/GAMEPAD_DATA', qos_profile_sensor_data)
        self.js = None
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
        except Exception as e: self.node.get_logger().error(f"[Joystick] Init error: {e}")

    def update(self):
        if not self.active or self.js is None: return
        try:
            self.pygame.event.pump()
            msg = GamepadData()
            msg.header = MetaType(); msg.header.stamp = self.node.get_clock().now().to_msg(); msg.header.frame_id = 0
            msg.left_axis_x = 0.0; msg.left_axis_y = 0.0; msg.right_axis_x = 0.0; msg.right_axis_y = 0.0; msg.buttons = 0
            num_axes = self.js.get_numaxes()
            if num_axes > 0: msg.left_axis_x = -float(self.js.get_axis(0))
            if num_axes > 1: msg.left_axis_y = -float(self.js.get_axis(1))
            if num_axes >= 4:
                turn_axis_idx = 3 if num_axes >= 5 else 2 
                msg.right_axis_x = -float(self.js.get_axis(turn_axis_idx)) 
            buttons_mask = 0
            for i in range(min(16, self.js.get_numbuttons())):
                if self.js.get_button(i): buttons_mask |= (1 << i)
            msg.buttons = buttons_mask
            self.pub.publish(msg)
        except Exception as e:
            self.node.get_logger().error(f"[Joystick] Runtime error: {e}"); self.active = False

class MuJoCoSimulationNode(Node):
    def __init__(self, model_key: str = MODEL_NAME, use_joystick: bool = False):
        super().__init__('mujoco_simulation')
        xml_string = merge_xmls(ROBOT_XML_PATH, TERRAIN_XML_PATH)
        robot_dir = Path(ROBOT_XML_PATH).parent
        merged_xml_path = robot_dir / "merged_scene_autogen.xml"
        try:
            with open(merged_xml_path, 'w') as f: f.write(xml_string)
            self.model = mujoco.MjModel.from_xml_path(str(merged_xml_path))
        except Exception as e:
            self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)
        self.actuator_ids = [a for a in range(self.model.nu)]
        self.dof_num = len(self.actuator_ids)
        self._set_initial_pose(model_key)
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32); self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd); self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd); self.input_tq = np.zeros_like(self.kp_cmd)
        self.timestamp = 0.0
        
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.cmd_sub = self.create_subscription(JointsDataCmd, '/JOINTS_CMD', self._cmd_callback, 50)
        
        # ================= 真实 Mid360 LiDAR 初始化 =================
        self.lidar_pub = self.create_publisher(PointCloud2, '/LIDAR_POINT_CLOUD_MERGED', qos_profile_sensor_data)
        self.world_lidar_pts = np.empty((0, 3)) 
        self.vis_geoms_initialized = False 
        try:
            # 采用 geomgroup 过滤：射线只与 group 0 (地形) 碰撞，避免扫到自身的大腿
            lidar_args = {"geomgroup": [1, 0, 0, 0, 0, 0]}
            
            # 单例模式初始化，使用 Taichi 渲染
            self.lidar = MjLidarWrapper(
                self.model, site_name="lidar_site_front", backend="taichi", args=lidar_args
            )
            self.livox_generator = scan_gen.LivoxGenerator("mid360") 
            self.get_logger().info("[Sim] Physical Mid360 LiDAR engine initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"[Sim] Failed to init LiDAR: {e}")
            self.lidar = None
        # =======================================================
        
        self.use_joystick = use_joystick
        if self.use_joystick: self.joystick = JoystickInterface(self)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if USE_VIEWER else None

    def _set_initial_pose(self, key: str):
        if key in JOINT_INIT:
            qpos0 = self.data.qpos.copy(); qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
            qpos0[:3] = np.array([0, 0, 0.45]); qpos0[3:7] = np.array([1, 0, 0, 0])
            self.data.qpos[:] = qpos0; mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsDataCmd):
        if hasattr(msg.data, 'joints_data'): data_list = msg.data.joints_data
        elif hasattr(msg.data, 'joints_cmd'): data_list = msg.data.joints_cmd
        else: return
        pub_pos = np.zeros(self.dof_num, dtype=np.float32); pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = data_list[i]; self.kp_cmd[i] = joint_cmd.kp; self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position; pub_vel[i] = joint_cmd.velocity; self.tau_ff[i] = joint_cmd.torque 
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD; self.vel_cmd.flat = pub_vel * JOINT_DIR

    def _apply_joint_torque(self):
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1); dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
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
        body_acc = self.data.sensordata[4:7]; angvel_b = self.data.sensordata[7:10]
        imu_msg = ImuData(); imu_msg.header = MetaType(); imu_msg.header.frame_id = 0
        stamp = Time(); sec = int(self.timestamp); nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec; stamp.nanosec = nanosec; imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_rad[0]*180/np.pi); imu_msg.data.pitch = float(rpy_rad[1]*180/np.pi); imu_msg.data.yaw = float(rpy_rad[2]*180/np.pi)
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

    # ================= LiDAR 物理扫描与点云发布 =================
    def _publish_lidar_state(self, step: int):
        if self.lidar is None: return
            
        rays_theta, rays_phi = self.livox_generator.sample_ray_angles()
        
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_pos = self.data.xpos[base_id]
        base_mat = self.data.xmat[base_id].reshape(3, 3)
        
        world_pts_list = []
        
        # 扫描前置雷达
        self.lidar.trace_rays(self.data, rays_theta, rays_phi, site_name="lidar_site_front")
        local_pts_front = self.lidar.get_hit_points()
        if len(local_pts_front) > 0:
            world_pts_front = local_pts_front @ self.lidar.sensor_rotation.T + self.lidar.sensor_position
            base_pts_front = (world_pts_front - base_pos) @ base_mat
            world_pts_list.append(world_pts_front)
        else: base_pts_front = np.empty((0, 3))
            
        # 扫描后置雷达
        self.lidar.trace_rays(self.data, rays_theta, rays_phi, site_name="lidar_site_rear")
        local_pts_rear = self.lidar.get_hit_points()
        if len(local_pts_rear) > 0:
            world_pts_rear = local_pts_rear @ self.lidar.sensor_rotation.T + self.lidar.sensor_position
            base_pts_rear = (world_pts_rear - base_pos) @ base_mat
            world_pts_list.append(world_pts_rear)
        else: base_pts_rear = np.empty((0, 3))
            
        # 缓存世界坐标点云，供本地 GUI 渲染
        if len(world_pts_list) > 0: self.world_lidar_pts = np.vstack(world_pts_list)
        else: self.world_lidar_pts = np.empty((0, 3))
            
        merged_points = np.vstack((base_pts_front, base_pts_rear))
        if len(merged_points) == 0: return

        # 发布至 ROS 2 (供你的 C++ 算法订阅并切片处理)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = len(merged_points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = merged_points.astype(np.float32).tobytes()
        self.lidar_pub.publish(msg)
    # =======================================================

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
                
                # 10Hz LiDAR 物理扫描
                if step % int(0.1 / DT) == 0: self._publish_lidar_state(step)
                
                if self.use_joystick and step % 10 == 0: self.joystick.update()
                    
                if self.viewer and step % RENDER_INTERVAL == 0: 
                    # ================= 密集可视化渲染 =================
                    if hasattr(self, 'world_lidar_pts') and len(self.world_lidar_pts) > 0:
                        with self.viewer.lock(): 
                            max_g = self.viewer.user_scn.maxgeom
                            if not self.vis_geoms_initialized:
                                for i in range(max_g):
                                    mujoco.mjv_initGeom(
                                        self.viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_SPHERE,  
                                        # 【修改 1】缩小球体半径到 0.01，让密集的点云看起来更精细
                                        size=[0.01, 0, 0], pos=np.zeros(3), mat=np.eye(3).flatten(), rgba=np.array([0.0, 1.0, 0.0, 0.6]) 
                                    )
                                self.vis_geoms_initialized = True
                            
                            # 【修改 2】改为 [::1] 不降采样，或者 [::2] 保留一半。
                            vis_pts = self.world_lidar_pts[::2] 
                            
                            # 注意：MuJoCo 的 user_scn 默认有一个最大几何体数量上限（通常是 1000 左右）
                            # 这里会安全地截断，自动渲染到系统支持的极限点数
                            num_pts = min(len(vis_pts), max_g)
                            self.viewer.user_scn.ngeom = num_pts
                            
                            for i in range(num_pts): 
                                self.viewer.user_scn.geoms[i].pos[:] = vis_pts[i]
                    # ==============================================================
                    self.viewer.sync()
            rclpy.spin_once(self, timeout_sec=0.0)

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    parser = argparse.ArgumentParser(description="MuJoCo Simulation for M20")
    parser.add_argument("--joystick", action="store_true", help="Enable joystick/gamepad control")
    args, unknown = parser.parse_known_args()
    
    try:
        rclpy.init(args=unknown)
        sim_node = MuJoCoSimulationNode(use_joystick=args.joystick)
        sim_node.start()
    except KeyboardInterrupt: print("[Sim] Stopping...", file=sys.stderr)
    except Exception: traceback.print_exc()
    finally:
        if 'sim_node' in locals():
            if sim_node.viewer: sim_node.viewer.close()
            sim_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()