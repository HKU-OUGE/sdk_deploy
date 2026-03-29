"""
 * @file mujoco_simulation_ros2.py
 * @brief simulation in mujoco with Real Mid360 LiDAR Simulation (Aligned with Isaac Sim Pose)
 * @author Bo (Percy) Peng / Modified for Sim2Sim True PointCloud & Elevation Mapping
 * @version 2.8
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
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd, GamepadData
    from grid_map_msgs.msg import GridMap
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
        
        # ================= [修复2] 使用 Reliable QoS =================
        # 强制使用深度队列的 Reliable QoS，匹配建图节点
        self.lidar_pub = self.create_publisher(PointCloud2, '/LIDAR_POINT_CLOUD_MERGED', 10)
        # =============================================================
        self.elevation_sub = self.create_subscription(
            GridMap, 
            '/elevation_mapping_node/elevation_map_raw', 
            self._grid_map_callback, 
            10)
        self.elevation_pts = np.empty((0, 3))
        self.world_lidar_pts = np.empty((0, 3)) 
        self.vis_geoms_initialized = False 

        self.num_lasers = 32
        self.num_horizontal = 180
        self.num_rays = self.num_lasers * self.num_horizontal
        
        theta_angles = np.linspace(-np.pi, np.pi, self.num_horizontal)
        phi_angles = np.linspace(0.0, np.pi/2, self.num_lasers) 
        Theta, Phi = np.meshgrid(theta_angles, phi_angles)
        
        X = np.cos(Phi) * np.cos(Theta)
        Y = np.cos(Phi) * np.sin(Theta)
        Z = np.sin(Phi)
        self.ray_dirs_local = np.stack([X.flatten(), Y.flatten(), Z.flatten()], axis=1)
        
        self.front_pos = np.array([0.32028, 0.0, -0.013])
        self.front_rot = np.array([
            [ 0.0,  0.0,  1.0],
            [ 0.0, -1.0,  0.0],
            [ 1.0,  0.0,  0.0]
        ], dtype=np.float64)

        self.rear_pos = np.array([-0.32028, 0.0, -0.013])
        self.rear_rot = np.array([
            [ 0.0,  0.0, -1.0],
            [ 0.0,  1.0,  0.0],
            [ 1.0,  0.0,  0.0]
        ], dtype=np.float64)

        self.geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        
        self.get_logger().info("[Sim] Native mj_multiRay LiDARs aligned with config.yaml.")
        # =================================================================
        self.use_joystick = use_joystick
        if self.use_joystick: self.joystick = JoystickInterface(self)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if USE_VIEWER else None

        self.tf_broadcaster = TransformBroadcaster(self)

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
        
        # ================= [修复1] 统一使用 Wall Time =================
        # 这是修复 RViz2 中 TF 报错 odom does not exist 的关键！
        stamp = self.get_clock().now().to_msg()
        # ==============================================================

        imu_msg = ImuData(); imu_msg.header = MetaType(); imu_msg.header.frame_id = 0
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_rad[0]*180/np.pi); imu_msg.data.pitch = float(rpy_rad[1]*180/np.pi); imu_msg.data.yaw = float(rpy_rad[2]*180/np.pi)
        imu_msg.data.omega_x = float(angvel_b[0]); imu_msg.data.omega_y = float(angvel_b[1]); imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0]); imu_msg.data.acc_y = float(body_acc[1]); imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)
        
        q = self.data.qpos[7:7 + self.dof_num]; dq = self.data.qvel[6:6 + self.dof_num]; tau = self.input_tq.flatten()
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR; pub_vel = dq * JOINT_DIR; pub_tau = tau * JOINT_DIR 
        joints_msg = JointsData(); joints_msg.header = MetaType(); joints_msg.header.frame_id = 0; 
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue(); joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]; joint.name = [32]*4; joint.status_word = 1
            joint.position = float(pub_pos[i]); joint.torque = float(pub_tau[i]); joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0; joint.driver_temp = 40.0
        self.joints_pub.publish(joints_msg)

        # 发布 odom -> base_link TF 
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        base_pos = self.data.qpos[:3]
        base_quat = self.data.qpos[3:7]  
        
        t.transform.translation.x = float(base_pos[0])
        t.transform.translation.y = float(base_pos[1])
        t.transform.translation.z = float(base_pos[2])
        
        t.transform.rotation.x = float(base_quat[1])
        t.transform.rotation.y = float(base_quat[2])
        t.transform.rotation.z = float(base_quat[3])
        t.transform.rotation.w = float(base_quat[0])
        
        self.tf_broadcaster.sendTransform(t)

    def _trace_single_lidar(self, site_name, base_pos, base_mat):
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1: return np.empty((0, 3))
        
        sensor_pos = self.data.site_xpos[site_id]
        sensor_mat = self.data.site_xmat[site_id].reshape(3, 3)
        
        pnt = sensor_pos.astype(np.float64)
        vec = (self.ray_dirs_local @ sensor_mat.T).astype(np.float64).flatten()
        
        geomid = np.zeros(self.num_rays, dtype=np.int32)
        dist = np.zeros(self.num_rays, dtype=np.float64)
        
        # 排除身体
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        
        mujoco.mj_multiRay(
            self.model, self.data,
            pnt, vec,
            self.geomgroup,
            1,          # flg_static
            body_id,    # bodyexclude
            geomid, dist,
            None,
            self.num_rays,
            10.0        # cutoff (最大量程 10 米)
        )
        
        # 排除打中天空的 -1 和无效距离
        valid_mask = (geomid != -1) & (dist > 0) & (dist < 3.0)
        if not np.any(valid_mask): return np.empty((0, 3))
        
        valid_dists = dist[valid_mask]
        valid_dirs_local = self.ray_dirs_local[valid_mask]
        
        # 从距离反推坐标：局部 -> 世界 -> base_link
        local_pts = valid_dirs_local * valid_dists[:, np.newaxis]
        world_pts = local_pts @ sensor_mat.T + sensor_pos
        base_pts = (world_pts - base_pos) @ base_mat
        
        return base_pts

    def _publish_lidar_state(self, step: int):
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_pos = self.data.xpos[base_id]
        base_mat = self.data.xmat[base_id].reshape(3, 3)

        def trace_rays(sensor_pos_offset, sensor_rot):

            rays_base = self.ray_dirs_local @ sensor_rot.T
            rays_world = rays_base @ base_mat.T
            sensor_pos_world = base_pos + base_mat @ sensor_pos_offset
            
            pnt = sensor_pos_world.astype(np.float64)
            vec = rays_world.astype(np.float64).flatten() # C++ 需要 1D array
            
            geomid = np.zeros(self.num_rays, dtype=np.int32)
            dist = np.zeros(self.num_rays, dtype=np.float64)
            
            mujoco.mj_multiRay(
                self.model, self.data,
                pnt, vec,
                self.geomgroup,
                1, base_id,
                geomid, dist,
                None,
                self.num_rays, 10.0
            )
            
            valid = (geomid != -1) & (dist > 0) & (dist < 3.0)
            if not np.any(valid): return np.empty((0, 3))
            
  
            pts_world = pnt + rays_world[valid] * dist[valid][:, np.newaxis]
            pts_base = (pts_world - base_pos) @ base_mat
            return pts_base


        base_pts_front = trace_rays(self.front_pos, self.front_rot)
        base_pts_rear = trace_rays(self.rear_pos, self.rear_rot)

        merged_points = np.vstack((base_pts_front, base_pts_rear))
        if len(merged_points) == 0: 
            self.world_lidar_pts = np.empty((0, 3))
            return
            
        # 用于 RViz 的仿真绿点可视化
        self.world_lidar_pts = merged_points @ base_mat.T + base_pos

        # 发布点云
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

    def _grid_map_callback(self, msg: GridMap):
        try:
            layer_idx = msg.layers.index('elevation')
        except ValueError:
            return
            
        resolution = msg.info.resolution
        length_x = msg.info.length_x
        length_y = msg.info.length_y
        center_x = msg.info.pose.position.x
        center_y = msg.info.pose.position.y
        # 正确的代码
        frame_id = msg.header.frame_id
        
        cells_x = int(round(length_x / resolution))
        cells_y = int(round(length_y / resolution))
        data = msg.data[layer_idx].data
        
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if base_id != -1:
            base_pos = self.data.xpos[base_id]
            base_mat = self.data.xmat[base_id].reshape(3, 3)
            rx, ry = base_pos[0], base_pos[1]
        else:
            return  # 若获取不到基座信息，则安全退出
        
        pts = []
        step = 2 
        
        for idx_x in range(0, cells_x, step):
            for idx_y in range(0, cells_y, step):
                idx = idx_x * cells_y + idx_y
                if idx >= len(data):
                    continue
                
                val = data[idx]
                if not math.isnan(val):

                    px = center_x + length_x / 2.0 - (idx_y + 0.5) * resolution
                    py = center_y + length_y / 2.0 - (idx_x + 0.5) * resolution
                    pz = val
                    

                    if frame_id == "base_link":
                        local_pt = np.array([px, py, pz])
                        world_pt = base_mat @ local_pt + base_pos
                        wx, wy, wz = world_pt[0], world_pt[1], world_pt[2]
                    else:

                        wx, wy, wz = px, py, pz
                    

                    # if (wx - rx)**2 + (wy - ry)**2 < 2.25:
                    pts.append([wx, wy, wz])
                        
        self.elevation_pts = np.array(pts)
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
                
                if step % int(0.1 / DT) == 0: self._publish_lidar_state(step)
                
                if self.use_joystick and step % 10 == 0: self.joystick.update()
                    
                if self.viewer and step % RENDER_INTERVAL == 0: 
                    with self.viewer.lock(): 
                        max_g = self.viewer.user_scn.maxgeom
                        if not self.vis_geoms_initialized:
                            for i in range(max_g):
                                mujoco.mjv_initGeom(
                                    self.viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_SPHERE,  
                                    size=[0.02, 0, 0], pos=np.zeros(3), mat=np.eye(3).flatten(), rgba=np.array([0.0, 0.0, 0.0, 0.0]) 
                                )
                            self.vis_geoms_initialized = True
                        
                        geom_idx = 0
                        
                        # ================= LiDAR (绿色) =================
                        if hasattr(self, 'world_lidar_pts') and len(self.world_lidar_pts) > 0:
                            vis_pts = self.world_lidar_pts[::2]  # 降采样
                            num_pts = min(len(vis_pts), max_g // 2)
                            for i in range(num_pts):
                                self.viewer.user_scn.geoms[geom_idx].pos[:] = vis_pts[i]
                                self.viewer.user_scn.geoms[geom_idx].rgba[:] = [0.0, 1.0, 0.0, 0.6]
                                geom_idx += 1
                                
                        # ================= 高程图 (红色) =================
                        if hasattr(self, 'elevation_pts') and len(self.elevation_pts) > 0:
                            vis_elev = self.elevation_pts
                            num_elev = min(len(vis_elev), max_g - geom_idx) # 使用剩余的几何体配额
                            for i in range(num_elev):
                                self.viewer.user_scn.geoms[geom_idx].pos[:] = vis_elev[i]
                                self.viewer.user_scn.geoms[geom_idx].rgba[:] = [1.0, 0.0, 0.0, 0.8]
                                geom_idx += 1
                        
                        self.viewer.user_scn.ngeom = geom_idx
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