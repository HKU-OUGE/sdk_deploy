import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom
import os

class MujocoTerrainGenerator:
    def __init__(self, output_file="m20_terrain.xml"):
        self.output_file = output_file
        self.root = ET.Element("mujoco", model="M20_Terrain")
        
        # 基础设置
        self._setup_defaults()
        self.worldbody = ET.SubElement(self.root, "worldbody")
        
        # 深渊底板 (防止完全掉出世界报错)
        ET.SubElement(self.worldbody, "geom", name="abyss_floor", type="plane", size="0 0 0.05", pos="0 0 -5.0", material="grid_mat", condim="3")
        ET.SubElement(self.worldbody, "light", name="sun", mode="targetbodycom", target="abyss_floor", diffuse=".8 .8 .8", dir="0 0 -1", pos="0 0 10")

        # 统一薄底板厚度。所有其他几何均坐在 z=floor_z 之上，地板本身在 save() 时统一生成
        self.floor_z = 0.01
        # 主赛道顶面：从地板顶 (floor_z) 向上 0.5m，整体抬升 floor_z 留出薄底板空间
        self.track_z = 0.5 + self.floor_z
        # 记录赛道起始 X，用于 save() 时生成覆盖全段的薄底板
        self.x_start = -1.0

        # 机器人出生平台 (顶面 = floor_z + 0.1)
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"0 0 {self.floor_z + 0.05:.3f}",
                      size="1.0 2.0 0.05", material="stone_mat", name="start_platform")

        # 出生平台上坡 -> 引导机器人走向主赛道
        ramp_x_len = 2.0
        ramp_dz = self.track_z - self.floor_z  # 0.5
        pitch = -np.arctan(ramp_dz / ramp_x_len)
        ramp_actual_len = np.sqrt(ramp_x_len**2 + ramp_dz**2)
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"{1.0 + ramp_x_len/2:.3f} 0.0 {self.floor_z + ramp_dz/2 - 0.05:.3f}",
                      size=f"{ramp_actual_len/2:.3f} 2.0 0.05",
                      euler=f"0 {np.degrees(pitch):.3f} 0", material="stone_mat", name="start_ramp")

        self.current_x = 1.0 + ramp_x_len
        # 生成第一段平地缓冲
        self.add_platform(length=2.0)

    def _setup_defaults(self):
        compiler = ET.SubElement(self.root, "compiler", angle="degree", coordinate="local", inertiafromgeom="true")
        option = ET.SubElement(self.root, "option", timestep="0.002", gravity="0 0 -9.81", iterations="50", tolerance="1e-10")
        
        asset = ET.SubElement(self.root, "asset")
        ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="307")
        ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", width="512", height="512", rgb1=".1 .2 .3", rgb2=".2 .3 .4")
        ET.SubElement(asset, "material", name="grid_mat", texture="grid", texrepeat="1 1", texuniform="true", reflectance=".2")
        ET.SubElement(asset, "material", name="stone_mat", rgba="0.7 0.7 0.7 1")
        ET.SubElement(asset, "material", name="rail_mat", rgba="0.8 0.2 0.2 1")
        ET.SubElement(asset, "material", name="box_mat", rgba="0.3 0.8 0.3 1")
        ET.SubElement(asset, "material", name="ring_mat", rgba="0.9 0.6 0.1 1")

    def add_platform(self, length=2.0, width=3.0):
        """生成标准连接地台：从 z=floor_z 到 z=track_z 的实心 box"""
        h = self.track_z - self.floor_z
        z_center = (self.floor_z + self.track_z) / 2
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"{self.current_x + length/2:.3f} 0.0 {z_center:.3f}",
                      size=f"{length/2:.3f} {width/2:.3f} {h/2:.3f}", material="stone_mat")
        self.current_x += length

    # 1 & 2. pyramid_stairs & pyramid_stairs_inv
    def add_stairs(self, steps=5, step_run=0.3, step_rise=0.15, width=3.0, inverted=False):
        terrain_type = "Inverted Stairs" if inverted else "Stairs"
        print(f"Generating {terrain_type} at X > {self.current_x}...")
        
        start_x = self.current_x
        current_z = self.track_z
        direction = -1 if inverted else 1
        
        max_depth = steps * step_rise
        base_z = self.track_z - max_depth - 0.5 
        
        for i in range(steps):
            current_z += step_rise * direction
            x = start_x + i * step_run + step_run/2
            height = current_z - base_z
            z_center = base_z + height / 2.0
            ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} 0.0 {z_center:.3f}", 
                          size=f"{step_run/2:.3f} {width/2:.3f} {height/2:.3f}", material="stone_mat")
        
        platform_len = 3.0
        x_plat = start_x + steps * step_run + platform_len/2
        height = current_z - base_z
        z_center = base_z + height / 2.0
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x_plat:.3f} 0.0 {z_center:.3f}", 
                      size=f"{platform_len/2:.3f} {width/2:.3f} {height/2:.3f}", material="stone_mat")
                        
        start_x_2 = start_x + steps * step_run + platform_len
        for i in range(steps):
            x = start_x_2 + i * step_run + step_run/2
            height = current_z - base_z
            z_center = base_z + height / 2.0
            ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} 0.0 {z_center:.3f}", 
                          size=f"{step_run/2:.3f} {width/2:.3f} {height/2:.3f}", material="stone_mat")
            current_z -= step_rise * direction

        self.current_x += (steps * 2 * step_run + platform_len)
        self.add_platform(2.0)

    # 4. rail
    def add_rails(self, num_rails=4, gap=1.5, rail_width=0.1, rail_height=0.25, yaw=90):
        print(f"Generating Rails at X > {self.current_x}...")
        start_x = self.current_x
        length = 3.0
        
        for i in range(num_rails):
            x = start_x + i * gap + rail_width/2
            ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} 0.0 {self.track_z - 0.05:.3f}", size=f"{gap/2:.3f} {length/2:.3f} 0.05", material="stone_mat")
            ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} 0.0 {self.track_z + rail_height/2:.3f}",
                          size=f"{rail_width/2:.3f} {length/2:.3f} {rail_height/2:.3f}", material="rail_mat")

        self.current_x += (num_rails * gap)
        self.add_platform(2.0)

    # 5. 凸起高台 (连续赛道 + 局部高台)
    def add_pit(self, pit_depth=0.2, gap_length=2.0, double_pit=True, width=3.0):
        """连续赛道 (track_z) 上的凸起高台 —— 不再掉到地板，赛道全程连续。
        - 高台前先铺 gap_length 长的 track_z 跑道 (连续，不落差)。
        - 再放一个实心高台 box，顶面 = track_z + pit_depth (高出赛道 pit_depth，机器人上下翻越)。
        - pit_depth = 高台高出赛道的高度 (m)。
        """
        print(f"Generating Raised Platform (h_above_track={pit_depth}) at X > {self.current_x}...")
        num_pits = 2 if double_pit else 1
        plat_len = 1.0

        for _ in range(num_pits):
            # 高台前的连续赛道 (铺 track_z 平地，current_x += gap_length)
            self.add_platform(length=gap_length, width=width)

            # --- 凸起高台 box：实心，从 floor_z 一直到 track_z + pit_depth ---
            top_z = self.track_z + pit_depth
            h_plat = top_z - self.floor_z
            z_center_plat = (self.floor_z + top_z) / 2.0
            ET.SubElement(self.worldbody, "geom", type="box",
                          pos=f"{self.current_x + plat_len/2:.3f} 0.0 {z_center_plat:.3f}",
                          size=f"{plat_len/2:.3f} {width/2:.3f} {h_plat/2:.3f}", material="stone_mat")
            self.current_x += plat_len

        self.add_platform(2.0)

    # 真实的悬空断崖 (Gap)
    def add_gap(self, gap_length=1.0, gap_depth=0.5, double_gap=True, width=3.0):
        print(f"Generating Gap (Width={gap_length}, Depth={gap_depth}) at X > {self.current_x}...")
        num_gaps = 2 if double_gap else 1
        plat_len = 1.0

        for i in range(num_gaps):
            # 坑底板
            z_top_pit = self.track_z - gap_depth
            ground_z = 0.0
            if z_top_pit > ground_z:
                h_pit = z_top_pit - ground_z
                z_center_pit = ground_z + h_pit / 2.0
                ET.SubElement(self.worldbody, "geom", type="box",
                              pos=f"{self.current_x + gap_length/2:.3f} 0.0 {z_center_pit:.3f}",
                              size=f"{gap_length/2:.3f} {width/2:.3f} {h_pit/2:.3f}", material="stone_mat")
            else:
                ET.SubElement(self.worldbody, "geom", type="box",
                              pos=f"{self.current_x + gap_length/2:.3f} 0.0 {z_top_pit - 0.05:.3f}",
                              size=f"{gap_length/2:.3f} {width/2:.3f} 0.05", material="stone_mat")
            self.current_x += gap_length

            # 中间平台
            h_plat = self.track_z - ground_z
            z_center_plat = ground_z + h_plat / 2.0
            ET.SubElement(self.worldbody, "geom", type="box",
                          pos=f"{self.current_x + plat_len/2:.3f} 0.0 {z_center_plat:.3f}",
                          size=f"{plat_len/2:.3f} {width/2:.3f} {h_plat/2:.3f}", material="stone_mat")
            self.current_x += plat_len

        self.add_platform(2.0)

    # 8 & 9. hf_pyramid_slope & hf_pyramid_slope_inv
    def add_slope(self, length=3.0, slope=0.4, width=3.0, inverted=False):
        terrain_type = "Inverted Slope" if inverted else "Slope"
        print(f"Generating {terrain_type} at X > {self.current_x}...")
        
        start_x = self.current_x
        angle_rad = np.arctan(slope) 
        direction = -1 if inverted else 1
        
        ramp_x_len = length
        ramp_z_height = length * slope * direction
        thickness = 0.5 
        
        x_1 = start_x + ramp_x_len / 2
        z_1 = self.track_z + ramp_z_height / 2 - thickness/2
        pitch_1 = -angle_rad if not inverted else angle_rad
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x_1:.3f} 0.0 {z_1:.3f}",
                      size=f"{length/2 / np.cos(angle_rad):.3f} {width/2:.3f} {thickness/2:.3f}",
                      euler=f"0 {np.degrees(pitch_1):.3f} 0", material="stone_mat")
        
        plat_len = 2.0
        x_plat = start_x + ramp_x_len + plat_len / 2
        z_plat = self.track_z + ramp_z_height - thickness/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x_plat:.3f} 0.0 {z_plat:.3f}", 
                      size=f"{plat_len/2:.3f} {width/2:.3f} {thickness/2:.3f}", material="stone_mat")
                      
        x_2 = start_x + ramp_x_len + plat_len + ramp_x_len / 2
        z_2 = self.track_z + ramp_z_height / 2 - thickness/2
        pitch_2 = angle_rad if not inverted else -angle_rad
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x_2:.3f} 0.0 {z_2:.3f}",
                      size=f"{length/2 / np.cos(angle_rad):.3f} {width/2:.3f} {thickness/2:.3f}",
                      euler=f"0 {np.degrees(pitch_2):.3f} 0", material="stone_mat")

        self.current_x += (ramp_x_len * 2 + plat_len)
        self.add_platform(2.0)

    # 11. Hurdle (crawl-mode floating bar, aligned with rl_training course terrain)
    def add_hurdle(self, hurdle_height=0.4, width=None, bar_thickness=None, bar_vertical_height=0.30):
        """
        生成 rl_training course 风格的 crawl-mode hurdle：
        - 地板连续，障碍长度 0.5m。
        - hurdle_height 是横杆底部到主赛道顶面的 clearance。
        - bar_thickness 是行进方向厚度/侧柱厚度，默认 0.08m。
        - bar_vertical_height 是横杆竖向高度，rl_training 中为 0.30m。
        """
        if width is None:
            width = 3.0
        if bar_thickness is None:
            bar_thickness = 0.08

        print(
            f"Generating Crawl Hurdle (Clearance={hurdle_height}, Width={width}, "
            f"Thickness={bar_thickness}, BarZ={bar_vertical_height}) at X > {self.current_x}..."
        )
        start_x = self.current_x
        
        # 底部通行地台：rl_training hurdle patch 长度约 0.5m，地板连续。
        plat_len = 0.5
        h_base = self.track_z - self.floor_z
        z_base_center = (self.floor_z + self.track_z) / 2
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"{start_x + plat_len/2:.3f} 0.0 {z_base_center:.3f}",
                      size=f"{plat_len/2:.3f} {width/2:.3f} {h_base/2:.3f}", material="stone_mat")
                      
        # 跨栏的中心X坐标
        hurdle_x = start_x + plat_len/2
        
        # 左右侧细柱：柱顶到横杆底部，保持 clearance = hurdle_height。
        left_y = width/2 - bar_thickness/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{hurdle_x:.3f} {left_y:.3f} {self.track_z + hurdle_height/2:.3f}",
                      size=f"{bar_thickness/2:.3f} {bar_thickness/2:.3f} {hurdle_height/2:.3f}", material="rail_mat")
        
        # 右侧立柱
        right_y = -width/2 + bar_thickness/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{hurdle_x:.3f} {right_y:.3f} {self.track_z + hurdle_height/2:.3f}",
                      size=f"{bar_thickness/2:.3f} {bar_thickness/2:.3f} {hurdle_height/2:.3f}", material="rail_mat")
        
        # 水平横杆：x 厚度 0.08m，z 高度 0.30m；底部在 track_z + hurdle_height。
        bar_z = self.track_z + hurdle_height + bar_vertical_height/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{hurdle_x:.3f} 0.0 {bar_z:.3f}",
                      size=f"{bar_thickness/2:.3f} {width/2:.3f} {bar_vertical_height/2:.3f}", material="ring_mat")

        self.current_x += plat_len
        self.add_platform(2.0)

    def save(self):
        # 终点大平地 (实心 box，从 floor_z 到 track_z)
        h_end = self.track_z - self.floor_z
        z_end_center = (self.floor_z + self.track_z) / 2
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"{self.current_x + 5.0:.3f} 0.0 {z_end_center:.3f}",
                      size=f"5.0 2.0 {h_end/2:.3f}", material="stone_mat")

        # 统一薄底板，从 z=0 到 z=floor_z，覆盖整段赛道
        floor_x_end = self.current_x + 10.0  # 含终点大平地末端
        floor_len = floor_x_end - self.x_start
        floor_x_center = (self.x_start + floor_x_end) / 2
        ET.SubElement(self.worldbody, "geom", type="box",
                      pos=f"{floor_x_center:.3f} 0.0 {self.floor_z/2:.3f}",
                      size=f"{floor_len/2:.3f} 5.0 {self.floor_z/2:.3f}",
                      material="stone_mat", name="unified_floor")
        
        raw_string = ET.tostring(self.root, 'utf-8')
        reparsed = minidom.parseString(raw_string)
        pretty_string = reparsed.toprettyxml(indent="  ")
        
        with open(self.output_file, "w") as f:
            f.write(pretty_string)
        print(f"\nSuccessfully generated terrain file: {self.output_file}")

if __name__ == "__main__":
    # 默认写到 sim 读取的位置 (相对本脚本路径), 直接覆盖原有 cfg —— 无需 cd 或 env -C
    _here = os.path.dirname(os.path.abspath(__file__))
    OUT = os.path.normpath(os.path.join(_here, "..", "interface", "robot", "simulation", "m20_terrain.xml"))
    gen = MujocoTerrainGenerator(OUT)
    
    # 1. pyramid_stairs (正向楼梯)
    gen.add_stairs(steps=10, step_run=0.3, step_rise=0.15, inverted=False)
    
    # # 3. hf_pyramid_slope
    # gen.add_slope(length=3.0, slope=0.4, inverted=False)

    # # 4. pit
    # gen.add_pit(pit_depth=0.2, gap_length=2.0, double_pit=False)
    # gen.add_pit(pit_depth=0.4, gap_length=2.0, double_pit=False)
    # gen.add_pit(pit_depth=0.6, gap_length=2.0, double_pit=False)
    # # 5. Hurdle (跨栏)
    # gen.add_hurdle(hurdle_height=0.2, width=3.0) 
    # 6. Hurdle (跨栏-钻)
    # gen.add_hurdle(hurdle_height=0.6, width=3.0)
    # 凸起高台 (连续赛道, 高出赛道 pit_depth; gap_length=前置跑道长度)。高度可按需调。
    gen.add_pit(pit_depth=0.2, gap_length=2.0, double_pit=False)
    gen.add_pit(pit_depth=0.35, gap_length=2.0, double_pit=False)
    gen.add_pit(pit_depth=0.5, gap_length=2.0, double_pit=False)
    # Gap 缝隙地形: 跨越 30cm / 40cm 宽的坑 (深 0.20m, 坑底降到赛道下方, 中间留 track_z 平台)
    gen.add_gap(gap_length=0.3, gap_depth=0.20, double_gap=False)
    gen.add_gap(gap_length=0.4, gap_depth=0.20, double_gap=False)
    gen.add_hurdle(hurdle_height=0.50, bar_thickness=0.08, width=3.0)
    gen.add_hurdle(hurdle_height=0.40, bar_thickness=0.08, width=3.0)
    gen.add_hurdle(hurdle_height=0.35, bar_thickness=0.08, width=3.0)


    # 7. Gap 地形 (4种不同宽度)
    # gen.add_gap(gap_length=0.2, gap_depth=0.3, double_gap=False)
    # gen.add_gap(gap_length=0.4, gap_depth=0.5, double_gap=False)
    # gen.add_gap(gap_length=0.6, gap_depth=0.7, double_gap=False)
    # gen.add_gap(gap_length=0.8, gap_depth=1.0, double_gap=False)

    gen.save()
