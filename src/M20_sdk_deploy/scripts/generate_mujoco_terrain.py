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

        # 【核心修改】：主赛道基准高度抬升，避开用户主环境自带的 Z=0 默认地面！
        self.track_z = 0.5 
        
        # 机器人出生平台 (保持 Z=0，防止机器人出生坠落)
        ET.SubElement(self.worldbody, "geom", type="box", pos="0 0 -0.05", size="1.0 2.0 0.05", material="stone_mat", name="start_platform")
        
        # 出生平台上坡 -> 引导机器人走向 Z=0.5 的主赛道
        ramp_x_len = 2.0
        pitch = -np.arctan(self.track_z / ramp_x_len)
        ramp_actual_len = np.sqrt(ramp_x_len**2 + self.track_z**2)
        ET.SubElement(self.worldbody, "geom", type="box", 
                      pos=f"{1.0 + ramp_x_len/2:.3f} 0.0 {self.track_z/2 - 0.05:.3f}", 
                      size=f"{ramp_actual_len/2:.3f} 2.0 0.05", 
                      euler=f"0 {pitch:.3f} 0", material="stone_mat", name="start_ramp")
        
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
        """生成标准连接地台，所有Z=0替换为Z=track_z"""
        ET.SubElement(self.worldbody, "geom", type="box", 
                      pos=f"{self.current_x + length/2:.3f} 0.0 {self.track_z - 0.05:.3f}", 
                      size=f"{length/2:.3f} {width/2:.3f} 0.05", material="stone_mat")
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

    # 3. stepping_stones
    def add_stepping_stones(self, length=8.0, stone_size=0.4, gap=0.3, height_noise=0.01, width=2.0):
        print(f"Generating Stepping Stones at X > {self.current_x}...")
        cols = int(length / (stone_size + gap))
        rows = int(width / (stone_size + gap))
        start_x = self.current_x
        
        for i in range(cols):
            for j in range(rows):
                x_noise = np.random.uniform(-0.02, 0.02)
                y_noise = np.random.uniform(-0.02, 0.02)
                h_noise = np.random.uniform(0, height_noise)
                
                x = start_x + i * (stone_size + gap) + stone_size/2 + x_noise
                y = (j - rows/2.0 + 0.5) * (stone_size + gap) + y_noise 
                
                # 梅花桩稍微凸起于跑道表面
                z_top = self.track_z + 0.15 + h_noise 
                base_z = self.track_z - 0.5
                height = z_top - base_z
                z_center = base_z + height / 2.0
                
                ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} {y:.3f} {z_center:.3f}", 
                              size=f"{stone_size/2:.3f} {stone_size/2:.3f} {height/2:.3f}", material="stone_mat")
        
        self.current_x += length
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

    # 5. pit (有底的凹坑)
    def add_pit(self, pit_depth=0.5, gap_length=1.0, double_pit=True, width=3.0):
        print(f"Generating Pit (Depth={pit_depth}) at X > {self.current_x}...")
        num_pits = 2 if double_pit else 1
        plat_len = 1.0 
        
        for i in range(num_pits):
            ET.SubElement(self.worldbody, "geom", type="box", 
                          pos=f"{self.current_x + gap_length/2:.3f} 0.0 {self.track_z - pit_depth - 0.05:.3f}", 
                          size=f"{gap_length/2:.3f} {width/2:.3f} 0.05", material="stone_mat")
            self.current_x += gap_length
            
            ET.SubElement(self.worldbody, "geom", type="box", 
                          pos=f"{self.current_x + plat_len/2:.3f} 0.0 {self.track_z - 0.05:.3f}", 
                          size=f"{plat_len/2:.3f} {width/2:.3f} 0.05", material="stone_mat")
            self.current_x += plat_len
            
        self.add_platform(2.0)

    # 【新增】：真实的悬空断崖 (Gap)
    def add_gap(self, gap_length=1.0, double_gap=True, width=3.0):
        print(f"Generating Real Gap (Empty space) at X > {self.current_x}...")
        num_gaps = 2 if double_gap else 1
        plat_len = 1.0 
        
        for i in range(num_gaps):
            # 仅仅增加坐标，不生成任何底板，让它直通深渊
            self.current_x += gap_length
            
            # 断崖后的立足点
            ET.SubElement(self.worldbody, "geom", type="box", 
                          pos=f"{self.current_x + plat_len/2:.3f} 0.0 {self.track_z - 0.05:.3f}", 
                          size=f"{plat_len/2:.3f} {width/2:.3f} 0.05", material="stone_mat")
            self.current_x += plat_len
            
        self.add_platform(2.0)

    # 6. boxes
    def add_random_boxes(self, length=8.0, grid_size=0.45, height_range=(0.05, 0.2), width=3.0):
        print(f"Generating Random Boxes at X > {self.current_x}...")
        cols = int(length / grid_size)
        rows = int(width / grid_size)
        start_x = self.current_x
        
        for i in range(cols):
            for j in range(rows):
                height = np.random.uniform(*height_range)
                x = start_x + i * grid_size + grid_size/2
                y = (j - rows/2.0 + 0.5) * grid_size
                ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} {y:.3f} {self.track_z + height/2:.3f}",
                              size=f"{grid_size/2 - 0.01:.3f} {grid_size/2 - 0.01:.3f} {height/2:.3f}", material="box_mat")
                              
        self.current_x += length
        self.add_platform(2.0)

    # 7. random_rough
    def add_random_rough(self, length=8.0, width=3.0, grid_size=0.1, noise_range=(0.02, 0.16)):
        print(f"Generating Random Rough Terrain at X > {self.current_x}...")
        cols = int(length / grid_size)
        rows = int(width / grid_size)
        start_x = self.current_x
        
        for i in range(cols):
            for j in range(rows):
                h = np.random.uniform(*noise_range)
                x = start_x + i * grid_size + grid_size/2
                y = (j - rows/2.0 + 0.5) * grid_size
                z_top = self.track_z + h
                base_z = self.track_z - 0.5
                height = z_top - base_z
                z_center = base_z + height / 2.0
                ET.SubElement(self.worldbody, "geom", type="box", pos=f"{x:.3f} {y:.3f} {z_center:.3f}",
                              size=f"{grid_size/2:.3f} {grid_size/2:.3f} {height/2:.3f}", material="stone_mat")
                              
        self.current_x += length
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
                      euler=f"0 {pitch_1:.3f} 0", material="stone_mat")
        
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
                      euler=f"0 {pitch_2:.3f} 0", material="stone_mat")

        self.current_x += (ramp_x_len * 2 + plat_len)
        self.add_platform(2.0)

    # 10. floating_ring (水平钻圈网)
    def add_floating_ring(self, inner_length=1.5, inner_width=2.0, ring_thickness=0.1, ring_width=0.15, ring_height=0.4):
        """生成一个平行于地面的长方形框，悬空高度为 ring_height，机器人必须从下面钻过"""
        print(f"Generating Horizontal Floating Ring at X > {self.current_x}...")
        start_x = self.current_x
        
        # 底部通行地台
        plat_len = inner_length + 2.0
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{start_x + plat_len/2:.3f} 0.0 {self.track_z - 0.05:.3f}", 
                      size=f"{plat_len/2:.3f} 1.5 0.05", material="stone_mat")
                      
        # 水平环的中心高
        ring_x = start_x + plat_len/2
        ring_z = self.track_z + ring_height + ring_thickness/2
        
        # 构建平躺的四个边框 (左右前后)
        # 1. 左侧横梁
        left_y = inner_width/2 + ring_width/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{ring_x:.3f} {left_y:.3f} {ring_z:.3f}",
                      size=f"{(inner_length + 2*ring_width)/2:.3f} {ring_width/2:.3f} {ring_thickness/2:.3f}", material="ring_mat")
        # 2. 右侧横梁
        right_y = -(inner_width/2 + ring_width/2)
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{ring_x:.3f} {right_y:.3f} {ring_z:.3f}",
                      size=f"{(inner_length + 2*ring_width)/2:.3f} {ring_width/2:.3f} {ring_thickness/2:.3f}", material="ring_mat")
        # 3. 前方横梁 (+X)
        front_x = ring_x + inner_length/2 + ring_width/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{front_x:.3f} 0.0 {ring_z:.3f}",
                      size=f"{ring_width/2:.3f} {inner_width/2:.3f} {ring_thickness/2:.3f}", material="ring_mat")
        # 4. 后方横梁 (-X)
        back_x = ring_x - inner_length/2 - ring_width/2
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{back_x:.3f} 0.0 {ring_z:.3f}",
                      size=f"{ring_width/2:.3f} {inner_width/2:.3f} {ring_thickness/2:.3f}", material="ring_mat")

        self.current_x += plat_len
        self.add_platform(2.0)

    def save(self):
        # 终点大平地
        ET.SubElement(self.worldbody, "geom", type="box", pos=f"{self.current_x + 5.0:.3f} 0.0 {self.track_z - 0.05:.3f}", size="5.0 2.0 0.05", material="stone_mat")
        
        raw_string = ET.tostring(self.root, 'utf-8')
        reparsed = minidom.parseString(raw_string)
        pretty_string = reparsed.toprettyxml(indent="  ")
        
        with open(self.output_file, "w") as f:
            f.write(pretty_string)
        print(f"\nSuccessfully generated terrain file: {self.output_file}")

if __name__ == "__main__":
    gen = MujocoTerrainGenerator("m20_terrain.xml")
    
    # 1. pyramid_stairs (正向楼梯)
    gen.add_stairs(steps=4, step_run=0.3, step_rise=0.15, inverted=False)
    
    # 2. pyramid_stairs_inv (倒向/坑状楼梯)
    gen.add_stairs(steps=4, step_run=0.3, step_rise=0.15, inverted=True)
    
    # # 3. stepping_stones (梅花桩，高度有微小扰动)
    # gen.add_stepping_stones(length=6.0, stone_size=0.4, gap=0.2, height_noise=0.01)
    
    # # 4. rail (跨栏)
    # gen.add_rails(num_rails=3, gap=2.0, rail_width=0.1, rail_height=0.25)
    
    # # 5. pit (有底基座的凹坑)
    # gen.add_pit(pit_depth=0.5, gap_length=1.0, double_pit=True)
    
    # # 6. gap (完全没有底座的真实断崖)
    # gen.add_gap(gap_length=0.4, double_gap=True)
    
    # # 7. boxes (随机高低方块阵列)
    # gen.add_random_boxes(length=6.0, grid_size=0.45, height_range=(0.05, 0.2))
    
    # # 8. random_rough (细密随机粗糙地形)
    # gen.add_random_rough(length=6.0, grid_size=0.1, noise_range=(0.02, 0.16))
    
    # 9. hf_pyramid_slope (正向斜坡)
    gen.add_slope(length=3.0, slope=0.4, inverted=False)
    
    # # 10. hf_pyramid_slope_inv (倒向/凹陷斜坡)
    # gen.add_slope(length=3.0, slope=0.4, inverted=True)
    
    # 11. floating_ring (水平方框网，离地0.4m高，供机器人钻爬)
    gen.add_floating_ring(inner_length=1.5, inner_width=2.0, ring_thickness=0.3, ring_width=0.15, ring_height=0.4)
    gen.add_pit(pit_depth=0.5, gap_length=2.0, double_pit=True)
    gen.save()