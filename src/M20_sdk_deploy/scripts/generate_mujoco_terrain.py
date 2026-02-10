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
        
        # 添加基础地面 (无限远)
        ET.SubElement(self.worldbody, "geom", name="floor", type="plane", size="0 0 0.05", material="grid_mat", condim="3")
        ET.SubElement(self.worldbody, "light", name="sun", mode="targetbodycom", target="floor", diffuse=".8 .8 .8", dir="0 0 -1", pos="0 0 10")

        # 当前生成的起始坐标
        self.current_x = 2.0  # 从 x=2 开始，留出原点给机器人
        self.row_spacing = 4.0 # 每一列地形的宽度

    def _setup_defaults(self):
        """设置材质、纹理和编译选项"""
        compiler = ET.SubElement(self.root, "compiler", angle="degree", coordinate="local", inertiafromgeom="true")
        option = ET.SubElement(self.root, "option", timestep="0.002", gravity="0 0 -9.81", iterations="50", tolerance="1e-10")
        
        asset = ET.SubElement(self.root, "asset")
        ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="307")
        ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", width="512", height="512", rgb1=".1 .2 .3", rgb2=".2 .3 .4")
        ET.SubElement(asset, "material", name="grid_mat", texture="grid", texrepeat="1 1", texuniform="true", reflectance=".2")
        ET.SubElement(asset, "material", name="stone_mat", rgba="0.7 0.7 0.7 1")
        ET.SubElement(asset, "material", name="rail_mat", rgba="0.8 0.2 0.2 1")
        ET.SubElement(asset, "material", name="box_mat", rgba="0.3 0.8 0.3 1")

    def add_stepping_stones(self, length=10.0, stone_size=0.4, gap=0.15, height_noise=0.0):
        """模仿 HfSteppingStonesTerrainCfg"""
        print(f"Generating Stepping Stones at X > {self.current_x}...")
        
        # 计算行数和列数
        cols = int(length / (stone_size + gap))
        rows = 3 # 宽度方向放 3 排
        
        start_x = self.current_x
        center_y = 0.0
        
        for i in range(cols):
            for j in range(rows):
                # 随机扰动
                x_noise = np.random.uniform(-0.02, 0.02)
                y_noise = np.random.uniform(-0.02, 0.02)
                h_noise = np.random.uniform(0, height_noise)
                
                x = start_x + i * (stone_size + gap) + stone_size/2 + x_noise
                y = center_y + (j - 1) * (stone_size + gap) + y_noise # j=0,1,2 -> -1, 0, 1
                z = h_noise + 0.05 # 基础高度
                
                ET.SubElement(self.worldbody, "geom", 
                              type="box", 
                              pos=f"{x:.3f} {y:.3f} {z:.3f}", 
                              size=f"{stone_size/2:.3f} {stone_size/2:.3f} 0.05", 
                              material="stone_mat",
                              name=f"stone_{i}_{j}")
        
        self.current_x += length + 2.0

    def add_random_boxes(self, length=10.0, grid_size=0.45, height_range=(0.05, 0.2)):
        """模仿 MeshRandomGridTerrainCfg"""
        print(f"Generating Random Boxes at X > {self.current_x}...")
        
        cols = int(length / grid_size)
        rows = 4 # 宽度 
        start_x = self.current_x
        
        for i in range(cols):
            for j in range(rows):
                height = np.random.uniform(*height_range)
                
                x = start_x + i * grid_size + grid_size/2
                y = (j - 1.5) * grid_size
                z = height / 2.0
                
                ET.SubElement(self.worldbody, "geom",
                              type="box",
                              pos=f"{x:.3f} {y:.3f} {z:.3f}",
                              size=f"{grid_size/2 - 0.01:.3f} {grid_size/2 - 0.01:.3f} {height/2:.3f}",
                              material="box_mat",
                              name=f"box_{i}_{j}")
                              
        self.current_x += length + 2.0

    def add_rails(self, length=10.0, rail_width=0.1, rail_height=0.15):
        """模仿 MeshRailsTerrainCfg"""
        print(f"Generating Rails at X > {self.current_x}...")
        
        # 生成两段独木桥
        start_x = self.current_x
        
        # Rail 1
        ET.SubElement(self.worldbody, "geom",
                      type="box",
                      pos=f"{start_x + length/2:.3f} 0.0 {rail_height/2:.3f}",
                      size=f"{length/2:.3f} {rail_width/2:.3f} {rail_height/2:.3f}",
                      material="rail_mat",
                      name=f"rail_1")
        
        # Rail 2 (Side step)
        ET.SubElement(self.worldbody, "geom",
                      type="box",
                      pos=f"{start_x + length/2:.3f} 1.0 {rail_height/2:.3f}",
                      size=f"{length/2:.3f} {rail_width/2:.3f} {rail_height/2:.3f}",
                      material="rail_mat",
                      name=f"rail_2")

        self.current_x += length + 2.0

    def add_stairs(self, steps=10, step_run=0.3, step_rise=0.1, width=2.0):
        """模仿 MeshInvertedPyramidStairsTerrainCfg (简化版：单向楼梯)"""
        print(f"Generating Stairs at X > {self.current_x}...")
        
        start_x = self.current_x
        current_z = 0.0
        
        # 上楼
        for i in range(steps):
            current_z += step_rise
            x = start_x + i * step_run + step_run/2
            
            ET.SubElement(self.worldbody, "geom",
                          type="box",
                          pos=f"{x:.3f} 0.0 {current_z/2:.3f}",
                          size=f"{step_run/2:.3f} {width/2:.3f} {current_z/2:.3f}",
                          material="stone_mat",
                          name=f"stair_up_{i}")
        
        # 平台
        platform_len = 2.0
        x_plat = start_x + steps * step_run + platform_len/2
        ET.SubElement(self.worldbody, "geom",
                        type="box",
                        pos=f"{x_plat:.3f} 0.0 {current_z/2:.3f}",
                        size=f"{platform_len/2:.3f} {width/2:.3f} {current_z/2:.3f}",
                        material="stone_mat",
                        name="stair_platform")
                        
        # 下楼
        start_x_down = start_x + steps * step_run + platform_len
        for i in range(steps):
            x = start_x_down + i * step_run + step_run/2
            
            ET.SubElement(self.worldbody, "geom",
                          type="box",
                          pos=f"{x:.3f} 0.0 {current_z/2:.3f}",
                          size=f"{step_run/2:.3f} {width/2:.3f} {current_z/2:.3f}",
                          material="stone_mat",
                          name=f"stair_down_{i}")
            current_z -= step_rise

        self.current_x += (steps * 2 * step_run + platform_len) + 2.0

    def save(self):
        # 美化 XML
        raw_string = ET.tostring(self.root, 'utf-8')
        reparsed = minidom.parseString(raw_string)
        pretty_string = reparsed.toprettyxml(indent="  ")
        
        with open(self.output_file, "w") as f:
            f.write(pretty_string)
        print(f"Successfully generated terrain file: {self.output_file}")

if __name__ == "__main__":
    # 配置你的地形生成参数
    gen = MujocoTerrainGenerator("m20_terrain.xml")
    
    # 1. 独木桥
    gen.add_rails(length=6.0, rail_width=0.2, rail_height=0.1)
    
    # 2. 梅花桩 (模拟 Stepping Stones)
    gen.add_stepping_stones(length=6.0, stone_size=0.4, gap=0.2, height_noise=0.02)
    
    # 3. 乱石阵 (模拟 Random Boxes)
    gen.add_random_boxes(length=6.0, grid_size=0.4, height_range=(0.05, 0.15))
    
    # 4. 楼梯 (模拟 Stairs)
    gen.add_stairs(steps=8, step_run=0.3, step_rise=0.08)
    
    gen.save()