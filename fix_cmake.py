import os
import re

pkg_name = "drdds"
msg_dir = f"src/{pkg_name}/msg"

if not os.path.exists(msg_dir):
    print(f"❌ 错误：找不到 {msg_dir}")
    exit(1)

# 1. 安全地获取所有 msg 文件的相对路径
msg_files = [f"\"msg/{f}\"" for f in os.listdir(msg_dir) if f.endswith(".msg")]
msg_files_str = "\n  ".join(msg_files)

# 2. 提取外部依赖 (去重)
deps = {"builtin_interfaces"}
for f in os.listdir(msg_dir):
    if not f.endswith(".msg"): continue
    with open(os.path.join(msg_dir, f), 'r') as file:
        content = file.read()
        # 寻找诸如 std_msgs/Header 这样的依赖
        matches = re.findall(r'\b([a-zA-Z0-9_]+)/[a-zA-Z0-9_]+\b', content)
        for match in matches:
            if match != pkg_name:
                deps.add(match)

deps_list = sorted(list(deps))

# 3. 完美生成 CMakeLists.txt
cmake_content = f"""cmake_minimum_required(VERSION 3.14)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

"""

for dep in deps_list:
    cmake_content += f"find_package({dep} REQUIRED)\n"

cmake_content += f"""
rosidl_generate_interfaces(${{PROJECT_NAME}}
  {msg_files_str}
  DEPENDENCIES {" ".join(deps_list)}
)

ament_package()
"""

with open(f"src/{pkg_name}/CMakeLists.txt", "w") as f:
    f.write(cmake_content)

print("✅ CMakeLists.txt 已生成！")
