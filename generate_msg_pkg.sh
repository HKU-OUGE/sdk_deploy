#!/bin/bash

PKG_NAME=$1

if [ -z "$PKG_NAME" ]; then
    echo -e "\033[31m❌ 请提供包名！例如: ./generate_msg_pkg.sh drdds\033[0m"
    exit 1
fi

PKG_DIR="src/$PKG_NAME"
MSG_DIR="$PKG_DIR/msg"

if [ ! -d "$MSG_DIR" ]; then
    echo -e "\033[31m❌ 找不到目录 $MSG_DIR！请确保 msg 文件已经放对位置。\033[0m"
    exit 1
fi

echo -e "\033[34m🔍 正在扫描 [$PKG_NAME] 的消息依赖...\033[0m"

# 提取外部依赖
EXT_DEPS=$(cat $MSG_DIR/*.msg 2>/dev/null | grep -v '^#' | grep -oE '\b[a-zA-Z0-9_]+/[a-zA-Z0-9_]+\b' | cut -d'/' -f1 | sort -u | grep -v "^$PKG_NAME$")
DEPS=$(echo -e "builtin_interfaces\n$EXT_DEPS" | sort -u)

echo -e "\033[32m✅ 扫描到依赖: $(echo $DEPS)\033[0m"

MSG_FILES_STR=""
for f in $MSG_DIR/*.msg; do
    filename=$(basename "$f")
    MSG_FILES_STR="$MSG_FILES_STR\n  \"msg/$filename\""
done

# ================= 生成 package.xml =================
echo "📝 正在生成 package.xml..."
cat << XML_EOF > $PKG_DIR/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$PKG_NAME</name>
  <version>0.0.0</version>
  <description>Auto-generated message package for $PKG_NAME</description>
  <maintainer email="dev@todo.com">dev</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

XML_EOF

for dep in $DEPS; do
    echo "  <depend>$dep</depend>" >> $PKG_DIR/package.xml
done

cat << XML_EOF >> $PKG_DIR/package.xml
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
XML_EOF

# ================= 生成 CMakeLists.txt =================
echo "📝 正在生成 CMakeLists.txt..."
cat << CMAKE_EOF > $PKG_DIR/CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project($PKG_NAME)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

CMAKE_EOF

for dep in $DEPS; do
    echo "find_package($dep REQUIRED)" >> $PKG_DIR/CMakeLists.txt
done

cat << CMAKE_EOF >> $PKG_DIR/CMakeLists.txt

rosidl_generate_interfaces(\${PROJECT_NAME}
$MSG_FILES_STR
  DEPENDENCIES $DEPS
)

ament_package()
CMAKE_EOF

echo -e "\033[32m🎉 [$PKG_NAME] 构建文件生成完毕！\033[0m"
