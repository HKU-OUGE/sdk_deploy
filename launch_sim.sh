#!/bin/bash
# Sim2Sim launch script - Terminator (preferred) or gnome-terminal fallback
#
# 自 ElevationAE 移除后，elevation_mapping_cupy 与对应 UDP map bridge 已不再需要。
# 当前 5 个 pane:
#   - MuJoCo:       仿真 + lidar 点云发布 (/LIDAR_SIM_RAW)
#   - UDP SDK:      跨机器 lidar 接收（同机器 sim2sim 实际不需要，但保留以兼容真机部署）
#   - Scan992:      订阅 /LIDAR_SIM_RAW，输出旧策略需要的 /scan/multi_layer_features_array (992)
#                   RB platform / LB crawl / BACK gap 等旧 1049 scan 策略依赖它；不包含 height_map
#   - NoisyElev:    订阅 /LIDAR_SIM_RAW + /height_map + /LIO_ODOM，输出 691-dim noisy_elevation
#                   Y 新 unified+elevation 策略=748 依赖它
#   - RL Deploy:    onnx policy + state machine
SDK_DIR="$HOME/Software/sdk_deploy"
DOMAIN_ID=1
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SOURCE_ROS="if [ -f /opt/ros/$ROS_DISTRO/setup.zsh ]; then source /opt/ros/$ROS_DISTRO/setup.zsh; else source /opt/ros/$ROS_DISTRO/setup.bash; fi"
PRE_SDK="conda deactivate 2>/dev/null; export ROS_DOMAIN_ID=$DOMAIN_ID; $SOURCE_ROS; cd $SDK_DIR && source install/setup.zsh"

CMD1="$PRE_SDK && python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py; exec zsh"
CMD4="$PRE_SDK && python3 src/M20_sdk_deploy/scripts/udp_bridge_sdk_node.py; exec zsh"
CMD_SCAN="$PRE_SDK && ros2 run m20_sdk_deploy lidar_to_scan_cpp --ros-args -p lidar_topic:=/LIDAR_SIM_RAW; exec zsh"
CMD5="$PRE_SDK && python3 src/M20_sdk_deploy/scripts/noisy_elevation_node.py; exec zsh"
CMD6="$PRE_SDK && ros2 run m20_sdk_deploy rl_deploy; exec zsh"

if command -v terminator &>/dev/null; then
    # Create wrapper scripts to avoid quoting issues in Terminator's ConfigObj parser
    cat > /tmp/sim_cmd_mujoco.sh << 'SCRIPT'
#!/bin/zsh -f
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh; else source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; fi
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_udp_sdk.sh << 'SCRIPT'
#!/bin/zsh -f
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh; else source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; fi
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/scripts/udp_bridge_sdk_node.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_scan992.sh << 'SCRIPT'
#!/bin/zsh -f
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh; else source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; fi
cd ~/Software/sdk_deploy && source install/setup.zsh
ros2 run m20_sdk_deploy lidar_to_scan_cpp --ros-args -p lidar_topic:=/LIDAR_SIM_RAW
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_noisy_elev.sh << 'SCRIPT'
#!/bin/zsh -f
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh; else source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; fi
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/scripts/noisy_elevation_node.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_rl_deploy.sh << 'SCRIPT'
#!/bin/zsh -f
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO:-jazzy}/setup.zsh; else source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; fi
cd ~/Software/sdk_deploy && source install/setup.zsh
ros2 run m20_sdk_deploy rl_deploy
exec zsh
SCRIPT

    chmod +x /tmp/sim_cmd_*.sh

    CFG="/tmp/sim_launch_terminator.conf"
    cat > "$CFG" << 'TCONF'
[global_config]
  title_hide_sizetext = True
[profiles]
  [[default]]
    use_system_font = True
    scrollback_lines = 10000
    show_titlebar = True
[layouts]
  [[default]]
    [[[child0]]]
      type = Window
      parent = ""
    [[[terminal1]]]
      type = Terminal
      parent = child0
  [[sim]]
    [[[child0]]]
      type = Window
      parent = ""
      order = 0
    [[[child1]]]
      type = VPaned
      parent = child0
      order = 0
      ratio = 0.5
    [[[row1]]]
      type = HPaned
      parent = child1
      order = 0
      ratio = 0.5
    [[[mujoco]]]
      type = Terminal
      parent = row1
      order = 0
      title = MuJoCo
      command = /tmp/sim_cmd_mujoco.sh
    [[[udp_sdk]]]
      type = Terminal
      parent = row1
      order = 1
      title = UDP SDK (lidar)
      command = /tmp/sim_cmd_udp_sdk.sh
    [[[row2]]]
      type = HPaned
      parent = child1
      order = 1
      ratio = 0.5
    [[[scan992]]]
      type = Terminal
      parent = row2
      order = 0
      title = Scan992(/scan)
      command = /tmp/sim_cmd_scan992.sh
    [[[right_bottom]]]
      type = HPaned
      parent = row2
      order = 1
      ratio = 0.5
    [[[noisy_elev]]]
      type = Terminal
      parent = right_bottom
      order = 0
      title = NoisyElev(691)
      command = /tmp/sim_cmd_noisy_elev.sh
    [[[rl_deploy]]]
      type = Terminal
      parent = right_bottom
      order = 1
      title = RL Deploy
      command = /tmp/sim_cmd_rl_deploy.sh
[plugins]
TCONF
    echo "Launching Terminator with 2x2 grid (elevation pipeline removed)..."
    terminator --no-dbus -g "$CFG" -l sim -m 2>/dev/null &
else
    echo "Terminator not found, using gnome-terminal..."
    gnome-terminal --title="MuJoCo"          -- zsh -ic "$CMD1" &
    sleep 1
    gnome-terminal --title="UDP SDK"         -- zsh -ic "$CMD4" &
    gnome-terminal --title="Scan992"         -- zsh -ic "$CMD_SCAN" &
    gnome-terminal --title="NoisyElev"       -- zsh -ic "$CMD5" &
    gnome-terminal --title="RL Deploy"       -- zsh -ic "$CMD6" &
fi

echo "All nodes launched (5 panes: MuJoCo / UDP SDK / Scan992 / NoisyElev / RL Deploy)."
