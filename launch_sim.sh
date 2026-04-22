#!/bin/bash
# Sim2Sim launch script - Terminator (preferred) or gnome-terminal fallback
SDK_DIR="$HOME/Software/sdk_deploy"
RL_WS_DIR="$HOME/Software/rl_ws"
DOMAIN_ID=1

PRE_SDK="conda deactivate 2>/dev/null; export ROS_DOMAIN_ID=$DOMAIN_ID; cd $SDK_DIR && source install/setup.zsh"
PRE_RLWS="conda deactivate 2>/dev/null; export ROS_DOMAIN_ID=$DOMAIN_ID; cd $RL_WS_DIR && source install/setup.zsh"

CMD1="$PRE_SDK && python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py; exec zsh"
CMD2="$PRE_RLWS && ros2 launch elevation_mapping_cupy elevation_mapping.launch.py robot_config:=m20.yaml; exec zsh"
CMD3="$PRE_RLWS && python3 src/elevation_mapping_cupy/scripts/udp_brideg_mapping_node.py; exec zsh"
CMD4="$PRE_SDK && python3 src/M20_sdk_deploy/scripts/udp_bridge_sdk_node.py; exec zsh"
CMD5="$PRE_SDK && python3 src/M20_sdk_deploy/scripts/lidar_to_scan.py; exec zsh"
CMD6="$PRE_SDK && ros2 run m20_sdk_deploy rl_deploy; exec zsh"

if command -v terminator &>/dev/null; then
    # Create wrapper scripts to avoid quoting issues in Terminator's ConfigObj parser
    cat > /tmp/sim_cmd_mujoco.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_elevmap.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
cd ~/Software/rl_ws && source install/setup.zsh
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py robot_config:=m20.yaml
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_udp_map.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
cd ~/Software/rl_ws && source install/setup.zsh
python3 src/elevation_mapping_cupy/scripts/udp_brideg_mapping_node.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_udp_sdk.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/scripts/udp_bridge_sdk_node.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_lidar2scan.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
cd ~/Software/sdk_deploy && source install/setup.zsh
python3 src/M20_sdk_deploy/scripts/lidar_to_scan.py
exec zsh
SCRIPT

    cat > /tmp/sim_cmd_rl_deploy.sh << 'SCRIPT'
#!/bin/zsh
conda deactivate 2>/dev/null
export ROS_DOMAIN_ID=1
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
      ratio = 0.66
    [[[child2]]]
      type = VPaned
      parent = child1
      order = 0
      ratio = 0.5
    [[[row1]]]
      type = HPaned
      parent = child2
      order = 0
      ratio = 0.5
    [[[mujoco]]]
      type = Terminal
      parent = row1
      order = 0
      title = MuJoCo
      command = /tmp/sim_cmd_mujoco.sh
    [[[elevmap]]]
      type = Terminal
      parent = row1
      order = 1
      title = ElevMap
      command = /tmp/sim_cmd_elevmap.sh
    [[[row2]]]
      type = HPaned
      parent = child2
      order = 1
      ratio = 0.5
    [[[udp_map]]]
      type = Terminal
      parent = row2
      order = 0
      title = UDP Map (Server)
      command = /tmp/sim_cmd_udp_map.sh
    [[[udp_sdk]]]
      type = Terminal
      parent = row2
      order = 1
      title = UDP SDK (Client)
      command = /tmp/sim_cmd_udp_sdk.sh
    [[[row3]]]
      type = HPaned
      parent = child1
      order = 1
      ratio = 0.5
    [[[lidar2scan]]]
      type = Terminal
      parent = row3
      order = 0
      title = LiDAR2Scan
      command = /tmp/sim_cmd_lidar2scan.sh
    [[[rl_deploy]]]
      type = Terminal
      parent = row3
      order = 1
      title = RL Deploy
      command = /tmp/sim_cmd_rl_deploy.sh
[plugins]
TCONF
    echo "Launching Terminator with 3x2 grid..."
    terminator --no-dbus -g "$CFG" -l sim -m 2>/dev/null &
else
    echo "Terminator not found, using gnome-terminal..."
    gnome-terminal --title="MuJoCo"          -- zsh -ic "$CMD1" &
    gnome-terminal --title="ElevMap"         -- zsh -ic "$CMD2" &
    gnome-terminal --title="UDP Map"         -- zsh -ic "$CMD3" &
    sleep 2
    gnome-terminal --title="UDP SDK"         -- zsh -ic "$CMD4" &
    gnome-terminal --title="LiDAR2Scan"      -- zsh -ic "$CMD5" &
    gnome-terminal --title="RL Deploy"       -- zsh -ic "$CMD6" &
fi

echo "All nodes launched."
