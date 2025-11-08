# RL_DEPLOY

```bash
colcon build --packages-up-to rl_deploy --cmake-args -DBUILD_PLATFORM=arm
colcon build --packages-up-to rl_deploy --cmake-args -DBUILD_PLATFORM=x86
```

```bash
# 运行环境下将m20_deploy更新到最新
sudo dpkg -i drdds-ros2-msgs.v1.0.2+.arm64.2510291519.deb # 运行arm版本需安装drdds-ros2-msgs

sudo su # 使用管理员权限
source /opt/ros/foxy/setup.bash #source ROS2 环境变量
source /opt/robot/scripts/setup_ros2.sh
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 200 # /JOINTS_DATA话题发布频率，建议不超过500hz
# 运行
source install/setup.bash
ros2 run rl_deploy rl_deploy

# 退出sdk模式：
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 0

# 键盘输入状态切换
- z： 机器狗站立进入默认状态
- c： 机器狗站立进入rl控制状态
- wasd：前后左右
- qe：顺逆时针旋转
```

/SDK_MODE：sdk模式服务  
```srv
# StdSrvInt32.srv
int32 command # 大于0设定/JOINTS_DATA话题发布频率，建议不超过500hz，否则退出sdk模式
---
int32 result 
```

/JOINTS_DATA：
电机状态  
```msg
# JointsData.msg
MetaType header
JointsDataValue data
```
```msg
# JointsDataValue.msg
JointData[16] joints_data
```
```msg
# JointData.msg
char[4] name        # 关节名称
uint16 data_id      # 数据id
uint16 status_word  # 电机状态
float32 position    # 位置（rad）
float32 torque      # 力矩（N·m）
float32 velocity    # 速度（rad/s）
float32 motion_temp # 电机温度
float32 driver_temp # 驱动器温度
```

/JOINTS_CMD：电机命令  
```msg
# JointsDataCmd.msg
MetaType header
JointsDataCmdValue data
```
```msg
# JointsDataCmdValue.msg
JointDataCmd[16] joints_data
```
```msg
# JointDataCmd.msg
char[4] name        # 关节名称
uint16 data_id      # 数据id
uint16 control_word # 电机控制字
float32 position    # 位置（rad）
float32 torque      # 力矩（N·m）
float32 velocity    # 速度（rad/s）
float32 kp          # kp
float32 kd          # kd
```



