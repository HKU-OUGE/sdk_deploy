cmake_minimum_required(VERSION 3.14)
project(drdds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MetaType.msg"
  "msg/LossStatusValue.msg"
  "msg/JointsCmd.msg"
  "msg/PlannerStatusValue.msg"
  "msg/NavCmdValue.msg"
  "msg/AiryLidarStatus.msg"
  "msg/StdStatus.msg"
  "msg/MotionInfo.msg"
  "msg/StdMsgInt32.msg"
  "msg/Joints.msg"
  "msg/JointDataCmd.msg"
  "msg/NavSatValue.msg"
  "msg/ExecStatusValue.msg"
  "msg/NavSat.msg"
  "msg/JointsDataValue.msg"
  "msg/FaultStatus.msg"
  "msg/JointData.msg"
  "msg/LedStatusLightValue.msg"
  "msg/JointValue.msg"
  "msg/MotionStatusValue.msg"
  "msg/BatteryData.msg"
  "msg/Steer.msg"
  "msg/LedStatusLight.msg"
  "msg/JointsData.msg"
  "msg/CpuData.msg"
  "msg/AiryLidarValue.msg"
  "msg/JointsDataCmdValue.msg"
  "msg/StdMsgFloat32.msg"
  "msg/NodeStatusValue.msg"
  "msg/MotionStateValue.msg"
  "msg/JointsDataCmd.msg"
  "msg/NodeStatus.msg"
  "msg/LocationStatus.msg"
  "msg/SteerValue.msg"
  "msg/GamepadData.msg"
  "msg/LocationStatusValue.msg"
  "msg/FaultStatusValue.msg"
  "msg/GaitValue.msg"
  "msg/NavCmd.msg"
  "msg/ImuDataValue.msg"
  "msg/JointCmdValue.msg"
  "msg/MotionStatus.msg"
  "msg/Timestamp.msg"
  "msg/Gait.msg"
  "msg/InputStatusValue.msg"
  "msg/MotionInfoValue.msg"
  "msg/MotionState.msg"
  "msg/ImuData.msg"
  "msg/CpuDataValue.msg"
  "msg/BatteryDataValue.msg"
  "msg/PlannerStatus.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
