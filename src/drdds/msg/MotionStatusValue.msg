uint32 drivers_over_heat                   # 驱动器过温
uint32 joints_over_heat                    # 关节过温
uint32 joints_pos_error                    # 关节角度超限
uint32 joints_vel_error                    # 关节角速度超限
uint32 joints_data_nan_error               # 关节数据nan
uint32 joints_update_error                 # 30ms数据不更新
uint32 joints_timeout                      # 30ms没数据
uint32 driver_status_error                 # 状态字
uint8 hipy_pos_outlimit                    # 髋关节位置超限
uint64 common_error                        # 按位使能: 从低位开始 依次motion_posture_error、body_posture_error、imu_timeout、imu_nan_error 、is_lift ...
