bool imu_error          # IMU 错误
bool wifi_error         # WiFi 错误

# 驱动器温度警告/错误计数或标志
uint32 driver_heat_warn
uint32 driver_heat_error

# 电机温度警告/错误
uint32 motor_heat_warn
uint32 motor_heat_error

# 电池相关
uint32 battery_low_warn
uint32 battery_low_error
uint32 battery_voltage_warn
uint32 battery_voltage_error
uint32 battery_heat_warn
uint32 battery_heat_error

# CPU / 频率
uint32 cpu_heat_warn
uint32 cpu_freq_warn
