uint8 type  # 样式

# [color] 颜色
# 渐变流水灯样式时: 数组[0] 是起点颜色，数组[1] 是终点颜色
# 双色交替闪烁样式时: 数组[0] 是颜色1，数组[1] 是颜色2
uint8[] color

uint8 num     # 灯珠数量
bool dir      # 方向: false=正向, true=反向
uint8 cycle   # 周期
