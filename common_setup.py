# common_setup.py
# ========== 公共配置模块 ==========
# 这个文件保存发送端和接收端都需要用到的配置信息
# 目的是避免在两个文件中重复写相同的代码

# ----- 网络配置 -----
# IP 地址: 127.0.0.1 是"本地回环地址"
# 意思是数据只在电脑内部传输，不会发送到网络上
# 适合在同一台电脑上测试使用
IP_ADDRESS = "127.0.0.1"

# 端口号: 用于区分不同的网络服务
# 14550 是 MAVLink 地面站常用的默认端口
# 发送端和接收端需要使用相同的端口才能通信
PORT = 14550

# 组合成完整的连接地址
# 格式: udpout:IP:端口 (用于发送)
# 格式: udpin:IP:端口 (用于接收)
SENDER_ADDRESS = f"udpout:{IP_ADDRESS}:{PORT}"   # 发送端使用的地址
RECEIVER_ADDRESS = f"udpin:{IP_ADDRESS}:{PORT}"  # 接收端使用的地址

# ----- MAVLink 系统配置 -----
# 系统 ID: 标识发送设备的唯一编号
# 范围: 1-255
# 在同一网络中，每个设备应该有唯一的系统 ID
SYSTEM_ID = 1

# 组件 ID: 标识设备内的具体组件
# 1 = 自动驾驶仪（飞控）
# 说明：这里我们模拟的是一个飞控设备
COMPONENT_ID = 1

# ----- 心跳消息配置 -----
# 飞行器类型
# MAV_TYPE_QUADROTOR = 2 表示四旋翼无人机
# 其他常见值：
#   0 = 通用类型
#   1 = 固定翼
#   2 = 四旋翼
#   13 = 六旋翼
VEHICLE_TYPE = 2  # 四旋翼

# 自动驾驶仪类型
# MAV_AUTOPILOT_GENERIC = 0 表示通用自动驾驶仪
# MAV_AUTOPILOT_ARDUPILOTMEGA = 3 表示 ArduPilot
AUTOPILOT_TYPE = 0  # 通用类型

# 基础模式
# 0 表示手动/稳定模式
BASE_MODE = 0

# 自定义模式
# 用于表示具体的飞行模式（如定高、定点等）
# 0 表示默认模式
CUSTOM_MODE = 0

# 系统状态
# MAV_STATE_ACTIVE = 4 表示系统正常工作中
# 其他值：
#   1 = 正在初始化
#   2 = 正在校准
#   3 = 待机
#   4 = 激活/工作中
#   5 = 故障
#   6 = 关闭中
SYSTEM_STATE = 4  # 激活状态

# ----- 发送间隔配置 -----
# 心跳发送间隔，单位：秒
# MAVLink 标准要求至少每 1 秒发送一次心跳
# 这里设置为 1 秒，符合标准要求
HEARTBEAT_INTERVAL = 1.0
