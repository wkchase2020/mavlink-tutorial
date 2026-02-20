# sender.py
# ========== MAVLink 心跳发送示例 ==========
# 这个程序演示如何发送 MAVLink 心跳消息
# 适合初学者学习，每一行都有详细注释

# ----- 导入需要的模块 -----
# time 模块：用于控制发送间隔
import time

# 从 pymavlink 导入 mavutil 工具模块
# mavutil 是 MAVLink 的 Python 工具库，封装了常用的功能
from pymavlink import mavutil

# 导入公共配置文件中的设置
from common_setup import (
    SENDER_ADDRESS,      # 发送端连接地址
    SYSTEM_ID,           # 系统 ID
    COMPONENT_ID,        # 组件 ID
    VEHICLE_TYPE,        # 飞行器类型
    AUTOPILOT_TYPE,      # 自动驾驶仪类型
    BASE_MODE,           # 基础模式
    CUSTOM_MODE,         # 自定义模式
    SYSTEM_STATE,        # 系统状态
    HEARTBEAT_INTERVAL,  # 发送间隔
)


def create_connection():
    """
    创建 MAVLink 连接
    
    返回:
        连接对象，用于后续发送消息
    """
    # 打印连接信息，方便调试
    print(f"正在连接到: {SENDER_ADDRESS}")
    
    # mavutil.mavlink_connection() 函数创建连接
    # 参数说明:
    #   SENDER_ADDRESS - 目标地址（在哪里发送数据）
    #   source_system  - 本设备的系统 ID
    #   source_component - 本设备的组件 ID
    #
    # udpout 表示 UDP 输出模式（发送数据）
    connection = mavutil.mavlink_connection(
        SENDER_ADDRESS,
        source_system=SYSTEM_ID,
        source_component=COMPONENT_ID
    )
    
    print("连接创建成功！")
    return connection


def send_heartbeat(connection, counter):
    """
    发送一次心跳消息
    
    参数:
        connection: MAVLink 连接对象
        counter: 心跳计数器，用于显示发送了多少次
    """
    # mavutil.mavlink.MAVLink_heartbeat_message() 创建心跳消息
    # 参数说明（按顺序）:
    #   type - 飞行器类型（如四旋翼、固定翼等）
    #   autopilot - 自动驾驶仪类型
    #   base_mode - 基础模式标志
    #   custom_mode - 自定义模式
    #   system_status - 系统状态
    #   mavlink_version - MAVLink 协议版本（通常用 3，表示 v2.0）
    
    # 创建心跳消息对象
    heartbeat_msg = mavutil.mavlink.MAVLink_heartbeat_message(
        type=VEHICLE_TYPE,           # 飞行器类型
        autopilot=AUTOPILOT_TYPE,    # 自动驾驶仪类型
        base_mode=BASE_MODE,         # 基础模式
        custom_mode=CUSTOM_MODE,     # 自定义模式
        system_status=SYSTEM_STATE,  # 系统状态
        mavlink_version=3            # 协议版本 v2.0
    )
    
    # 通过连接发送消息
    # send() 方法会将 MAVLink 消息编码并通过网络发送
    connection.mav.send(heartbeat_msg)
    
    # 打印发送信息，方便观察
    print(f"发送心跳 #{counter}")


def main():
    """
    主函数 - 程序入口点
    """
    # ----- 打印程序标题 -----
    print("=" * 40)
    print("  MAVLink 心跳发送器")
    print("=" * 40)
    print()
    
    # ----- 创建连接 -----
    # 调用 create_connection() 函数创建连接
    # 这个连接对象将被用于发送所有消息
    connection = create_connection()
    
    # ----- 等待一小会儿 -----
    # 给接收端一些启动时间
    # 在真实场景中，这一步通常不需要
    print("等待 2 秒让接收端准备...")
    time.sleep(2)
    print()
    
    # ----- 主循环：持续发送心跳 -----
    # counter 用于计数，记录发送了多少次心跳
    counter = 0
    
    print("开始发送心跳消息...")
    print(f"发送间隔: {HEARTBEAT_INTERVAL} 秒")
    print("按 Ctrl+C 停止程序")
    print()
    
    # while True 创建一个无限循环
    # 程序会持续运行，直到用户手动停止（Ctrl+C）
    while True:
        try:
            # 计数器加 1
            counter = counter + 1
            
            # 调用 send_heartbeat() 函数发送心跳
            send_heartbeat(connection, counter)
            
            # 等待指定的时间间隔
            # time.sleep() 让程序暂停执行，单位是秒
            time.sleep(HEARTBEAT_INTERVAL)
            
        except KeyboardInterrupt:
            # KeyboardInterrupt 是当用户按 Ctrl+C 时触发的异常
            # 这里捕获这个异常，让程序优雅地退出
            print()
            print("-" * 40)
            print("程序被用户停止")
            print(f"总共发送了 {counter} 条心跳消息")
            print("再见！")
            break
        except Exception as e:
            # Exception 是所有其他异常的基类
            # 这里捕获所有未预料的错误，防止程序崩溃
            print()
            print(f"发生错误: {e}")
            print("等待 1 秒后重试...")
            time.sleep(1)


# ----- 程序入口 -----
# 当直接运行这个文件时，__name__ 的值是 "__main__"
# 当作为模块导入时，__name__ 的值是模块名
# 这个检查确保 main() 只在直接运行时被调用
if __name__ == "__main__":
    main()
