# receiver.py
# ========== MAVLink 心跳接收示例 ==========
# 这个程序演示如何接收 MAVLink 心跳消息
# 代码简单易懂，适合初学者学习

# ----- 导入需要的模块 -----
# time 模块：用于获取当前时间
import time

# 从 pymavlink 导入 mavutil 工具模块
# mavutil 提供了连接管理和消息处理的功能
from pymavlink import mavutil

# 导入公共配置文件中的设置
from common_setup import RECEIVER_ADDRESS  # 接收端监听地址


def create_connection():
    """
    创建 MAVLink 接收连接
    
    返回:
        连接对象，用于接收消息
    """
    # 打印监听信息
    print(f"正在创建监听: {RECEIVER_ADDRESS}")
    
    # mavutil.mavlink_connection() 创建连接
    # udpin 表示 UDP 输入模式（接收数据）
    # 程序会在这个地址上等待传入的消息
    connection = mavutil.mavlink_connection(RECEIVER_ADDRESS)
    
    print("监听已启动！")
    return connection


def wait_for_heartbeat(connection):
    """
    等待并接收心跳消息
    
    参数:
        connection: MAVLink 连接对象
    
    返回:
        收到的心跳消息对象，如果没有收到则返回 None
    """
    # wait_msg() 方法等待指定类型的消息
    # 参数:
    #   'HEARTBEAT' - 要等待的消息类型
    #   timeout=5.0 - 超时时间（秒），5秒内没收到就返回 None
    #   blocking=True - 阻塞模式，会一直等待直到收到消息或超时
    #
    # 注意：第一次连接时可能需要等待一段时间
    # 因为 MAVLink 需要完成协议版本协商
    
    message = connection.recv_match(
        type='HEARTBEAT',     # 指定消息类型为心跳
        blocking=True,        # 阻塞模式，没有消息就等待
        timeout=5.0           # 5秒超时
    )
    
    return message


def parse_heartbeat(message):
    """
    解析心跳消息，提取有用的信息
    
    参数:
        message: MAVLink 心跳消息对象
    
    返回:
        包含心跳信息的字典
    """
    # 创建一个字典存储解析后的信息
    # 字典是 Python 中存储键值对的数据结构
    info = {}
    
    # 获取发送方的系统 ID
    # 每个 MAVLink 设备都有唯一的系统 ID
    info['system_id'] = message.get_srcSystem()
    
    # 获取发送方的组件 ID
    # 组件 ID 标识设备内的具体功能模块
    info['component_id'] = message.get_srcComponent()
    
    # 获取飞行器类型
    # type 字段表示这是什么类型的飞行器
    info['vehicle_type'] = message.type
    
    # 将数字类型转换为人类可读的名称
    # 这里使用自定义字典，避免不同版本 pymavlink 的兼容性问题
    vehicle_types = {
        0: "通用",
        1: "固定翼",
        2: "四旋翼",
        3: "直升机",
        4: "扑翼机",
        5: "飞艇",
        6: "自由气球",
        7: "火箭",
        8: "地面漫游车",
        9: "水面船只",
        10: "水下潜航器",
        11: "其他",
        12: "八旋翼",
        13: "三旋翼",
        14: "六旋翼",
        15: "六旋翼",
        16: "八旋翼",
        17: "八旋翼",
        18: "八旋翼",
        19: "八旋翼",
        20: "VTOL 固定翼",
        21: "VTOL 四旋翼",
        22: "VTOL 尾坐式",
        23: "VTOL 倾转旋翼",
        24: "VTOL 混合",
        25: "八旋翼",
        26: "八旋翼",
        27: "八旋翼"
    }
    vehicle_type_name = vehicle_types.get(
        message.type, 
        f"未知类型({message.type})"
    )
    info['vehicle_type_name'] = vehicle_type_name
    
    # 获取自动驾驶仪类型
    info['autopilot'] = message.autopilot
    
    # 获取基础模式
    info['base_mode'] = message.base_mode
    
    # 获取自定义模式（飞行模式）
    info['custom_mode'] = message.custom_mode
    
    # 获取系统状态
    info['system_status'] = message.system_status
    
    # 将状态码转换为人类可读的名称
    status_names = {
        1: "初始化中",
        2: "校准中",
        3: "待机",
        4: "激活",
        5: "故障",
        6: "关闭中"
    }
    info['status_name'] = status_names.get(
        message.system_status, 
        f"未知状态({message.system_status})"
    )
    
    return info


def display_heartbeat(info):
    """
    显示心跳消息的信息
    
    参数:
        info: 包含心跳信息的字典
    """
    # 获取当前时间
    current_time = time.strftime("%Y-%m-%d %H:%M:%S")
    
    # 打印心跳信息
    print()
    print("=" * 40)
    print("收到心跳消息！")
    print("-" * 40)
    print(f"  发送方系统 ID: {info['system_id']}")
    print(f"  发送方组件 ID: {info['component_id']}")
    print(f"  飞行器类型: {info['vehicle_type']} ({info['vehicle_type_name']})")
    print(f"  自动驾驶仪: {info['autopilot']}")
    print(f"  基础模式: {info['base_mode']}")
    print(f"  自定义模式: {info['custom_mode']}")
    print(f"  系统状态: {info['system_status']} ({info['status_name']})")
    print(f"  接收时间: {current_time}")
    print("=" * 40)


def main():
    """
    主函数 - 程序入口点
    """
    # ----- 打印程序标题 -----
    print("=" * 40)
    print("  MAVLink 心跳接收器")
    print("=" * 40)
    print()
    
    # ----- 创建连接 -----
    connection = create_connection()
    
    # ----- 等待第一条心跳（特殊处理）-----
    # 第一条心跳通常需要等待较长时间
    # 因为 MAVLink 需要完成协议版本协商
    print()
    print("等待第一条心跳消息...")
    print("（首次连接可能需要几秒钟）")
    print()
    
    # 循环直到收到第一条心跳
    first_heartbeat = None
    while first_heartbeat is None:
        first_heartbeat = wait_for_heartbeat(connection)
        if first_heartbeat is None:
            print("超时，重新等待...")
    
    # 解析并显示第一条心跳
    info = parse_heartbeat(first_heartbeat)
    display_heartbeat(info)
    print("连接已建立！继续接收心跳...")
    print()
    
    # ----- 主循环：持续接收心跳 -----
    print("按 Ctrl+C 停止程序")
    print()
    
    # 计数器：记录收到的心跳数量
    heartbeat_count = 1
    
    while True:
        try:
            # 等待下一条心跳消息
            message = wait_for_heartbeat(connection)
            
            if message is not None:
                # 收到消息，计数加 1
                heartbeat_count = heartbeat_count + 1
                
                # 解析消息
                info = parse_heartbeat(message)
                
                # 显示消息
                display_heartbeat(info)
                
                # 打印计数
                print(f"[已接收 {heartbeat_count} 条心跳]")
            else:
                # 超时，没有收到消息
                print("等待心跳中...（5秒未收到消息）")
                
        except KeyboardInterrupt:
            # 用户按 Ctrl+C 停止程序
            print()
            print("-" * 40)
            print("程序被用户停止")
            print(f"总共接收了 {heartbeat_count} 条心跳消息")
            print("再见！")
            break
        except Exception as e:
            # 处理其他错误
            print()
            print(f"发生错误: {e}")
            print("继续接收...")


# ----- 程序入口 -----
if __name__ == "__main__":
    main()
