# quick_test.py
# ========== 快速测试脚本 ==========
# 这个脚本用于快速验证 MAVLink 环境是否安装正确
# 适合初学者运行，检查是否可以正常导入和使用 pymavlink

print("=" * 50)
print("  MAVLink 环境快速测试")
print("=" * 50)
print()

# ----- 测试 1: 导入模块 -----
print("测试 1: 导入 pymavlink 模块...")
try:
    from pymavlink import mavutil
    print("  ✓ 导入成功!")
except ImportError as e:
    print(f"  ✗ 导入失败: {e}")
    print("  请运行: pip install pymavlink")
    exit(1)

# ----- 测试 2: 创建 MAVLink 消息 -----
print("测试 2: 创建 MAVLink 心跳消息...")
try:
    # 创建一条心跳消息
    heartbeat = mavutil.mavlink.MAVLink_heartbeat_message(
        type=2,           # 四旋翼
        autopilot=0,      # 通用自动驾驶仪
        base_mode=0,      # 基础模式
        custom_mode=0,    # 自定义模式
        system_status=4,  # 激活状态
        mavlink_version=3 # 协议版本
    )
    print("  ✓ 消息创建成功!")
    print(f"    消息类型: {heartbeat.get_type()}")
except Exception as e:
    print(f"  ✗ 创建失败: {e}")
    exit(1)

# ----- 测试 3: 编码消息 -----
print("测试 3: 编码消息...")
try:
    # 将消息编码为字节
    bytes_data = heartbeat.pack(mavutil.mavlink.MAVLink(0, 0, 0))
    print("  ✓ 消息编码成功!")
    print(f"    消息大小: {len(bytes_data)} 字节")
except Exception as e:
    print(f"  ✗ 编码失败: {e}")
    exit(1)

# ----- 测试 4: 消息解码 -----
print("测试 4: 解码消息...")
try:
    # 创建 MAVLink 解析器
    mav = mavutil.mavlink.MAVLink(0, 0, 0)
    
    # 解析字节数据
    decoded = mav.decode(bytes_data)
    print("  ✓ 消息解码成功!")
    print(f"    飞行器类型: {decoded.type}")
    print(f"    系统状态: {decoded.system_status}")
except Exception as e:
    print(f"  ✗ 解码失败: {e}")
    exit(1)

# ----- 测试完成 -----
print()
print("=" * 50)
print("  所有测试通过! ✓")
print("=" * 50)
print()
print("你现在可以运行:")
print("  1. python receiver.py  (窗口 1)")
print("  2. python sender.py    (窗口 2)")
print()
