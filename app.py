import streamlit as st
import time
from datetime import datetime, timedelta
from collections import deque

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 心跳包演示",
    page_icon="🚁",
    layout="wide"
)

# ==================== 时区转换函数 ====================
def get_local_time():
    """获取中国标准时间 (UTC+8)"""
    utc_time = datetime.utcnow()
    local_time = utc_time + timedelta(hours=8)
    return local_time

# ==================== MAVLink 常量 ====================
MAV_TYPE = {
    0: "GENERIC", 1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL",
    4: "HELICOPTER", 5: "ANTENNA_TRACKER", 6: "GCS", 7: "AIRSHIP",
    8: "FREE_BALLOON", 9: "ROCKET", 10: "GROUND_ROVER",
    11: "SURFACE_BOAT", 12: "SUBMARINE", 13: "HEXAROTOR",
    14: "OCTOROTOR", 15: "TRICOPTER", 16: "FLAPPING_WING",
    17: "KITE", 18: "ONBOARD_CONTROLLER", 19: "VTOL_DUOROTOR",
    20: "VTOL_QUADROTOR", 21: "VTOL_TILTROTOR", 22: "VTOL_RESERVED2",
    23: "VTOL_RESERVED3", 24: "VTOL_RESERVED4", 25: "VTOL_RESERVED5",
    26: "GIMBAL", 27: "ADSB", 28: "PARAFOIL", 29: "DODECAROTOR",
    30: "CAMERA", 31: "CHARGING_STATION", 32: "FLARM",
    33: "SERVO", 34: "ODID", 35: "DECAROTOR", 36: "BATTERY",
    37: "PARACHUTE", 38: "LOG", 39: "OSD", 40: "IMU",
    41: "GPS", 42: "WINCH"
}

MAV_AUTOPILOT = {0: "GENERIC", 3: "ARDUPILOTMEGA", 12: "PX4"}
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY", 4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}

# 系统ID定义
SYSTEM_ID_MAP = {
    0: "广播地址",
    1: "自动驾驶仪/主飞行器",
    2: "地面控制站 (GCS)",
    3: "任务规划系统",
    4: "相机/成像系统",
    5: "云台/稳定系统",
    6: "遥测无线电",
    7: "Companion Computer",
    8: "路径规划系统",
    9: "遥控接收机",
    10: "电池管理系统",
    11: "伺服/执行器系统",
    12: "避障系统",
    13: "降落伞系统",
    14: "日志系统",
    15: "ADSB 接收机",
    16: "光学流量传感器",
    17: "视觉系统",
    18: "红外系统",
    19: "声纳/雷达",
    20: "GPS 模块",
    21: "气压计",
    22: "磁力计",
    23: "IMU 传感器",
    24: "激光雷达",
    25: "VIO 系统"
}

# 组件ID定义
COMPONENT_ID_MAP = {
    0: "广播（所有组件）",
    1: "自动驾驶仪（主控）",
    2: "任务计算机",
    3: "遥控输入",
    4: "遥测输出",
    5: "相机 #1",
    6: "相机 #2",
    7: "相机 #3",
    8: "云台 #1",
    9: "云台 #2",
    10: "伺服 #1",
    11: "伺服 #2",
    12: "伺服 #3",
    13: "伺服 #4",
    14: "伺服 #5",
    15: "伺服 #6",
    16: "伺服 #7",
    17: "伺服 #8",
    18: "GPS #1",
    19: "GPS #2",
    20: "气压计 #1",
    21: "气压计 #2",
    22: "IMU #1",
    23: "IMU #2",
    24: "IMU #3",
    25: "磁力计 #1",
    26: "磁力计 #2",
    27: "激光雷达 #1",
    28: "激光雷达 #2",
    29: "光流传感器",
    30: "视觉系统 #1",
    31: "视觉系统 #2",
    32: "红外传感器",
    33: "超声波传感器",
    34: "雷达 #1",
    35: "雷达 #2",
    36: "ADS-B 接收机",
    37: "应答机",
    38: "TCAS",
    39: "ACAS",
    40: "地形感知",
    41: "近地警告",
    42: "避障系统 #1",
    43: "避障系统 #2",
    44: "路径规划",
    45: "任务规划",
    46: "地理围栏",
    47: "返航系统",
    48: "降落系统",
    49: "伞降系统",
    50: "气囊系统",
    51: "浮力系统"
}

# ==================== 预定义通信场景 ====================
COMMUNICATION_SCENARIOS = {
    "drone_to_gcs": {
        "name": "🚁 无人机 → 地面站",
        "description": "无人机飞控发送心跳包给地面控制站，用于状态监控和连接保持",
        "sender_sys": 1,
        "sender_comp": 1,
        "sender_name": "无人机飞控",
        "receiver_sys": 2,
        "receiver_comp": 4,
        "receiver_name": "地面控制站 (GCS)",
        "mav_type": 2,
        "icon": "🚁→🖥️"
    },
    "sensor_to_fc": {
        "name": "📡 传感器 → 飞控",
        "description": "机载传感器（GPS、雷达等）向飞控上报数据和状态",
        "sender_sys": 20,
        "sender_comp": 18,
        "sender_name": "GPS 模块",
        "receiver_sys": 1,
        "receiver_comp": 1,
        "receiver_name": "飞控主控",
        "mav_type": 0,
        "icon": "📡→🧠"
    },
    "gcs_to_drone": {
        "name": "🖥️ 地面站 → 无人机",
        "description": "地面站发送任务指令或控制命令给无人机",
        "sender_sys": 2,
        "sender_comp": 3,
        "sender_name": "地面控制站",
        "receiver_sys": 1,
        "receiver_comp": 1,
        "receiver_name": "无人机飞控",
        "mav_type": 6,
        "icon": "🖥️→🚁"
    },
    "companion_to_fc": {
        "name": "💻 伴机电脑 → 飞控",
        "description": "Companion Computer 向飞控发送高级控制指令或任务数据",
        "sender_sys": 7,
        "sender_comp": 2,
        "sender_name": "Companion Computer",
        "receiver_sys": 1,
        "receiver_comp": 1,
        "receiver_name": "飞控主控",
        "mav_type": 18,
        "icon": "💻→🧠"
    },
    "custom": {
        "name": "⚙️ 自定义配置",
        "description": "手动配置发送端和接收端身份，灵活模拟各种场景",
        "sender_sys": 1,
        "sender_comp": 1,
        "sender_name": "自定义",
        "receiver_sys": 2,
        "receiver_comp": 4,
        "receiver_name": "自定义",
        "mav_type": 2,
        "icon": "⚙️"
    }
}

# ==================== 会话状态 ====================
if 'send_log' not in st.session_state:
    st.session_state.send_log = deque(maxlen=20)
if 'recv_log' not in st.session_state:
    st.session_state.recv_log = deque(maxlen=20)
if 'is_running' not in st.session_state:
    st.session_state.is_running = False
if 'send_count' not in st.session_state:
    st.session_state.send_count = 0
if 'recv_count' not in st.session_state:
    st.session_state.recv_count = 0

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 心跳包实时演示")
st.caption("模拟 MAVLink 通信协议 | 支持多种典型应用场景 | 北京时间 (UTC+8)")

# ==================== 场景选择 ====================
st.header("📋 选择通信场景")

scenario_cols = st.columns(len(COMMUNICATION_SCENARIOS))

selected_scenario = None
for idx, (key, scenario) in enumerate(COMMUNICATION_SCENARIOS.items()):
    with scenario_cols[idx]:
        # 创建卡片式按钮
        card_style = """
        <style>
        .scenario-card {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
            cursor: pointer;
            transition: transform 0.2s;
            border: 3px solid transparent;
        }
        .scenario-card:hover {
            transform: scale(1.05);
            border-color: #00FF00;
        }
        .scenario-card.selected {
            border-color: #00FF00;
            box-shadow: 0 0 20px rgba(0,255,0,0.5);
        }
        </style>
        """
        st.markdown(card_style, unsafe_allow_html=True)
        
        # 使用 radio 按钮实现单选
        if st.radio(
            label="",
            options=[key],
            format_func=lambda x: f"{scenario['icon']}\n\n**{scenario['name']}**\n\n<small>{scenario['description'][:30]}...</small>",
            key=f"scenario_{key}",
            label_visibility="collapsed"
        ):
            selected_scenario = key

# 获取选中的场景配置
if selected_scenario is None:
    selected_scenario = "drone_to_gcs"  # 默认场景

scenario = COMMUNICATION_SCENARIOS[selected_scenario]

# 显示选中场景的详细信息
st.info(f"""
**当前场景:** {scenario['name']} {scenario['icon']}

{scenario['description']}

**发送端:** {scenario['sender_name']} (系统ID: {scenario['sender_sys']}, 组件ID: {scenario['sender_comp']})  
**接收端:** {scenario['receiver_name']} (系统ID: {scenario['receiver_sys']}, 组件ID: {scenario['receiver_comp']})
""")

# ==================== 自定义配置（仅在自定义场景显示） ====================
if selected_scenario == "custom":
    st.subheader("⚙️ 自定义配置")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.markdown("**📤 发送端配置**")
        custom_sender_sys = st.selectbox(
            "发送端系统 ID",
            options=list(SYSTEM_ID_MAP.keys()),
            format_func=lambda x: f"{x}: {SYSTEM_ID_MAP[x]}",
            index=1,
            key="custom_sender_sys"
        )
        custom_sender_comp = st.selectbox(
            "发送端组件 ID",
            options=list(COMPONENT_ID_MAP.keys()),
            format_func=lambda x: f"{x}: {COMPONENT_ID_MAP[x]}",
            index=1,
            key="custom_sender_comp"
        )
    
    with col2:
        st.markdown("**📥 接收端配置**")
        custom_receiver_sys = st.selectbox(
            "接收端系统 ID",
            options=list(SYSTEM_ID_MAP.keys()),
            format_func=lambda x: f"{x}: {SYSTEM_ID_MAP[x]}",
            index=2,
            key="custom_receiver_sys"
        )
        custom_receiver_comp = st.selectbox(
            "接收端组件 ID",
            options=list(COMPONENT_ID_MAP.keys()),
            format_func=lambda x: f"{x}: {COMPONENT_ID_MAP[x]}",
            index=4,
            key="custom_receiver_comp"
        )
    
    # 使用自定义配置
    sender_sys = custom_sender_sys
    sender_comp = custom_sender_comp
    receiver_sys = custom_receiver_sys
    receiver_comp = custom_receiver_comp
    mav_type = st.selectbox("飞行器类型", list(MAV_TYPE.keys()), 
                           format_func=lambda x: f"{x}: {MAV_TYPE[x]}", index=2)
else:
    # 使用场景预设配置
    sender_sys = scenario['sender_sys']
    sender_comp = scenario['sender_comp']
    receiver_sys = scenario['receiver_sys']
    receiver_comp = scenario['receiver_comp']
    mav_type = scenario['mav_type']

# ==================== 控制面板 ====================
st.markdown("---")
col_ctrl1, col_ctrl2, col_ctrl3 = st.columns([2, 1, 2])

with col_ctrl1:
    st.markdown(f"""
    <div style="background: linear-gradient(135deg, #FF6B6B 0%, #EE5A6F 100%); padding: 20px; border-radius: 15px; text-align: center; box-shadow: 0 4px 15px rgba(255,107,107,0.4);">
        <h3 style="color: white; margin: 0 0 10px 0;">📤 发送端</h3>
        <div style="background: rgba(255,255,255,0.2); padding: 10px; border-radius: 8px; margin: 5px 0;">
            <p style="color: white; margin: 0; font-size: 18px; font-weight: bold;">{SYSTEM_ID_MAP.get(sender_sys, '未知')}</p>
            <p style="color: #FFE66D; margin: 5px 0 0 0; font-size: 14px;">系统ID: {sender_sys} | 组件ID: {sender_comp}</p>
        </div>
        <p style="color: white; margin: 10px 0 0 0; font-size: 12px;">
            {COMPONENT_ID_MAP.get(sender_comp, '未知组件')}
        </p>
    </div>
    """, unsafe_allow_html=True)

with col_ctrl2:
    # 动态箭头和控制按钮
    arrow_color = "#00FF00" if st.session_state.is_running else "#888888"
    pulse_anim = "🔥" if st.session_state.is_running else "⚡"
    
    st.markdown(f"""
    <div style="text-align: center; padding-top: 20px;">
        <div style="color: {arrow_color}; font-size: 32px; margin-bottom: 10px;">
            ➤➤➤
        </div>
        <div style="color: #888; font-size: 12px; margin-bottom: 20px;">
            MAVLink 2.0<br>UDP:14550
        </div>
    </div>
    """, unsafe_allow_html=True)
    
    # 控制按钮放在中间列底部
    btn_col1, btn_col2 = st.columns(2)
    with btn_col1:
        if st.button("▶️ 启动", disabled=st.session_state.is_running, type="primary", use_container_width=True):
            st.session_state.is_running = True
            st.rerun()
    with btn_col2:
        if st.button("⏹️ 停止", disabled=not st.session_state.is_running, type="secondary", use_container_width=True):
            st.session_state.is_running = False
            st.rerun()

with col_ctrl3:
    st.markdown(f"""
    <div style="background: linear-gradient(135deg, #4ECDC4 0%, #44A08D 100%); padding: 20px; border-radius: 15px; text-align: center; box-shadow: 0 4px 15px rgba(78,205,196,0.4);">
        <h3 style="color: white; margin: 0 0 10px 0;">📥 接收端</h3>
        <div style="background: rgba(255,255,255,0.2); padding: 10px; border-radius: 8px; margin: 5px 0;">
            <p style="color: white; margin: 0; font-size: 18px; font-weight: bold;">{SYSTEM_ID_MAP.get(receiver_sys, '未知')}</p>
            <p style="color: #FFE66D; margin: 5px 0 0 0; font-size: 14px;">系统ID: {receiver_sys} | 组件ID: {receiver_comp}</p>
        </div>
        <p style="color: white; margin: 10px 0 0 0; font-size: 12px;">
            {COMPONENT_ID_MAP.get(receiver_comp, '未知组件')}
        </p>
    </div>
    """, unsafe_allow_html=True)

# ==================== 统计区域 ====================
st.markdown("---")
st.subheader("📊 实时统计")
col1, col2, col3, col4 = st.columns(4)
col1.metric("📤 已发送", st.session_state.send_count)
col2.metric("📥 已接收", st.session_state.recv_count)
col3.metric("⏱️ 发送间隔", f"{st.slider('间隔(秒)', 0.5, 3.0, 1.0, 0.1, label_visibility='collapsed')}s")
col4.metric("🚁 飞行器", MAV_TYPE.get(mav_type, "UNKNOWN"))

# ==================== 发送/接收日志 ====================
st.markdown("---")

col_send_log, col_recv_log = st.columns(2)

# 左侧：发送端日志
with col_send_log:
    st.subheader(f"📤 发送日志")
    st.markdown(f"<small>来自: {SYSTEM_ID_MAP.get(sender_sys, '未知')} (SYS:{sender_sys}/COMP:{sender_comp})</small>", unsafe_allow_html=True)
    
    send_container = st.container()
    with send_container:
        if st.session_state.send_log:
            for log in reversed(list(st.session_state.send_log)[-8:]):
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:10px;margin:5px 0;border-radius:5px;font-family:'Courier New',monospace;font-size:12px;border-left:4px solid #FF6B6B;">
                    <span style="color:#AAAAAA;">[{log['time']}]</span>
                    <span style="color:#FF6B6B;font-weight:bold;margin-left:6px;">📤 SEND</span>
                    <span style="color:#FFFFFF;margin-left:6px;">SEQ:{log['seq']}</span>
                    <span style="color:#FFD700;margin-left:6px;">TO:{log['receiver']}</span>
                    <br>
                    <span style="color:#87CEEB;font-size:11px;">{log['sender_name']} → {log['receiver_name']}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待发送数据...")

# 右侧：接收端日志
with col_recv_log:
    st.subheader(f"📥 接收日志")
    st.markdown(f"<small>目标: {SYSTEM_ID_MAP.get(receiver_sys, '未知')} (SYS:{receiver_sys}/COMP:{receiver_comp})</small>", unsafe_allow_html=True)
    
    recv_container = st.container()
    with recv_container:
        if st.session_state.recv_log:
            for log in reversed(list(st.session_state.recv_log)[-8:]):
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:10px;margin:5px 0;border-radius:5px;font-family:'Courier New',monospace;font-size:12px;border-left:4px solid #4ECDC4;">
                    <span style="color:#AAAAAA;">[{log['time']}]</span>
                    <span style="color:#4ECDC4;font-weight:bold;margin-left:6px;">📥 RECV</span>
                    <span style="color:#FFFFFF;margin-left:6px;">SEQ:{log['seq']}</span>
                    <span style="color:#FFD700;margin-left:6px;">FROM:{log['sender']}</span>
                    <br>
                    <span style="color:#87CEEB;font-size:11px;">{log['type_name']} | {log['status_name']}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待接收数据...")

# ==================== 数据包详情 ====================
st.markdown("---")
st.subheader("📦 最新 MAVLink 数据包 (HEX)")

hex_col1, hex_col2 = st.columns(2)
with hex_col1:
    st.markdown(f"<small><b>发送端 [{SYSTEM_ID_MAP.get(sender_sys, '未知')}] 发出</b></small>", unsafe_allow_html=True)
    if st.session_state.send_log:
        last_send = list(st.session_state.send_log)[-1]
        st.code(last_send['hex'], language='hex')
    else:
        st.code("等待数据...", language='text')

with hex_col2:
    st.markdown(f"<small><b>接收端 [{SYSTEM_ID_MAP.get(receiver_sys, '未知')}] 收到</b></small>", unsafe_allow_html=True)
    if st.session_state.recv_log:
        last_recv = list(st.session_state.recv_log)[-1]
        st.code(last_recv['hex'], language='hex')
    else:
        st.code("等待数据...", language='text')

# ==================== 通信循环 ====================
if st.session_state.is_running:
    seq = st.session_state.send_count + 1
    current_time = get_local_time()
    timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
    
    # 构建 HEX 数据
    hex_data = f"FD 09 00 00 {seq % 256:02X} {sender_sys:02X} {sender_comp:02X} 00 00 00 {sender_sys:02X} 00 00 00 00 51 04 03 {mav_type:02X} 0C"
    
    # 发送日志
    send_entry = {
        'time': timestamp,
        'seq': seq,
        'sender': sender_sys,
        'sender_name': SYSTEM_ID_MAP.get(sender_sys, '未知'),
        'receiver': receiver_sys,
        'receiver_name': SYSTEM_ID_MAP.get(receiver_sys, '未知'),
        'hex': hex_data
    }
    st.session_state.send_log.append(send_entry)
    st.session_state.send_count += 1
    
    # 模拟网络延迟
    time.sleep(0.1)
    
    # 接收日志
    recv_entry = {
        'time': timestamp,
        'seq': seq,
        'sender': sender_sys,
        'sender_name': SYSTEM_ID_MAP.get(sender_sys, '未知'),
        'receiver': receiver_sys,
        'receiver_name': SYSTEM_ID_MAP.get(receiver_sys, '未知'),
        'type_name': MAV_TYPE.get(mav_type, "UNKNOWN"),
        'status_name': MAV_STATE.get(4, "ACTIVE"),
        'hex': hex_data
    }
    st.session_state.recv_log.append(recv_entry)
    st.session_state.recv_count += 1
    
    time.sleep(max(0, 1.0 - 0.1))  # 使用固定间隔简化
    st.rerun()

st.markdown("---")
st.caption(f"MAVLink 2.0 Protocol Simulator | 当前场景: {scenario['name']} | 北京时间 (UTC+8)")
