import streamlit as st
import time
from datetime import datetime
from collections import deque

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 心跳包演示",
    page_icon="🚁",
    layout="wide"
)

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
st.caption("模拟 MAVLink 心跳包发送与接收过程")

# 侧边栏控制
with st.sidebar:
    st.header("⚙️ 控制面板")
    
    system_id = st.number_input("系统 ID", 1, 255, 1)
    component_id = st.number_input("组件 ID", 1, 255, 1)
    mav_type = st.selectbox("飞行器类型", list(MAV_TYPE.keys()), 
                           format_func=lambda x: f"{x}: {MAV_TYPE[x]}", index=2)
    interval = st.slider("发送间隔(秒)", 0.5, 3.0, 1.0, 0.1)
    
    st.markdown("---")
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("▶️ 启动", disabled=st.session_state.is_running, type="primary", use_container_width=True):
            st.session_state.is_running = True
            st.rerun()
    with col2:
        if st.button("⏹️ 停止", disabled=not st.session_state.is_running, type="secondary", use_container_width=True):
            st.session_state.is_running = False
            st.rerun()
    
    status = "🟢 运行中" if st.session_state.is_running else "🔴 已停止"
    st.markdown(f"**状态:** {status}")

# ==================== 统计区域 ====================
st.subheader("📊 实时统计")
col1, col2, col3 = st.columns(3)
col1.metric("📤 已发送", st.session_state.send_count)
col2.metric("📥 已接收", st.session_state.recv_count)
col3.metric("⏱️ 当前间隔", f"{interval}s")

# ==================== 发送/接收详情区域 ====================
st.markdown("---")

col_send, col_recv = st.columns(2)

# 左侧：发送端信息
with col_send:
    st.subheader("📤 发送端详情")
    
    # 当前配置
    with st.container():
        st.markdown("**当前配置:**")
        st.json({
            "系统 ID": system_id,
            "组件 ID": component_id,
            "飞行器类型": f"{mav_type} ({MAV_TYPE[mav_type]})",
            "自动驾驶仪": "PX4 (12)",
            "基础模式": 81,
            "系统状态": "ACTIVE (4)",
            "MAVLink 版本": 3
        })
    
    # 发送日志
    st.markdown("**发送记录:**")
    send_container = st.container()
    with send_container:
        if st.session_state.send_log:
            for log in reversed(list(st.session_state.send_log)[-8:]):
                st.markdown(f"""
                <div style="background:#1E1E1E;padding:8px;margin:4px 0;border-radius:4px;font-family:monospace;font-size:12px;">
                <span style="color:#888;">[{log['time']}]</span> 
                <span style="color:#4CAF50;">➜ SEND</span> 
                SEQ:{log['seq']} | SYS:{log['sys']} | COMP:{log['comp']}
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待发送...")

# 右侧：接收端信息
with col_recv:
    st.subheader("📥 接收端详情")
    
    # 接收统计
    with st.container():
        st.markdown("**接收统计:**")
        if st.session_state.recv_count > 0:
            latest = list(st.session_state.recv_log)[-1] if st.session_state.recv_log else None
            if latest:
                st.json({
                    "最后接收时间": latest['time'],
                    "来源系统": latest['sys'],
                    "飞行器类型": latest['type_name'],
                    "系统状态": latest['status_name'],
                    "消息序列号": latest['seq']
                })
        else:
            st.json({"状态": "等待接收..."})
    
    # 接收日志
    st.markdown("**接收记录:**")
    recv_container = st.container()
    with recv_container:
        if st.session_state.recv_log:
            for log in reversed(list(st.session_state.recv_log)[-8:]):
                st.markdown(f"""
                <div style="background:#1E1E1E;padding:8px;margin:4px 0;border-radius:4px;font-family:monospace;font-size:12px;">
                <span style="color:#888;">[{log['time']}]</span> 
                <span style="color:#2196F3;">⬅ RECV</span> 
                SEQ:{log['seq']} | SYS:{log['sys']} | {log['type_name']}
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待接收...")

# ==================== 原始数据展示 ====================
st.markdown("---")
st.subheader("📦 最新数据包 (HEX)")

hex_col1, hex_col2 = st.columns(2)
with hex_col1:
    if st.session_state.send_log:
        last_send = list(st.session_state.send_log)[-1]
        st.text_area("发送数据包", last_send['hex'], height=100, disabled=True)
    else:
        st.text_area("发送数据包", "无数据", height=100, disabled=True)

with hex_col2:
    if st.session_state.recv_log:
        last_recv = list(st.session_state.recv_log)[-1]
        st.text_area("接收数据包", last_recv['hex'], height=100, disabled=True)
    else:
        st.text_area("接收数据包", "无数据", height=100, disabled=True)

# ==================== 通信循环 ====================
if st.session_state.is_running:
    # 生成模拟数据
    seq = st.session_state.send_count + 1
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    # 构建模拟 HEX 数据
    hex_data = f"FD 09 00 00 {seq % 256:02X} {system_id:02X} {component_id:02X} 00 00 00 {system_id:02X} 00 00 00 00 51 04 03 {mav_type:02X} 0C"
    
    # 发送日志
    send_entry = {
        'time': timestamp,
        'seq': seq,
        'sys': system_id,
        'comp': component_id,
        'hex': hex_data
    }
    st.session_state.send_log.append(send_entry)
    st.session_state.send_count += 1
    
    # 模拟接收（延迟100ms）
    time.sleep(0.1)
    
    # 接收日志
    recv_entry = {
        'time': datetime.now().strftime("%H:%M:%S.%f")[:-3],
        'seq': seq,
        'sys': system_id,
        'type_name': MAV_TYPE.get(mav_type, "UNKNOWN"),
        'status_name': MAV_STATE.get(4, "UNKNOWN"),
        'hex': hex_data
    }
    st.session_state.recv_log.append(recv_entry)
    st.session_state.recv_count += 1
    
    # 继续循环
    time.sleep(max(0, interval - 0.1))
    st.rerun()

st.markdown("---")
st.caption("MAVLink Simulator | 发送端 ➜ 网络 ➜ 接收端")
