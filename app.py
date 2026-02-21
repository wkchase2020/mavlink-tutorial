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

# ==================== MAVLink 常量定义 ====================
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

MAV_AUTOPILOT = {
    0: "GENERIC", 1: "RESERVED", 2: "SLUGS", 3: "ARDUPILOTMEGA",
    4: "OPENPILOT", 5: "GENERIC_WAYPOINTS_ONLY",
    6: "GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
    7: "GENERIC_MISSION_FULL", 8: "INVALID", 9: "PPZ",
    10: "UDB", 11: "FP", 12: "PX4", 13: "SMACCMPILOT",
    14: "AUTOQUAD", 15: "ARMAZILA", 16: "AEROB",
    17: "ASLUAV", 18: "SMARTAP", 19: "AIRRAILS"
}

MAV_STATE = {
    0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
    4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY",
    7: "POWEROFF", 8: "FLIGHT_TERMINATION"
}

# ==================== 会话状态初始化 ====================
if 'messages' not in st.session_state:
    st.session_state.messages = deque(maxlen=50)
if 'is_running' not in st.session_state:
    st.session_state.is_running = False
if 'send_count' not in st.session_state:
    st.session_state.send_count = 0
if 'recv_count' not in st.session_state:
    st.session_state.recv_count = 0

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 心跳包实时演示")
st.caption("基于 Streamlit 的交互式 MAVLink 通信模拟器")

# 侧边栏控制
with st.sidebar:
    st.header("⚙️ 控制面板")
    
    # 连接设置
    st.subheader("连接配置")
    system_id = st.number_input("系统 ID", 1, 255, 1)
    component_id = st.number_input("组件 ID", 1, 255, 1)
    mav_type = st.selectbox(
        "飞行器类型",
        options=list(MAV_TYPE.keys()),
        format_func=lambda x: f"{x}: {MAV_TYPE[x]}",
        index=2
    )
    send_interval = st.slider("发送间隔(秒)", 0.1, 3.0, 1.0, 0.1)
    
    st.markdown("---")
    
    # 控制按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("▶️ 启动", type="primary", 
                    disabled=st.session_state.is_running,
                    use_container_width=True):
            st.session_state.is_running = True
            st.rerun()
    
    with col2:
        if st.button("⏹️ 停止", type="secondary",
                    disabled=not st.session_state.is_running,
                    use_container_width=True):
            st.session_state.is_running = False
            st.rerun()
    
    # 状态显示
    status = "🟢 运行中" if st.session_state.is_running else "🔴 已停止"
    st.markdown(f"**状态:** {status}")

# ==================== 主界面 ====================
col1, col2, col3 = st.columns(3)
with col1:
    st.metric("📤 已发送", st.session_state.send_count)
with col2:
    st.metric("📥 已接收", st.session_state.recv_count)
with col3:
    loss = 0
    if st.session_state.send_count > 0:
        loss = (st.session_state.send_count - st.session_state.recv_count) / st.session_state.send_count * 100
    st.metric("📉 丢包率", f"{max(0, loss):.1f}%")

# 实时日志
st.subheader("📋 通信日志")
log_container = st.container()

# ==================== 模拟通信循环 ====================
if st.session_state.is_running:
    # 模拟发送心跳
    st.session_state.send_count += 1
    
    # 构建模拟心跳消息
    msg = {
        'timestamp': datetime.now(),
        'system_id': system_id,
        'component_id': component_id,
        'mav_type': mav_type,
        'mav_type_name': MAV_TYPE.get(mav_type, "UNKNOWN"),
        'autopilot': 12,
        'autopilot_name': MAV_AUTOPILOT.get(12, "UNKNOWN"),
        'system_status': 4,
        'system_status_name': MAV_STATE.get(4, "UNKNOWN"),
        'seq': st.session_state.send_count
    }
    
    # 模拟接收（自发自收）
    st.session_state.messages.append(msg)
    st.session_state.recv_count += 1
    
    # 自动刷新
    time.sleep(send_interval)
    st.rerun()

# 显示日志
with log_container:
    if st.session_state.messages:
        for msg in reversed(list(st.session_state.messages)[-10:]):
            time_str = msg['timestamp'].strftime("%H:%M:%S.%f")[:-3]
            st.text(f"[{time_str}] 系统{msg['system_id']} | {msg['mav_type_name']} | {msg['system_status_name']}")
    else:
        st.info("💡 点击'启动'开始模拟 MAVLink 心跳通信...")

# 页脚
st.markdown("---")
st.caption("MAVLink Simulator | Built with Streamlit")
