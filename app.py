"""
MAVLink 高级分析器 - 新增功能：
- 实时图表可视化
- 真实 UDP 通信 + 模拟模式
- 日志导出 (CSV/JSON)
- 多飞行器支持
"""

import streamlit as st
import socket
import threading
import time
import queue
from datetime import datetime
from collections import deque, defaultdict

# 导入自定义模块
from mavlink_utils import MAVLinkParser, MAV_TYPE, MAV_AUTOPILOT, MAV_STATE
from utils import ChartManager, LogExporter

# 页面配置
st.set_page_config(
    page_title="MAVLink 高级分析器",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# 自定义样式
st.markdown("""
<style>
    .stProgress > div > div > div > div {
        background-image: linear-gradient(to right, #FF4B4B, #FF8C42);
    }
    .metric-container {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        padding: 1rem;
        border-radius: 0.5rem;
        color: white;
    }
</style>
""", unsafe_allow_html=True)

# ==================== 会话状态 ====================

def init_session():
    defaults = {
        'messages': deque(maxlen=200),
        'is_running': False,
        'send_count': 0,
        'recv_count': 0,
        'start_time': None,
        'parser': MAVLinkParser(),
        'chart_manager': ChartManager(),
        'systems': defaultdict(lambda: {'count': 0, 'last_seen': None}),
        'udp_socket': None,
        'receive_thread': None,
        'message_queue': queue.Queue(),
        'connection_mode': 'simulation'  # 'simulation' 或 'real'
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session()

# ==================== UDP 通信线程 ====================

def udp_receiver_thread():
    """UDP 接收线程（真实网络模式）"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)  # 1秒超时，便于检查停止标志
        sock.bind(('0.0.0.0', st.session_state.udp_port))
        st.session_state.udp_socket = sock
        
        while st.session_state.is_running and st.session_state.connection_mode == 'real':
            try:
                data, addr = sock.recvfrom(1024)
                st.session_state.message_queue.put(('recv', data, addr))
            except socket.timeout:
                continue
            except Exception as e:
                if st.session_state.is_running:
                    st.session_state.message_queue.put(('error', str(e), None))
                break
    except Exception as e:
        st.session_state.message_queue.put(('error', f"Socket error: {e}", None))
    finally:
        if sock:
            sock.close()

# ==================== 页面布局 ====================

st.title("🚁 MAVLink 高级分析器")
st.caption("支持真实 UDP 通信 | 实时图表 | 日志导出 | 多飞行器追踪")

# 侧边栏
with st.sidebar:
    st.header("⚙️ 配置")
    
    # 连接模式选择
    st.subheader("🔌 连接模式")
    mode = st.radio(
        "选择模式",
        options=['simulation', 'real'],
        format_func=lambda x: "🔄 自发自收模拟" if x == 'simulation' else "📡 真实 UDP 通信",
        index=0 if st.session_state.connection_mode == 'simulation' else 1
    )
    st.session_state.connection_mode = mode
    
    # 网络配置
    with st.expander("网络设置", expanded=True):
        udp_ip = st.text_input("目标 IP", value="127.0.0.1")
        udp_port = st.number_input("端口", value=14550, min_value=1024, max_value=65535)
        st.session_state.udp_port = udp_port
        
        if mode == 'real':
            st.info("💡 真实模式：将监听 0.0.0.0:" + str(udp_port))
    
    # 心跳包配置
    with st.expander("心跳包设置"):
        system_id = st.number_input("系统 ID", 1, 255, 1)
        component_id = st.number_input("组件 ID", 1, 255, 1)
        mav_type = st.selectbox("飞行器类型", list(MAV_TYPE.keys()), 
                               format_func=lambda x: f"{x}: {MAV_TYPE[x]}", index=2)
        send_interval = st.slider("发送间隔(秒)", 0.1, 5.0, 1.0, 0.1)
    
    # 控制按钮
    st.markdown("---")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("▶️ 启动", disabled=st.session_state.is_running, 
                    type="primary", use_container_width=True):
            st.session_state.is_running = True
            st.session_state.start_time = datetime.now()
            
            # 真实模式下启动接收线程
            if mode == 'real':
                thread = threading.Thread(target=udp_receiver_thread, daemon=True)
                st.session_state.receive_thread = thread
                thread.start()
            
            st.rerun()
    
    with col2:
        if st.button("⏹️ 停止", disabled=not st.session_state.is_running,
                    type="secondary", use_container_width=True):
            st.session_state.is_running = False
            if st.session_state.udp_socket:
                st.session_state.udp_socket.close()
            st.rerun()
    
    # 状态显示
    status = "🟢 运行中" if st.session_state.is_running else "🔴 已停止"
    st.markdown(f"**状态:** {status}")

# ==================== 主界面：多标签页 ====================

tab1, tab2, tab3, tab4 = st.tabs(["📊 实时监控", "🛸 飞行器追踪", "📁 日志导出", "📈 统计分析"])

# ---------- 标签页1: 实时监控 ----------
with tab1:
    st.header("实时通信监控")
    
    # 关键指标
    cols = st.columns(4)
    metrics = [
        ("📤 发送", st.session_state.send_count),
        ("📥 接收", st.session_state.recv_count),
        ("🛸 飞行器", len(st.session_state.systems)),
        ("⏱️ 运行时长", 
         f"{(datetime.now() - st.session_state.start_time).seconds}s" 
         if st.session_state.start_time else "0s")
    ]
    for col, (label, value) in zip(cols, metrics):
        with col:
            st.metric(label, value)
    
    # 图表
    st.subheader("通信图表")
    st.session_state.chart_manager.render_charts()
    
    # 最新日志
    st.subheader("最近消息")
    log_container = st.container()
    with log_container:
        recent_msgs = list(st.session_state.messages)[-10:]
        for msg in reversed(recent_msgs):
            time_str = msg.get('timestamp', datetime.now()).strftime("%H:%M:%S.%f")[:-3]
            st.text(f"[{time_str}] {msg.get('msg_type', 'UNKNOWN')} - "
                   f"系统{msg.get('system_id', '?')} - {msg.get('mav_type_name', 'UNKNOWN')}")

# ---------- 标签页2: 飞行器追踪 ----------
with tab2:
    st.header("多飞行器追踪")
    
    if st.session_state.systems:
        for sys_id, info in st.session_state.systems.items():
            with st.expander(f"🛸 系统 ID: {sys_id}", expanded=True):
                cols = st.columns(3)
                cols[0].metric("消息数", info['count'])
                cols[1].metric("最后活跃", 
                              info['last_seen'].strftime("%H:%M:%S") if info['last_seen'] else "N/A")
                
                # 查找该系统的最新消息
                recent = [m for m in st.session_state.messages 
                         if m.get('system_id') == sys_id][-1:]
                if recent:
                    cols[2].metric("状态", recent[0].get('system_status_name', 'UNKNOWN'))
    else:
        st.info("暂无飞行器数据")

# ---------- 标签页3: 日志导出 ----------
with tab3:
    st.header("日志导出")
    
    if st.session_state.messages:
        exporter = LogExporter()
        
        col1, col2 = st.columns(2)
        
        with col1:
            st.subheader("CSV 格式")
            csv_data = exporter.to_csv(st.session_state.messages)
            st.download_button(
                label="⬇️ 下载 CSV",
                data=csv_data,
                file_name=f"mavlink_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                mime="text/csv"
            )
        
        with col2:
            st.subheader("JSON 格式")
            json_data = exporter.to_json(st.session_state.messages)
            st.download_button(
                label="⬇️ 下载 JSON",
                data=json_data,
                file_name=f"mavlink_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                mime="application/json"
            )
        
        # 摘要
        st.subheader("会话摘要")
        summary = exporter.get_summary(st.session_state.messages)
        st.json(summary)
    else:
        st.warning("暂无数据可导出")

# ---------- 标签页4: 统计分析 ----------
with tab4:
    st.header("通信统计")
    st.info("功能开发中...")

# ==================== 主循环 ====================

if st.session_state.is_running:
    parser = st.session_state.parser
    
    # 处理消息队列（真实模式）
    if st.session_state.connection_mode == 'real':
        try:
            while True:
                msg_type, data, addr = st.session_state.message_queue.get_nowait()
                if msg_type == 'recv':
                    parsed = parser.parse_any_message(data)
                    if parsed:
                        parsed['source_addr'] = str(addr)
                        st.session_state.messages.append(parsed)
                        st.session_state.recv_count += 1
                        
                        # 更新系统统计
                        sys_id = parsed.get('system_id', 0)
                        st.session_state.systems[sys_id]['count'] += 1
                        st.session_state.systems[sys_id]['last_seen'] = datetime.now()
                elif msg_type == 'error':
                    st.error(f"接收错误: {data}")
        except queue.Empty:
            pass
    
    # 模拟模式：发送并自收
    if st.session_state.connection_mode == 'simulation':
        heartbeat = parser.create_heartbeat_v2(
            system_id=system_id,
            component_id=component_id,
            mav_type=mav_type
        )
        
        st.session_state.send_count += 1
        
        # 模拟接收
        parsed = parser.parse_heartbeat_v2(heartbeat)
        if parsed:
            st.session_state.messages.append(parsed)
            st.session_state.recv_count += 1
            
            sys_id = parsed.get('system_id', 0)
            st.session_state.systems[sys_id]['count'] += 1
            st.session_state.systems[sys_id]['last_seen'] = datetime.now()
    
    # 更新图表数据
    elapsed = (datetime.now() - st.session_state.start_time).total_seconds() if st.session_state.start_time else 1
    freq = st.session_state.recv_count / elapsed if elapsed > 0 else 0
    
    st.session_state.chart_manager.add_data_point(
        datetime.now(),
        st.session_state.send_count,
        st.session_state.recv_count,
        freq
    )
    
    # 自动刷新
    time.sleep(send_interval)
    st.rerun()
