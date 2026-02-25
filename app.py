import streamlit as st
import time
import math
import random
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, MarkerCluster
from streamlit_folium import st_folium

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 航线规划与避障系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
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

MAV_CMD = {
    16: "NAV_WAYPOINT", 22: "NAV_TAKEOFF", 21: "NAV_LAND",
    20: "NAV_RETURN_TO_LAUNCH", 81: "NAV_PATHPLANNING"
}

# 系统ID定义
SYSTEM_ID_MAP = {
    0: "广播地址", 1: "自动驾驶仪/主飞行器", 2: "地面控制站 (GCS)",
    3: "任务规划系统", 4: "相机/成像系统", 5: "云台/稳定系统",
    6: "遥测无线电", 7: "Companion Computer", 8: "路径规划系统",
    9: "遥控接收机", 10: "电池管理系统", 11: "伺服/执行器系统",
    12: "避障系统", 13: "降落伞系统", 14: "日志系统",
    15: "ADSB 接收机", 16: "光学流量传感器", 17: "视觉系统",
    18: "红外系统", 19: "声纳/雷达", 20: "GPS 模块",
    21: "气压计", 22: "磁力计", 23: "IMU 传感器",
    24: "激光雷达", 25: "VIO 系统"
}

# 组件ID定义
COMPONENT_ID_MAP = {
    0: "广播（所有组件）", 1: "自动驾驶仪（主控）", 2: "任务计算机",
    3: "遥控输入", 4: "遥测输出", 5: "相机 #1", 6: "相机 #2",
    7: "相机 #3", 8: "云台 #1", 9: "云台 #2", 10: "伺服 #1",
    18: "GPS #1", 27: "激光雷达 #1", 30: "视觉系统 #1",
    42: "避障系统 #1", 44: "路径规划", 45: "任务规划"
}

# ==================== 预定义通信场景 ====================
COMMUNICATION_SCENARIOS = {
    "drone_to_gcs": {
        "name": "无人机 → 地面站",
        "description": "无人机飞控发送心跳包给地面控制站，用于状态监控和连接保持",
        "sender_sys": 1, "sender_comp": 1, "sender_name": "无人机飞控",
        "receiver_sys": 2, "receiver_comp": 4, "receiver_name": "地面控制站 (GCS)",
        "mav_type": 2, "icon": "🚁→🖥️"
    },
    "gcs_to_drone": {
        "name": "地面站 → 无人机",
        "description": "地面站发送任务指令或控制命令给无人机",
        "sender_sys": 2, "sender_comp": 3, "sender_name": "地面控制站",
        "receiver_sys": 1, "receiver_comp": 1, "receiver_name": "无人机飞控",
        "mav_type": 6, "icon": "🖥️→🚁"
    },
    "custom": {
        "name": "自定义配置",
        "description": "手动配置发送端和接收端身份，灵活模拟各种场景",
        "sender_sys": 1, "sender_comp": 1, "sender_name": "自定义",
        "receiver_sys": 2, "receiver_comp": 4, "receiver_name": "自定义",
        "mav_type": 2, "icon": "⚙️"
    }
}

# ==================== 航线规划相关类 ====================
class Waypoint:
    """航点类"""
    def __init__(self, lat, lon, alt=50, cmd=16, seq=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.cmd = cmd
        self.seq = seq
    
    def to_dict(self):
        return {
            "lat": self.lat, "lon": self.lon, "alt": self.alt,
            "cmd": self.cmd, "cmd_name": MAV_CMD.get(self.cmd, "UNKNOWN")
        }

class Obstacle:
    """障碍物类"""
    def __init__(self, lat, lon, radius, height):
        self.lat = lat
        self.lon = lon
        self.radius = radius
        self.height = height

class PathPlanner:
    """路径规划器 - 支持避障路径规划"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 10
    
    def add_obstacle(self, lat, lon, radius, height):
        self.obstacles.append(Obstacle(lat, lon, radius, height))
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间距离（米）"""
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def check_collision(self, lat, lon, alt):
        """检查点是否与障碍物碰撞"""
        for obs in self.obstacles:
            dist = self.haversine_distance(lat, lon, obs.lat, obs.lon)
            if dist < (obs.radius + self.safety_margin) and alt < obs.height:
                return True
        return False
    
    def plan_path(self, start_wp, end_wp, step_size=50):
        """使用改进的 A* 算法规划避障路径"""
        path = [start_wp]
        current = start_wp
        max_iterations = 50
        
        for _ in range(max_iterations):
            dist_to_end = self.haversine_distance(current.lat, current.lon, end_wp.lat, end_wp.lon)
            
            if dist_to_end < step_size:
                path.append(end_wp)
                break
            
            dlat = end_wp.lat - current.lat
            dlon = end_wp.lon - current.lon
            ratio = step_size / dist_to_end
            
            next_lat = current.lat + dlat * ratio
            next_lon = current.lon + dlon * ratio
            next_alt = current.alt + (end_wp.alt - current.alt) * ratio
            
            if not self.check_collision(next_lat, next_lon, next_alt):
                next_wp = Waypoint(next_lat, next_lon, next_alt)
                path.append(next_wp)
                current = next_wp
            else:
                # 绕行策略
                found = False
                for angle in [45, -45, 90, -90]:
                    rad = math.radians(angle)
                    new_dlat = dlat * math.cos(rad) - dlon * math.sin(rad)
                    new_dlon = dlat * math.sin(rad) + dlon * math.cos(rad)
                    test_lat = current.lat + new_dlat * ratio
                    test_lon = current.lon + new_dlon * ratio
                    test_alt = current.alt + 20
                    
                    if not self.check_collision(test_lat, test_lon, test_alt):
                        next_wp = Waypoint(test_lat, test_lon, test_alt)
                        path.append(next_wp)
                        current = next_wp
                        found = True
                        break
                
                if not found:
                    path.append(end_wp)
                    break
        
        # 重新编号
        for i, wp in enumerate(path):
            wp.seq = i
        
        return path

# ==================== 会话状态初始化 ====================
def init_session_state():
    defaults = {
        'send_log': deque(maxlen=20), 'recv_log': deque(maxlen=20),
        'is_running': False, 'send_count': 0, 'recv_count': 0,
        'selected_scenario': "drone_to_gcs",
        'waypoints': [], 'obstacles': [], 'planned_path': [],
        'drone_position': None, 'mission_sent': False, 'mission_executing': False,
        'map_center': [39.9042, 116.4074], 'map_zoom': 13,
        'path_planner': PathPlanner(), 'last_map_click': None,
        'point_a': None, 'point_b': None, 'avoidance_enabled': True,
        'flight_altitude': 50, 'obstacle_radius': 30, 'obstacle_height': 100,
        'current_waypoint_index': 0
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 地面站 - 航线规划与避障系统")
st.caption("模拟 MAVLink 通信协议 | 支持航线规划、避障路径计算 | 北京时间 (UTC+8)")

# ==================== 侧边栏导航 ====================
with st.sidebar:
    st.header("📋 功能导航")
    page = st.radio("选择功能模块", ["💓 心跳包监控", "🗺️ 航线规划", "🛰️ 任务监控"])
    
    st.markdown("---")
    st.header("📡 系统状态")
    
    if st.session_state.is_running:
        st.success("🟢 通信正常")
    else:
        st.warning("🟡 通信待机")
    
    st.metric("发送包数", st.session_state.send_count)
    st.metric("接收包数", st.session_state.recv_count)
    
    if st.session_state.mission_sent:
        st.info(f"📍 航线点数: {len(st.session_state.waypoints)}")

# ==================== 心跳包监控页面 ====================
if page == "💓 心跳包监控":
    st.header("💓 MAVLink 心跳包实时演示")
    
    st.subheader("📋 选择通信场景")
    
    scenario_options = list(COMMUNICATION_SCENARIOS.keys())
    scenario_labels = [f"{COMMUNICATION_SCENARIOS[k]['icon']} {COMMUNICATION_SCENARIOS[k]['name']}" for k in scenario_options]
    
    selected_index = scenario_options.index(st.session_state.selected_scenario)
    selected_label = st.selectbox("选择通信场景", options=scenario_labels, index=selected_index, label_visibility="collapsed")
    
    selected_scenario = scenario_options[scenario_labels.index(selected_label)]
    st.session_state.selected_scenario = selected_scenario
    scenario = COMMUNICATION_SCENARIOS[selected_scenario]
    
    st.info(f"""
    **当前场景:** {scenario['icon']} {scenario['name']}
    
    {scenario['description']}
    
    **发送端:** {scenario['sender_name']} (系统ID: {scenario['sender_sys']}, 组件ID: {scenario['sender_comp']})  
    **接收端:** {scenario['receiver_name']} (系统ID: {scenario['receiver_sys']}, 组件ID: {scenario['receiver_comp']})
    """)
    
    if selected_scenario == "custom":
        st.subheader("⚙️ 自定义配置")
        col1, col2 = st.columns(2)
        
        with col1:
            st.markdown("**📤 发送端配置**")
            custom_sender_sys = st.selectbox("发送端系统 ID", options=list(SYSTEM_ID_MAP.keys()),
                format_func=lambda x: f"{x}: {SYSTEM_ID_MAP[x]}", index=1, key="custom_sender_sys")
            custom_sender_comp = st.selectbox("发送端组件 ID", options=list(COMPONENT_ID_MAP.keys()),
                format_func=lambda x: f"{x}: {COMPONENT_ID_MAP[x]}", index=1, key="custom_sender_comp")
        
        with col2:
            st.markdown("**📥 接收端配置**")
            custom_receiver_sys = st.selectbox("接收端系统 ID", options=list(SYSTEM_ID_MAP.keys()),
                format_func=lambda x: f"{x}: {SYSTEM_ID_MAP[x]}", index=2, key="custom_receiver_sys")
            custom_receiver_comp = st.selectbox("接收端组件 ID", options=list(COMPONENT_ID_MAP.keys()),
                format_func=lambda x: f"{x}: {COMPONENT_ID_MAP[x]}", index=4, key="custom_receiver_comp")
        
        sender_sys, sender_comp = custom_sender_sys, custom_sender_comp
        receiver_sys, receiver_comp = custom_receiver_sys, custom_receiver_comp
        mav_type = st.selectbox("飞行器类型", list(MAV_TYPE.keys()), 
                               format_func=lambda x: f"{x}: {MAV_TYPE[x]}", index=2)
    else:
        sender_sys, sender_comp = scenario['sender_sys'], scenario['sender_comp']
        receiver_sys, receiver_comp = scenario['receiver_sys'], scenario['receiver_comp']
        mav_type = scenario['mav_type']
    
    st.markdown("---")
    col_ctrl1, col_ctrl2, col_ctrl3 = st.columns([2, 1, 2])
    
    with col_ctrl1:
        st.markdown(f"""
        <div style="background: linear-gradient(135deg, #FF6B6B 0%, #EE5A6F 100%); padding: 20px; border-radius: 15px; text-align: center;">
            <h3 style="color: white; margin: 0;">📤 发送端</h3>
            <p style="color: white; font-size: 16px; font-weight: bold;">{SYSTEM_ID_MAP.get(sender_sys, '未知')}</p>
            <p style="color: #FFE66D; font-size: 12px;">SYS:{sender_sys} | COMP:{sender_comp}</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col_ctrl2:
        arrow_color = "#00FF00" if st.session_state.is_running else "#888888"
        st.markdown(f"""
        <div style="text-align: center; padding-top: 30px;">
            <div style="color: {arrow_color}; font-size: 32px;">➤➤➤</div>
            <div style="color: #888; font-size: 12px;">MAVLink 2.0</div>
        </div>
        """, unsafe_allow_html=True)
        
        btn_col1, btn_col2 = st.columns(2)
        with btn_col1:
            if st.button("▶️ 启动", disabled=st.session_state.is_running, type="primary", use_container_width=True):
                st.session_state.is_running = True
                st.rerun()
        with btn_col2:
            if st.button("⏹️ 停止", disabled=not st.session_state.is_running, use_container_width=True):
                st.session_state.is_running = False
                st.rerun()
    
    with col_ctrl3:
        st.markdown(f"""
        <div style="background: linear-gradient(135deg, #4ECDC4 0%, #44A08D 100%); padding: 20px; border-radius: 15px; text-align: center;">
            <h3 style="color: white; margin: 0;">📥 接收端</h3>
            <p style="color: white; font-size: 16px; font-weight: bold;">{SYSTEM_ID_MAP.get(receiver_sys, '未知')}</p>
            <p style="color: #FFE66D; font-size: 12px;">SYS:{receiver_sys} | COMP:{receiver_comp}</p>
        </div>
        """, unsafe_allow_html=True)
    
    st.markdown("---")
    st.subheader("📊 实时统计")
    interval = st.slider("发送间隔", 0.5, 3.0, 1.0, 0.1)
    
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("📤 已发送", st.session_state.send_count)
    col2.metric("📥 已接收", st.session_state.recv_count)
    col3.metric("⏱️ 当前间隔", f"{interval}s")
    col4.metric("🚁 飞行器", MAV_TYPE.get(mav_type, "UNKNOWN"))
    
    st.markdown("---")
    col_send_log, col_recv_log = st.columns(2)
    
    with col_send_log:
        st.subheader("📤 发送日志")
        if st.session_state.send_log:
            for log in reversed(list(st.session_state.send_log)[-8:]):
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:8px;margin:4px 0;border-radius:5px;font-family:monospace;font-size:11px;border-left:4px solid #FF6B6B;">
                    <span style="color:#AAAAAA;">[{log['time']}]</span>
                    <span style="color:#FF6B6B;font-weight:bold;">📤 SEND</span>
                    <span style="color:#FFFFFF;">SEQ:{log['seq']}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待发送数据...")
    
    with col_recv_log:
        st.subheader("📥 接收日志")
        if st.session_state.recv_log:
            for log in reversed(list(st.session_state.recv_log)[-8:]):
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:8px;margin:4px 0;border-radius:5px;font-family:monospace;font-size:11px;border-left:4px solid #4ECDC4;">
                    <span style="color:#AAAAAA;">[{log['time']}]</span>
                    <span style="color:#4ECDC4;font-weight:bold;">📥 RECV</span>
                    <span style="color:#FFFFFF;">SEQ:{log['seq']}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待接收数据...")
    
    # 通信循环
    if st.session_state.is_running:
        seq = st.session_state.send_count + 1
        current_time = get_local_time()
        timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
        
        hex_data = f"FD 09 00 00 {seq % 256:02X} {sender_sys:02X} {sender_comp:02X} 00 00 00 {mav_type:02X} 0C"
        
        send_entry = {
            'time': timestamp, 'seq': seq, 'sender': sender_sys,
            'sender_name': SYSTEM_ID_MAP.get(sender_sys, '未知'),
            'receiver': receiver_sys, 'receiver_name': SYSTEM_ID_MAP.get(receiver_sys, '未知'),
            'hex': hex_data
        }
        st.session_state.send_log.append(send_entry)
        st.session_state.send_count += 1
        
        time.sleep(0.1)
        
        recv_entry = {
            'time': timestamp, 'seq': seq, 'sender': sender_sys,
            'sender_name': SYSTEM_ID_MAP.get(sender_sys, '未知'),
            'type_name': MAV_TYPE.get(mav_type, "UNKNOWN"),
            'status_name': MAV_STATE.get(4, "ACTIVE"), 'hex': hex_data
        }
        st.session_state.recv_log.append(recv_entry)
        st.session_state.recv_count += 1
        
        time.sleep(max(0, interval - 0.1))
        st.rerun()

# ==================== 航线规划页面 ====================
elif page == "🗺️ 航线规划":
    st.header("🗺️ 航线规划与避障系统")
    
    col_left, col_right = st.columns([3, 2])
    
    with col_left:
        st.subheader("🗺️ 地图操作")
        
        map_type = st.selectbox("选择地图图层", ["OpenStreetMap", "CartoDB positron", "CartoDB dark_matter"], index=0)
        
        m = folium.Map(location=st.session_state.map_center, zoom_start=st.session_state.map_zoom, tiles=map_type)
        
        # 显示航点
        for i, wp in enumerate(st.session_state.waypoints):
            color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints) - 1 else 'blue'
            folium.Marker([wp.lat, wp.lon], popup=f"航点 {i+1}<br>高度: {wp.alt}m",
                         icon=folium.Icon(color=color, icon='play' if i == 0 else 'stop' if i == len(st.session_state.waypoints)-1 else 'dot', prefix='glyphicon')).add_to(m)
        
        # 显示障碍物
        for obs in st.session_state.obstacles:
            folium.Circle([obs.lat, obs.lon], radius=obs.radius, popup=f"障碍物<br>半径:{obs.radius}m<br>高度:{obs.height}m",
                         color='red', fill=True, fillColor='red', fillOpacity=0.3).add_to(m)
        
        # 显示规划路径
        if st.session_state.planned_path:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
            folium.PolyLine(path_coords, color='green', weight=4, opacity=0.8).add_to(m)
        
        # 显示无人机位置
        if st.session_state.drone_position:
            folium.Marker(st.session_state.drone_position, popup="无人机",
                         icon=folium.Icon(color='orange', icon='plane', prefix='fa')).add_to(m)
        
        map_data = st_folium(m, width=700, height=500, key="map")
        
        if map_data['last_clicked']:
            click_lat = map_data['last_clicked']['lat']
            click_lng = map_data['last_clicked']['lng']
            st.session_state.last_map_click = (click_lat, click_lng)
            st.info(f"📍 点击坐标: 纬度 {click_lat:.6f}, 经度 {click_lng:.6f}")
    
    with col_right:
        st.subheader("⚙️ 航线设置")
        
        st.markdown("**📍 设置起点 (A) 和终点 (B)**")
        
        col_a, col_b = st.columns(2)
        
        with col_a:
            st.markdown("🟢 **起点 A**")
            if st.session_state.point_a:
                st.success(f"已设置<br>{st.session_state.point_a[0]:.4f}, {st.session_state.point_a[1]:.4f}", unsafe_allow_html=True)
                if st.button("清除 A", key="clear_a"):
                    st.session_state.point_a = None
                    st.rerun()
            else:
                st.info("未设置")
                if st.button("设为 A", key="set_a"):
                    if st.session_state.last_map_click:
                        st.session_state.point_a = st.session_state.last_map_click
                        st.rerun()
                    else:
                        st.warning("请先在地图上点击")
            
            with st.expander("手动输入 A"):
                lat_a = st.number_input("纬度 A", value=39.9042, format="%.6f", key="lat_a")
                lon_a = st.number_input("经度 A", value=116.4074, format="%.6f", key="lon_a")
                if st.button("确认 A"):
                    st.session_state.point_a = (lat_a, lon_a)
                    st.rerun()
        
        with col_b:
            st.markdown("🔴 **终点 B**")
            if st.session_state.point_b:
                st.success(f"已设置<br>{st.session_state.point_b[0]:.4f}, {st.session_state.point_b[1]:.4f}", unsafe_allow_html=True)
                if st.button("清除 B", key="clear_b"):
n                    st.session_state.point_b = None
                    st.rerun()
            else:
                st.info("未设置")
                if st.button("设为 B", key="set_b"):
                    if st.session_state.last_map_click:
                        st.session_state.point_b = st.session_state.last_map_click
                        st.rerun()
                    else:
                        st.warning("请先在地图上点击")
            
            with st.expander("手动输入 B"):
                lat_b = st.number_input("纬度 B", value=39.9142, format="%.6f", key="lat_b")
                lon_b = st.number_input("经度 B", value=116.4174, format="%.6f", key="lon_b")
                if st.button("确认 B"):
                    st.session_state.point_b = (lat_b, lon_b)
                    st.rerun()
        
        st.markdown("---")
        st.markdown("**✈️ 飞行参数**")
        st.session_state.flight_altitude = st.slider("飞行高度 (m)", 10, 200, 50)
        st.session_state.avoidance_enabled = st.checkbox("启用避障", value=True)
        
        if st.session_state.avoidance_enabled:
            st.markdown("**🚧 障碍物设置**")
            obs_lat = st.number_input("障碍物纬度", value=st.session_state.map_center[0], format="%.6f")
            obs_lon = st.number_input("障碍物经度", value=st.session_state.map_center[1], format="%.6f")
            st.session_state.obstacle_radius = st.slider("障碍物半径 (m)", 10, 100, 30)
            st.session_state.obstacle_height = st.slider("障碍物高度 (m)", 20, 200, 100)
            
            col_obs1, col_obs2 = st.columns(2)
            with col_obs1:
                if st.button("➕ 添加障碍物"):
                    obs = Obstacle(obs_lat, obs_lon, st.session_state.obstacle_radius, st.session_state.obstacle_height)
                    st.session_state.obstacles.append(obs)
                    st.session_state.path_planner.add_obstacle(obs_lat, obs_lon, st.session_state.obstacle_radius, st.session_state.obstacle_height)
                    st.success("障碍物已添加")
                    st.rerun()
            with col_obs2:
                if st.button("🗑️ 清除障碍物"):
                    st.session_state.obstacles = []
                    st.session_state.path_planner.clear_obstacles()
                    st.rerun()
        
        st.markdown("---")
        
        if st.button("🧮 规划路径", type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], st.session_state.flight_altitude, cmd=22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], st.session_state.flight_altitude, cmd=16)
                
                if st.session_state.avoidance_enabled and st.session_state.obstacles:
                    path = st.session_state.path_planner.plan_path(start_wp, end_wp)
                    st.session_state.planned_path = path
                    st.success(f"✅ 避障路径规划完成！共 {len(path)} 个航点")
                else:
                    st.session_state.planned_path = [start_wp, end_wp]
                    st.success("✅ 直线路径规划完成！")
                
                st.session_state.waypoints = st.session_state.planned_path
                st.rerun()
            else:
                st.error("❌ 请先设置起点 A 和终点 B")
        
        if st.session_state.planned_path:
            if st.button("📡 发送航线到飞控", type="primary", use_container_width=True):
                st.session_state.mission_sent = True
                
                current_time = get_local_time()
                timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
                
                send_entry = {
                    'time': timestamp, 'seq': st.session_state.send_count + 1,
                    'sender': 2, 'sender_name': "地面控制站",
                    'receiver': 1, 'receiver_name': "无人机飞控",
                    'hex': f"MISSION_ITEM_COUNT: {len(st.session_state.planned_path)}"
                }
                st.session_state.send_log.append(send_entry)
                st.session_state.send_count += 1
                
                st.success(f"📡 航线已发送！共 {len(st.session_state.planned_path)} 个航点")
                st.balloons()
        
        if st.session_state.waypoints:
            st.markdown("---")
            st.subheader("📋 航点列表")
            for i, wp in enumerate(st.session_state.waypoints):
                with st.expander(f"航点 {i+1}: {MAV_CMD.get(wp.cmd, 'UNKNOWN')}"):
                    st.write(f"纬度: {wp.lat:.6f}")
                    st.write(f"经度: {wp.lon:.6f}")
                    st.write(f"高度: {wp.alt}m")

# ==================== 任务监控页面 ====================
elif page == "🛰️ 任务监控":
    st.header("🛰️ 任务执行监控")
    
    if not st.session_state.mission_sent:
        st.warning("⚠️ 尚未发送航线任务，请先前往 '🗺️ 航线规划' 页面规划并发送航线")
    else:
        st.success("✅ 航线任务已加载")
        
        col1, col2, col3 = st.columns(3)
        
        with col1:
            if st.button("▶️ 开始任务", type="primary", use_container_width=True):
                st.session_state.mission_executing = True
                st.session_state.current_waypoint_index = 0
                if st.session_state.waypoints:
                    st.session_state.drone_position = [st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]
                st.rerun()
        
        with col2:
            if st.button("⏸️ 暂停任务", use_container_width=True):
                st.session_state.mission_executing = False
                st.rerun()
        
        with col3:
            if st.button("⏹️ 终止任务", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.session_state.current_waypoint_index = 0
                st.rerun()
        
        if st.session_state.mission_executing:
            st.markdown("---")
            st.subheader("📊 实时飞行数据")
            
            # 计算进度
            total_wp = len(st.session_state.waypoints)
            current_idx = st.session_state.current_waypoint_index
            
            if total_wp > 0:
                progress = int((current_idx / (total_wp - 1)) * 100) if total_wp > 1 else 0
                st.progress(min(100, progress))
                st.write(f"当前航点: {current_idx + 1} / {total_wp}")
            
            # 模拟飞行
            if st.session_state.drone_position and current_idx < total_wp - 1:
                current_wp = st.session_state.waypoints[current_idx]
                next_wp = st.session_state.waypoints[current_idx + 1]
                
                # 向下一航点移动
                step = 0.0005
                curr_lat, curr_lon = st.session_state.drone_position
                
                if abs(curr_lat - next_wp.lat) > step or abs(curr_lon - next_wp.lon) > step:
                    new_lat = curr_lat + (next_wp.lat - curr_lat) * 0.1
                    new_lon = curr_lon + (next_wp.lon - curr_lon) * 0.1
                    st.session_state.drone_position = [new_lat, new_lon]
                else:
                    st.session_state.current_waypoint_index += 1
                    if st.session_state.current_waypoint_index >= total_wp - 1:
                        st.success("🎉 任务完成！")
                        st.session_state.mission_executing = False
                
                time.sleep(0.3)
                st.rerun()
            
            # 遥测数据
            col_tel1, col_tel2, col_tel3, col_tel4 = st.columns(4)
            col_tel1.metric("地速", f"{random.uniform(8, 12):.1f} m/s")
            col_tel2.metric("空速", f"{random.uniform(10, 15):.1f} m/s")
            col_tel3.metric("相对高度", f"{st.session_state.flight_altitude} m")
            col_tel4.metric("卫星数", random.randint(8, 15))
            
            # 实时地图
            st.markdown("---")
            st.subheader("🗺️ 实时位置")
            
            m = folium.Map(location=st.session_state.drone_position or st.session_state.map_center, zoom_start=15)
            
            if st.session_state.planned_path:
                path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
                folium.PolyLine(path_coords, color='blue', weight=3, opacity=0.6).add_to(m)
            
            if st.session_state.drone_position:
                folium.Marker(st.session_state.drone_position, popup="无人机",
                             icon=folium.Icon(color='orange', icon='plane', prefix='fa')).add_to(m)
            
            st_folium(m, width=700, height=400, key="mission_map")

st.markdown("---")
st.caption(f"MAVLink Ground Control Station | 航线规划与避障系统 | 北京时间 (UTC+8)")
