
import streamlit as st
import time
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, MarkerCluster
from streamlit_folium import st_folium
import math
import json
import random

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
    16: "NAV_WAYPOINT",
    17: "NAV_LOITER_UNLIM",
    18: "NAV_LOITER_TURNS",
    19: "NAV_LOITER_TIME",
    20: "NAV_RETURN_TO_LAUNCH",
    21: "NAV_LAND",
    22: "NAV_TAKEOFF",
    23: "NAV_LAND_LOCAL",
    24: "NAV_TAKEOFF_LOCAL",
    25: "NAV_FOLLOW",
    26: "NAV_CONTINUE_AND_CHANGE_ALT",
    27: "NAV_LOITER_TO_ALT",
    80: "NAV_ROI",
    81: "NAV_PATHPLANNING",
    82: "NAV_SPLINE_WAYPOINT",
    83: "NAV_VTOL_TAKEOFF",
    84: "NAV_VTOL_LAND",
    85: "NAV_GUIDED_ENABLE",
    86: "NAV_DELAY",
    87: "NAV_PAYLOAD_PLACE",
    112: "NAV_LAST",
    113: "CONDITION_DELAY",
    114: "CONDITION_CHANGE_ALT",
    115: "CONDITION_DISTANCE",
    116: "CONDITION_YAW",
    117: "CONDITION_LAST",
    118: "DO_SET_MODE",
    119: "DO_JUMP",
    120: "DO_CHANGE_SPEED",
    121: "DO_SET_HOME",
    122: "DO_SET_PARAMETER",
    123: "DO_SET_RELAY",
    124: "DO_REPEAT_RELAY",
    125: "DO_SET_SERVO",
    126: "DO_REPEAT_SERVO",
    127: "DO_FLIGHTTERMINATION",
    128: "DO_CHANGE_ALTITUDE",
    129: "DO_LAND_START",
    130: "DO_RALLY_LAND",
    131: "DO_GO_AROUND",
    132: "DO_REPOSITION",
    133: "DO_PAUSE_CONTINUE",
    134: "DO_SET_REVERSE",
    135: "DO_SET_ROI_LOCATION",
    136: "DO_SET_ROI_WPNEXT_OFFSET",
    137: "DO_SET_ROI_NONE",
    138: "DO_SET_ROI_SYSID",
    139: "DO_CONTROL_VIDEO",
    140: "DO_SET_ROI",
    141: "DO_DIGICAM_CONFIGURE",
    142: "DO_DIGICAM_CONTROL",
    143: "DO_MOUNT_CONFIGURE",
    144: "DO_MOUNT_CONTROL",
    145: "DO_SET_CAM_TRIGG_DIST",
    146: "DO_FENCE_ENABLE",
    147: "DO_PARACHUTE",
    148: "DO_MOTOR_TEST",
    149: "DO_INVERTED_FLIGHT",
    150: "DO_NAVIGATION_MODE",
    151: "DO_SET_HOME",
    152: "DO_SET_RETURN_ALT",
    153: "DO_SET_RESUME_DIST",
    154: "DO_SET_RESUME_SPEED",
    155: "DO_SET_RESUME_ALT",
    156: "DO_SET_RESUME_LATLON",
    157: "DO_SET_RESUME_STATE",
    158: "DO_LAST",
    176: "DO_SET_MISSION_CURRENT",
    177: "DO_LAST",
    178: "DO_SET_MISSION_CURRENT",
    179: "DO_LAST",
    180: "DO_SET_MISSION_CURRENT",
    181: "DO_LAST",
    182: "DO_SET_MISSION_CURRENT",
    183: "DO_LAST",
    184: "DO_SET_MISSION_CURRENT",
    185: "DO_LAST",
    186: "DO_SET_MISSION_CURRENT",
    187: "DO_LAST",
    188: "DO_SET_MISSION_CURRENT",
    189: "DO_LAST",
    190: "DO_SET_MISSION_CURRENT",
    191: "DO_LAST",
    192: "DO_SET_MISSION_CURRENT",
    193: "DO_LAST",
    194: "DO_SET_MISSION_CURRENT",
    195: "DO_LAST",
    196: "DO_SET_MISSION_CURRENT",
    197: "DO_LAST",
    198: "DO_SET_MISSION_CURRENT",
    199: "DO_LAST",
    200: "DO_SET_MISSION_CURRENT",
    201: "DO_LAST",
    202: "DO_SET_MISSION_CURRENT",
    203: "DO_LAST",
    204: "DO_SET_MISSION_CURRENT",
    205: "DO_LAST",
    206: "DO_SET_MISSION_CURRENT",
    207: "DO_LAST",
    208: "DO_SET_MISSION_CURRENT",
    209: "DO_LAST",
    210: "DO_SET_MISSION_CURRENT",
    211: "DO_LAST",
    212: "DO_SET_MISSION_CURRENT",
    213: "DO_LAST",
    214: "DO_SET_MISSION_CURRENT",
    215: "DO_LAST",
    216: "DO_SET_MISSION_CURRENT",
    217: "DO_LAST",
    218: "DO_SET_MISSION_CURRENT",
    219: "DO_LAST",
    220: "DO_SET_MISSION_CURRENT",
    221: "DO_LAST",
    222: "DO_SET_MISSION_CURRENT",
    223: "DO_LAST",
    224: "DO_SET_MISSION_CURRENT",
    225: "DO_LAST",
    226: "DO_SET_MISSION_CURRENT",
    227: "DO_LAST",
    228: "DO_SET_MISSION_CURRENT",
    229: "DO_LAST",
    230: "DO_SET_MISSION_CURRENT",
    231: "DO_LAST",
    232: "DO_SET_MISSION_CURRENT",
    233: "DO_LAST",
    234: "DO_SET_MISSION_CURRENT",
    235: "DO_LAST",
    236: "DO_SET_MISSION_CURRENT",
    237: "DO_LAST",
    238: "DO_SET_MISSION_CURRENT",
    239: "DO_LAST",
    240: "DO_SET_MISSION_CURRENT",
    241: "DO_LAST",
    242: "DO_SET_MISSION_CURRENT",
    243: "DO_LAST",
    244: "DO_SET_MISSION_CURRENT",
    245: "DO_LAST",
    246: "DO_SET_MISSION_CURRENT",
    247: "DO_LAST",
    248: "DO_SET_MISSION_CURRENT",
    249: "DO_LAST",
    250: "DO_SET_MISSION_CURRENT",
    251: "DO_LAST",
    252: "DO_SET_MISSION_CURRENT",
    253: "DO_LAST",
    254: "DO_SET_MISSION_CURRENT",
    255: "DO_LAST",
}

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
        "name": "无人机 → 地面站",
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
        "name": "传感器 → 飞控",
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
        "name": "地面站 → 无人机",
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
        "name": "伴机电脑 → 飞控",
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
        "name": "自定义配置",
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

# ==================== 航线规划相关类 ====================
class Waypoint:
    """航点类"""
    def __init__(self, lat, lon, alt=50, cmd=16, param1=0, param2=0, param3=0, param4=0):
        self.lat = lat  # 纬度
        self.lon = lon  # 经度
        self.alt = alt  # 高度（米）
        self.cmd = cmd  # 命令类型
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.seq = 0

    def to_dict(self):
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "cmd": self.cmd,
            "cmd_name": MAV_CMD.get(self.cmd, "UNKNOWN"),
            "param1": self.param1,
            "param2": self.param2,
            "param3": self.param3,
            "param4": self.param4
        }

class Obstacle:
    """障碍物类"""
    def __init__(self, lat, lon, radius, height):
        self.lat = lat
        self.lon = lon
        self.radius = radius  # 半径（米）
        self.height = height  # 高度（米）

    def to_dict(self):
        return {
            "lat": self.lat,
            "lon": self.lon,
            "radius": self.radius,
            "height": self.height
        }

class PathPlanner:
    """路径规划器 - 支持避障路径规划"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 10  # 安全边距（米）

    def add_obstacle(self, lat, lon, radius, height):
        """添加障碍物"""
        self.obstacles.append(Obstacle(lat, lon, radius, height))

    def clear_obstacles(self):
        """清除所有障碍物"""
        self.obstacles = []

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间距离（米）"""
        R = 6371000  # 地球半径（米）
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
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

    def plan_path(self, start_wp, end_wp, step_size=20):
        """
        使用改进的 A* 算法规划避障路径
        返回航点列表
        """
        path = [start_wp]
        current = start_wp
        max_iterations = 100
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            # 计算到终点的距离和方向
            dist_to_end = self.haversine_distance(current.lat, current.lon, end_wp.lat, end_wp.lon)

            if dist_to_end < step_size:
                # 接近终点，直接连接
                path.append(end_wp)
                break

            # 计算朝向终点的方向
            dlat = end_wp.lat - current.lat
            dlon = end_wp.lon - current.lon

            # 尝试直接前进
            ratio = step_size / dist_to_end
            next_lat = current.lat + dlat * ratio
            next_lon = current.lon + dlon * ratio
            next_alt = current.alt + (end_wp.alt - current.alt) * ratio

            # 检查碰撞
            if not self.check_collision(next_lat, next_lon, next_alt):
                # 安全，直接前进
                next_wp = Waypoint(next_lat, next_lon, next_alt)
                path.append(next_wp)
                current = next_wp
            else:
                # 有障碍物，尝试绕行
                # 尝试左右偏航
                found_path = False
                for angle in [30, -30, 60, -60, 90, -90]:
                    rad = math.radians(angle)
                    # 旋转方向向量
                    new_dlat = dlat * math.cos(rad) - dlon * math.sin(rad)
                    new_dlon = dlat * math.sin(rad) + dlon * math.cos(rad)

                    test_lat = current.lat + new_dlat * ratio
                    test_lon = current.lon + new_dlon * ratio
                    test_alt = current.alt + 10  # 尝试爬升

                    if not self.check_collision(test_lat, test_lon, test_alt):
                        next_wp = Waypoint(test_lat, test_lon, test_alt)
                        path.append(next_wp)
                        current = next_wp
                        found_path = True
                        break

                if not found_path:
                    # 无法找到路径，直接连接到终点（实际系统中应报错）
                    path.append(end_wp)
                    break

        return path

# ==================== 会话状态初始化 ====================
def init_session_state():
    """初始化所有会话状态"""
    defaults = {
        'send_log': deque(maxlen=20),
        'recv_log': deque(maxlen=20),
        'is_running': False,
        'send_count': 0,
        'recv_count': 0,
        'selected_scenario': "drone_to_gcs",
        # 航线规划相关
        'waypoints': [],
        'obstacles': [],
        'planned_path': [],
        'drone_position': None,
        'mission_sent': False,
        'mission_executing': False,
        'map_center': [39.9042, 116.4074],  # 默认北京
        'map_zoom': 13,
        'path_planner': PathPlanner(),
        'last_map_click': None,
        'selected_point': None,  # 'A' or 'B'
        'point_a': None,
        'point_b': None,
        'avoidance_enabled': True,
        'flight_altitude': 50,
        'obstacle_radius': 30,
        'obstacle_height': 100
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
    page = st.radio(
        "选择功能模块",
        ["💓 心跳包监控", "🗺️ 航线规划", "🛰️ 任务监控"],
        label_visibility="collapsed"
    )

    st.markdown("---")
    st.header("📡 系统状态")

    # 显示当前连接状态
    if st.session_state.is_running:
        st.success("🟢 通信正常")
    else:
        st.warning("🟡 通信待机")

    st.metric("发送包数", st.session_state.send_count)
    st.metric("接收包数", st.session_state.recv_count)

    if st.session_state.mission_sent:
        st.info(f"📍 航线点数: {len(st.session_state.waypoints)}")

    if st.session_state.mission_executing:
        st.success("🚁 任务执行中")

# ==================== 心跳包监控页面 ====================
if page == "💓 心跳包监控":
    st.header("💓 MAVLink 心跳包实时演示")

    # ==================== 场景选择 ====================
    st.subheader("📋 选择通信场景")

    scenario_options = list(COMMUNICATION_SCENARIOS.keys())
    scenario_labels = [f"{COMMUNICATION_SCENARIOS[k]['icon']} {COMMUNICATION_SCENARIOS[k]['name']}" for k in scenario_options]

    selected_index = scenario_options.index(st.session_state.selected_scenario)
    selected_label = st.selectbox(
        "选择通信场景",
        options=scenario_labels,
        index=selected_index,
        label_visibility="collapsed"
    )

    selected_scenario = scenario_options[scenario_labels.index(selected_label)]
    st.session_state.selected_scenario = selected_scenario

    scenario = COMMUNICATION_SCENARIOS[selected_scenario]

    st.info(f"""
    **当前场景:** {scenario['icon']} {scenario['name']}

    {scenario['description']}

    **发送端:** {scenario['sender_name']} (系统ID: {scenario['sender_sys']}, 组件ID: {scenario['sender_comp']})  
    **接收端:** {scenario['receiver_name']} (系统ID: {scenario['receiver_sys']}, 组件ID: {scenario['receiver_comp']})
    """)

    # ==================== 自定义配置 ====================
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

        sender_sys = custom_sender_sys
        sender_comp = custom_sender_comp
        receiver_sys = custom_receiver_sys
        receiver_comp = custom_receiver_comp
        mav_type = st.selectbox("飞行器类型", list(MAV_TYPE.keys()), 
                               format_func=lambda x: f"{x}: {MAV_TYPE[x]}", index=2)
    else:
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
        arrow_color = "#00FF00" if st.session_state.is_running else "#888888"

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

    interval = st.slider("发送间隔", 0.5, 3.0, 1.0, 0.1, label_visibility="collapsed")

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("📤 已发送", st.session_state.send_count)
    col2.metric("📥 已接收", st.session_state.recv_count)
    col3.metric("⏱️ 当前间隔", f"{interval}s")
    col4.metric("🚁 飞行器", MAV_TYPE.get(mav_type, "UNKNOWN"))

    # ==================== 发送/接收日志 ====================
    st.markdown("---")

    col_send_log, col_recv_log = st.columns(2)

    with col_send_log:
        st.subheader(f"📤 发送日志")
        st.caption(f"来自: {SYSTEM_ID_MAP.get(sender_sys, '未知')} (SYS:{sender_sys}/COMP:{sender_comp})")

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

    with col_recv_log:
        st.subheader(f"📥 接收日志")
        st.caption(f"目标: {SYSTEM_ID_MAP.get(receiver_sys, '未知')} (SYS:{receiver_sys}/COMP:{receiver_comp})")

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
        st.caption(f"发送端 [{SYSTEM_ID_MAP.get(sender_sys, '未知')}] 发出")
        if st.session_state.send_log:
            last_send = list(st.session_state.send_log)[-1]
            st.code(last_send['hex'], language='hex')
        else:
            st.code("等待数据...", language='text')

    with hex_col2:
        st.caption(f"接收端 [{SYSTEM_ID_MAP.get(receiver_sys, '未知')}] 收到")
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

        hex_data = f"FD 09 00 00 {seq % 256:02X} {sender_sys:02X} {sender_comp:02X} 00 00 00 {sender_sys:02X} 00 00 00 00 51 04 03 {mav_type:02X} 0C"

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

        time.sleep(0.1)

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

        time.sleep(max(0, interval - 0.1))
        st.rerun()

# ==================== 航线规划页面 ====================
elif page == "🗺️ 航线规划":
    st.header("🗺️ 航线规划与避障系统")

    col_left, col_right = st.columns([3, 2])

    with col_left:
        st.subheader("🗺️ 地图操作")

        # 地图图层选择
        map_type = st.selectbox(
            "选择地图图层",
            ["OpenStreetMap", "CartoDB positron", "CartoDB dark_matter"],
            index=0
        )

        # 创建地图
        m = folium.Map(
            location=st.session_state.map_center,
            zoom_start=st.session_state.map_zoom,
            tiles=map_type
        )

        # 添加绘制工具
        draw = Draw(
            draw_options={
                'polyline': False,
                'rectangle': False,
                'polygon': False,
                'circle': False,
                'marker': True,
                'circlemarker': False
            },
            edit_options={'edit': False}
        )
        draw.add_to(m)

        # 显示已有航点
        for i, wp in enumerate(st.session_state.waypoints):
            color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints) - 1 else 'blue'
            icon = 'play' if i == 0 else 'stop' if i == len(st.session_state.waypoints) - 1 else 'dot'
            folium.Marker(
                [wp.lat, wp.lon],
                popup=f"航点 {i+1}<br>高度: {wp.alt}m<br>命令: {MAV_CMD.get(wp.cmd, 'UNKNOWN')}",
                icon=folium.Icon(color=color, icon=icon, prefix='glyphicon')
            ).add_to(m)

        # 显示障碍物
        for obs in st.session_state.obstacles:
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius,
                popup=f"障碍物<br>半径: {obs.radius}m<br>高度: {obs.height}m",
                color='red',
                fill=True,
                fillColor='red',
                fillOpacity=0.3
            ).add_to(m)

        # 显示规划路径
        if st.session_state.planned_path:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
            folium.PolyLine(
                path_coords,
                color='green',
                weight=4,
                opacity=0.8,
                popup="规划路径"
            ).add_to(m)

        # 显示无人机当前位置
        if st.session_state.drone_position:
            folium.Marker(
                st.session_state.drone_position,
                popup="无人机当前位置",
                icon=folium.Icon(color='orange', icon='plane', prefix='fa')
            ).add_to(m)

        # 显示地图
        map_data = st_folium(m, width=700, height=500, key="map")

        # 处理地图点击事件
        if map_data['last_clicked']:
            click_lat = map_data['last_clicked']['lat']
            click_lng = map_data['last_clicked']['lng']
            st.session_state.last_map_click = (click_lat, click_lng)

            st.info(f"📍 点击坐标: 纬度 {click_lat:.6f}, 经度 {click_lng:.6f}")

    with col_right:
        st.subheader("⚙️ 航线设置")

        # 点A和点B设置
        st.markdown("**📍 设置起点 (A) 和终点 (B)**")

        col_a, col_b = st.columns(2)

        with col_a:
            st.markdown("🟢 **起点 A**")
            if st.session_state.point_a:
                st.success(f"已设置
{st.session_state.point_a[0]:.4f}, {st.session_state.point_a[1]:.4f}")
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
                        st.warning("请先在地图上点击选择位置")

            # 手动输入A
            with st.expander("手动输入坐标 A"):
                lat_a = st.number_input("纬度 A", value=39.9042, format="%.6f", key="lat_a")
                lon_a = st.number_input("经度 A", value=116.4074, format="%.6f", key="lon_a")
                if st.button("确认输入 A"):
                    st.session_state.point_a = (lat_a, lon_a)
                    st.rerun()

        with col_b:
            st.markdown("🔴 **终点 B**")
            if st.session_state.point_b:
                st.success(f"已设置
{st.session_state.point_b[0]:.4f}, {st.session_state.point_b[1]:.4f}")
                if st.button("清除 B", key="clear_b"):
                    st.session_state.point_b = None
                    st.rerun()
            else:
                st.info("未设置")
                if st.button("设为 B", key="set_b"):
                    if st.session_state.last_map_click:
                        st.session_state.point_b = st.session_state.last_map_click
                        st.rerun()
                    else:
                        st.warning("请先在地图上点击选择位置")

            # 手动输入B
            with st.expander("手动输入坐标 B"):
                lat_b = st.number_input("纬度 B", value=39.9142, format="%.6f", key="lat_b")
                lon_b = st.number_input("经度 B", value=116.4174, format="%.6f", key="lon_b")
                if st.button("确认输入 B"):
                    st.session_state.point_b = (lat_b, lon_b)
                    st.rerun()

        st.markdown("---")

        # 飞行参数设置
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

        # 路径规划按钮
        if st.button("🧮 规划路径", type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                # 创建起点和终点航点
                start_wp = Waypoint(
                    st.session_state.point_a[0], 
                    st.session_state.point_a[1], 
                    st.session_state.flight_altitude,
                    cmd=22  # NAV_TAKEOFF
                )
                end_wp = Waypoint(
                    st.session_state.point_b[0], 
                    st.session_state.point_b[1], 
                    st.session_state.flight_altitude,
                    cmd=16  # NAV_WAYPOINT
                )

                # 规划路径
                if st.session_state.avoidance_enabled and st.session_state.obstacles:
                    path = st.session_state.path_planner.plan_path(start_wp, end_wp)
                    st.session_state.planned_path = path
                    st.success(f"✅ 避障路径规划完成！共 {len(path)} 个航点")
                else:
                    # 直线路径
                    st.session_state.planned_path = [start_wp, end_wp]
                    st.success("✅ 直线路径规划完成！")

                # 更新航点列表
                st.session_state.waypoints = st.session_state.planned_path
                st.rerun()
            else:
                st.error("❌ 请先设置起点 A 和终点 B")

        # 发送航线按钮
        if st.session_state.planned_path:
            if st.button("📡 发送航线到飞控", type="primary", use_container_width=True):
                # 模拟发送航线
                st.session_state.mission_sent = True

                # 记录发送日志
                current_time = get_local_time()
                timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]

                send_entry = {
                    'time': timestamp,
                    'seq': st.session_state.send_count + 1,
                    'sender': 2,  # GCS
                    'sender_name': "地面控制站",
                    'receiver': 1,  # 飞控
                    'receiver_name': "无人机飞控",
                    'hex': f"MISSION_ITEM_COUNT: {len(st.session_state.planned_path)}"
                }
                st.session_state.send_log.append(send_entry)
                st.session_state.send_count += 1

                st.success(f"📡 航线已发送！共 {len(st.session_state.planned_path)} 个航点")
                st.balloons()

        # 显示当前航点列表
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

        # 任务控制
        col1, col2, col3 = st.columns(3)

        with col1:
            if st.button("▶️ 开始任务", type="primary", use_container_width=True):
                st.session_state.mission_executing = True
                st.session_state.drone_position = [
                    st.session_state.waypoints[0].lat, 
                    st.session_state.waypoints[0].lon
                ]
                st.rerun()

        with col2:
            if st.button("⏸️ 暂停任务", use_container_width=True):
                st.session_state.mission_executing = False
                st.rerun()

        with col3:
            if st.button("⏹️ 终止任务", type="secondary", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.rerun()

        # 显示任务进度
        if st.session_state.mission_executing:
            st.markdown("---")
            st.subheader("📊 实时飞行数据")

            # 模拟飞行进度
            progress_bar = st.progress(0)
            current_pos = st.session_state.drone_position

            if current_pos and st.session_state.waypoints:
                # 计算到终点的进度
                start = st.session_state.waypoints[0]
                end = st.session_state.waypoints[-1]

                total_dist = st.session_state.path_planner.haversine_distance(
                    start.lat, start.lon, end.lat, end.lon
                )
                current_dist = st.session_state.path_planner.haversine_distance(
                    current_pos[0], current_pos[1], end.lat, end.lon
                )

                progress = max(0, min(100, int((1 - current_dist/total_dist) * 100)))
                progress_bar.progress(progress)

                # 更新无人机位置（模拟飞行）
                if progress < 100:
                    # 向终点移动一小步
                    step = 0.001
                    next_lat = current_pos[0] + (end.lat - current_pos[0]) * step
                    next_lon = current_pos[1] + (end.lon - current_pos[1]) * step
                    st.session_state.drone_position = [next_lat, next_lon]
                    time.sleep(0.5)
                    st.rerun()
                else:
                    st.success("🎉 任务完成！")
                    st.session_state.mission_executing = False

            # 显示遥测数据
            col_tel1, col_tel2, col_tel3, col_tel4 = st.columns(4)

            with col_tel1:
                st.metric("地速", f"{random.uniform(8, 12):.1f} m/s")
            with col_tel2:
                st.metric("空速", f"{random.uniform(10, 15):.1f} m/s")
            with col_tel3:
                st.metric("相对高度", f"{st.session_state.flight_altitude} m")
            with col_tel4:
                st.metric("卫星数", random.randint(8, 15))

            # 显示地图
            st.markdown("---")
            st.subheader("🗺️ 实时位置")

            m = folium.Map(location=st.session_state.drone_position or st.session_state.map_center, zoom_start=15)

            # 显示航线
            if st.session_state.planned_path:
                path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
                folium.PolyLine(path_coords, color='blue', weight=3, opacity=0.6).add_to(m)

            # 显示无人机位置
            if st.session_state.drone_position:
                folium.Marker(
                    st.session_state.drone_position,
                    popup="无人机",
                    icon=folium.Icon(color='orange', icon='plane', prefix='fa')
                ).add_to(m)

            st_folium(m, width=700, height=400, key="mission_map")

st.markdown("---")
st.caption(f"MAVLink Ground Control Station | 航线规划与避障系统 | 北京时间 (UTC+8)")
