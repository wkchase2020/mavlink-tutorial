import streamlit as st
import time
import math
import random
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, MarkerCluster, AntPath
from streamlit_folium import st_folium

# ==================== 坐标系转换函数 ====================
def wgs84_to_gcj02(lng, lat):
    """WGS-84转GCJ-02（火星坐标系）"""
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transformlat(lng - 105.0, lat - 35.0)
    dlng = transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - 0.00669342162296594323 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (6378245.0 / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def gcj02_to_wgs84(lng, lat):
    """GCJ-02（火星坐标系）转WGS-84"""
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transformlat(lng - 105.0, lat - 35.0)
    dlng = transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - 0.00669342162296594323 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (6378245.0 / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def transformlat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transformlng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    """判断是否在中国范围外"""
    return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 3D避障航线规划系统",
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

MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY", 4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}
MAV_CMD = {16: "NAV_WAYPOINT", 22: "NAV_TAKEOFF", 21: "NAV_LAND"}

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

COMPONENT_ID_MAP = {
    0: "广播（所有组件）", 1: "自动驾驶仪（主控）", 2: "任务计算机",
    3: "遥控输入", 4: "遥测输出", 5: "相机 #1", 6: "相机 #2",
    7: "相机 #3", 8: "云台 #1", 9: "云台 #2", 10: "伺服 #1",
    18: "GPS #1", 27: "激光雷达 #1", 30: "视觉系统 #1",
    42: "避障系统 #1", 44: "路径规划", 45: "任务规划"
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
    def __init__(self, lat, lon, radius, height, name="障碍物"):
        self.lat = lat
        self.lon = lon
        self.radius = radius
        self.height = height
        self.name = name

class PathPlanner:
    """路径规划器 - 支持避障路径规划"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 15
        self.max_flight_altitude = 100  # 最大飞行高度限制
    
    def add_obstacle(self, lat, lon, radius, height, name="障碍物"):
        self.obstacles.append(Obstacle(lat, lon, radius, height, name))
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def set_max_altitude(self, max_alt):
        """设置最大飞行高度"""
        self.max_flight_altitude = max_alt
    
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
            # 水平距离在安全边距内 且 飞行高度低于障碍物高度 = 碰撞
            if dist < (obs.radius + self.safety_margin) and alt < obs.height:
                return True, obs
        return False, None
    
    def can_fly_over(self, obs, current_alt):
        """判断是否可以飞越该障碍物（爬升策略）"""
        # 飞越需要的高度 = 障碍物高度 + 安全余量
        required_alt = obs.height + 20  # 20米安全余量
        # 只有在飞越高度不超过最大飞行高度时才允许
        return required_alt <= self.max_flight_altitude
    
    def plan_path(self, start_wp, end_wp, step_size=30):
        """
        智能避障路径规划算法
        规则：
        1. 如果障碍物高度 < 飞行高度：可以从上方飞越（爬升）
        2. 如果障碍物高度 >= 飞行高度：必须绕行（水平避让）
        """
        path = [start_wp]
        current = start_wp
        max_iterations = 100
        
        for iteration in range(max_iterations):
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
            
            collision, obs = self.check_collision(next_lat, next_lon, next_alt)
            
            if not collision:
                # 无碰撞，直接前进
                next_wp = Waypoint(next_lat, next_lon, next_alt)
                path.append(next_wp)
                current = next_wp
            else:
                # 有碰撞，需要避障
                # 关键判断：障碍物是否高于最大飞行高度？
                if obs.height >= self.max_flight_altitude:
                    # 障碍物太高，无法飞越，必须绕行（水平方向）
                    found = self.try_detour(current, end_wp, dlat, dlon, ratio, path, obs)
                    if not found:
                        # 绕行失败，尝试反向绕行
                        found = self.try_detour_reverse(current, end_wp, dlat, dlon, ratio, path, obs)
                    if not found:
                        st.error(f"无法规划路径：障碍物 '{obs.name}' 过高且无法绕行")
                        return path
                else:
                    # 障碍物可以飞越，尝试爬升策略
                    if self.can_fly_over(obs, current.alt):
                        # 尝试爬升飞越
                        flyover_alt = obs.height + 20
                        if not self.check_collision(next_lat, next_lon, flyover_alt)[0]:
                            # 爬升后可以安全通过
                            flyover_wp = Waypoint(next_lat, next_lon, flyover_alt)
                            path.append(flyover_wp)
                            current = flyover_wp
                        else:
                            # 爬升后仍有碰撞，需要绕行
                            found = self.try_detour(current, end_wp, dlat, dlon, ratio, path, obs)
                            if not found:
                                found = self.try_detour_reverse(current, end_wp, dlat, dlon, ratio, path, obs)
                    else:
                        # 无法飞越，必须绕行
                        found = self.try_detour(current, end_wp, dlat, dlon, ratio, path, obs)
                        if not found:
                            found = self.try_detour_reverse(current, end_wp, dlat, dlon, ratio, path, obs)
        
        # 重新编号
        for i, wp in enumerate(path):
            wp.seq = i
        
        return path
    
    def try_detour(self, current, end_wp, dlat, dlon, ratio, path, blocking_obs):
        """尝试向右绕行（顺时针方向）"""
        angles = [30, 60, 90, 120, 150, -30, -60, -90, -120, -150]
        
        for angle in angles:
            rad = math.radians(angle)
            new_dlat = dlat * math.cos(rad) - dlon * math.sin(rad)
            new_dlon = dlat * math.sin(rad) + dlon * math.cos(rad)
            
            # 绕行距离要足够远，确保避开障碍物
            detour_ratio = ratio * 1.5  # 增加绕行距离
            
            test_lat = current.lat + new_dlat * detour_ratio
            test_lon = current.lon + new_dlon * detour_ratio
            test_alt = current.alt  # 保持当前高度绕行
            
            # 检查绕行点是否安全
            if not self.check_collision(test_lat, test_lon, test_alt)[0]:
                # 还要检查从绕行点到终点的路径是否安全
                if self.check_path_clear(test_lat, test_lon, end_wp.lat, end_wp.lon, test_alt):
                    next_wp = Waypoint(test_lat, test_lon, test_alt)
                    path.append(next_wp)
                    return True
        
        return False
    
    def try_detour_reverse(self, current, end_wp, dlat, dlon, ratio, path, blocking_obs):
        """尝试反向绕行（逆时针方向，更大的角度）"""
        angles = [-30, -60, -90, -120, -150, 30, 60, 90, 120, 150]
        
        for angle in angles:
            rad = math.radians(angle)
            new_dlat = dlat * math.cos(rad) - dlon * math.sin(rad)
            new_dlon = dlat * math.sin(rad) + dlon * math.cos(rad)
            
            detour_ratio = ratio * 2.0  # 更大的绕行距离
            
            test_lat = current.lat + new_dlat * detour_ratio
            test_lon = current.lon + new_dlon * detour_ratio
            test_alt = current.alt
            
            if not self.check_collision(test_lat, test_lon, test_alt)[0]:
                if self.check_path_clear(test_lat, test_lon, end_wp.lat, end_wp.lon, test_alt):
                    next_wp = Waypoint(test_lat, test_lon, test_alt)
                    path.append(next_wp)
                    return True
        
        return False
    
    def check_path_clear(self, lat1, lon1, lat2, lon2, alt):
        """检查两点间直线路径是否安全"""
        dist = self.haversine_distance(lat1, lon1, lat2, lon2)
        steps = max(1, int(dist / 10))  # 每10米检查一个点
        
        for i in range(steps + 1):
            ratio = i / steps
            check_lat = lat1 + (lat2 - lat1) * ratio
            check_lon = lon1 + (lon2 - lon1) * ratio
            
            if self.check_collision(check_lat, check_lon, alt)[0]:
                return False
        
        return True

# ==================== 会话状态初始化 ====================
def init_session_state():
    defaults = {
        'send_log': deque(maxlen=20), 'recv_log': deque(maxlen=20),
        'is_running': False, 'send_count': 0, 'recv_count': 0,
        'selected_scenario': "drone_to_gcs",
        'waypoints': [], 'obstacles': [], 'planned_path': [],
        'drone_position': None, 'mission_sent': False, 'mission_executing': False,
        'map_center': [32.0603, 118.7969],  # 默认南京
        'map_zoom': 14,
        'path_planner': PathPlanner(),
        'last_map_click': None,
        'point_a': None, 'point_b': None,
        'point_a_gcj': None, 'point_b_gcj': None,  # 存储原始GCJ坐标
        'avoidance_enabled': True,
        'flight_altitude': 50,  # 默认50米
        'obstacle_radius': 30, 'obstacle_height': 40,
        'current_waypoint_index': 0,
        'flight_path_history': [],  # 飞行轨迹历史
        'animation_step': 0,
        'show_animation': False,
        'coord_system': 'WGS-84'  # 默认坐标系
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 地面站 - 3D避障航线规划系统")
st.caption("实时避障路径规划 | 智能绕行算法 | 坐标系自动转换 | 北京时间 (UTC+8)")

# ==================== 侧边栏导航 ====================
with st.sidebar:
    st.header("📋 功能导航")
    page = st.radio("选择功能模块", ["🗺️ 航线规划与避障", "🛰️ 飞行仿真监控", "💓 MAVLink通信"])
    
    st.markdown("---")
    st.header("⚙️ 坐标系设置")
    
    # 坐标系选择
    coord_options = ["WGS-84 (GPS/国际标准)", "GCJ-02 (火星坐标/高德百度)"]
    selected_coord = st.radio("输入坐标系", coord_options, index=0 if st.session_state.coord_system == 'WGS-84' else 1)
    st.session_state.coord_system = 'WGS-84' if 'WGS' in selected_coord else 'GCJ-02'
    
    st.info(f"""
    **当前设置:** {st.session_state.coord_system}
    
    **说明:**
    - **WGS-84**: GPS原始坐标、国际标准
    - **GCJ-02**: 中国国测局坐标，用于高德/百度地图
    
    程序会自动转换到WGS-84在地图上显示
    """)
    
    st.markdown("---")
    st.header("📡 系统状态")
    
    col1, col2 = st.columns(2)
    with col1:
        if st.session_state.point_a:
            st.success("🟢 A点已设")
        else:
            st.error("🔴 A点未设")
    with col2:
        if st.session_state.point_b:
            st.success("🟢 B点已设")
        else:
            st.error("🔴 B点未设")
    
    st.metric("障碍物数量", len(st.session_state.obstacles))
    st.metric("航线航点数", len(st.session_state.waypoints))
    
    if st.session_state.mission_executing:
        st.success("🚁 飞行中")
    elif st.session_state.mission_sent:
        st.info("⏳ 待起飞")

# ==================== 航线规划与避障页面 ====================
if page == "🗺️ 航线规划与避障":
    st.header("🗺️ 航线规划与避障系统")
    
    # 避障策略说明
    with st.expander("📖 避障策略说明", expanded=True):
        st.markdown("""
        **🤖 智能避障规则：**
        
        1. **🛫 飞越策略**（优先尝试）
           - 当障碍物高度 **< 飞行高度** 时
           - 无人机爬升至障碍物上方安全高度通过
           - 安全余量：障碍物高度 + 20米
        
        2. **🔄 绕行策略**（当无法飞越时）
           - 当障碍物高度 **≥ 飞行高度** 时
           - 无人机保持当前高度，水平方向绕行
           - 绕行距离：障碍物半径 + 安全边距 × 1.5
        
        3. **⚠️ 无法规划**
           - 当障碍物过高且无法绕行时
           - 系统会提示错误，需要调整航线或移除障碍物
        """)
    
    col_left, col_right = st.columns([3, 2])
    
    with col_left:
        st.subheader("🗺️ 实时地图")
        
        # 自动调整地图中心
        if st.session_state.point_a and st.session_state.point_b:
            center_lat = (st.session_state.point_a[0] + st.session_state.point_b[0]) / 2
            center_lon = (st.session_state.point_a[1] + st.session_state.point_b[1]) / 2
            map_center = [center_lat, center_lon]
        else:
            map_center = st.session_state.map_center
        
        m = folium.Map(location=map_center, zoom_start=15, tiles="CartoDB positron")
        
        # 添加卫星图层选项
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='卫星影像',
            overlay=False,
            control=True
        ).add_to(m)
        
        # 显示起点A（绿色）
        if st.session_state.point_a:
            popup_a = f"""
            <b>🟢 起点 A</b><br>
            <b>显示坐标(WGS-84):</b><br>
            纬度: {st.session_state.point_a[0]:.6f}<br>
            经度: {st.session_state.point_a[1]:.6f}<br>
            """
            if st.session_state.point_a_gcj:
                popup_a += f"""
                <b>原始输入({st.session_state.coord_system}):</b><br>
                纬度: {st.session_state.point_a_gcj[0]:.6f}<br>
                经度: {st.session_state.point_a_gcj[1]:.6f}<br>
                """
            
            folium.Marker(
                st.session_state.point_a,
                popup=folium.Popup(popup_a, max_width=300),
                icon=folium.Icon(color='green', icon='play', prefix='glyphicon'),
                tooltip="起点 A"
            ).add_to(m)
            folium.Circle(
                st.session_state.point_a, radius=10, color='green', fill=True, fillOpacity=0.3
            ).add_to(m)
        
        # 显示终点B（红色）
        if st.session_state.point_b:
            popup_b = f"""
            <b>🔴 终点 B</b><br>
            <b>显示坐标(WGS-84):</b><br>
            纬度: {st.session_state.point_b[0]:.6f}<br>
            经度: {st.session_state.point_b[1]:.6f}<br>
            """
            if st.session_state.point_b_gcj:
                popup_b += f"""
                <b>原始输入({st.session_state.coord_system}):</b><br>
                纬度: {st.session_state.point_b_gcj[0]:.6f}<br>
                经度: {st.session_state.point_b_gcj[1]:.6f}<br>
                """
            
            folium.Marker(
                st.session_state.point_b,
                popup=folium.Popup(popup_b, max_width=300),
                icon=folium.Icon(color='red', icon='stop', prefix='glyphicon'),
                tooltip="终点 B"
            ).add_to(m)
            folium.Circle(
                st.session_state.point_b, radius=10, color='red', fill=True, fillOpacity=0.3
            ).add_to(m)
        
        # 显示障碍物（红色圆柱效果）
        for i, obs in enumerate(st.session_state.obstacles):
            # 判断障碍物类型（可飞越 vs 必须绕行）
            can_fly_over = obs.height < st.session_state.flight_altitude
            
            # 外圈 - 危险区域
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius + st.session_state.path_planner.safety_margin,
                popup=f"<b>{obs.name} #{i+1}</b><br>半径: {obs.radius}m<br>高度: {obs.height}m<br>危险半径: {obs.radius + st.session_state.path_planner.safety_margin}m<br>{'<span style=\"color:orange\">⚠️ 必须绕行（过高）</span>' if not can_fly_over else '<span style=\"color:green\">✓ 可以飞越</span>'}",
                color='darkred',
                fill=True,
                fillColor='red',
                fillOpacity=0.2,
                weight=2,
                tooltip=f"障碍物 #{i+1} - {'必须绕行' if not can_fly_over else '可以飞越'}"
            ).add_to(m)
            
            # 内圈 - 实际障碍物
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius,
                popup=f"<b>{obs.name} #{i+1}</b><br>半径: {obs.radius}m<br>高度: {obs.height}m",
                color='red',
                fill=True,
                fillColor='red',
                fillOpacity=0.4,
                weight=3,
                tooltip=f"障碍物 #{i+1}"
            ).add_to(m)
            
            # 中心标记
            color_code = "orange" if not can_fly_over else "red"
            folium.Marker(
                [obs.lat, obs.lon],
                icon=folium.DivIcon(
                    html=f'<div style="background-color:{color_code};color:white;border-radius:50%;width:24px;height:24px;text-align:center;line-height:24px;font-weight:bold;">{i+1}</div>'
                ),
                tooltip=f"障碍物 #{i+1} {'(需绕行)' if not can_fly_over else '(可飞越)'}"
            ).add_to(m)
        
        # 显示规划路径（蓝色虚线）
        if st.session_state.planned_path:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
            
            # 使用AntPath实现动态效果
            AntPath(
                locations=path_coords,
                color='blue',
                weight=4,
                opacity=0.8,
                dash_array=[10, 20],
                delay=800,
                tooltip="规划航线"
            ).add_to(m)
            
            # 显示所有航点
            for i, wp in enumerate(st.session_state.planned_path[1:-1], 1):
                # 判断是绕行点还是爬升点
                prev_wp = st.session_state.planned_path[i]
                is_detour = abs(wp.lat - prev_wp.lat) > 0.0001 or abs(wp.lon - prev_wp.lon) > 0.0001
                
                folium.CircleMarker(
                    [wp.lat, wp.lon],
                    radius=6,
                    color='blue',
                    fill=True,
                    fillColor='white',
                    fillOpacity=0.9,
                    weight=2,
                    popup=f"航点 {i}<br>高度: {wp.alt}m<br>{'🔄 绕行点' if is_detour else '⬆️ 爬升点'}"
                ).add_to(m)
        
        # 显示飞行轨迹（橙色实线）
        if st.session_state.flight_path_history:
            folium.PolyLine(
                st.session_state.flight_path_history,
                color='orange',
                weight=3,
                opacity=0.9,
                tooltip="实际飞行轨迹"
            ).add_to(m)
        
        # 显示无人机当前位置
        if st.session_state.drone_position:
            # 无人机图标
            folium.Marker(
                st.session_state.drone_position,
                popup=f"<b>无人机</b><br>高度: {st.session_state.flight_altitude}m",
                icon=folium.Icon(color='orange', icon='plane', prefix='fa', angle=45),
                tooltip="无人机当前位置"
            ).add_to(m)
            # 位置圆圈
            folium.Circle(
                st.session_state.drone_position,
                radius=5,
                color='orange',
                fill=True,
                fillOpacity=0.8
            ).add_to(m)
        
        # 添加图例
        legend_html = '''
        <div style="position: fixed; 
                    bottom: 50px; left: 50px; width: 220px;
                    border:2px solid grey; z-index:9999; font-size:12px;
                    background-color:white; padding: 10px; border-radius: 5px;">
        <b>图例</b><br>
        <i class="glyphicon glyphicon-play" style="color:green"></i> 起点 A (WGS-84)<br>
        <i class="glyphicon glyphicon-stop" style="color:red"></i> 终点 B (WGS-84)<br>
        <span style="color:red">●</span> 障碍物(可飞越)<br>
        <span style="color:orange">●</span> 障碍物(需绕行)<br>
        <span style="color:blue">---</span> 规划航线<br>
        <span style="color:orange">—</span> 实际轨迹<br>
        <span style="color:orange">✈</span> 无人机<br>
        <hr style="margin:5px 0;">
        <small>地图使用WGS-84坐标系<br>输入坐标自动转换</small>
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        folium.LayerControl().add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        if map_data['last_clicked']:
            click_lat = map_data['last_clicked']['lat']
            click_lng = map_data['last_clicked']['lng']
            st.session_state.last_map_click = (click_lat, click_lng)
            st.info(f"📍 点击坐标(WGS-84): 纬度 {click_lat:.6f}, 经度 {click_lng:.6f}")
    
    with col_right:
        st.subheader("⚙️ 航线设置")
        
        # 坐标系提示
        st.info(f"当前输入坐标系: **{st.session_state.coord_system}**\n\n程序将自动转换为WGS-84在地图上显示")
        
        # 起点A设置
        st.markdown("**🟢 起点 A 设置**")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            lat_a = st.number_input("纬度 A", value=st.session_state.point_a_gcj[0] if st.session_state.point_a_gcj else 32.0603, format="%.6f", key="lat_a")
        with col_a2:
            lon_a = st.number_input("经度 A", value=st.session_state.point_a_gcj[1] if st.session_state.point_a_gcj else 118.7969, format="%.6f", key="lon_a")
        
        col_a_btn1, col_a_btn2 = st.columns(2)
        with col_a_btn1:
            if st.button("✅ 设置 A 点", use_container_width=True):
                # 保存原始输入
                st.session_state.point_a_gcj = (lat_a, lon_a)
                # 根据坐标系转换
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
                    st.session_state.point_a = (lat_wgs, lon_wgs)
                    st.success(f"A点已转换!\n输入(GCJ-02): {lat_a:.4f}, {lon_a:.4f}\n显示(WGS-84): {lat_wgs:.4f}, {lon_wgs:.4f}")
                else:
                    st.session_state.point_a = (lat_a, lon_a)
                    st.success(f"A点已设置(WGS-84): {lat_a:.4f}, {lon_a:.4f}")
                st.rerun()
        with col_a_btn2:
            if st.button("🗑️ 清除 A", use_container_width=True):
                st.session_state.point_a = None
                st.session_state.point_a_gcj = None
                st.rerun()
        
        # 终点B设置
        st.markdown("**🔴 终点 B 设置**")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            lat_b = st.number_input("纬度 B", value=st.session_state.point_b_gcj[0] if st.session_state.point_b_gcj else 32.0703, format="%.6f", key="lat_b")
        with col_b2:
            lon_b = st.number_input("经度 B", value=st.session_state.point_b_gcj[1] if st.session_state.point_b_gcj else 118.8069, format="%.6f", key="lon_b")
        
        col_b_btn1, col_b_btn2 = st.columns(2)
        with col_b_btn1:
            if st.button("✅ 设置 B 点", use_container_width=True):
                # 保存原始输入
                st.session_state.point_b_gcj = (lat_b, lon_b)
                # 根据坐标系转换
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
                    st.session_state.point_b = (lat_wgs, lon_wgs)
                    st.success(f"B点已转换!\n输入(GCJ-02): {lat_b:.4f}, {lon_b:.4f}\n显示(WGS-84): {lat_wgs:.4f}, {lon_wgs:.4f}")
                else:
                    st.session_state.point_b = (lat_b, lon_b)
                    st.success(f"B点已设置(WGS-84): {lat_b:.4f}, {lon_b:.4f}")
                st.rerun()
        with col_b_btn2:
            if st.button("🗑️ 清除 B", use_container_width=True):
                st.session_state.point_b = None
                st.session_state.point_b_gcj = None
                st.rerun()
        
        st.markdown("---")
        st.markdown("**✈️ 飞行参数**")
        
        # 修改飞行高度范围为10-100米
        col_alt, col_margin = st.columns(2)
        with col_alt:
            new_altitude = st.slider("飞行高度 (m)", 10, 100, st.session_state.flight_altitude)
            if new_altitude != st.session_state.flight_altitude:
                st.session_state.flight_altitude = new_altitude
                st.session_state.path_planner.set_max_altitude(new_altitude)
                st.rerun()
        with col_margin:
            st.session_state.path_planner.safety_margin = st.slider("安全边距 (m)", 5, 30, 15)
        
        # 显示当前最大飞行高度
        st.info(f"🚁 当前最大飞行高度: **{st.session_state.flight_altitude}m**\n\n⚠️ 高于此高度的障碍物将触发**绕行策略**")
        
        st.session_state.avoidance_enabled = st.checkbox("启用智能避障", value=True)
        
        # 障碍物设置
        st.markdown("**🚧 障碍物设置**")
        
        # 预设障碍物模板
        obstacle_templates = {
            "自定义": None,
            "低矮建筑 (半径20m, 高15m) - 可飞越": (20, 15),
            "中等建筑 (半径30m, 高40m) - 需绕行": (30, 40),
            "高楼 (半径40m, 高80m) - 需绕行": (40, 80),
            "超高建筑 (半径50m, 高120m) - 需绕行": (50, 120),
            "山峰 (半径100m, 高150m) - 需绕行": (100, 150),
            "电线塔 (半径15m, 高60m) - 需绕行": (15, 60)
        }
        
        template = st.selectbox("选择障碍物类型", list(obstacle_templates.keys()))
        if template != "自定义" and obstacle_templates[template]:
            st.session_state.obstacle_radius, st.session_state.obstacle_height = obstacle_templates[template]
        
        col_obs1, col_obs2 = st.columns(2)
        with col_obs1:
            obs_lat_input = st.number_input("障碍物纬度", value=st.session_state.map_center[0], format="%.6f")
        with col_obs2:
            obs_lon_input = st.number_input("障碍物经度", value=st.session_state.map_center[1], format="%.6f")
        
        col_obs3, col_obs4 = st.columns(2)
        with col_obs3:
            st.session_state.obstacle_radius = st.slider("半径 (m)", 10, 150, st.session_state.obstacle_radius)
        with col_obs4:
            # 障碍物高度可以高于飞行高度
            st.session_state.obstacle_height = st.slider("高度 (m)", 5, 200, st.session_state.obstacle_height)
        
        # 显示障碍物类型判断
        if st.session_state.obstacle_height >= st.session_state.flight_altitude:
            st.error(f"⚠️ 此障碍物高度({st.session_state.obstacle_height}m) ≥ 飞行高度({st.session_state.flight_altitude}m)\n\n**无人机将绕行（水平避让）**")
        else:
            st.success(f"✓ 此障碍物高度({st.session_state.obstacle_height}m) < 飞行高度({st.session_state.flight_altitude}m)\n\n**无人机可以飞越**")
        
        col_obs_btn1, col_obs_btn2 = st.columns(2)
        with col_obs_btn1:
            if st.button("➕ 添加障碍物", use_container_width=True):
                if st.session_state.point_a and st.session_state.point_b:
                    # 障碍物也需要坐标转换
                    if st.session_state.coord_system == 'GCJ-02':
                        lon_wgs, lat_wgs = gcj02_to_wgs84(obs_lon_input, obs_lat_input)
                        obs_lat, obs_lon = lat_wgs, lon_wgs
                    else:
                        obs_lat, obs_lon = obs_lat_input, obs_lon_input
                    
                    # 判断障碍物类型
                    obs_type = "需绕行" if st.session_state.obstacle_height >= st.session_state.flight_altitude else "可飞越"
                    obs_name = f"障碍物{len(st.session_state.obstacles)+1}({obs_type})"
                    
                    obs = Obstacle(obs_lat, obs_lon, st.session_state.obstacle_radius, 
                                  st.session_state.obstacle_height, obs_name)
                    st.session_state.obstacles.append(obs)
                    st.session_state.path_planner.add_obstacle(obs_lat, obs_lon, 
                                                               st.session_state.obstacle_radius, 
                                                               st.session_state.obstacle_height, 
                                                               obs_name)
                    st.success(f"障碍物已添加！类型: {obs_type}")
                    st.rerun()
                else:
                    st.error("请先设置A点和B点")
        with col_obs_btn2:
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacles = []
                st.session_state.path_planner.clear_obstacles()
                st.rerun()
        
        # 显示当前障碍物列表
        if st.session_state.obstacles:
            with st.expander(f"📋 当前障碍物列表 ({len(st.session_state.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.obstacles):
                    obs_type = "🔴 需绕行" if obs.height >= st.session_state.flight_altitude else "🟢 可飞越"
                    st.write(f"#{i+1}: {obs_type} | 位置({obs.lat:.4f}, {obs.lon:.4f}), 半径{obs.radius}m, 高度{obs.height}m")
        
        st.markdown("---")
        
        # 路径规划按钮
        if st.button("🧮 智能规划避障路径", type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                # 更新路径规划器的最大高度
                st.session_state.path_planner.set_max_altitude(st.session_state.flight_altitude)
                
                # 创建起点和终点
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, cmd=22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, cmd=16)
                
                if st.session_state.avoidance_enabled and st.session_state.obstacles:
                    with st.spinner("正在计算避障路径..."):
                        path = st.session_state.path_planner.plan_path(start_wp, end_wp)
                        st.session_state.planned_path = path
                        
                        # 统计避障信息
                        avoidance_count = len(path) - 2
                        total_dist = sum([st.session_state.path_planner.haversine_distance(path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon) for i in range(len(path)-1)])
                        
                        # 分析路径类型
                        detour_count = 0
                        climb_count = 0
                        for i in range(1, len(path)-1):
                            prev_wp = path[i-1]
                            curr_wp = path[i]
                            # 判断是绕行还是爬升
                            if abs(curr_wp.lat - prev_wp.lat) > 0.0001 or abs(curr_wp.lon - prev_wp.lon) > 0.0001:
                                detour_count += 1
                            elif curr_wp.alt > prev_wp.alt:
                                climb_count += 1
                        
                        st.success(f"""
                        ✅ 避障路径规划完成！
                        - 总航点数: {len(path)}
                        - 预计飞行距离: {total_dist:.0f}m
                        - 🔄 绕行点: {detour_count}个
                        - ⬆️ 爬升点: {climb_count}个
                        """)
                else:
                    st.session_state.planned_path = [start_wp, end_wp]
                    dist = st.session_state.path_planner.haversine_distance(
                        start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon
                    )
                    st.success(f"✅ 直线路径规划完成！\n- 距离: {dist:.0f}m\n- 航点数: 2")
                
                st.session_state.waypoints = st.session_state.planned_path
                st.rerun()
            else:
                st.error("❌ 请先设置起点 A 和终点 B")
        
        # 发送航线按钮
        if st.session_state.planned_path:
            if st.button("📡 上传航线到飞控", type="primary", use_container_width=True):
                st.session_state.mission_sent = True
                
                # 记录日志
                current_time = get_local_time()
                timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
                
                send_entry = {
                    'time': timestamp, 'seq': st.session_state.send_count + 1,
                    'sender': 2, 'sender_name': "地面控制站",
                    'receiver': 1, 'receiver_name': "无人机飞控",
                    'hex': f"MISSION_COUNT:{len(st.session_state.planned_path)}|AVOID:{len(st.session_state.obstacles)}"
                }
                st.session_state.send_log.append(send_entry)
                st.session_state.send_count += 1
                
                # 统计避障信息
                detour_count = sum(1 for i in range(1, len(st.session_state.planned_path)-1) 
                                  if abs(st.session_state.planned_path[i].lat - st.session_state.planned_path[i-1].lat) > 0.0001)
                
                st.success(f"""
                📡 航线已上传到飞控！
                - 总航点数: {len(st.session_state.planned_path)}
                - 避障机动点: {detour_count}个
                """)
                st.balloons()

# ==================== 飞行仿真监控页面 ====================
elif page == "🛰️ 飞行仿真监控":
    st.header("🛰️ 实时飞行仿真监控")
    
    if not st.session_state.mission_sent:
        st.warning("⚠️ 尚未上传航线任务，请先前往 '🗺️ 航线规划与避障' 页面规划并上传航线")
    else:
        # 控制面板
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始飞行", type="primary", use_container_width=True):
                    st.session_state.mission_executing = True
                    st.session_state.current_waypoint_index = 0
                    st.session_state.flight_path_history = []
                    st.session_state.animation_step = 0
                    if st.session_state.waypoints:
                        st.session_state.drone_position = [st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]
                    st.rerun()
            else:
                st.button("▶️ 开始飞行", disabled=True, use_container_width=True)
        
        with col2:
            if st.session_state.mission_executing:
                if st.button("⏸️ 暂停", use_container_width=True):
                    st.session_state.mission_executing = False
                    st.rerun()
            else:
                st.button("⏸️ 暂停", disabled=True, use_container_width=True)
        
        with col3:
            if st.button("⏹️ 终止/重置", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.session_state.current_waypoint_index = 0
                st.session_state.flight_path_history = []
                st.session_state.animation_step = 0
                st.rerun()
        
        with col4:
            if st.button("🔄 重新规划", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.mission_sent = False
                st.session_state.drone_position = None
                st.rerun()
        
        # 飞行状态显示
        if st.session_state.mission_executing or st.session_state.drone_position:
            st.markdown("---")
            
            # 进度条
            total_wp = len(st.session_state.waypoints)
            current_idx = st.session_state.current_waypoint_index
            
            if total_wp > 0:
                progress = min(100, int((current_idx / max(1, total_wp - 1)) * 100))
                st.progress(progress)
                
                col_info1, col_info2, col_info3 = st.columns(3)
                with col_info1:
                    st.metric("当前航点", f"{current_idx + 1} / {total_wp}")
                with col_info2:
                    st.metric("完成进度", f"{progress}%")
                with col_info3:
                    if current_idx < total_wp - 1:
                        next_wp = st.session_state.waypoints[current_idx + 1]
                        st.metric("下一航点高度", f"{next_wp.alt}m")
            
            # 遥测数据
            st.subheader("📊 实时遥测数据")
            
            col_tel1, col_tel2, col_tel3, col_tel4, col_tel5 = st.columns(5)
            with col_tel1:
                st.metric("地速", f"{random.uniform(8, 12):.1f} m/s")
            with col_tel2:
                st.metric("空速", f"{random.uniform(10, 15):.1f} m/s")
            with col_tel3:
                current_alt = st.session_state.waypoints[current_idx].alt if current_idx < len(st.session_state.waypoints) else st.session_state.flight_altitude
                st.metric("相对高度", f"{current_alt:.0f} m")
            with col_tel4:
                st.metric("垂直速度", f"{random.uniform(-2, 2):.1f} m/s")
            with col_tel5:
                st.metric("GPS卫星", random.randint(10, 16))
            
            # 实时地图
            st.markdown("---")
            st.subheader("🗺️ 实时飞行轨迹")
            
            # 创建实时地图
            if st.session_state.drone_position:
                map_center = st.session_state.drone_position
            else:
                map_center = st.session_state.map_center
            
            m_realtime = folium.Map(location=map_center, zoom_start=16, tiles="CartoDB dark_matter")
            
            # 添加卫星图层
            folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='卫星影像',
                overlay=False,
                control=True
            ).add_to(m_realtime)
            
            # 显示完整航线（半透明）
            if st.session_state.planned_path:
                full_path = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
                folium.PolyLine(full_path, color='gray', weight=2, opacity=0.5, dash_array='5,10').add_to(m_realtime)
                
                # 显示未飞过的航点
                for i in range(current_idx + 1, len(st.session_state.waypoints)):
                    wp = st.session_state.waypoints[i]
                    folium.CircleMarker([wp.lat, wp.lon], radius=4, color='blue', fill=True, fillOpacity=0.5).add_to(m_realtime)
            
            # 显示已飞过的路径（绿色）
            if st.session_state.flight_path_history and len(st.session_state.flight_path_history) > 1:
                folium.PolyLine(st.session_state.flight_path_history, color='green', weight=4, opacity=0.9).add_to(m_realtime)
            
            # 显示当前位置（闪烁效果）
            if st.session_state.drone_position:
                # 无人机位置
                folium.Marker(
                    st.session_state.drone_position,
                    popup=f"<b>无人机</b><br>航点: {current_idx + 1}<br>高度: {st.session_state.waypoints[current_idx].alt if current_idx < len(st.session_state.waypoints) else 0}m",
                    icon=folium.Icon(color='orange', icon='plane', prefix='fa')
                ).add_to(m_realtime)
                
                # 脉冲圆圈效果
                folium.Circle(
                    st.session_state.drone_position,
                    radius=20,
                    color='orange',
                    fill=True,
                    fillColor='orange',
                    fillOpacity=0.3
                ).add_to(m_realtime)
            
            folium.LayerControl().add_to(m_realtime)
            
            st_folium(m_realtime, width=800, height=500, key="realtime_map")
            
            # 飞行动画逻辑
            if st.session_state.mission_executing and st.session_state.drone_position:
                if current_idx < total_wp - 1:
                    current_wp = st.session_state.waypoints[current_idx]
                    next_wp = st.session_state.waypoints[current_idx + 1]
                    
                    # 计算插值位置
                    steps = 20  # 每个航点间分20步
                    step = st.session_state.animation_step
                    
                    if step < steps:
                        ratio = step / steps
                        new_lat = current_wp.lat + (next_wp.lat - current_wp.lat) * ratio
                        new_lon = current_wp.lon + (next_wp.lon - current_wp.lon) * ratio
                        
                        st.session_state.drone_position = [new_lat, new_lon]
                        st.session_state.flight_path_history.append([new_lat, new_lon])
                        st.session_state.animation_step += 1
                    else:
                        # 到达下一航点
                        st.session_state.current_waypoint_index += 1
                        st.session_state.animation_step = 0
                        
                        if st.session_state.current_waypoint_index >= total_wp - 1:
                            st.success("🎉 飞行任务完成！已成功降落在目标点")
                            st.session_state.mission_executing = False
                            st.balloons()
                    
                    time.sleep(0.1)
                    st.rerun()
                else:
                    st.success("🎉 飞行任务完成！")
                    st.session_state.mission_executing = False

# ==================== MAVLink通信页面 ====================
elif page == "💓 MAVLink通信":
    st.header("💓 MAVLink 心跳包与通信日志")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📤 发送日志")
        if st.session_state.send_log:
            for log in reversed(list(st.session_state.send_log)[-10:]):
                st.markdown(f"""
                <div style="background:#1e1e1e;padding:8px;margin:4px 0;border-radius:5px;font-family:monospace;font-size:11px;border-left:4px solid #FF6B6B;color:#fff;">
                    <span style="color:#888;">[{log['time']}]</span>
                    <span style="color:#FF6B6B;font-weight:bold;">📤 SEND</span>
                    <span style="color:#fff;">SEQ:{log['seq']}</span><br>
                    <span style="color:#4ECDC4;">{log['sender_name']} → {log['receiver_name']}</span><br>
                    <span style="color:#FFE66D;font-size:10px;">{log['hex'][:50]}...</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("暂无发送记录")
    
    with col2:
        st.subheader("📥 接收日志")
        if st.session_state.recv_log:
            for log in reversed(list(st.session_state.recv_log)[-10:]):
                st.markdown(f"""
                <div style="background:#1e1e1e;padding:8px;margin:4px 0;border-radius:5px;font-family:monospace;font-size:11px;border-left:4px solid #4ECDC4;color:#fff;">
                    <span style="color:#888;">[{log['time']}]</span>
                    <span style="color:#4ECDC4;font-weight:bold;">📥 RECV</span>
                    <span style="color:#fff;">SEQ:{log['seq']}</span><br>
                    <span style="color:#FF6B6B;">{log.get('type_name', 'UNKNOWN')}</span> | 
                    <span style="color:#FFE66D;">{log.get('status_name', 'UNKNOWN')}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("暂无接收记录")
    
    # 通信控制
    st.markdown("---")
    col_ctrl1, col_ctrl2, col_ctrl3 = st.columns(3)
    
    with col_ctrl1:
        if st.button("▶️ 启动心跳", disabled=st.session_state.is_running, type="primary", use_container_width=True):
            st.session_state.is_running = True
            st.rerun()
    
    with col_ctrl2:
        if st.button("⏹️ 停止心跳", disabled=not st.session_state.is_running, use_container_width=True):
            st.session_state.is_running = False
            st.rerun()
    
    with col_ctrl3:
        if st.button("🗑️ 清空日志", use_container_width=True):
            st.session_state.send_log.clear()
            st.session_state.recv_log.clear()
            st.rerun()
    
    # 心跳循环
    if st.session_state.is_running:
        seq = st.session_state.send_count + 1
        current_time = get_local_time()
        timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
        
        hex_data = f"FD 09 00 00 {seq % 256:02X} 01 01 00 00 00 02 00 00 00 00 51 04 03 02 0C"
        
        send_entry = {
            'time': timestamp, 'seq': seq,
            'sender': 1, 'sender_name': "无人机飞控",
            'receiver': 2, 'receiver_name': "地面控制站",
            'hex': hex_data
        }
        st.session_state.send_log.append(send_entry)
        st.session_state.send_count += 1
        
        recv_entry = {
            'time': timestamp, 'seq': seq,
            'sender': 1, 'sender_name': "无人机飞控",
            'type_name': "QUADROTOR", 'status_name': "ACTIVE"
        }
        st.session_state.recv_log.append(recv_entry)
        st.session_state.recv_count += 1
        
        time.sleep(1.0)
        st.rerun()

st.markdown("---")
st.caption(f"MAVLink Ground Control Station | 3D避障航线规划系统 v2.2 | 智能绕行算法 | 北京时间 (UTC+8)")
