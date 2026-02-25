import streamlit as st
import time
import math
import random
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, MarkerCluster, AntPath
from streamlit_folium import st_folium
import json

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
    
    def add_obstacle(self, lat, lon, radius, height, name="障碍物"):
        self.obstacles.append(Obstacle(lat, lon, radius, height, name))
    
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
                return True, obs
        return False, None
    
    def plan_path(self, start_wp, end_wp, step_size=30):
        """使用改进的 A* 算法规划避障路径"""
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
                next_wp = Waypoint(next_lat, next_lon, next_alt)
                path.append(next_wp)
                current = next_wp
            else:
                # 绕行策略 - 尝试多个方向
                found = False
                angles = [30, -30, 60, -60, 90, -90, 120, -120, 150, -150]
                
                for angle in angles:
                    rad = math.radians(angle)
                    new_dlat = dlat * math.cos(rad) - dlon * math.sin(rad)
                    new_dlon = dlat * math.sin(rad) + dlon * math.cos(rad)
                    test_lat = current.lat + new_dlat * ratio * 1.2
                    test_lon = current.lon + new_dlon * ratio * 1.2
                    test_alt = current.alt + 30  # 爬升避障
                    
                    if not self.check_collision(test_lat, test_lon, test_alt)[0]:
                        next_wp = Waypoint(test_lat, test_lon, test_alt)
                        path.append(next_wp)
                        current = next_wp
                        found = True
                        break
                
                if not found:
                    # 如果无法绕行，尝试直接飞越（如果高度足够）
                    if current.alt + 50 > max([o.height for o in self.obstacles]):
                        flyover_wp = Waypoint(next_lat, next_lon, max([o.height for o in self.obstacles]) + 50)
                        path.append(flyover_wp)
                        current = flyover_wp
                    else:
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
        'map_center': [32.0603, 118.7969],  # 默认南京
        'map_zoom': 14,
        'path_planner': PathPlanner(),
        'last_map_click': None,
        'point_a': None, 'point_b': None,
        'avoidance_enabled': True,
        'flight_altitude': 80,
        'obstacle_radius': 50, 'obstacle_height': 120,
        'current_waypoint_index': 0,
        'flight_path_history': [],  # 飞行轨迹历史
        'animation_step': 0,
        'show_animation': False
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 地面站 - 3D避障航线规划系统")
st.caption("实时避障路径规划 | 动态飞行仿真 | 障碍物检测与绕行 | 北京时间 (UTC+8)")

# ==================== 侧边栏导航 ====================
with st.sidebar:
    st.header("📋 功能导航")
    page = st.radio("选择功能模块", ["🗺️ 航线规划与避障", "🛰️ 飞行仿真监控", "💓 MAVLink通信"])
    
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
            folium.Marker(
                st.session_state.point_a,
                popup=f"<b>起点 A</b><br>纬度: {st.session_state.point_a[0]:.6f}<br>经度: {st.session_state.point_a[1]:.6f}",
                icon=folium.Icon(color='green', icon='play', prefix='glyphicon'),
                tooltip="起点 A"
            ).add_to(m)
            folium.Circle(
                st.session_state.point_a, radius=10, color='green', fill=True, fillOpacity=0.3
            ).add_to(m)
        
        # 显示终点B（红色）
        if st.session_state.point_b:
            folium.Marker(
                st.session_state.point_b,
                popup=f"<b>终点 B</b><br>纬度: {st.session_state.point_b[0]:.6f}<br>经度: {st.session_state.point_b[1]:.6f}",
                icon=folium.Icon(color='red', icon='stop', prefix='glyphicon'),
                tooltip="终点 B"
            ).add_to(m)
            folium.Circle(
                st.session_state.point_b, radius=10, color='red', fill=True, fillOpacity=0.3
            ).add_to(m)
        
        # 显示障碍物（红色圆柱效果）
        for i, obs in enumerate(st.session_state.obstacles):
            # 外圈 - 危险区域
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius + 15,
                popup=f"<b>{obs.name} #{i+1}</b><br>半径: {obs.radius}m<br>高度: {obs.height}m<br>危险半径: {obs.radius+15}m",
                color='darkred',
                fill=True,
                fillColor='red',
                fillOpacity=0.2,
                weight=2,
                tooltip=f"障碍物 #{i+1} - 危险区"
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
            folium.Marker(
                [obs.lat, obs.lon],
                icon=folium.DivIcon(
                    html=f'<div style="background-color:red;color:white;border-radius:50%;width:24px;height:24px;text-align:center;line-height:24px;font-weight:bold;">{i+1}</div>'
                ),
                tooltip=f"障碍物 #{i+1} 中心"
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
                folium.CircleMarker(
                    [wp.lat, wp.lon],
                    radius=6,
                    color='blue',
                    fill=True,
                    fillColor='white',
                    fillOpacity=0.9,
                    weight=2,
                    popup=f"航点 {i}<br>高度: {wp.alt}m"
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
                    bottom: 50px; left: 50px; width: 180px;
                    border:2px solid grey; z-index:9999; font-size:12px;
                    background-color:white; padding: 10px; border-radius: 5px;">
        <b>图例</b><br>
        <i class="glyphicon glyphicon-play" style="color:green"></i> 起点 A<br>
        <i class="glyphicon glyphicon-stop" style="color:red"></i> 终点 B<br>
        <span style="color:red">●</span> 障碍物<br>
        <span style="color:blue">---</span> 规划航线<br>
        <span style="color:orange">—</span> 实际轨迹<br>
        <span style="color:orange">✈</span> 无人机
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        folium.LayerControl().add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        if map_data['last_clicked']:
            click_lat = map_data['last_clicked']['lat']
            click_lng = map_data['last_clicked']['lng']
            st.session_state.last_map_click = (click_lat, click_lng)
            st.info(f"📍 点击坐标: 纬度 {click_lat:.6f}, 经度 {click_lng:.6f}")
    
    with col_right:
        st.subheader("⚙️ 航线设置")
        
        # 起点A设置
        st.markdown("**🟢 起点 A 设置**")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            lat_a = st.number_input("纬度 A", value=st.session_state.point_a[0] if st.session_state.point_a else 32.0603, format="%.6f", key="lat_a")
        with col_a2:
            lon_a = st.number_input("经度 A", value=st.session_state.point_a[1] if st.session_state.point_a else 118.7969, format="%.6f", key="lon_a")
        
        col_a_btn1, col_a_btn2 = st.columns(2)
        with col_a_btn1:
            if st.button("✅ 设置 A 点", use_container_width=True):
                st.session_state.point_a = (lat_a, lon_a)
                st.success(f"A点已设置: {lat_a:.4f}, {lon_a:.4f}")
                st.rerun()
        with col_a_btn2:
            if st.button("🗑️ 清除 A", use_container_width=True):
                st.session_state.point_a = None
                st.rerun()
        
        # 终点B设置
        st.markdown("**🔴 终点 B 设置**")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            lat_b = st.number_input("纬度 B", value=st.session_state.point_b[0] if st.session_state.point_b else 32.0703, format="%.6f", key="lat_b")
        with col_b2:
            lon_b = st.number_input("经度 B", value=st.session_state.point_b[1] if st.session_state.point_b else 118.8069, format="%.6f", key="lon_b")
        
        col_b_btn1, col_b_btn2 = st.columns(2)
        with col_b_btn1:
            if st.button("✅ 设置 B 点", use_container_width=True):
                st.session_state.point_b = (lat_b, lon_b)
                st.success(f"B点已设置: {lat_b:.4f}, {lon_b:.4f}")
                st.rerun()
        with col_b_btn2:
            if st.button("🗑️ 清除 B", use_container_width=True):
                st.session_state.point_b = None
                st.rerun()
        
        st.markdown("---")
        st.markdown("**✈️ 飞行参数**")
        
        col_alt, col_margin = st.columns(2)
        with col_alt:
            st.session_state.flight_altitude = st.slider("飞行高度 (m)", 30, 200, 80)
        with col_margin:
            st.session_state.path_planner.safety_margin = st.slider("安全边距 (m)", 5, 30, 15)
        
        st.session_state.avoidance_enabled = st.checkbox("启用智能避障", value=True)
        
        # 障碍物设置
        st.markdown("**🚧 障碍物设置**")
        
        # 预设障碍物模板
        obstacle_templates = {
            "自定义": None,
            "高楼 (半径30m, 高100m)": (30, 100),
            "塔吊 (半径20m, 高80m)": (20, 80),
            "山峰 (半径100m, 高150m)": (100, 150),
            "电线塔 (半径15m, 高60m)": (15, 60)
        }
        
        template = st.selectbox("选择障碍物类型", list(obstacle_templates.keys()))
        if template != "自定义" and obstacle_templates[template]:
            st.session_state.obstacle_radius, st.session_state.obstacle_height = obstacle_templates[template]
        
        col_obs1, col_obs2 = st.columns(2)
        with col_obs1:
            obs_lat = st.number_input("障碍物纬度", value=st.session_state.map_center[0], format="%.6f")
        with col_obs2:
            obs_lon = st.number_input("障碍物经度", value=st.session_state.map_center[1], format="%.6f")
        
        col_obs3, col_obs4 = st.columns(2)
        with col_obs3:
            st.session_state.obstacle_radius = st.slider("半径 (m)", 10, 150, st.session_state.obstacle_radius)
        with col_obs4:
            st.session_state.obstacle_height = st.slider("高度 (m)", 20, 200, st.session_state.obstacle_height)
        
        col_obs_btn1, col_obs_btn2 = st.columns(2)
        with col_obs_btn1:
            if st.button("➕ 添加障碍物", use_container_width=True):
                if st.session_state.point_a and st.session_state.point_b:
                    # 检查障碍物是否在航线上
                    obs = Obstacle(obs_lat, obs_lon, st.session_state.obstacle_radius, 
                                  st.session_state.obstacle_height, f"障碍物{len(st.session_state.obstacles)+1}")
                    st.session_state.obstacles.append(obs)
                    st.session_state.path_planner.add_obstacle(obs_lat, obs_lon, 
                                                               st.session_state.obstacle_radius, 
                                                               st.session_state.obstacle_height, 
                                                               obs.name)
                    st.success(f"障碍物已添加！位置: ({obs_lat:.4f}, {obs_lon:.4f})")
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
                    st.write(f"#{i+1}: 位置({obs.lat:.4f}, {obs.lon:.4f}), 半径{obs.radius}m, 高度{obs.height}m")
        
        st.markdown("---")
        
        # 路径规划按钮
        if st.button("🧮 智能规划避障路径", type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
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
                        st.success(f"✅ 避障路径规划完成！\n- 总航点数: {len(path)}\n- 绕行次数: {avoidance_count}\n- 预计飞行距离: {sum([st.session_state.path_planner.haversine_distance(path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon) for i in range(len(path)-1)]):.0f}m")
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
                
                st.success(f"📡 航线已上传到飞控！\n- 航点数: {len(st.session_state.planned_path)}\n- 避障点: {len([wp for wp in st.session_state.planned_path[1:-1] if wp.alt > st.session_state.flight_altitude])}")
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
st.caption(f"MAVLink Ground Control Station | 3D避障航线规划系统 v2.0 | 北京时间 (UTC+8)")
