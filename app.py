import streamlit as st
import time
import math
import random
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, MarkerCluster, AntPath
from streamlit_folium import st_folium

# ==================== 坐标系转换函数 ====================
def gcj02_to_wgs84(lng, lat):
    """GCJ-02（火星坐标系）转WGS-84"""
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
        return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

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

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 智能避障航线规划系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

def get_local_time():
    return datetime.utcnow() + timedelta(hours=8)

# ==================== MAVLink 常量 ====================
MAV_CMD = {16: "NAV_WAYPOINT", 22: "NAV_TAKEOFF", 21: "NAV_LAND"}

# ==================== 核心类定义 ====================
class Waypoint:
    def __init__(self, lat, lon, alt=50, cmd=16, seq=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.cmd = cmd
        self.seq = seq
    
    def to_dict(self):
        return {"lat": self.lat, "lon": self.lon, "alt": self.alt}

class Obstacle:
    def __init__(self, lat, lon, radius, height, name="障碍物"):
        self.lat = lat
        self.lon = lon
        self.radius = radius
        self.height = height
        self.name = name

class Node:
    """A*算法节点"""
    def __init__(self, lat, lon, alt, g_cost=0, h_cost=0, parent=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.g_cost = g_cost  # 从起点到当前节点的实际代价
        self.h_cost = h_cost  # 从当前节点到终点的估计代价
        self.f_cost = g_cost + h_cost  # 总代价
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        if other is None:
            return False
        return (abs(self.lat - other.lat) < 1e-8 and 
                abs(self.lon - other.lon) < 1e-8 and 
                abs(self.alt - other.alt) < 0.1)
    
    def __hash__(self):
        return hash((round(self.lat, 8), round(self.lon, 8), round(self.alt)))

class PathPlanner:
    """改进的路径规划器 - 使用A*算法"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 15
        self.max_flight_altitude = 100
    
    def add_obstacle(self, lat, lon, radius, height, name="障碍物"):
        self.obstacles.append(Obstacle(lat, lon, radius, height, name))
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def set_max_altitude(self, max_alt):
        self.max_flight_altitude = max_alt
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间水平距离（米）"""
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
            # 水平距离在危险范围内 且 高度低于障碍物顶部 = 碰撞
            if dist < (obs.radius + self.safety_margin) and alt < obs.height:
                return True, obs
        return False, None
    
    def get_neighbors(self, node, end_node, step_size=20):
        """获取当前节点的邻居节点（8个方向）"""
        neighbors = []
        
        # 计算到终点的方向
        dlat = end_node.lat - node.lat
        dlon = end_node.lon - node.lon
        dist = math.sqrt(dlat**2 + dlon**2)
        
        if dist > 0:
            # 归一化方向
            dlat_norm = dlat / dist
            dlon_norm = dlon / dist
            
            # 8个方向：前、后、左、右、4个对角
            directions = [
                (dlat_norm, dlon_norm),  # 朝向终点
                (dlat_norm, dlon_norm + 0.5),  # 右前方
                (dlat_norm, dlon_norm - 0.5),  # 左前方
                (dlat_norm + 0.5, dlon_norm),  # 右方
                (dlat_norm - 0.5, dlon_norm),  # 左方
                (-dlat_norm, dlon_norm),  # 侧向
                (dlat_norm, -dlon_norm),  # 反向侧向
                (0.5, 0.5),  # 其他方向
                (-0.5, -0.5),
                (0.5, -0.5),
                (-0.5, 0.5)
            ]
        else:
            directions = [(0, 0)]
        
        # 将步长转换为经纬度偏移（约111km/度）
        lat_step = step_size / 111000.0
        lon_step = step_size / (111000.0 * math.cos(math.radians(node.lat)))
        
        for dlat_dir, dlon_dir in directions:
            new_lat = node.lat + dlat_dir * lat_step
            new_lon = node.lon + dlon_dir * lon_step
            
            # 高度保持不变（水平绕行）
            new_alt = node.alt
            
            # 检查是否碰撞
            collision, obs = self.check_collision(new_lat, new_lon, new_alt)
            
            if not collision:
                # 计算代价
                g_cost = node.g_cost + step_size
                h_cost = self.haversine_distance(new_lat, new_lon, end_node.lat, end_node.lon)
                
                neighbors.append(Node(new_lat, new_lon, new_alt, g_cost, h_cost, node))
        
        return neighbors
    
    def plan_path(self, start_wp, end_wp, step_size=25):
        """
        使用A*算法规划避障路径
        关键：当障碍物高于飞行高度时，强制水平绕行，禁止从下方穿行
        """
        # 创建起点和终点节点
        start_node = Node(start_wp.lat, start_wp.lon, start_wp.alt, 0, 
                         self.haversine_distance(start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon))
        end_node = Node(end_wp.lat, end_wp.lon, end_wp.alt)
        
        # 检查起点和终点是否安全
        if self.check_collision(start_node.lat, start_node.lon, start_node.alt)[0]:
            st.error("起点位于障碍物内，请重新设置")
            return [start_wp, end_wp]
        
        if self.check_collision(end_node.lat, end_node.lon, end_node.alt)[0]:
            st.error("终点位于障碍物内，请重新设置")
            return [start_wp, end_wp]
        
        # A*算法主循环
        open_list = []  # 优先队列
        heapq.heappush(open_list, start_node)
        
        closed_set = set()  # 已访问节点
        
        max_iterations = 2000
        iteration = 0
        
        while open_list and iteration < max_iterations:
            iteration += 1
            
            # 取出f_cost最小的节点
            current_node = heapq.heappop(open_list)
            
            # 检查是否到达终点（允许一定误差）
            dist_to_end = self.haversine_distance(current_node.lat, current_node.lon, 
                                                 end_node.lat, end_node.lon)
            if dist_to_end < step_size * 2:
                # 重构路径
                path = self.reconstruct_path(current_node, end_node)
                # 转换为Waypoint对象
                waypoints = [Waypoint(node.lat, node.lon, node.alt, seq=i) 
                           for i, node in enumerate(path)]
                waypoints[0].cmd = 22  # 起飞
                waypoints[-1].cmd = 16  # 航点
                
                return waypoints
            
            # 将当前节点加入关闭列表
            closed_set.add(current_node)
            
            # 获取邻居节点
            neighbors = self.get_neighbors(current_node, end_node, step_size)
            
            for neighbor in neighbors:
                # 检查是否已访问
                if any(neighbor == node for node in closed_set):
                    continue
                
                # 检查是否已在开放列表中且代价更高
                existing = [n for n in open_list if n == neighbor]
                if existing and existing[0].g_cost <= neighbor.g_cost:
                    continue
                
                heapq.heappush(open_list, neighbor)
        
        # 未找到路径
        st.warning("⚠️ 无法找到完全避障路径，尝试生成近似路径...")
        return self.generate_detour_path(start_wp, end_wp)
    
    def reconstruct_path(self, current_node, end_node):
        """重构路径"""
        path = []
        node = current_node
        
        while node is not None:
            path.append(node)
            node = node.parent
        
        path.reverse()
        
        # 添加终点
        if path[-1].lat != end_node.lat or path[-1].lon != end_node.lon:
            path.append(end_node)
        
        return path
    
    def generate_detour_path(self, start_wp, end_wp):
        """当A*失败时，生成简单的绕行路径"""
        path = [start_wp]
        
        # 尝试在障碍物周围生成绕行点
        for obs in self.obstacles:
            if obs.height >= self.max_flight_altitude:
                # 生成绕行点（障碍物周围4个点）
                detour_distance = (obs.radius + self.safety_margin) * 1.5 / 111000.0
                
                detour_points = [
                    Waypoint(obs.lat + detour_distance, obs.lon, start_wp.alt),
                    Waypoint(obs.lat, obs.lon + detour_distance, start_wp.alt),
                    Waypoint(obs.lat - detour_distance, obs.lon, start_wp.alt),
                    Waypoint(obs.lat, obs.lon - detour_distance, start_wp.alt)
                ]
                
                # 选择距离当前位置最近的绕行点
                if path:
                    current = path[-1]
                    best_point = min(detour_points, 
                                   key=lambda p: self.haversine_distance(current.lat, current.lon, p.lat, p.lon))
                    path.append(best_point)
        
        path.append(end_wp)
        
        # 重新编号
        for i, wp in enumerate(path):
            wp.seq = i
        
        return path

# ==================== 会话状态初始化 ====================
def init_session_state():
    defaults = {
        'send_log': deque(maxlen=20), 'recv_log': deque(maxlen=20),
        'is_running': False, 'send_count': 0, 'recv_count': 0,
        'waypoints': [], 'obstacles': [], 'planned_path': [],
        'drone_position': None, 'mission_sent': False, 'mission_executing': False,
        'map_center': [32.0603, 118.7969],
        'map_zoom': 14,
        'path_planner': PathPlanner(),
        'last_map_click': None,
        'point_a': None, 'point_b': None,
        'point_a_gcj': None, 'point_b_gcj': None,
        'avoidance_enabled': True,
        'flight_altitude': 50,
        'obstacle_radius': 30, 'obstacle_height': 40,
        'current_waypoint_index': 0,
        'flight_path_history': [],
        'animation_step': 0,
        'coord_system': 'WGS-84'
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 地面站 - 智能避障航线规划系统")
st.caption("A*寻路算法 | 强制水平绕行 | 完整路径规划 | 北京时间 (UTC+8)")

# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("📋 功能导航")
    page = st.radio("选择功能模块", ["🗺️ 航线规划与避障", "🛰️ 飞行仿真监控", "💓 MAVLink通信"])
    
    st.markdown("---")
    st.header("⚙️ 坐标系设置")
    
    coord_options = ["WGS-84 (GPS/国际标准)", "GCJ-02 (火星坐标/高德百度)"]
    selected_coord = st.radio("输入坐标系", coord_options, 
                             index=0 if st.session_state.coord_system == 'WGS-84' else 1)
    st.session_state.coord_system = 'WGS-84' if 'WGS' in selected_coord else 'GCJ-02'
    
    st.info(f"当前: **{st.session_state.coord_system}**")
    
    st.markdown("---")
    st.header("📡 系统状态")
    
    col1, col2 = st.columns(2)
    with col1:
        st.success("🟢 A点已设") if st.session_state.point_a else st.error("🔴 A点未设")
    with col2:
        st.success("🟢 B点已设") if st.session_state.point_b else st.error("🔴 B点未设")
    
    st.metric("障碍物", len(st.session_state.obstacles))
    st.metric("航线点", len(st.session_state.waypoints))

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划与避障":
    st.header("🗺️ 航线规划与避障系统")
    
    # 避障规则说明
    with st.expander("📖 避障规则（重要）", expanded=True):
        st.markdown("""
        ### 🚫 严禁穿行规则
        
        **当障碍物高度 ≥ 飞行高度时：**
        - ❌ **禁止从下方穿行**（会导致碰撞）
        - ✅ **强制水平绕行**（保持高度不变）
        
        **算法逻辑：**
        1. 使用A*寻路算法在水平面上搜索路径
        2. 只考虑8个水平方向（前、后、左、右、4对角）
        3. **不考虑垂直方向（不爬升、不下降）**
        4. 确保生成从起点到终点的**完整连续路径**
        """)
    
    col_left, col_right = st.columns([3, 2])
    
    with col_left:
        st.subheader("🗺️ 实时地图")
        
        # 地图中心
        if st.session_state.point_a and st.session_state.point_b:
            map_center = [(st.session_state.point_a[0] + st.session_state.point_b[0]) / 2,
                         (st.session_state.point_a[1] + st.session_state.point_b[1]) / 2]
        else:
            map_center = st.session_state.map_center
        
        m = folium.Map(location=map_center, zoom_start=16, tiles="CartoDB positron")
        
        # 卫星图层
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='卫星影像',
            overlay=False,
            control=True
        ).add_to(m)
        
        # 显示起点A
        if st.session_state.point_a:
            folium.Marker(
                st.session_state.point_a,
                popup=f"<b>🟢 起点 A</b><br> lat:{st.session_state.point_a[0]:.6f}<br>lon:{st.session_state.point_a[1]:.6f}",
                icon=folium.Icon(color='green', icon='play', prefix='glyphicon'),
                tooltip="起点 A"
            ).add_to(m)
            folium.Circle(st.session_state.point_a, radius=8, color='green', fill=True).add_to(m)
        
        # 显示终点B
        if st.session_state.point_b:
            folium.Marker(
                st.session_state.point_b,
                popup=f"<b>🔴 终点 B</b><br>lat:{st.session_state.point_b[0]:.6f}<br>lon:{st.session_state.point_b[1]:.6f}",
                icon=folium.Icon(color='red', icon='stop', prefix='glyphicon'),
                tooltip="终点 B"
            ).add_to(m)
            folium.Circle(st.session_state.point_b, radius=8, color='red', fill=True).add_to(m)
        
        # 显示障碍物
        for i, obs in enumerate(st.session_state.obstacles):
            must_detour = obs.height >= st.session_state.flight_altitude
            
            # 危险区域
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius + st.session_state.path_planner.safety_margin,
                color='darkred' if must_detour else 'orange',
                fill=True,
                fillColor='red',
                fillOpacity=0.3,
                weight=2,
                popup=f"<b>障碍物 #{i+1}</b><br>高度:{obs.height}m<br>{'<span style=\"color:red\">🚫 必须绕行</span>' if must_detour else '<span style=\"color:green\">✓ 可飞越</span>'}"
            ).add_to(m)
            
            # 实际障碍物
            folium.Circle(
                [obs.lat, obs.lon],
                radius=obs.radius,
                color='red',
                fill=True,
                fillOpacity=0.5
            ).add_to(m)
            
            # 中心标记
            folium.Marker(
                [obs.lat, obs.lon],
                icon=folium.DivIcon(
                    html=f'<div style="background-color:{"red" if must_detour else "orange"};color:white;border-radius:50%;width:28px;height:28px;text-align:center;line-height:28px;font-weight:bold;border:2px solid white;">{i+1}</div>'
                )
            ).add_to(m)
        
        # 显示规划路径
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
            
            # 使用AntPath显示动态路径
            AntPath(
                locations=path_coords,
                color='blue',
                weight=5,
                opacity=0.9,
                dash_array=[15, 30],
                delay=600
            ).add_to(m)
            
            # 显示所有航点
            for i, wp in enumerate(st.session_state.planned_path):
                color = 'green' if i == 0 else 'red' if i == len(st.session_state.planned_path)-1 else 'blue'
                folium.CircleMarker(
                    [wp.lat, wp.lon],
                    radius=6,
                    color=color,
                    fill=True,
                    fillColor='white',
                    fillOpacity=0.9,
                    popup=f"航点 {i}<br>高度:{wp.alt}m"
                ).add_to(m)
        
        # 图例
        legend = '''
        <div style="position:fixed;bottom:50px;left:50px;width:200px;background:white;padding:10px;border:2px solid grey;border-radius:5px;font-size:12px;">
        <b>图例</b><br>
        🟢 起点A | 🔴 终点B<br>
        🔴 障碍物(需绕行)<br>
        🟠 障碍物(可飞越)<br>
        🔵 规划航线<br>
        ⚪ 航点<br>
        <hr>
        <b>算法:</b> A*水平绕行<br>
        <b>禁止:</b> 垂直穿行
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend))
        
        folium.LayerControl().add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        if map_data['last_clicked']:
            st.session_state.last_map_click = (map_data['last_clicked']['lat'], map_data['last_clicked']['lng'])
            st.info(f"📍 点击: {st.session_state.last_map_click[0]:.6f}, {st.session_state.last_map_click[1]:.6f}")
    
    with col_right:
        st.subheader("⚙️ 参数设置")
        
        # A点设置
        st.markdown("**🟢 起点 A**")
        c1, c2 = st.columns(2)
        with c1:
            lat_a = st.number_input("纬度A", value=st.session_state.point_a_gcj[0] if st.session_state.point_a_gcj else 32.0603, format="%.6f")
        with c2:
            lon_a = st.number_input("经度A", value=st.session_state.point_a_gcj[1] if st.session_state.point_a_gcj else 118.7969, format="%.6f")
        
        c3, c4 = st.columns(2)
        with c3:
            if st.button("✅ 设置A", use_container_width=True):
                st.session_state.point_a_gcj = (lat_a, lon_a)
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
                    st.session_state.point_a = (lat_wgs, lon_wgs)
                else:
                    st.session_state.point_a = (lat_a, lon_a)
                st.rerun()
        with c4:
            if st.button("🗑️ 清除A", use_container_width=True):
                st.session_state.point_a = None
                st.session_state.point_a_gcj = None
                st.rerun()
        
        # B点设置
        st.markdown("**🔴 终点 B**")
        c5, c6 = st.columns(2)
        with c5:
            lat_b = st.number_input("纬度B", value=st.session_state.point_b_gcj[0] if st.session_state.point_b_gcj else 32.0703, format="%.6f")
        with c6:
            lon_b = st.number_input("经度B", value=st.session_state.point_b_gcj[1] if st.session_state.point_b_gcj else 118.8069, format="%.6f")
        
        c7, c8 = st.columns(2)
        with c7:
            if st.button("✅ 设置B", use_container_width=True):
                st.session_state.point_b_gcj = (lat_b, lon_b)
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
                    st.session_state.point_b = (lat_wgs, lon_wgs)
                else:
                    st.session_state.point_b = (lat_b, lon_b)
                st.rerun()
        with c8:
            if st.button("🗑️ 清除B", use_container_width=True):
                st.session_state.point_b = None
                st.session_state.point_b_gcj = None
                st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("**✈️ 飞行参数**")
        
        new_alt = st.slider("飞行高度 (m)", 10, 100, st.session_state.flight_altitude)
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
            st.session_state.path_planner.set_max_altitude(new_alt)
            st.rerun()
        
        st.session_state.path_planner.safety_margin = st.slider("安全边距 (m)", 10, 50, 20)
        
        # 显示当前规则
        st.info(f"""
        **当前飞行高度: {st.session_state.flight_altitude}m**
        
        **避障规则:**
        - 障碍物 < {st.session_state.flight_altitude}m: 可飞越
        - 障碍物 ≥ {st.session_state.flight_altitude}m: **强制绕行**
        """)
        
        # 障碍物设置
        st.markdown("**🚧 障碍物设置**")
        
        templates = {
            "低矮(20m,可飞越)": (20, 15),
            "中等(40m,需绕行)": (30, 40),
            "高楼(80m,需绕行)": (40, 80),
            "超高(120m,需绕行)": (50, 120)
        }
        tmpl = st.selectbox("模板", list(templates.keys()))
        if tmpl != "自定义":
            st.session_state.obstacle_radius, st.session_state.obstacle_height = templates[tmpl]
        
        c9, c10 = st.columns(2)
        with c9:
            obs_lat = st.number_input("障碍物纬度", value=st.session_state.map_center[0], format="%.6f")
        with c10:
            obs_lon = st.number_input("障碍物经度", value=st.session_state.map_center[1], format="%.6f")
        
        c11, c12 = st.columns(2)
        with c11:
            st.session_state.obstacle_radius = st.slider("半径(m)", 10, 100, st.session_state.obstacle_radius)
        with c12:
            st.session_state.obstacle_height = st.slider("高度(m)", 5, 150, st.session_state.obstacle_height)
        
        # 判断类型
        if st.session_state.obstacle_height >= st.session_state.flight_altitude:
            st.error(f"🚫 此障碍物({st.session_state.obstacle_height}m) ≥ 飞行高度({st.session_state.flight_altitude}m)\n\n**将强制水平绕行**")
        else:
            st.success(f"✓ 此障碍物({st.session_state.obstacle_height}m) < 飞行高度({st.session_state.flight_altitude}m)\n\n**可以飞越**")
        
        c13, c14 = st.columns(2)
        with c13:
            if st.button("➕ 添加障碍物", use_container_width=True):
                if st.session_state.point_a and st.session_state.point_b:
                    if st.session_state.coord_system == 'GCJ-02':
                        lon_wgs, lat_wgs = gcj02_to_wgs84(obs_lon, obs_lat)
                        obs_lat, obs_lon = lat_wgs, lon_wgs
                    
                    obs_type = "需绕行" if st.session_state.obstacle_height >= st.session_state.flight_altitude else "可飞越"
                    st.session_state.path_planner.add_obstacle(
                        obs_lat, obs_lon, 
                        st.session_state.obstacle_radius, 
                        st.session_state.obstacle_height,
                        f"障碍物{len(st.session_state.obstacles)+1}({obs_type})"
                    )
                    st.success(f"已添加: {obs_type}")
                    st.rerun()
                else:
                    st.error("请先设置A点和B点")
        with c14:
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacles = []
                st.session_state.path_planner.clear_obstacles()
                st.rerun()
        
        # 显示障碍物列表
        if st.session_state.obstacles:
            with st.expander(f"📋 障碍物列表 ({len(st.session_state.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.obstacles):
                    t = "🔴 需绕行" if obs.height >= st.session_state.flight_altitude else "🟢 可飞越"
                    st.write(f"#{i+1}: {t} 高{obs.height}m 半径{obs.radius}m")
        
        st.markdown("---")
        
        # 规划按钮
        if st.button("🧮 规划避障路径", type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                st.session_state.path_planner.set_max_altitude(st.session_state.flight_altitude)
                
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, cmd=22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, cmd=16)
                
                with st.spinner("A*算法计算中..."):
                    path = st.session_state.path_planner.plan_path(start_wp, end_wp)
                    st.session_state.planned_path = path
                    st.session_state.waypoints = path
                
                # 分析结果
                detour_points = sum(1 for wp in path[1:-1] 
                                  if any(st.session_state.path_planner.check_collision(wp.lat, wp.lon, wp.alt)[0] 
                                        for _ in [0]))  # 简化判断
                
                straight_dist = st.session_state.path_planner.haversine_distance(
                    start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon)
                actual_dist = sum(st.session_state.path_planner.haversine_distance(
                    path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon) 
                    for i in range(len(path)-1))
                
                st.success(f"""
                ✅ 路径规划完成！
                
                **统计信息:**
                - 总航点数: {len(path)}
                - 直线距离: {straight_dist:.0f}m
                - 实际距离: {actual_dist:.0f}m
                - 绕行增加: {((actual_dist/straight_dist-1)*100):.1f}%
                
                **路径类型:** 水平绕行（高度不变）
                """)
                st.rerun()
            else:
                st.error("❌ 请先设置起点A和终点B")
        
        # 上传按钮
        if st.session_state.planned_path:
            if st.button("📡 上传到飞控", type="primary", use_container_width=True):
                st.session_state.mission_sent = True
                
                # 记录日志
                timestamp = get_local_time().strftime("%H:%M:%S.%f")[:-3]
                st.session_state.send_log.append({
                    'time': timestamp, 'seq': st.session_state.send_count + 1,
                    'sender': 2, 'sender_name': "地面控制站",
                    'receiver': 1, 'receiver_name': "无人机飞控",
                    'hex': f"MISSION:{len(st.session_state.planned_path)}"
                })
                st.session_state.send_count += 1
                
                st.success(f"📡 已上传！航点数: {len(st.session_state.planned_path)}")
                st.balloons()

# ==================== 飞行仿真页面 ====================
elif page == "🛰️ 飞行仿真监控":
    st.header("🛰️ 飞行仿真监控")
    
    if not st.session_state.mission_sent:
        st.warning("⚠️ 请先规划并上传航线")
    else:
        # 控制按钮
        c1, c2, c3, c4 = st.columns(4)
        with c1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始飞行", type="primary", use_container_width=True):
                    st.session_state.mission_executing = True
                    st.session_state.current_waypoint_index = 0
                    st.session_state.flight_path_history = []
                    st.session_state.animation_step = 0
                    if st.session_state.waypoints:
                        st.session_state.drone_position = [
                            st.session_state.waypoints[0].lat, 
                            st.session_state.waypoints[0].lon
                        ]
                    st.rerun()
            else:
                st.button("▶️ 开始飞行", disabled=True, use_container_width=True)
        
        with c2:
            if st.session_state.mission_executing and st.button("⏸️ 暂停", use_container_width=True):
                st.session_state.mission_executing = False
                st.rerun()
        
        with c3:
            if st.button("⏹️ 终止", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.rerun()
        
        with c4:
            if st.button("🔄 重新规划", use_container_width=True):
                st.session_state.mission_sent = False
                st.session_state.mission_executing = False
                st.rerun()
        
        # 状态显示
        if st.session_state.mission_executing or st.session_state.drone_position:
            total_wp = len(st.session_state.waypoints)
            current_idx = st.session_state.current_waypoint_index
            
            if total_wp > 0:
                progress = min(100, int((current_idx / max(1, total_wp-1)) * 100))
                st.progress(progress)
                
                c5, c6, c7 = st.columns(3)
                c5.metric("当前航点", f"{current_idx+1}/{total_wp}")
                c6.metric("进度", f"{progress}%")
                c7.metric("高度", f"{st.session_state.waypoints[current_idx].alt if current_idx < len(st.session_state.waypoints) else 0}m")
            
            # 实时地图
            st.subheader("🗺️ 实时轨迹")
            
            center = st.session_state.drone_position if st.session_state.drone_position else st.session_state.map_center
            m_rt = folium.Map(location=center, zoom_start=17, tiles="CartoDB dark_matter")
            
            # 添加卫星图
            folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='卫星影像',
                overlay=False,
                control=True
            ).add_to(m_rt)
            
            # 显示完整规划路径（灰色）
            if st.session_state.planned_path:
                full_path = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
                folium.PolyLine(full_path, color='gray', weight=2, opacity=0.4, dash_array='5,10').add_to(m_rt)
            
            # 显示已飞路径（绿色）
            if len(st.session_state.flight_path_history) > 1:
                folium.PolyLine(st.session_state.flight_path_history, color='lime', weight=4, opacity=0.9).add_to(m_rt)
            
            # 显示无人机
            if st.session_state.drone_position:
                folium.Marker(
                    st.session_state.drone_position,
                    icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
                    popup=f"航点{current_idx+1}"
                ).add_to(m_rt)
                folium.Circle(st.session_state.drone_position, radius=15, color='orange', fill=True, fillOpacity=0.3).add_to(m_rt)
            
            folium.LayerControl().add_to(m_rt)
            st_folium(m_rt, width=800, height=500, key="rt_map")
            
            # 动画逻辑
            if st.session_state.mission_executing and st.session_state.drone_position:
                if current_idx < total_wp - 1:
                    curr_wp = st.session_state.waypoints[current_idx]
                    next_wp = st.session_state.waypoints[current_idx + 1]
                    
                    steps = 15
                    step = st.session_state.animation_step
                    
                    if step < steps:
                        ratio = step / steps
                        new_lat = curr_wp.lat + (next_wp.lat - curr_wp.lat) * ratio
                        new_lon = curr_wp.lon + (next_wp.lon - curr_wp.lon) * ratio
                        
                        st.session_state.drone_position = [new_lat, new_lon]
                        st.session_state.flight_path_history.append([new_lat, new_lon])
                        st.session_state.animation_step += 1
                    else:
                        st.session_state.current_waypoint_index += 1
                        st.session_state.animation_step = 0
                        
                        if st.session_state.current_waypoint_index >= total_wp - 1:
                            st.success("🎉 任务完成！")
                            st.session_state.mission_executing = False
                            st.balloons()
                    
                    time.sleep(0.08)
                    st.rerun()

# ==================== MAVLink通信页面 ====================
elif page == "💓 MAVLink通信":
    st.header("💓 MAVLink通信日志")
    
    c1, c2 = st.columns(2)
    with c1:
        st.subheader("📤 发送")
        for log in reversed(list(st.session_state.send_log)[-10:]):
            st.text(f"[{log['time']}] SEQ:{log['seq']} {log['sender_name']}→{log['receiver_name']}")
    
    with c2:
        st.subheader("📥 接收")
        for log in reversed(list(st.session_state.recv_log)[-10:]):
            st.text(f"[{log['time']}] SEQ:{log['seq']} {log.get('type_name','')}")
    
    if st.button("🗑️ 清空日志"):
        st.session_state.send_log.clear()
        st.session_state.recv_log.clear()
        st.rerun()

st.markdown("---")
st.caption("MAVLink GCS v3.0 | A*避障算法 | 强制水平绕行 | 北京时间 (UTC+8)")
