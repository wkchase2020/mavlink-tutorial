import streamlit as st
import time
import math
import random
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, AntPath
from streamlit_folium import st_folium
import json

# ==================== 坐标系转换 ====================
def gcj02_to_wgs84(lng, lat):
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

def wgs84_to_gcj02(lng, lat):
    """WGS-84转GCJ-02"""
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
    return mglng, mglat

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 多边形避障规划系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

def get_local_time():
    return datetime.utcnow() + timedelta(hours=8)

# ==================== 几何工具函数（替代shapely）====================
def point_in_polygon(lat, lon, polygon_points):
    """射线法判断点是否在多边形内"""
    n = len(polygon_points)
    if n < 3:
        return False
    
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = polygon_points[i][0], polygon_points[i][1]
        yj, xj = polygon_points[j][0], polygon_points[j][1]
        
        if ((yi > lat) != (yj > lat)) and (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside

def point_to_segment_distance(lat, lon, lat1, lon1, lat2, lon2):
    """计算点到线段的距离（米）"""
    lat_diff = lat2 - lat1
    lon_diff = lon2 - lon1
    
    if abs(lat_diff) < 1e-10 and abs(lon_diff) < 1e-10:
        return math.sqrt((lat - lat1)**2 + (lon - lon1)**2) * 111000
    
    t = max(0, min(1, ((lat - lat1) * lat_diff + (lon - lon1) * lon_diff) / (lat_diff**2 + lon_diff**2)))
    
    proj_lat = lat1 + t * lat_diff
    proj_lon = lon1 + t * lon_diff
    
    return math.sqrt((lat - proj_lat)**2 + (lon - proj_lon)**2) * 111000

def rotate_point(cx, cy, x, y, angle_deg):
    """绕中心点旋转坐标"""
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    dx = x - cx
    dy = y - cy
    
    new_dx = dx * cos_a - dy * sin_a
    new_dy = dx * sin_a + dy * cos_a
    
    return cx + new_dx, cy + new_dy

def create_rotated_rectangle(center_lat, center_lon, width_m, height_m, rotation_deg):
    """创建旋转矩形，返回4个角点 [(lat,lon), ...]"""
    lat_offset = (height_m / 2) / 111000
    lon_offset = (width_m / 2) / (111000 * math.cos(math.radians(center_lat)))
    
    corners = [
        (center_lat + lat_offset, center_lon + lon_offset),
        (center_lat + lat_offset, center_lon - lon_offset),
        (center_lat - lat_offset, center_lon - lon_offset),
        (center_lat - lat_offset, center_lon + lon_offset),
    ]
    
    rotated_corners = []
    for lat, lon in corners:
        new_lat, new_lon = rotate_point(center_lat, center_lon, lat, lon, rotation_deg)
        rotated_corners.append((new_lat, new_lon))
    
    return rotated_corners

# ==================== 核心类 ====================
class Waypoint:
    def __init__(self, lat, lon, alt=50, cmd=16, seq=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.cmd = cmd
        self.seq = seq

class Obstacle:
    """支持多边形和圆形的障碍物，新增旋转矩形支持"""
    def __init__(self, points, height, name="障碍物", obs_type="polygon", rotation=0, width=0, height_m=0):
        self.points = points if isinstance(points, list) else [points]
        self.height = height
        self.name = name
        self.type = obs_type
        self.rotation = rotation
        self.width = width
        self.height_m = height_m
        
        if obs_type == "polygon" and len(self.points) > 0:
            self.center_lat = sum(p[0] for p in self.points) / len(self.points)
            self.center_lon = sum(p[1] for p in self.points) / len(self.points)
            self.radius = max(
                math.sqrt((p[0]-self.center_lat)**2 + (p[1]-self.center_lon)**2) * 111000 
                for p in self.points
            )
        elif obs_type == "rectangle":
            self.center_lat = sum(p[0] for p in self.points) / len(self.points)
            self.center_lon = sum(p[1] for p in self.points) / len(self.points)
            self.radius = math.sqrt((width/2)**2 + (height_m/2)**2)
        else:
            self.center_lat = self.points[0][0]
            self.center_lon = self.points[0][1]
            self.radius = 30
    
    def contains_point(self, lat, lon):
        """判断点是否在障碍物内"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center_lat)**2 + (lon-self.center_lon)**2) * 111000
            return dist < self.radius
        
        return point_in_polygon(lat, lon, self.points)

class Node:
    def __init__(self, lat, lon, alt, g_cost=0, h_cost=0, parent=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        if other is None:
            return False
        return (abs(self.lat - other.lat) < 1e-8 and 
                abs(self.lon - other.lon) < 1e-8)
    
    def __hash__(self):
        return hash((round(self.lat, 8), round(self.lon, 8)))

class PathPlanner:
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 20
        self.max_flight_altitude = 100
    
    def add_polygon_obstacle(self, points, height, name="多边形障碍物"):
        self.obstacles.append(Obstacle(points, height, name, "polygon"))
    
    def add_circle_obstacle(self, center_lat, center_lon, radius, height, name="圆形障碍物"):
        obs = Obstacle([(center_lat, center_lon)], height, name, "circle")
        obs.radius = radius
        self.obstacles.append(obs)
    
    def add_rotated_rectangle_obstacle(self, center_lat, center_lon, width_m, height_m, rotation, obs_height, name="矩形障碍物"):
        """添加可旋转的矩形障碍物"""
        points = create_rotated_rectangle(center_lat, center_lon, width_m, height_m, rotation)
        obs = Obstacle(points, obs_height, name, "rectangle", rotation, width_m, height_m)
        self.obstacles.append(obs)
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def set_max_altitude(self, max_alt):
        self.max_flight_altitude = max_alt
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def check_collision(self, lat, lon, alt):
        """检查是否与任何障碍物碰撞"""
        for obs in self.obstacles:
            if alt < obs.height:
                if obs.type in ["polygon", "rectangle"]:
                    if obs.contains_point(lat, lon):
                        return True, obs
                    n = len(obs.points)
                    for i in range(n):
                        p1 = obs.points[i]
                        p2 = obs.points[(i+1) % n]
                        dist = point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1])
                        if dist < self.safety_margin:
                            return True, obs
                else:
                    dist = self.haversine_distance(lat, lon, obs.center_lat, obs.center_lon)
                    if dist < (obs.radius + self.safety_margin):
                        return True, obs
        return False, None
    
    def get_neighbors(self, node, end_node, step_size=20):
        """获取邻居节点"""
        neighbors = []
        
        dlat = end_node.lat - node.lat
        dlon = end_node.lon - node.lon
        dist = math.sqrt(dlat**2 + dlon**2)
        
        if dist > 0:
            dlat_norm = dlat / dist
            dlon_norm = dlon / dist
            
            directions = []
            for i in range(16):
                angle = i * 22.5
                rad = math.radians(angle)
                directions.append((
                    dlat_norm * math.cos(rad) - dlon_norm * math.sin(rad),
                    dlat_norm * math.sin(rad) + dlon_norm * math.cos(rad)
                ))
        else:
            directions = [(0, 0)]
        
        lat_step = step_size / 111000.0
        lon_step = step_size / (111000.0 * math.cos(math.radians(node.lat)))
        
        for dlat_dir, dlon_dir in directions:
            new_lat = node.lat + dlat_dir * lat_step
            new_lon = node.lon + dlon_dir * lon_step
            new_alt = node.alt
            
            collision, obs = self.check_collision(new_lat, new_lon, new_alt)
            
            if not collision:
                g_cost = node.g_cost + step_size
                h_cost = self.haversine_distance(new_lat, new_lon, end_node.lat, end_node.lon)
                neighbors.append(Node(new_lat, new_lon, new_alt, g_cost, h_cost, node))
        
        return neighbors
    
    def plan_path(self, start_wp, end_wp, step_size=20):
        """A*路径规划"""
        start_node = Node(start_wp.lat, start_wp.lon, start_wp.alt, 0,
                         self.haversine_distance(start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon))
        end_node = Node(end_wp.lat, end_wp.lon, end_wp.alt)
        
        if self.check_collision(start_node.lat, start_node.lon, start_node.alt)[0]:
            st.error("❌ 起点在障碍物内")
            return [start_wp, end_wp]
        
        if self.check_collision(end_node.lat, end_node.lon, end_node.alt)[0]:
            st.error("❌ 终点在障碍物内")
            return [start_wp, end_wp]
        
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        
        max_iter = 5000
        for _ in range(max_iter):
            if not open_list:
                break
            
            current = heapq.heappop(open_list)
            
            if self.haversine_distance(current.lat, current.lon, end_node.lat, end_node.lon) < step_size * 2:
                path = []
                node = current
                while node:
                    path.append(node)
                    node = node.parent
                path.reverse()
                if path[-1].lat != end_node.lat:
                    path.append(end_node)
                
                waypoints = [Waypoint(n.lat, n.lon, n.alt, seq=i) for i, n in enumerate(path)]
                waypoints[0].cmd = 22
                waypoints[-1].cmd = 16
                return waypoints
            
            closed_set.add((round(current.lat, 8), round(current.lon, 8)))
            
            for neighbor in self.get_neighbors(current, end_node, step_size):
                key = (round(neighbor.lat, 8), round(neighbor.lon, 8))
                if key in closed_set:
                    continue
                
                existing = [n for n in open_list if abs(n.lat-neighbor.lat)<1e-8 and abs(n.lon-neighbor.lon)<1e-8]
                if existing and existing[0].g_cost <= neighbor.g_cost:
                    continue
                
                heapq.heappush(open_list, neighbor)
        
        st.warning("⚠️ 未找到完整路径，返回直线路径")
        return [start_wp, end_wp]

# ==================== 初始化 ====================
def init_session_state():
    defaults = {
        'send_log': deque(maxlen=20), 
        'recv_log': deque(maxlen=20),
        'is_running': False, 
        'send_count': 0, 
        'recv_count': 0,
        'waypoints': [], 
        'planned_path': [],
        'drone_position': None, 
        'mission_sent': False, 
        'mission_executing': False,
        'map_center': [32.0603, 118.7969],
        'path_planner': PathPlanner(),
        'point_a': None, 
        'point_b': None,
        'point_a_gcj': None, 
        'point_b_gcj': None,
        'flight_altitude': 50,
        'current_waypoint_index': 0,
        'flight_path_history': [],
        'animation_step': 0,
        'coord_system': 'WGS-84',
        'map_draw_data': None,
        'temp_obstacle_points': [],
        'temp_obstacle_type': None,
        'temp_circle_radius': 30,
        'obstacle_height_input': 40,
        'pending_drawing': None,  # 存储待处理的绘制数据
        'rect_width': 50,
        'rect_height': 80,
        'rect_rotation': 0,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面 ====================
st.title("🚁 MAVLink 地面站 - 多边形避障规划系统")
st.caption("支持多边形框选障碍物 | 旋转矩形 | A*水平绕行 | 北京时间 (UTC+8)")

with st.sidebar:
    st.header("📋 导航")
    page = st.radio("功能", ["🗺️ 航线规划", "🛰️ 飞行监控", "💓 通信日志"])
    
    st.markdown("---")
    st.header("⚙️ 坐标系")
    coord_opt = ["WGS-84", "GCJ-02(高德/百度)"]
    sel = st.radio("输入坐标系", coord_opt, 0 if st.session_state.coord_system=='WGS-84' else 1)
    st.session_state.coord_system = 'WGS-84' if 'WGS' in sel else 'GCJ-02'
    
    st.markdown("---")
    st.header("📊 状态")
    if st.session_state.point_a:
        st.success("🟢 A点已设")
    else:
        st.error("🔴 A点未设")
    if st.session_state.point_b:
        st.success("🟢 B点已设")
    else:
        st.error("🔴 B点未设")
    
    st.metric("障碍物数", len(st.session_state.path_planner.obstacles))
    st.metric("航线点数", len(st.session_state.waypoints))

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.header("🗺️ 航线规划与避障")
    
    with st.expander("📖 使用说明", expanded=True):
        st.markdown("""
        ### 🎯 操作步骤：
        
        1. **设置A/B点**：在右侧输入起点和终点坐标（注意选择正确的坐标系）
        2. **添加障碍物**：
           - **地图绘制**：点击地图上的 🔵 按钮，画出多边形/矩形/圆形，然后点击"确认添加"
           - **参数设置**：在右侧直接输入坐标和尺寸添加旋转矩形
        3. **规划路径**：点击"规划避障路径"
        
        ### 🚫 避障规则：
        - 障碍物高度 ≥ 飞行高度：**强制水平绕行**
        - 障碍物高度 < 飞行高度：**可以飞越**
        
        ### ⚠️ 坐标系说明：
        - **地图绘制**：自动使用WGS-84坐标
        - **手动输入**：根据右侧选择的坐标系自动转换
        """)
    
    col_map, col_ctrl = st.columns([3, 2])
    
    with col_map:
        st.subheader("🗺️ 地图（可绘制障碍物）")
        
        # 确定地图中心
        if st.session_state.point_a and st.session_state.point_b:
            center = [(st.session_state.point_a[0]+st.session_state.point_b[0])/2,
                     (st.session_state.point_a[1]+st.session_state.point_b[1])/2]
        else:
            center = st.session_state.map_center
        
        # 创建地图
        m = folium.Map(location=center, zoom_start=16, tiles="CartoDB positron")
        
        # 添加绘制工具
        draw = Draw(
            draw_options={
                'polyline': False,
                'rectangle': True,
                'polygon': True,
                'circle': True,
                'marker': False,
                'circlemarker': False
            },
            edit_options={'edit': True, 'remove': True}
        )
        draw.add_to(m)
        
        # 卫星图层
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='卫星影像',
            overlay=False,
            control=True
        ).add_to(m)
        
        # 显示A点
        if st.session_state.point_a:
            folium.Marker(st.session_state.point_a, 
                         popup="起点A",
                         icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
            folium.Circle(st.session_state.point_a, radius=8, color='green', fill=True).add_to(m)
        
        # 显示B点
        if st.session_state.point_b:
            folium.Marker(st.session_state.point_b,
                         popup="终点B", 
                         icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
            folium.Circle(st.session_state.point_b, radius=8, color='red', fill=True).add_to(m)
        
        # 显示障碍物
        for i, obs in enumerate(st.session_state.path_planner.obstacles):
            color = 'red' if obs.height >= st.session_state.flight_altitude else 'orange'
            
            if obs.type == "polygon":
                folium.Polygon(
                    locations=obs.points,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.4,
                    weight=2
                ).add_to(m)
                folium.Marker(
                    [obs.center_lat, obs.center_lon],
                    icon=folium.DivIcon(
                        html=f'<div style="background:{color};color:white;padding:2px 6px;border-radius:3px;font-size:11px;">{obs.height}m</div>'
                    )
                ).add_to(m)
                
            elif obs.type == "rectangle":
                folium.Polygon(
                    locations=obs.points,
                    popup=f"{obs.name}<br>高度:{obs.height}m<br>旋转:{obs.rotation}°",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.4,
                    weight=2
                ).add_to(m)
                folium.PolyLine(
                    [[obs.center_lat, obs.center_lon], obs.points[0]],
                    color=color,
                    weight=1,
                    dash_array='5,5'
                ).add_to(m)
                folium.Marker(
                    [obs.center_lat, obs.center_lon],
                    icon=folium.DivIcon(
                        html=f'<div style="background:{color};color:white;padding:2px 6px;border-radius:3px;font-size:11px;">{obs.height}m<br>↻{obs.rotation}°</div>'
                    )
                ).add_to(m)
                
            else:  # circle
                folium.Circle(
                    [obs.center_lat, obs.center_lon],
                    radius=obs.radius,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillOpacity=0.4
                ).add_to(m)
        
        # 显示规划路径
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
            AntPath(path_coords, color='blue', weight=5, opacity=0.9, 
                   dash_array=[15, 30], delay=600).add_to(m)
            
            for i, wp in enumerate(st.session_state.planned_path):
                color = 'green' if i==0 else 'red' if i==len(st.session_state.planned_path)-1 else 'blue'
                folium.CircleMarker([wp.lat, wp.lon], radius=5, color=color, 
                                   fill=True, fillColor='white').add_to(m)
        
        # 显示地图并获取绘制数据
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        # 处理地图绘制数据 - 使用更简单直接的方式
        if map_data:
            # 检查是否有新的绘制数据
            if map_data.get('last_active_drawing'):
                drawing = map_data['last_active_drawing']
                
                # 检查是否是新绘制（通过类型和坐标判断）
                drawing_signature = f"{drawing.get('type')}_{str(drawing.get('geometry', {}).get('coordinates', []))[:50]}"
                
                if st.session_state.get('last_drawing_signature') != drawing_signature:
                    st.session_state['last_drawing_signature'] = drawing_signature
                    
                    geom_type = drawing.get('type')
                    
                    if geom_type == 'polygon':
                        coords = drawing['geometry']['coordinates'][0]
                        # Folium返回的是[lon, lat]，需要转换为[lat, lon]
                        points = [(coord[1], coord[0]) for coord in coords[:-1]]  # 去掉重复的最后一个点
                        st.session_state.pending_drawing = {
                            'type': 'polygon',
                            'points': points
                        }
                        st.rerun()
                        
                    elif geom_type == 'rectangle':
                        coords = drawing['geometry']['coordinates'][0]
                        points = [(coord[1], coord[0]) for coord in coords[:-1]]
                        st.session_state.pending_drawing = {
                            'type': 'rectangle',
                            'points': points
                        }
                        st.rerun()
                        
                    elif geom_type == 'circle':
                        center = drawing['geometry']['coordinates']
                        radius = drawing['properties']['radius']
                        # center是[lon, lat]，转换为[lat, lon]
                        st.session_state.pending_drawing = {
                            'type': 'circle',
                            'center': (center[1], center[0]),
                            'radius': radius
                        }
                        st.rerun()
    
    with col_ctrl:
        st.subheader("⚙️ 控制面板")
        
        # A点设置 - 根据坐标系自动转换
        st.markdown("**🟢 起点 A**")
        st.caption(f"输入坐标系: {st.session_state.coord_system}")
        c1, c2 = st.columns(2)
        
        # 如果有WGS坐标，转换为当前坐标系显示
        default_lat_a = 32.0603
        default_lon_a = 118.7969
        if st.session_state.point_a:
            if st.session_state.coord_system == 'GCJ-02':
                # WGS转GCJ显示
                lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_a[1], st.session_state.point_a[0])
                default_lat_a = lat_gcj
                default_lon_a = lon_gcj
            else:
                default_lat_a = st.session_state.point_a[0]
                default_lon_a = st.session_state.point_a[1]
        
        lat_a = c1.number_input("纬度", value=default_lat_a, format="%.6f", key="lat_a")
        lon_a = c2.number_input("经度", value=default_lon_a, format="%.6f", key="lon_a")
        
        if st.button("✅ 设置A点", key="set_a"):
            if st.session_state.coord_system == 'GCJ-02':
                # GCJ转WGS存储
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
                st.session_state.point_a = (lat_wgs, lon_wgs)
                st.session_state.point_a_gcj = (lat_a, lon_a)
            else:
                st.session_state.point_a = (lat_a, lon_a)
                st.session_state.point_a_gcj = None
            st.success(f"A点已设置 ({st.session_state.coord_system})")
            st.rerun()
        
        # B点设置
        st.markdown("**🔴 终点 B**")
        c3, c4 = st.columns(2)
        
        default_lat_b = 32.0703
        default_lon_b = 118.8069
        if st.session_state.point_b:
            if st.session_state.coord_system == 'GCJ-02':
                lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_b[1], st.session_state.point_b[0])
                default_lat_b = lat_gcj
                default_lon_b = lon_gcj
            else:
                default_lat_b = st.session_state.point_b[0]
                default_lon_b = st.session_state.point_b[1]
        
        lat_b = c3.number_input("纬度", value=default_lat_b, format="%.6f", key="lat_b")
        lon_b = c4.number_input("经度", value=default_lon_b, format="%.6f", key="lon_b")
        
        if st.button("✅ 设置B点", key="set_b"):
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
                st.session_state.point_b = (lat_wgs, lon_wgs)
                st.session_state.point_b_gcj = (lat_b, lon_b)
            else:
                st.session_state.point_b = (lat_b, lon_b)
                st.session_state.point_b_gcj = None
            st.success(f"B点已设置 ({st.session_state.coord_system})")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("**✈️ 飞行参数**")
        new_alt = st.slider("飞行高度(m)", 10, 100, st.session_state.flight_altitude, key="flight_alt")
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
            st.session_state.path_planner.set_max_altitude(new_alt)
            st.rerun()
        
        st.info(f"当前飞行高度: **{st.session_state.flight_altitude}m**")
        
        st.markdown("---")
        
        # 障碍物设置
        st.markdown("**🚧 障碍物管理**")
        
        # 显示待确认的地图绘制
        if st.session_state.pending_drawing:
            drawing = st.session_state.pending_drawing
            obs_type = drawing['type']
            
            if obs_type == 'circle':
                st.success(f"⭕ 地图绘制：圆形障碍")
                st.write(f"- 中心: ({drawing['center'][0]:.6f}, {drawing['center'][1]:.6f})")
                st.write(f"- 半径: {drawing['radius']:.1f}m")
            else:
                st.success(f"📐 地图绘制：{obs_type} ({len(drawing['points'])}顶点)")
                # 显示第一个点作为参考
                st.write(f"- 第一个顶点: ({drawing['points'][0][0]:.6f}, {drawing['points'][0][1]:.6f})")
            
            obs_height = st.number_input("障碍物高度(m)", 5, 200, 40, key="obs_h")
            
            col_add, col_cancel = st.columns(2)
            with col_add:
                if st.button("✅ 确认添加", key="add_obs", type="primary"):
                    if obs_type == 'circle':
                        center = drawing['center']
                        st.session_state.path_planner.add_circle_obstacle(
                            center[0], center[1], drawing['radius'], obs_height,
                            f"圆形障碍({obs_height}m)"
                        )
                    else:
                        st.session_state.path_planner.add_polygon_obstacle(
                            drawing['points'], obs_height,
                            f"{obs_type}障碍({obs_height}m)"
                        )
                    
                    st.session_state.pending_drawing = None
                    st.success("✅ 障碍物已添加！")
                    st.rerun()
            
            with col_cancel:
                if st.button("❌ 取消", key="cancel_obs"):
                    st.session_state.pending_drawing = None
                    st.rerun()
            
            st.markdown("---")
        
        # 旋转矩形快速添加（参数方式）- 同样处理坐标系
        with st.expander("⬜ 快速添加旋转矩形"):
            st.markdown("**设置矩形参数：**")
            
            # 使用地图中心或A点作为默认值，并考虑坐标系
            default_lat = st.session_state.map_center[0]
            default_lon = st.session_state.map_center[1]
            if st.session_state.point_a:
                if st.session_state.coord_system == 'GCJ-02' and st.session_state.point_a_gcj:
                    default_lat = st.session_state.point_a_gcj[0]
                    default_lon = st.session_state.point_a_gcj[1]
                else:
                    default_lat = st.session_state.point_a[0]
                    default_lon = st.session_state.point_a[1]
            
            rect_lat = st.number_input("中心纬度", value=default_lat, format="%.6f", key="rect_lat")
            rect_lon = st.number_input("中心经度", value=default_lon, format="%.6f", key="rect_lon")
            rect_width = st.slider("宽度(米)", 10, 200, 50, key="rect_w")
            rect_height = st.slider("长度(米)", 10, 200, 80, key="rect_h")
            rect_rotation = st.slider("旋转角度(度)", 0, 360, 0, key="rect_rot")
            rect_obs_height = st.number_input("矩形障碍物高度(m)", 5, 200, 40, key="rect_obs_h")
            
            if st.button("➕ 添加旋转矩形", key="add_rect"):
                # 坐标系转换
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(rect_lon, rect_lat)
                else:
                    lat_wgs, lon_wgs = rect_lat, rect_lon
                
                st.session_state.path_planner.add_rotated_rectangle_obstacle(
                    lat_wgs, lon_wgs, rect_width, rect_height, 
                    rect_rotation, rect_obs_height,
                    f"矩形障碍({rect_obs_height}m)"
                )
                st.success(f"✅ 已添加旋转矩形障碍物（旋转{rect_rotation}°）")
                st.rerun()
        
        # 显示障碍物列表
        if st.session_state.path_planner.obstacles:
            with st.expander(f"📋 障碍物列表({len(st.session_state.path_planner.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.path_planner.obstacles):
                    need_detour = "🔴" if obs.height >= st.session_state.flight_altitude else "🟢"
                    type_icon = "⬜" if obs.type == "rectangle" else "⭕" if obs.type == "circle" else "📐"
                    rot_info = f"↻{obs.rotation}°" if obs.type == "rectangle" and obs.rotation != 0 else ""
                    st.write(f"{need_detour} {type_icon} #{i+1}: {obs.name} {rot_info}")
                
                if st.button("🗑️ 清除全部障碍物", key="clear_all_obs"):
                    st.session_state.path_planner.clear_obstacles()
                    st.rerun()
        
        st.markdown("---")
        
        # 路径规划
        can_plan = st.session_state.point_a and st.session_state.point_b
        if not can_plan:
            st.warning("⚠️ 请先设置A点和B点")
        
        if st.button("🧮 规划避障路径", type="primary", disabled=not can_plan, key="plan_path"):
            st.session_state.path_planner.set_max_altitude(st.session_state.flight_altitude)
            
            start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                               st.session_state.flight_altitude, cmd=22)
            end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                             st.session_state.flight_altitude, cmd=16)
            
            with st.spinner("A*算法规划中..."):
                path = st.session_state.path_planner.plan_path(start_wp, end_wp)
                st.session_state.planned_path = path
                st.session_state.waypoints = path
            
            straight = st.session_state.path_planner.haversine_distance(
                start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon)
            actual = sum(st.session_state.path_planner.haversine_distance(
                path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon) 
                for i in range(len(path)-1))
            
            st.success(f"""
            ✅ 规划完成！
            - 航点数: {len(path)}
            - 直线: {straight:.0f}m
            - 实际: {actual:.0f}m
            - 增加: {((actual/max(straight,1)-1)*100):.1f}%
            """)
            st.rerun()
        
        # 上传
        if st.session_state.planned_path:
            if st.button("📡 上传到飞控", type="primary", key="upload"):
                st.session_state.mission_sent = True
                st.success(f"已上传 {len(st.session_state.planned_path)} 个航点")
                st.balloons()

# ==================== 飞行监控页面 ====================
elif page == "🛰️ 飞行监控":
    st.header("🛰️ 飞行监控")
    
    if not st.session_state.mission_sent:
        st.warning("请先规划航线")
    else:
        c1, c2, c3 = st.columns(3)
        with c1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始", type="primary", use_container_width=True):
                    st.session_state.mission_executing = True
                    st.session_state.current_waypoint_index = 0
                    st.session_state.flight_path_history = []
                    if st.session_state.waypoints:
                        st.session_state.drone_position = [
                            st.session_state.waypoints[0].lat,
                            st.session_state.waypoints[0].lon
                        ]
                    st.rerun()
            else:
                st.button("▶️ 开始", disabled=True, use_container_width=True)
        
        with c2:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.mission_executing = False
                st.rerun()
        
        with c3:
            if st.button("🔄️ 重置", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.rerun()
        
        if st.session_state.mission_executing or st.session_state.drone_position:
            total = len(st.session_state.waypoints)
            curr = st.session_state.current_waypoint_index
            if total > 0:
                prog = min(100, int((curr / max(1, total-1)) * 100))
                st.progress(prog)
                st.write(f"航点: {curr+1}/{total} ({prog}%)")
            
            center = st.session_state.drone_position if st.session_state.drone_position else st.session_state.map_center
            m = folium.Map(location=center, zoom_start=17, tiles="CartoDB dark_matter")
            
            if st.session_state.planned_path:
                full = [[wp.lat, wp.lon] for wp in st.session_state.planned_path]
                folium.PolyLine(full, color='gray', weight=2, opacity=0.5, dash_array='5,10').add_to(m)
            
            if len(st.session_state.flight_path_history) > 1:
                folium.PolyLine(st.session_state.flight_path_history, color='lime', weight=4).add_to(m)
            
            if st.session_state.drone_position:
                folium.Marker(st.session_state.drone_position,
                            icon=folium.Icon(color='orange', icon='plane', prefix='fa')).add_to(m)
            
            st_folium(m, width=800, height=500)
            
            if st.session_state.mission_executing and st.session_state.drone_position:
                if curr < total - 1:
                    curr_wp = st.session_state.waypoints[curr]
                    next_wp = st.session_state.waypoints[curr + 1]
                    
                    step = st.session_state.animation_step
                    if step < 15:
                        r = step / 15
                        new_lat = curr_wp.lat + (next_wp.lat - curr_wp.lat) * r
                        new_lon = curr_wp.lon + (next_wp.lon - curr_wp.lon) * r
                        st.session_state.drone_position = [new_lat, new_lon]
                        st.session_state.flight_path_history.append([new_lat, new_lon])
                        st.session_state.animation_step += 1
                    else:
                        st.session_state.current_waypoint_index += 1
                        st.session_state.animation_step = 0
                        if st.session_state.current_waypoint_index >= total - 1:
                            st.success("任务完成！")
                            st.session_state.mission_executing = False
                    
                    time.sleep(0.1)
                    st.rerun()

# ==================== 通信日志页面 ====================
elif page == "💓 通信日志":
    st.header("💓 MAVLink通信")
    
    c1, c2 = st.columns(2)
    with c1:
        st.subheader("发送")
        for log in list(st.session_state.send_log)[-10:]:
            st.text(f"{log['time']} SEQ:{log['seq']}")
    
    with c2:
        st.subheader("接收")
        for log in list(st.session_state.recv_log)[-10:]:
            st.text(f"{log['time']} SEQ:{log['seq']}")

st.markdown("---")
st.caption("MAVLink GCS v4.0 | 多边形避障 | 旋转矩形 | A*算法 | 北京时间 (UTC+8)")
