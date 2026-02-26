import streamlit as st
import time
import math
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, AntPath
from streamlit_folium import st_folium

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
    page_title="MAVLink 地面站 - 双策略3D避障系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

def get_local_time():
    return datetime.utcnow() + timedelta(hours=8)

# ==================== 几何工具函数 ====================
def point_in_polygon(lat, lon, polygon_points):
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
    lat_diff = lat2 - lat1
    lon_diff = lon2 - lon1
    if abs(lat_diff) < 1e-10 and abs(lon_diff) < 1e-10:
        return math.sqrt((lat - lat1)**2 + (lon - lon1)**2) * 111000
    t = max(0, min(1, ((lat - lat1) * lat_diff + (lon - lon1) * lon_diff) / (lat_diff**2 + lon_diff**2)))
    proj_lat = lat1 + t * lat_diff
    proj_lon = lon1 + t * lon_diff
    return math.sqrt((lat - proj_lat)**2 + (lon - proj_lon)**2) * 111000

def rotate_point(cx, cy, x, y, angle_deg):
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = x - cx
    dy = y - cy
    new_dx = dx * cos_a - dy * sin_a
    new_dy = dx * sin_a + dy * cos_a
    return cx + new_dx, cy + new_dy

def create_rotated_rectangle(center_lat, center_lon, width_m, height_m, rotation_deg):
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
        elif obs_type == "rectangle":
            self.center_lat = sum(p[0] for p in self.points) / len(self.points)
            self.center_lon = sum(p[1] for p in self.points) / len(self.points)
        else:
            self.center_lat = self.points[0][0]
            self.center_lon = self.points[0][1]
            self.radius = 30
    
    def is_inside(self, lat, lon, margin=0):
        """检查点是否在障碍物内（含边距）"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center_lat)**2 + (lon-self.center_lon)**2) * 111000
            return dist < (self.radius + margin)
        
        if point_in_polygon(lat, lon, self.points):
            return True
        
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i+1) % n]
            if point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1]) < margin:
                return True
        return False

class GridPathPlanner:
    """网格A*路径规划器 - 确保可靠绕行"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 25
        self.grid_size = 12  # 12米网格
    
    def add_polygon_obstacle(self, points, height, name="多边形障碍物"):
        obs = Obstacle(points, height, name, "polygon")
        self.obstacles.append(obs)
    
    def add_circle_obstacle(self, center_lat, center_lon, radius, height, name="圆形障碍物"):
        obs = Obstacle([(center_lat, center_lon)], height, name, "circle")
        obs.radius = radius
        self.obstacles.append(obs)
    
    def add_rotated_rectangle_obstacle(self, center_lat, center_lon, width_m, height_m, rotation, obs_height, name="矩形障碍物"):
        points = create_rotated_rectangle(center_lat, center_lon, width_m, height_m, rotation)
        obs = Obstacle(points, obs_height, name, "rectangle", rotation, width_m, height_m)
        self.obstacles.append(obs)
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def is_collision(self, lat, lon, flight_alt):
        """检查是否碰撞"""
        for obs in self.obstacles:
            if flight_alt >= obs.height + 10:
                continue
            if obs.is_inside(lat, lon, self.safety_margin):
                return True
        return False
    
    def latlon_to_grid(self, lat, lon, base_lat, base_lon):
        dlat = (lat - base_lat) * 111000
        dlon = (lon - base_lon) * 111000 * math.cos(math.radians(base_lat))
        return (int(dlon / self.grid_size), int(dlat / self.grid_size))
    
    def grid_to_latlon(self, grid_x, grid_y, base_lat, base_lon):
        lon = base_lon + (grid_x * self.grid_size) / (111000 * math.cos(math.radians(base_lat)))
        lat = base_lat + (grid_y * self.grid_size) / 111000
        return (lat, lon)
    
    def plan_horizontal_avoidance(self, start_wp, end_wp):
        """水平绕行 - 网格A*"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        flight_alt = start_wp.alt
        
        if self.is_collision(start[0], start[1], flight_alt):
            st.error("❌ 起点在障碍物内")
            return [start_wp, end_wp]
        if self.is_collision(end[0], end[1], flight_alt):
            st.error("❌ 终点在障碍物内")
            return [start_wp, end_wp]
        
        # 检查直线路径
        if self.is_line_clear(start, end, flight_alt):
            return [start_wp, end_wp]
        
        # 网格A*搜索
        base_lat = min(start[0], end[0]) - 0.002
        base_lon = min(start[1], end[1]) - 0.002
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        open_set = [(0, start_grid[0], start_grid[1], [start_grid])]
        visited = set()
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        max_iter = 6000
        iteration = 0
        
        while open_set and iteration < max_iter:
            iteration += 1
            cost, x, y, path = heapq.heappop(open_set)
            
            if abs(x - end_grid[0]) <= 1 and abs(y - end_grid[1]) <= 1:
                waypoints = [start_wp]
                for grid in path[1:]:
                    lat, lon = self.grid_to_latlon(grid[0], grid[1], base_lat, base_lon)
                    waypoints.append(Waypoint(lat, lon, flight_alt, 16, len(waypoints)))
                waypoints.append(end_wp)
                waypoints[-1].seq = len(waypoints) - 1
                return waypoints
            
            key = (x, y)
            if key in visited:
                continue
            visited.add(key)
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (nx, ny) in visited:
                    continue
                
                lat, lon = self.grid_to_latlon(nx, ny, base_lat, base_lon)
                if self.is_collision(lat, lon, flight_alt):
                    continue
                
                move_cost = math.sqrt(dx**2 + dy**2) * self.grid_size
                new_cost = cost + move_cost
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                
                heapq.heappush(open_set, (new_cost + h, nx, ny, path + [(nx, ny)]))
        
        st.warning("⚠️ 未找到绕行路径，返回直线路径")
        return [start_wp, end_wp]
    
    def is_line_clear(self, start, end, flight_alt):
        """检查直线路径是否畅通"""
        dist = self.haversine_distance(start[0], start[1], end[0], end[1])
        steps = max(int(dist / 5), 10)
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            if self.is_collision(lat, lon, flight_alt):
                return False
        return True
    
    def plan_climb_over(self, start_wp, end_wp, max_altitude):
        """爬升飞越"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        base_alt = start_wp.alt
        
        max_obs_height = 0
        steps = 20
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            for obs in self.obstacles:
                if obs.is_inside(lat, lon, 0):
                    max_obs_height = max(max_obs_height, obs.height)
        
        if max_obs_height == 0:
            return [start_wp, end_wp]
        
        fly_alt = max_obs_height + 15
        if fly_alt > max_altitude:
            st.warning(f"需要飞越高度{fly_alt}m超过最大限制{max_altitude}m")
            return None
        
        path = [start_wp]
        dist_total = self.haversine_distance(start[0], start[1], end[0], end[1])
        
        if dist_total > 100:
            climb_lat = start[0] + (end[0] - start[0]) * 0.15
            climb_lon = start[1] + (end[1] - start[1]) * 0.15
            path.append(Waypoint(climb_lat, climb_lon, fly_alt, 16, 1))
        
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        path.append(Waypoint(mid_lat, mid_lon, fly_alt, 16, len(path)))
        
        if dist_total > 100:
            descend_lat = start[0] + (end[0] - start[0]) * 0.85
            descend_lon = start[1] + (end[1] - start[1]) * 0.85
            path.append(Waypoint(descend_lat, descend_lon, fly_alt, 16, len(path)))
        
        path.append(end_wp)
        path[-1].seq = len(path) - 1
        return path

# ==================== 初始化 ====================
def init_session_state():
    defaults = {
        'send_log': deque(maxlen=20), 
        'recv_log': deque(maxlen=20),
        'is_running': False, 
        'send_count': 0, 
        'recv_count': 0,
        'waypoints': [], 
        'planned_path_horizontal': None,
        'planned_path_climb': None,
        'selected_path_type': None,
        'drone_position': None, 
        'mission_sent': False, 
        'mission_executing': False,
        'map_center': [32.0603, 118.7969],
        'planner': GridPathPlanner(),
        'point_a': None, 
        'point_b': None,
        'point_a_gcj': None, 
        'point_b_gcj': None,
        'flight_altitude': 50,
        'max_altitude': 120,
        'current_waypoint_index': 0,
        'flight_path_history': [],
        'animation_step': 0,
        'coord_system': 'WGS-84',
        'pending_drawing': None,
        'rect_width': 50,
        'rect_height': 80,
        'rect_rotation': 0,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("📋 导航")
    page = st.radio("功能页面", ["🗺️ 航线规划", "🛰️ 飞行监控", "💓 通信日志"])
    
    st.markdown("---")
    st.header("⚙️ 坐标系设置")
    coord_opt = ["WGS-84", "GCJ-02(高德/百度)"]
    sel = st.radio("输入坐标系", coord_opt, 
                   index=0 if st.session_state.coord_system=='WGS-84' else 1)
    st.session_state.coord_system = 'WGS-84' if 'WGS' in sel else 'GCJ-02'
    
    st.markdown("---")
    st.header("📊 系统状态")
    if st.session_state.point_a:
        st.success("🟢 A点已设")
    else:
        st.error("🔴 A点未设")
    if st.session_state.point_b:
        st.success("🟢 B点已设")
    else:
        st.error("🔴 B点未设")
    
    st.metric("障碍物数量", len(st.session_state.planner.obstacles))
    st.metric("当前航线点数", len(st.session_state.waypoints))

# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title("🚁 MAVLink 地面站 - 双策略3D避障系统")
    st.caption("网格A*水平绕行 | 爬升飞越 | 旋转矩形 | 多坐标系支持")
    
    with st.expander("📖 使用说明", expanded=False):
        st.markdown("""
        ### 🎯 操作步骤：
        1. **设置A/B点**：选择坐标系，输入起点终点
        2. **添加障碍物**：地图上绘制或参数设置旋转矩形
        3. **选择策略**：规划水平绕行或爬升飞越路径
        
        ### 🚁 双策略说明：
        - **🔵 水平绕行**：12米精度网格A*，强制绕行障碍物（距离较长）
        - **🟢 爬升飞越**：爬升到障碍物上方，直线飞过（距离较短）
        """)
    
    col_map, col_ctrl = st.columns([3, 2])
    
    with col_map:
        st.subheader("🗺️ 地图")
        
        if st.session_state.point_a and st.session_state.point_b:
            center = [(st.session_state.point_a[0]+st.session_state.point_b[0])/2,
                     (st.session_state.point_a[1]+st.session_state.point_b[1])/2]
        else:
            center = st.session_state.map_center
        
        m = folium.Map(location=center, zoom_start=16, tiles="CartoDB positron")
        
        Draw(
            draw_options={
                'polyline': False,
                'rectangle': True,
                'polygon': True,
                'circle': True,
                'marker': False,
                'circlemarker': False
            },
            edit_options={'edit': True, 'remove': True}
        ).add_to(m)
        
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='卫星影像',
            overlay=False,
            control=True
        ).add_to(m)
        
        # 显示A/B点
        if st.session_state.point_a:
            folium.Marker(st.session_state.point_a, 
                         popup="起点A",
                         icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
            folium.Circle(st.session_state.point_a, radius=8, color='green', fill=True).add_to(m)
        
        if st.session_state.point_b:
            folium.Marker(st.session_state.point_b,
                         popup="终点B", 
                         icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
            folium.Circle(st.session_state.point_b, radius=8, color='red', fill=True).add_to(m)
        
        # 显示障碍物
        for i, obs in enumerate(st.session_state.planner.obstacles):
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
                    color=color, weight=1, dash_array='5,5'
                ).add_to(m)
            else:
                folium.Circle(
                    [obs.center_lat, obs.center_lon],
                    radius=obs.radius,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillOpacity=0.4
                ).add_to(m)
        
        # 显示选中的路径
        if st.session_state.selected_path_type == 'horizontal' and st.session_state.planned_path_horizontal:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path_horizontal]
            AntPath(path_coords, color='blue', weight=5, opacity=0.9, 
                   dash_array=[10, 20], delay=500).add_to(m)
            for wp in st.session_state.planned_path_horizontal:
                folium.CircleMarker([wp.lat, wp.lon], radius=4, color='blue', fill=True,
                                   popup=f'高度:{wp.alt}m').add_to(m)
        
        elif st.session_state.selected_path_type == 'climb' and st.session_state.planned_path_climb:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.planned_path_climb]
            AntPath(path_coords, color='green', weight=5, opacity=0.9,
                   dash_array=[10, 20], delay=500).add_to(m)
            for wp in st.session_state.planned_path_climb:
                color = 'darkgreen' if wp.alt > st.session_state.flight_altitude + 5 else 'green'
                folium.CircleMarker([wp.lat, wp.lon], radius=5, color=color, fill=True,
                                   popup=f'高度:{wp.alt}m').add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            shape_id = f"{drawing.get('type')}_{id(drawing)}"
            
            if st.session_state.get("last_shape_id") != shape_id:
                st.session_state["last_shape_id"] = shape_id
                geom_type = drawing.get("type")
                
                if geom_type == "circle":
                    center = drawing["geometry"]["coordinates"]
                    radius = drawing["properties"]["radius"]
                    st.session_state.pending_drawing = {
                        'type': 'circle',
                        'center': (center[1], center[0]),
                        'radius': radius
                    }
                    st.rerun()
                elif geom_type in ["polygon", "rectangle"]:
                    coords = drawing["geometry"]["coordinates"][0]
                    points = [(c[1], c[0]) for c in coords[:-1]]
                    st.session_state.pending_drawing = {
                        'type': 'polygon',
                        'points': points
                    }
                    st.rerun()
    
    with col_ctrl:
        st.subheader("⚙️ 控制面板")
        
        # A点设置
        st.markdown("**🟢 起点 A**")
        st.caption(f"坐标系: {st.session_state.coord_system}")
        c1, c2 = st.columns(2)
        
        default_lat_a = 32.0603
        default_lon_a = 118.7969
        if st.session_state.point_a:
            if st.session_state.coord_system == 'GCJ-02':
                lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_a[1], st.session_state.point_a[0])
                default_lat_a, default_lon_a = lat_gcj, lon_gcj
            else:
                default_lat_a, default_lon_a = st.session_state.point_a
        
        lat_a = c1.number_input("纬度", value=default_lat_a, format="%.6f", key="lat_a")
        lon_a = c2.number_input("经度", value=default_lon_a, format="%.6f", key="lon_a")
        
        if st.button("✅ 设置A点", key="set_a"):
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
                st.session_state.point_a = (lat_wgs, lon_wgs)
                st.session_state.point_a_gcj = (lat_a, lon_a)
            else:
                st.session_state.point_a = (lat_a, lon_a)
            st.success("A点已设置")
            st.rerun()
        
        # B点设置
        st.markdown("**🔴 终点 B**")
        c3, c4 = st.columns(2)
        
        default_lat_b = 32.0703
        default_lon_b = 118.8069
        if st.session_state.point_b:
            if st.session_state.coord_system == 'GCJ-02':
                lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_b[1], st.session_state.point_b[0])
                default_lat_b, default_lon_b = lat_gcj, lon_gcj
            else:
                default_lat_b, default_lon_b = st.session_state.point_b
        
        lat_b = c3.number_input("纬度", value=default_lat_b, format="%.6f", key="lat_b")
        lon_b = c4.number_input("经度", value=default_lon_b, format="%.6f", key="lon_b")
        
        if st.button("✅ 设置B点", key="set_b"):
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
                st.session_state.point_b = (lat_wgs, lon_wgs)
                st.session_state.point_b_gcj = (lat_b, lon_b)
            else:
                st.session_state.point_b = (lat_b, lon_b)
            st.success("B点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("**✈️ 飞行参数**")
        new_alt = st.slider("设定飞行高度(m)", 10, 100, st.session_state.flight_altitude, key="flight_alt")
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
        
        max_alt = st.slider("最大允许高度(m)", st.session_state.flight_altitude + 10, 200, 
                           st.session_state.max_altitude, key="max_alt")
        if max_alt != st.session_state.max_altitude:
            st.session_state.max_altitude = max_alt
        
        st.info(f"飞行高度: **{st.session_state.flight_altitude}m** | 最大: **{st.session_state.max_altitude}m**")
        
        st.markdown("---")
        
        # 障碍物管理
        st.markdown("**🚧 障碍物管理**")
        
        if st.session_state.pending_drawing:
            drawing = st.session_state.pending_drawing
            
            if drawing['type'] == 'circle':
                st.success(f"⭕ 圆形: 半径{drawing['radius']:.1f}m")
            else:
                st.success(f"📐 多边形: {len(drawing['points'])}顶点")
            
            obs_height = st.number_input("障碍物高度(m)", 5, 200, 40, key="obs_h")
            
            col_add, col_cancel = st.columns(2)
            with col_add:
                if st.button("✅ 确认添加", type="primary"):
                    if drawing['type'] == 'circle':
                        st.session_state.planner.add_circle_obstacle(
                            drawing['center'][0], drawing['center'][1],
                            drawing['radius'], obs_height, f"圆形({obs_height}m)"
                        )
                    else:
                        st.session_state.planner.add_polygon_obstacle(
                            drawing['points'], obs_height, f"多边形({obs_height}m)"
                        )
                    st.session_state.pending_drawing = None
                    st.success("✅ 已添加")
                    st.rerun()
            
            with col_cancel:
                if st.button("❌ 取消"):
                    st.session_state.pending_drawing = None
                    st.rerun()
            
            st.markdown("---")
        
        # 旋转矩形
        with st.expander("⬜ 添加旋转矩形"):
            default_lat = st.session_state.point_a[0] if st.session_state.point_a else st.session_state.map_center[0]
            default_lon = st.session_state.point_a[1] if st.session_state.point_a else st.session_state.map_center[1]
            
            if st.session_state.coord_system == 'GCJ-02' and st.session_state.point_a_gcj:
                default_lat = st.session_state.point_a_gcj[0]
                default_lon = st.session_state.point_a_gcj[1]
            
            rect_lat = st.number_input("中心纬度", value=default_lat, format="%.6f", key="rect_lat")
            rect_lon = st.number_input("中心经度", value=default_lon, format="%.6f", key="rect_lon")
            rect_width = st.slider("宽度(m)", 10, 200, 50, key="rect_w")
            rect_height = st.slider("长度(m)", 10, 200, 80, key="rect_h")
            rect_rotation = st.slider("旋转角度(°)", 0, 360, 0, key="rect_rot")
            rect_obs_h = st.number_input("矩形高度(m)", 5, 200, 40, key="rect_obs_h")
            
            if st.button("➕ 添加旋转矩形"):
                if st.session_state.coord_system == 'GCJ-02':
                    lon_wgs, lat_wgs = gcj02_to_wgs84(rect_lon, rect_lat)
                else:
                    lat_wgs, lon_wgs = rect_lat, rect_lon
                
                st.session_state.planner.add_rotated_rectangle_obstacle(
                    lat_wgs, lon_wgs, rect_width, rect_height,
                    rect_rotation, rect_obs_h, f"矩形({rect_obs_h}m)"
                )
                st.success(f"✅ 已添加旋转矩形({rect_rotation}°)")
                st.rerun()
        
        # 障碍物列表
        if st.session_state.planner.obstacles:
            with st.expander(f"📋 障碍物列表({len(st.session_state.planner.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.planner.obstacles):
                    icon = "⭕" if obs.type == "circle" else "⬜" if obs.type == "rectangle" else "📐"
                    rot = f"↻{obs.rotation}°" if obs.type == "rectangle" else ""
                    st.write(f"{icon} #{i+1}: {obs.name} {rot}")
                
                if st.button("🗑️ 清除全部"):
                    st.session_state.planner.clear_obstacles()
                    st.rerun()
        
        st.markdown("---")
        
        # 双策略路径规划
        can_plan = st.session_state.point_a and st.session_state.point_b
        if not can_plan:
            st.warning("⚠️ 请先设置A点和B点")
        
        st.markdown("**🧮 双策略路径规划**")
        
        col_h, col_c = st.columns(2)
        
        with col_h:
            if st.button("🔵 水平绕行", disabled=not can_plan, use_container_width=True):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                with st.spinner("网格A*规划中..."):
                    path = st.session_state.planner.plan_horizontal_avoidance(start_wp, end_wp)
                    st.session_state.planned_path_horizontal = path
                    st.session_state.selected_path_type = 'horizontal'
                    st.session_state.waypoints = path
                
                dist = sum(st.session_state.planner.haversine_distance(
                    path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon)
                    for i in range(len(path)-1))
                st.success(f"✅ 水平绕行: {len(path)}航点, {dist:.0f}m")
                st.rerun()
        
        with col_c:
            if st.button("🟢 爬升飞越", disabled=not can_plan, use_container_width=True):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                with st.spinner("规划中..."):
                    path = st.session_state.planner.plan_climb_over(start_wp, end_wp, 
                                                                    st.session_state.max_altitude)
                    if path:
                        st.session_state.planned_path_climb = path
                        st.session_state.selected_path_type = 'climb'
                        st.session_state.waypoints = path
                        
                        max_fly = max(wp.alt for wp in path)
                        dist = sum(st.session_state.planner.haversine_distance(
                            path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon)
                            for i in range(len(path)-1))
                        st.success(f"✅ 爬升飞越: 最高{max_fly}m, {dist:.0f}m")
                        st.rerun()
        
        # 路径选择
        if st.session_state.planned_path_horizontal or st.session_state.planned_path_climb:
            st.markdown("**🎯 选择使用路径**")
            options = []
            if st.session_state.planned_path_horizontal:
                options.append("水平绕行")
            if st.session_state.planned_path_climb:
                options.append("爬升飞越")
            
            selected = st.radio("显示路径", options, horizontal=True,
                              index=0 if st.session_state.selected_path_type == 'horizontal' else 
                                    (1 if st.session_state.selected_path_type == 'climb' and len(options) > 1 else 0))
            
            new_type = 'horizontal' if selected == "水平绕行" else 'climb'
            if new_type != st.session_state.selected_path_type:
                st.session_state.selected_path_type = new_type
                st.session_state.waypoints = (st.session_state.planned_path_horizontal if new_type == 'horizontal' 
                                             else st.session_state.planned_path_climb)
                st.rerun()
            
            if st.button("📡 上传到飞控", type="primary"):
                st.session_state.mission_sent = True
                st.success(f"已上传 {len(st.session_state.waypoints)} 个航点")
                st.balloons()

# ==================== 飞行监控页面 ====================
elif page == "🛰️ 飞行监控":
    st.title("🛰️ 飞行监控")
    
    if not st.session_state.mission_sent:
        st.warning("请先规划并上传航线")
    else:
        col1, col2, col3 = st.columns(3)
        
        with col1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始任务", type="primary", use_container_width=True):
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
                st.button("▶️ 开始任务", disabled=True, use_container_width=True)
        
        with col2:
            if st.button("⏹️ 停止任务", use_container_width=True):
                st.session_state.mission_executing = False
                st.rerun()
        
        with col3:
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
                st.write(f"航点进度: {curr+1}/{total} ({prog}%)")
                
                if curr < total:
                    curr_alt = st.session_state.waypoints[curr].alt
                    st.metric("当前高度", f"{curr_alt}m")
            
            center = st.session_state.drone_position if st.session_state.drone_position else st.session_state.map_center
            m = folium.Map(location=center, zoom_start=17, tiles="CartoDB dark_matter")
            
            if st.session_state.waypoints:
                full_path = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
                folium.PolyLine(full_path, color='gray', weight=2, opacity=0.5, dash_array='5,10').add_to(m)
            
            if len(st.session_state.flight_path_history) > 1:
                folium.PolyLine(st.session_state.flight_path_history, color='lime', weight=4).add_to(m)
            
            if st.session_state.drone_position:
                folium.Marker(st.session_state.drone_position,
                            icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
                            popup="无人机").add_to(m)
            
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
                            st.success("🎉 任务完成！")
                            st.session_state.mission_executing = False
                    
                    time.sleep(0.1)
                    st.rerun()

# ==================== 通信日志页面 ====================
elif page == "💓 通信日志":
    st.title("💓 MAVLink通信日志")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📤 发送")
        if st.session_state.send_log:
            for log in list(st.session_state.send_log)[-10:]:
                st.text(f"{log}")
        else:
            st.info("暂无发送记录")
    
    with col2:
        st.subheader("📥 接收")
        if st.session_state.recv_log:
            for log in list(st.session_state.recv_log)[-10:]:
                st.text(f"{log}")
        else:
            st.info("暂无接收记录")

st.markdown("---")
st.caption("MAVLink GCS v5.0 | 网格A*双策略避障 | 12米精度 | 北京时间 (UTC+8)")
