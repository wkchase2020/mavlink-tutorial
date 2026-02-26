import streamlit as st
import time
import math
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, AntPath
from streamlit_folium import st_folium

# ==================== 坐标系转换工具类 ====================
class CoordinateConverter:
    """
    坐标系转换工具类
    
    重要说明：
    - folium 地图使用 WGS-84 坐标 (EPSG:4326)
    - 中国地图（高德、百度）使用 GCJ-02 坐标（火星坐标系）
    - 所有内部数据统一使用 WGS-84 (lat, lon) 格式存储
    - 用户界面根据选择进行坐标转换
    
    参数约定：
    - 所有函数使用 (lat, lon) 顺序（与地理惯例一致）
    """
    
    PI = 3.1415926535897932384626
    A = 6378245.0
    EE = 0.00669342162296594323
    
    @staticmethod
    def _transformlat(lng, lat):
        """内部转换辅助函数 - 注意参数是经度、纬度"""
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * CoordinateConverter.PI) + 40.0 * math.sin(lat / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * CoordinateConverter.PI) + 320 * math.sin(lat * CoordinateConverter.PI / 30.0)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _transformlng(lng, lat):
        """内部转换辅助函数 - 注意参数是经度、纬度"""
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * CoordinateConverter.PI) + 40.0 * math.sin(lng / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * CoordinateConverter.PI) + 300.0 * math.sin(lng / 30.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _out_of_china(lng, lat):
        """判断是否在中国境外"""
        return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)
    
    @classmethod
    def gcj02_to_wgs84(cls, lat, lon):
        """
        GCJ-02 (火星坐标系) 转 WGS-84
        参数: (lat, lon) - 纬度, 经度
        返回: (lat, lon) - 纬度, 经度
        """
        if cls._out_of_china(lon, lat):
            return lat, lon
        
        dlat = cls._transformlat(lon - 105.0, lat - 35.0)
        dlng = cls._transformlng(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * cls.PI
        magic = math.sin(radlat)
        magic = 1 - cls.EE * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.A * (1 - cls.EE)) / (magic * sqrtmagic) * cls.PI)
        dlng = (dlng * 180.0) / (cls.A / sqrtmagic * math.cos(radlat) * cls.PI)
        mglat = lat + dlat
        mglng = lon + dlng
        return lat * 2 - mglat, lon * 2 - mglng
    
    @classmethod
    def wgs84_to_gcj02(cls, lat, lon):
        """
        WGS-84 转 GCJ-02 (火星坐标系)
        参数: (lat, lon) - 纬度, 经度
        返回: (lat, lon) - 纬度, 经度
        """
        if cls._out_of_china(lon, lat):
            return lat, lon
        
        dlat = cls._transformlat(lon - 105.0, lat - 35.0)
        dlng = cls._transformlng(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * cls.PI
        magic = math.sin(radlat)
        magic = 1 - cls.EE * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.A * (1 - cls.EE)) / (magic * sqrtmagic) * cls.PI)
        dlng = (dlng * 180.0) / (cls.A / sqrtmagic * math.cos(radlat) * cls.PI)
        mglat = lat + dlat
        mglng = lon + dlng
        return mglat, mglng
    
    @classmethod
    def to_map_display(cls, lat, lon, coord_system='WGS-84'):
        """
        将内部 WGS-84 坐标转换为地图显示坐标
        - 如果用户选择 GCJ-02，需要将 WGS-84 转为 GCJ-02 显示
        - 如果用户选择 WGS-84，直接显示
        
        注意：folium 内部使用 WGS-84，但如果用户期望看到 GCJ-02 坐标，我们需要转换
        """
        if coord_system == 'GCJ-02':
            # 内部 WGS-84 -> 显示 GCJ-02
            return cls.wgs84_to_gcj02(lat, lon)
        return lat, lon
    
    @classmethod
    def from_user_input(cls, lat, lon, coord_system='WGS-84'):
        """
        将用户输入坐标转换为内部 WGS-84 坐标
        - 如果用户输入 GCJ-02，需要转为 WGS-84 存储
        - 如果用户输入 WGS-84，直接存储
        """
        if coord_system == 'GCJ-02':
            # 输入 GCJ-02 -> 内部 WGS-84
            return cls.gcj02_to_wgs84(lat, lon)
        return lat, lon
    
    @classmethod
    def from_map_drawing(cls, lat, lon):
        """
        处理地图绘制返回的坐标
        
        重要：folium/Leaflet.Draw 返回的是 WGS-84 坐标
        但如果用户在中国使用，可能需要考虑底图偏移问题
        
        实际上，folium 使用的是标准 WGS-84，不需要转换
        但如果用户期望输入是 GCJ-02，我们需要将 WGS-84 转为 GCJ-02 再转回来？
        
        简化处理：地图绘制总是返回 WGS-84，直接存储
        用户选择坐标系只影响手动输入的坐标
        """
        return lat, lon


# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 智能避障系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

def get_local_time():
    return datetime.utcnow() + timedelta(hours=8)


# ==================== 几何工具函数 ====================
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

def lines_intersect(p1, p2, p3, p4):
    """检查两条线段是否相交"""
    def ccw(A, B, C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    
    A, B = (p1[1], p1[0]), (p2[1], p2[0])
    C, D = (p3[1], p3[0]), (p4[1], p4[0])
    
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def line_intersects_polygon(p1, p2, polygon):
    """检查线段是否与多边形任何边相交"""
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i+1) % n]
        if lines_intersect(p1, p2, p3, p4):
            return True
    return False

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
    """绕中心点旋转"""
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = x - cx
    dy = y - cy
    new_dx = dx * cos_a - dy * sin_a
    new_dy = dx * sin_a + dy * cos_a
    return cx + new_dx, cy + new_dy

def create_rotated_rectangle(center_lat, center_lon, width_m, height_m, rotation_deg):
    """创建旋转矩形"""
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
        
        if obs_type in ["polygon", "rectangle"] and len(self.points) > 0:
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
        
        # 检查点是否在多边形内
        if point_in_polygon(lat, lon, self.points):
            return True
        
        # 如果设置了边距，检查点到各边的距离
        if margin > 0:
            n = len(self.points)
            for i in range(n):
                p1 = self.points[i]
                p2 = self.points[(i+1) % n]
                if point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1]) < margin:
                    return True
        return False
    
    def line_intersects(self, p1, p2):
        """检查线段是否与障碍物相交"""
        if self.type == "circle":
            num_samples = 20
            for i in range(num_samples + 1):
                t = i / num_samples
                lat = p1[0] + (p2[0] - p1[0]) * t
                lon = p1[1] + (p2[1] - p1[1]) * t
                if self.is_inside(lat, lon):
                    return True
            return False
        
        return line_intersects_polygon(p1, p2, self.points)


class GridPathPlanner:
    """增强版网格A*路径规划器"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 40
        self.grid_size = 8
        self.max_iterations = 20000
    
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
    
    def get_max_obstacle_height(self):
        """获取最高障碍物高度"""
        if not self.obstacles:
            return 0
        return max(obs.height for obs in self.obstacles)
    
    def should_force_avoidance(self, flight_alt):
        """判断是否需要强制绕行"""
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                return True
        return False
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间的球面距离"""
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def is_collision(self, lat, lon, flight_alt):
        """检查是否碰撞（严格模式）"""
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
            elif flight_alt < obs.height + 15:
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
        return False
    
    def line_hits_obstacle(self, p1, p2, flight_alt):
        """精确检测线段是否与任何障碍物相交"""
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                if obs.line_intersects(p1, p2):
                    return True
                mid_lat = (p1[0] + p2[0]) / 2
                mid_lon = (p1[1] + p2[1]) / 2
                if obs.is_inside(mid_lat, mid_lon, self.safety_margin):
                    return True
            elif flight_alt < obs.height + 15:
                if obs.line_intersects(p1, p2):
                    return True
                mid_lat = (p1[0] + p2[0]) / 2
                mid_lon = (p1[1] + p2[1]) / 2
                if obs.is_inside(mid_lat, mid_lon, self.safety_margin):
                    return True
        return False
    
    def latlon_to_grid(self, lat, lon, base_lat, base_lon):
        """将经纬度转换为网格坐标"""
        dlat = (lat - base_lat) * 111000
        dlon = (lon - base_lon) * 111000 * math.cos(math.radians(base_lat))
        return (round(dlon / self.grid_size), round(dlat / self.grid_size))
    
    def grid_to_latlon(self, grid_x, grid_y, base_lat, base_lon):
        """将网格坐标转换为经纬度"""
        lon = base_lon + (grid_x * self.grid_size) / (111000 * math.cos(math.radians(base_lat)))
        lat = base_lat + (grid_y * self.grid_size) / 111000
        return (lat, lon)
    
    def plan_horizontal_avoidance(self, start_wp, end_wp):
        """强制水平绕行路径规划"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        flight_alt = start_wp.alt
        
        if self.is_collision(start[0], start[1], flight_alt):
            st.error("起点在障碍物内或安全边界内，请调整起点位置")
            return None
        if self.is_collision(end[0], end[1], flight_alt):
            st.error("终点在障碍物内或安全边界内，请调整终点位置")
            return None
        
        direct_distance = self.haversine_distance(start[0], start[1], end[0], end[1])
        
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [start_wp, end_wp]
        
        margin_deg = max(0.008, min(0.02, direct_distance / 100000))
        
        lat_min = min(start[0], end[0]) - margin_deg
        lat_max = max(start[0], end[0]) + margin_deg
        lon_min = min(start[1], end[1]) - margin_deg
        lon_max = max(start[1], end[1]) + margin_deg
        
        base_lat = lat_min
        base_lon = lon_min
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        directions = [
            (0,1), (1,0), (0,-1), (-1,0),
            (1,1), (1,-1), (-1,1), (-1,-1),
            (0,2), (2,0), (0,-2), (-2,0),
            (2,2), (2,-2), (-2,2), (-2,-2),
        ]
        
        open_set = [(0, 0, start_grid[0], start_grid[1], [start_grid])]
        visited = {}
        
        iteration = 0
        best_path = None
        best_dist = float('inf')
        
        while open_set and iteration < self.max_iterations:
            iteration += 1
            f_cost, g_cost, x, y, path = heapq.heappop(open_set)
            
            if abs(x - end_grid[0]) <= 2 and abs(y - end_grid[1]) <= 2:
                waypoints = [start_wp]
                for grid in path[1:]:
                    lat, lon = self.grid_to_latlon(grid[0], grid[1], base_lat, base_lon)
                    waypoints.append(Waypoint(lat, lon, flight_alt, 16, len(waypoints)))
                
                waypoints.append(end_wp)
                waypoints[-1].seq = len(waypoints) - 1
                
                waypoints = self.smooth_path(waypoints, flight_alt)
                
                current_dist = sum(self.haversine_distance(
                    waypoints[i].lat, waypoints[i].lon, 
                    waypoints[i+1].lat, waypoints[i+1].lon) for i in range(len(waypoints)-1))
                
                if current_dist < best_dist:
                    best_dist = current_dist
                    best_path = waypoints
                
                if iteration > 5000:
                    break
                continue
            
            key = (x, y)
            if key in visited and visited[key] <= g_cost:
                continue
            visited[key] = g_cost
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                new_key = (nx, ny)
                
                lat, lon = self.grid_to_latlon(nx, ny, base_lat, base_lon)
                
                if not (lat_min <= lat <= lat_max and lon_min <= lon <= lon_max):
                    continue
                
                if self.is_collision(lat, lon, flight_alt):
                    continue
                
                curr_lat, curr_lon = self.grid_to_latlon(x, y, base_lat, base_lon)
                if self.line_hits_obstacle((curr_lat, curr_lon), (lat, lon), flight_alt):
                    continue
                
                move_cost = math.sqrt(dx**2 + dy**2) * self.grid_size
                new_g_cost = g_cost + move_cost
                
                if new_key in visited and visited[new_key] <= new_g_cost:
                    continue
                
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                
                heapq.heappush(open_set, (new_g_cost + h, new_g_cost, nx, ny, path + [(nx, ny)]))
        
        if best_path is not None:
            return best_path
        
        st.error("无法找到可行的绕行路径，请检查障碍物设置或调整飞行高度")
        return None
    
    def smooth_path(self, waypoints, flight_alt):
        """路径平滑：移除不必要的中间点"""
        if len(waypoints) <= 2:
            return waypoints
        
        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            j = len(waypoints) - 1
            while j > i + 1:
                p1 = (waypoints[i].lat, waypoints[i].lon)
                p2 = (waypoints[j].lat, waypoints[j].lon)
                if not self.line_hits_obstacle(p1, p2, flight_alt):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j
        
        for idx, wp in enumerate(smoothed):
            wp.seq = idx
        
        return smoothed
    
    def plan_climb_over(self, start_wp, end_wp, max_altitude):
        """爬升飞越"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        
        for obs in self.obstacles:
            if obs.height >= start_wp.alt:
                st.error("存在高于飞行高度的障碍物，无法使用爬升飞越策略")
                return None
        
        max_obs_height = 0
        steps = 50
        
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            
            for obs in self.obstacles:
                if obs.is_inside(lat, lon, 0):
                    max_obs_height = max(max_obs_height, obs.height)
        
        if max_obs_height == 0:
            return [start_wp, end_wp]
        
        fly_alt = max_obs_height + 25
        if fly_alt > max_altitude:
            st.warning(f"需要飞越高度{fly_alt}m超过最大限制{max_altitude}m，无法执行爬升飞越")
            return None
        
        path = [start_wp]
        dist_total = self.haversine_distance(start[0], start[1], end[0], end[1])
        
        if dist_total > 100:
            climb_lat = start[0] + (end[0] - start[0]) * 0.2
            climb_lon = start[1] + (end[1] - start[1]) * 0.2
            path.append(Waypoint(climb_lat, climb_lon, fly_alt, 16, 1))
            
            mid_lat = (start[0] + end[0]) / 2
            mid_lon = (start[1] + end[1]) / 2
            path.append(Waypoint(mid_lat, mid_lon, fly_alt, 16, len(path)))
            
            descend_lat = start[0] + (end[0] - start[0]) * 0.8
            descend_lon = start[1] + (end[1] - start[1]) * 0.8
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
        'flight_altitude': 50,
        'max_altitude': 120,
        'current_waypoint_index': 0,
        'flight_path_history': [],
        'animation_step': 0,
        'coord_system': 'WGS-84',
        'pending_drawing': None,
        'debug_info': []
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()


# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("🧭 导航")
    page = st.radio("功能页面", ["🗺️ 航线规划", "✈️ 飞行监控", "📡 通信日志"])
    
    st.markdown("---")
    st.header("⚙️ 坐标系设置")
    coord_opt = ["WGS-84", "GCJ-02(高德/百度)"]
    sel = st.radio("输入坐标系", coord_opt, 
                   index=0 if st.session_state.coord_system=='WGS-84' else 1)
    st.session_state.coord_system = 'WGS-84' if 'WGS' in sel else 'GCJ-02'
    
    st.markdown("---")
    st.header("📊 系统状态")
    if st.session_state.point_a:
        st.success("✅ A点已设")
    else:
        st.error("❌ A点未设")
    if st.session_state.point_b:
        st.success("✅ B点已设")
    else:
        st.error("❌ B点未设")
    
    st.metric("障碍物数量", len(st.session_state.planner.obstacles))
    
    max_obs_h = st.session_state.planner.get_max_obstacle_height()
    if max_obs_h > 0:
        st.metric("最高障碍物", f"{max_obs_h}m")
        if max_obs_h >= st.session_state.flight_altitude:
            st.error("⚠️ 强制绕行模式")
    
    if st.session_state.waypoints:
        path_type = "水平绕行" if st.session_state.selected_path_type == 'horizontal' else "爬升飞越" if st.session_state.selected_path_type == 'climb' else "无"
        st.metric("当前路径类型", path_type)
        st.metric("航点数量", len(st.session_state.waypoints))


# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title("🚁 MAVLink 地面站 - 智能避障系统")
    st.caption("强制绕行避障 | 坐标系自动转换 | 智能高度判断")
    
    with st.expander("📖 坐标系说明", expanded=True):
        st.markdown("""
        ### 📍 坐标系说明
        
        **WGS-84**: 国际标准GPS坐标系，folium地图使用此坐标系
        **GCJ-02**: 中国国测局坐标系（火星坐标），高德/百度地图使用此坐标系
        
        ### ⚠️ 重要提示
        - **内部存储**: 所有数据统一使用 WGS-84 坐标存储
        - **地图显示**: folium 原生使用 WGS-84 坐标
        - **用户输入**: 根据选择的坐标系自动转换
        - **地图绘制**: 返回 WGS-84 坐标，直接存储
        """)
    
    col_map, col_ctrl = st.columns([3, 2])
    
    with col_map:
        st.subheader("🗺️ 地图")
        
        if st.session_state.point_a and st.session_state.point_b:
            center = [(st.session_state.point_a[0]+st.session_state.point_b[0])/2,
                     (st.session_state.point_a[1]+st.session_state.point_b[1])/2]
        else:
            center = st.session_state.map_center
        
        # folium 使用 WGS-84 坐标，直接显示
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
        
        # 显示A/B点 - 内部存储的是 WGS-84，直接显示
        if st.session_state.point_a:
            lat, lon = st.session_state.point_a
            folium.Marker([lat, lon], 
                         popup=f"起点A<br>WGS84: {lat:.6f}, {lon:.6f}<br>GCJ02: {CoordinateConverter.wgs84_to_gcj02(lat, lon)[0]:.6f}, {CoordinateConverter.wgs84_to_gcj02(lat, lon)[1]:.6f}",
                         icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
            folium.Circle([lat, lon], radius=8, color='green', fill=True, fillOpacity=0.3).add_to(m)
        
        if st.session_state.point_b:
            lat, lon = st.session_state.point_b
            folium.Marker([lat, lon],
                         popup=f"终点B<br>WGS84: {lat:.6f}, {lon:.6f}<br>GCJ02: {CoordinateConverter.wgs84_to_gcj02(lat, lon)[0]:.6f}, {CoordinateConverter.wgs84_to_gcj02(lat, lon)[1]:.6f}", 
                         icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
            folium.Circle([lat, lon], radius=8, color='red', fill=True, fillOpacity=0.3).add_to(m)
        
        # 显示障碍物 - 内部存储 WGS-84，直接显示
        for i, obs in enumerate(st.session_state.planner.obstacles):
            color = 'red' if obs.height >= st.session_state.flight_altitude else 'orange'
            
            if obs.type in ["polygon", "rectangle"]:
                # points 存储为 (lat, lon) 列表
                folium.Polygon(
                    locations=obs.points,
                    popup=f"{obs.name}<br>高度:{obs.height}m<br>类型:{obs.type}",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.5,
                    weight=3
                ).add_to(m)
            else:
                folium.Circle(
                    [obs.center_lat, obs.center_lon],
                    radius=obs.radius,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillOpacity=0.5
                ).add_to(m)
        
        # 显示选中的路径
        if st.session_state.selected_path_type and st.session_state.waypoints:
            path_coords = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
            
            if st.session_state.selected_path_type == 'horizontal':
                AntPath(path_coords, color='blue', weight=6, opacity=0.9, 
                       dash_array=[15, 10], delay=800).add_to(m)
                for i, wp in enumerate(st.session_state.waypoints):
                    color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints)-1 else 'blue'
                    folium.CircleMarker([wp.lat, wp.lon], radius=4, color=color, fill=True,
                                       popup=f'航点{i}<br>高度:{wp.alt}m').add_to(m)
            
            elif st.session_state.selected_path_type == 'climb':
                AntPath(path_coords, color='green', weight=6, opacity=0.9,
                       dash_array=[15, 10], delay=800).add_to(m)
                for i, wp in enumerate(st.session_state.waypoints):
                    color = 'darkgreen' if wp.alt > st.session_state.flight_altitude + 5 else 'green'
                    folium.CircleMarker([wp.lat, wp.lon], radius=5, color=color, fill=True,
                                       popup=f'航点{i}<br>高度:{wp.alt}m').add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        # 处理地图绘制 - folium 返回的是 WGS-84 坐标，直接存储
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            shape_id = f"{drawing.get('type')}_{id(drawing)}"
            
            if st.session_state.get("last_shape_id") != shape_id:
                st.session_state["last_shape_id"] = shape_id
                geom_type = drawing.get("type")
                
                if geom_type == "circle":
                    # folium 返回 [lon, lat] 格式
                    center = drawing["geometry"]["coordinates"]
                    radius = drawing["properties"]["radius"]
                    # 转换为 (lat, lon) 存储
                    st.session_state.pending_drawing = {
                        'type': 'circle',
                        'center': (center[1], center[0]),  # (lat, lon)
                        'radius': radius
                    }
                    st.rerun()
                elif geom_type in ["polygon", "rectangle"]:
                    # folium 返回 [[lon, lat], ...] 格式
                    coords = drawing["geometry"]["coordinates"][0]
                    # 转换为 [(lat, lon), ...] 存储
                    points = [(c[1], c[0]) for c in coords[:-1]]  # 最后一个点是重复的
                    st.session_state.pending_drawing = {
                        'type': 'polygon',
                        'points': points
                    }
                    st.rerun()
    
    with col_ctrl:
        st.subheader("⚙️ 控制面板")
        
        # A点设置
        st.markdown("**📍 起点 A**")
        st.caption(f"输入坐标系: {st.session_state.coord_system}")
        c1, c2 = st.columns(2)
        
        # 显示默认值：如果已设置，显示对应坐标系的值
        default_lat_a, default_lon_a = 32.0603, 118.7969
        if st.session_state.point_a:
            lat_wgs, lon_wgs = st.session_state.point_a
            if st.session_state.coord_system == 'GCJ-02':
                lat_gcj, lon_gcj = CoordinateConverter.wgs84_to_gcj02(lat_wgs, lon_wgs)
                default_lat_a, default_lon_a = lat_gcj, lon_gcj
            else:
                default_lat_a, default_lon_a = lat_wgs, lon_wgs
        
        lat_a = c1.number_input("纬度", value=default_lat_a, format="%.6f", key="lat_a")
        lon_a = c2.number_input("经度", value=default_lon_a, format="%.6f", key="lon_a")
        
        if st.button("✅ 设置A点", key="set_a"):
            # 将用户输入转换为 WGS-84 存储
            lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat_a, lon_a, st.session_state.coord_system)
            st.session_state.point_a = (lat_wgs, lon_wgs)
            st.success(f"A点已设置 (WGS84: {lat_wgs:.6f}, {lon_wgs:.6f})")
            st.rerun()
        
        # B点设置
        st.markdown("**📍 终点 B**")
        c3, c4 = st.columns(2)
        
        default_lat_b, default_lon_b = 32.0703, 118.8069
        if st.session_state.point_b:
            lat_wgs, lon_wgs = st.session_state.point_b
            if st.session_state.coord_system == 'GCJ-02':
                lat_gcj, lon_gcj = CoordinateConverter.wgs84_to_gcj02(lat_wgs, lon_wgs)
                default_lat_b, default_lon_b = lat_gcj, lon_gcj
            else:
                default_lat_b, default_lon_b = lat_wgs, lon_wgs
        
        lat_b = c3.number_input("纬度", value=default_lat_b, format="%.6f", key="lat_b")
        lon_b = c4.number_input("经度", value=default_lon_b, format="%.6f", key="lon_b")
        
        if st.button("✅ 设置B点", key="set_b"):
            lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat_b, lon_b, st.session_state.coord_system)
            st.session_state.point_b = (lat_wgs, lon_wgs)
            st.success(f"B点已设置 (WGS84: {lat_wgs:.6f}, {lon_wgs:.6f})")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("**✈️ 飞行参数**")
        new_alt = st.slider("设定飞行高度(m)", 10, 200, st.session_state.flight_altitude, key="flight_alt")
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
            st.rerun()
        
        max_alt = st.slider("最大允许高度(m)", st.session_state.flight_altitude + 10, 300, 
                           st.session_state.max_altitude, key="max_alt")
        if max_alt != st.session_state.max_altitude:
            st.session_state.max_altitude = max_alt
        
        # 高度对比和警告
        max_obs_h = st.session_state.planner.get_max_obstacle_height()
        if max_obs_h > 0:
            st.markdown("---")
            st.markdown("**📊 高度分析**")
            col_h1, col_h2 = st.columns(2)
            col_h1.metric("飞行高度", f"{st.session_state.flight_altitude}m")
            col_h2.metric("最高障碍", f"{max_obs_h}m")
            
            if max_obs_h >= st.session_state.flight_altitude:
                st.error("🔒 障碍物高于飞行高度！强制使用水平绕行")
            elif max_obs_h >= st.session_state.flight_altitude - 15:
                st.warning("⚠️ 障碍物接近飞行高度，建议绕行")
        
        st.markdown("---")
        
        # 障碍物管理
        st.markdown("**🧱 障碍物管理**")
        
        if st.session_state.pending_drawing:
            drawing = st.session_state.pending_drawing
            
            if drawing['type'] == 'circle':
                st.success(f"⭕ 圆形障碍物: 半径{drawing['radius']:.1f}m")
            else:
                area = 0
                if len(drawing['points']) > 2:
                    pts = drawing['points']
                    for i in range(len(pts)):
                        j = (i+1) % len(pts)
                        area += pts[i][0] * pts[j][1]
                        area -= pts[j][0] * pts[i][1]
                    area = abs(area) * 111000 * 111000 / 2
                st.success(f"📐 多边形: {len(drawing['points'])}顶点, 面积约{area:.0f}m²")
            
            obs_height = st.number_input("障碍物高度(m)", 5, 300, 40, key="obs_h")
            
            col_add, col_cancel = st.columns(2)
            with col_add:
                if st.button("✅ 确认添加", type="primary"):
                    if drawing['type'] == 'circle':
                        lat, lon = drawing['center']
                        st.session_state.planner.add_circle_obstacle(
                            lat, lon, drawing['radius'], obs_height, f"圆形({obs_height}m)"
                        )
                    else:
                        st.session_state.planner.add_polygon_obstacle(
                            drawing['points'], obs_height, f"多边形({obs_height}m)"
                        )
                    st.session_state.pending_drawing = None
                    st.success("✅ 障碍物已添加")
                    st.rerun()
            
            with col_cancel:
                if st.button("❌ 取消"):
                    st.session_state.pending_drawing = None
                    st.rerun()
            
            st.markdown("---")
        
        # 旋转矩形（参数化输入）
        with st.expander("⬜ 添加旋转矩形（参数化）"):
            default_lat, default_lon = 32.0603, 118.7969
            if st.session_state.point_a:
                lat_wgs, lon_wgs = st.session_state.point_a
                if st.session_state.coord_system == 'GCJ-02':
                    lat_gcj, lon_gcj = CoordinateConverter.wgs84_to_gcj02(lat_wgs, lon_wgs)
                    default_lat, default_lon = lat_gcj, lon_gcj
                else:
                    default_lat, default_lon = lat_wgs, lon_wgs
            
            rect_lat = st.number_input("中心纬度", value=default_lat, format="%.6f", key="rect_lat")
            rect_lon = st.number_input("中心经度", value=default_lon, format="%.6f", key="rect_lon")
            rect_width = st.slider("宽度(m)", 10, 300, 60, key="rect_w")
            rect_height = st.slider("长度(m)", 10, 300, 100, key="rect_h")
            rect_rotation = st.slider("旋转角度(°)", 0, 360, 45, key="rect_rot")
            rect_obs_h = st.number_input("矩形高度(m)", 5, 300, 60, key="rect_obs_h")
            
            if st.button("➕ 添加旋转矩形"):
                lat_wgs, lon_wgs = CoordinateConverter.from_user_input(rect_lat, rect_lon, st.session_state.coord_system)
                
                st.session_state.planner.add_rotated_rectangle_obstacle(
                    lat_wgs, lon_wgs, rect_width, rect_height,
                    rect_rotation, rect_obs_h, f"矩形({rect_obs_h}m)"
                )
                st.success(f"✅ 已添加旋转矩形 {rect_width}m×{rect_height}m @ {rect_rotation}°")
                st.rerun()
        
        # 障碍物列表
        if st.session_state.planner.obstacles:
            with st.expander(f"📋 障碍物列表({len(st.session_state.planner.obstacles)}个)", expanded=True):
                for i, obs in enumerate(st.session_state.planner.obstacles):
                    icon = "⭕" if obs.type == "circle" else "⬜" if obs.type == "rectangle" else "📐"
                    rot = f"↻{obs.rotation}°" if obs.type == "rectangle" else ""
                    is_blocking = "🔴" if obs.height >= st.session_state.flight_altitude else "🟢"
                    st.write(f"{is_blocking} {icon} #{i+1}: {obs.name} {rot} - {obs.height}m")
                
                if st.button("🗑️ 清除全部障碍物"):
                    st.session_state.planner.clear_obstacles()
                    st.session_state.planned_path_horizontal = None
                    st.session_state.planned_path_climb = None
                    st.session_state.waypoints = []
                    st.rerun()
        
        st.markdown("---")
        
        # 路径规划
        can_plan = st.session_state.point_a and st.session_state.point_b
        if not can_plan:
            st.warning("⚠️ 请先设置A点和B点")
        
        st.markdown("**🧭 路径规划**")
        
        force_avoidance = st.session_state.planner.should_force_avoidance(st.session_state.flight_altitude)
        
        col_h, col_c = st.columns(2)
        
        with col_h:
            if st.button("🔄 水平绕行", disabled=not can_plan, use_container_width=True, type="primary"):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                with st.spinner("正在规划绕行路径..."):
                    path = st.session_state.planner.plan_horizontal_avoidance(start_wp, end_wp)
                    
                    if path is not None:
                        st.session_state.planned_path_horizontal = path
                        st.session_state.selected_path_type = 'horizontal'
                        st.session_state.waypoints = path
                        
                        dist = sum(st.session_state.planner.haversine_distance(
                            path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon)
                            for i in range(len(path)-1))
                        
                        is_safe = True
                        for i in range(len(path)-1):
                            p1 = (path[i].lat, path[i].lon)
                            p2 = (path[i+1].lat, path[i+1].lon)
                            if st.session_state.planner.line_hits_obstacle(p1, p2, st.session_state.flight_altitude):
                                is_safe = False
                                break
                        
                        if is_safe:
                            st.success(f"✅ 水平绕行成功！{len(path)}个航点, {dist:.0f}m")
                        else:
                            st.error("⚠️ 路径验证失败")
                            st.session_state.waypoints = []
                    else:
                        st.error("❌ 规划失败")
                        st.session_state.planned_path_horizontal = None
                        st.session_state.waypoints = []
                
                st.rerun()
        
        with col_c:
            climb_disabled = not can_plan or force_avoidance
            climb_help = "有障碍物高于飞行高度，强制绕行" if force_avoidance else ""
            
            if st.button("⬆️ 爬升飞越", disabled=climb_disabled, use_container_width=True, help=climb_help):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                with st.spinner("规划爬升路径..."):
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
                        st.success(f"✅ 爬升飞越: 最高{max_fly}m, 总长{dist:.0f}m")
                    else:
                        st.error("❌ 爬升飞越不可行")
                    st.rerun()
        
        # 路径选择与验证
        if st.session_state.planned_path_horizontal or st.session_state.planned_path_climb:
            st.markdown("**✅ 路径选择**")
            options = []
            if st.session_state.planned_path_horizontal:
                options.append("水平绕行")
            if st.session_state.planned_path_climb:
                options.append("爬升飞越")
            
            if options:
                selected = st.radio("当前使用", options, horizontal=True,
                                  index=0 if st.session_state.selected_path_type == 'horizontal' else 
                                        (1 if st.session_state.selected_path_type == 'climb' and len(options) > 1 else 0))
                
                new_type = 'horizontal' if selected == "水平绕行" else 'climb'
                if new_type != st.session_state.selected_path_type:
                    st.session_state.selected_path_type = new_type
                    st.session_state.waypoints = (st.session_state.planned_path_horizontal if new_type == 'horizontal' 
                                                 else st.session_state.planned_path_climb)
                    st.rerun()
            
            if st.session_state.waypoints:
                unsafe_segments = []
                for i in range(len(st.session_state.waypoints)-1):
                    p1 = (st.session_state.waypoints[i].lat, st.session_state.waypoints[i].lon)
                    p2 = (st.session_state.waypoints[i+1].lat, st.session_state.waypoints[i+1].lon)
                    if st.session_state.planner.line_hits_obstacle(p1, p2, st.session_state.flight_altitude):
                        unsafe_segments.append(i)
                
                if unsafe_segments:
                    st.error(f"⚠️ 警告：航段 {unsafe_segments} 存在碰撞风险！")
                else:
                    st.success("✅ 路径安全检查通过")
            
            if st.button("📤 上传到飞控", type="primary"):
                if st.session_state.waypoints:
                    st.session_state.mission_sent = True
                    st.success(f"已上传 {len(st.session_state.waypoints)} 个航点到飞控")
                    st.balloons()
                else:
                    st.error("没有可上传的航点")


# ==================== 飞行监控页面 ====================
elif page == "✈️ 飞行监控":
    st.title("✈️ 飞行监控")
    
    if not st.session_state.mission_sent:
        st.warning("请先规划并上传航线")
    else:
        col1, col2, col3 = st.columns(3)
        
        with col1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始执行任务", type="primary", use_container_width=True):
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
                st.button("⏳ 任务执行中...", disabled=True, use_container_width=True)
        
        with col2:
            if st.button("⏹️ 紧急停止", use_container_width=True):
                st.session_state.mission_executing = False
                st.warning("任务已停止")
                st.rerun()
        
        with col3:
            if st.button("🔄 重置任务", use_container_width=True):
                st.session_state.mission_executing = False
                st.session_state.drone_position = None
                st.session_state.current_waypoint_index = 0
                st.session_state.flight_path_history = []
                st.rerun()
        
        if st.session_state.mission_executing or st.session_state.drone_position:
            total = len(st.session_state.waypoints)
            curr = st.session_state.current_waypoint_index
            
            if total > 0:
                prog = min(100, int((curr / max(1, total-1)) * 100))
                st.progress(prog)
                cols = st.columns(3)
                cols[0].metric("当前航点", f"{min(curr+1, total)}/{total}")
                cols[1].metric("完成进度", f"{prog}%")
                if curr < total:
                    cols[2].metric("目标高度", f"{st.session_state.waypoints[min(curr, total-1)].alt}m")
                
                if st.session_state.mission_executing:
                    st.info("🚁 正在执行任务...")
                else:
                    st.warning("⏸️ 任务已暂停")
            
            # 地图显示 - 直接使用 WGS-84 坐标
            center = st.session_state.drone_position if st.session_state.drone_position else [st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]
            
            m = folium.Map(location=center, zoom_start=17, tiles="CartoDB dark_matter")
            
            if st.session_state.waypoints:
                full_path = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
                folium.PolyLine(full_path, color='blue', weight=3, opacity=0.6, dash_array='5,10').add_to(m)
                
                for i, wp in enumerate(st.session_state.waypoints):
                    color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints)-1 else 'blue'
                    folium.CircleMarker([wp.lat, wp.lon], radius=4, color=color, fill=True, 
                                       popup=f'航点{i}').add_to(m)
            
            if len(st.session_state.flight_path_history) > 1:
                folium.PolyLine(st.session_state.flight_path_history, color='lime', weight=5, opacity=0.9).add_to(m)
            
            if st.session_state.drone_position:
                folium.Marker(st.session_state.drone_position,
                            icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
                            popup="无人机当前位置").add_to(m)
                folium.Circle(st.session_state.drone_position, radius=10, color='orange', fill=True, fillOpacity=0.3).add_to(m)
            
            st_folium(m, width=800, height=500)
            
            if st.session_state.mission_executing and st.session_state.drone_position and curr < total - 1:
                curr_wp = st.session_state.waypoints[curr]
                next_wp = st.session_state.waypoints[curr + 1]
                
                step = st.session_state.animation_step
                total_steps = 20
                
                if step < total_steps:
                    r = step / total_steps
                    new_lat = curr_wp.lat + (next_wp.lat - curr_wp.lat) * r
                    new_lon = curr_wp.lon + (next_wp.lon - curr_wp.lon) * r
                    st.session_state.drone_position = [new_lat, new_lon]
                    st.session_state.flight_path_history.append([new_lat, new_lon])
                    st.session_state.animation_step += 1
                else:
                    st.session_state.current_waypoint_index += 1
                    st.session_state.animation_step = 0
                    if st.session_state.current_waypoint_index >= total - 1:
                        st.success("🎉 任务执行完成！")
                        st.session_state.mission_executing = False
                
                time.sleep(0.05)
                st.rerun()


# ==================== 通信日志页面 ====================
elif page == "📡 通信日志":
    st.title("📡 MAVLink通信日志")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📤 发送日志")
        if st.session_state.send_log:
            for log in list(st.session_state.send_log)[-10:]:
                st.text(f"{log}")
        else:
            st.info("暂无发送记录")
        
        if st.button("🗑️ 清空发送日志"):
            st.session_state.send_log.clear()
            st.rerun()
    
    with col2:
        st.subheader("📥 接收日志")
        if st.session_state.recv_log:
            for log in list(st.session_state.recv_log)[-10:]:
                st.text(f"{log}")
        else:
            st.info("暂无接收记录")
        
        if st.button("🗑️ 清空接收日志"):
            st.session_state.recv_log.clear()
            st.rerun()

st.markdown("---")
st.caption("MAVLink GCS v5.3 | 智能避障 | 坐标系自动转换 | 北京时间 (UTC+8)")
