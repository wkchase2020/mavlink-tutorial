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
    """统一坐标系转换工具类，避免代码重复"""
    
    X_PI = 3.14159265358979324 * 3000.0 / 180.0
    PI = 3.1415926535897932384626
    A = 6378245.0
    EE = 0.00669342162296594323
    
    @staticmethod
    def transformlat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * CoordinateConverter.PI) + 40.0 * math.sin(lat / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * CoordinateConverter.PI) + 320 * math.sin(lat * CoordinateConverter.PI / 30.0)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def transformlng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * CoordinateConverter.PI) + 40.0 * math.sin(lng / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * CoordinateConverter.PI) + 300.0 * math.sin(lng / 30.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def out_of_china(lng, lat):
        return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)
    
    @classmethod
    def gcj02_to_wgs84(cls, lng, lat):
        """GCJ-02 (火星坐标系) 转 WGS-84"""
        if cls.out_of_china(lng, lat):
            return lng, lat
        
        dlat = cls.transformlat(lng - 105.0, lat - 35.0)
        dlng = cls.transformlng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * cls.PI
        magic = math.sin(radlat)
        magic = 1 - cls.EE * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.A * (1 - cls.EE)) / (magic * sqrtmagic) * cls.PI)
        dlng = (dlng * 180.0) / (cls.A / sqrtmagic * math.cos(radlat) * cls.PI)
        mglat = lat + dlat
        mglng = lng + dlng
        return lng * 2 - mglng, lat * 2 - mglat
    
    @classmethod
    def wgs84_to_gcj02(cls, lng, lat):
        """WGS-84 转 GCJ-02 (火星坐标系)"""
        if cls.out_of_china(lng, lat):
            return lng, lat
        
        dlat = cls.transformlat(lng - 105.0, lat - 35.0)
        dlng = cls.transformlng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * cls.PI
        magic = math.sin(radlat)
        magic = 1 - cls.EE * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((cls.A * (1 - cls.EE)) / (magic * sqrtmagic) * cls.PI)
        dlng = (dlng * 180.0) / (cls.A / sqrtmagic * math.cos(radlat) * cls.PI)
        mglat = lat + dlat
        mglng = lng + dlng
        return mglng, mglat
    
    @classmethod
    def to_display_coords(cls, lat, lon, coord_system='WGS-84'):
        """将内部 WGS-84 坐标转换为显示坐标"""
        if coord_system == 'GCJ-02':
            lon_gcj, lat_gcj = cls.wgs84_to_gcj02(lon, lat)
            return [lat_gcj, lon_gcj]
        return [lat, lon]
    
    @classmethod
    def from_input_coords(cls, lat, lon, coord_system='WGS-84'):
        """将输入坐标转换为内部 WGS-84 坐标"""
        if coord_system == 'GCJ-02':
            lon_wgs, lat_wgs = cls.gcj02_to_wgs84(lon, lat)
            return lat_wgs, lon_wgs
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
            # 对圆形进行采样检测
            num_samples = 20
            for i in range(num_samples + 1):
                t = i / num_samples
                lat = p1[0] + (p2[0] - p1[0]) * t
                lon = p1[1] + (p2[1] - p1[1]) * t
                if self.is_inside(lat, lon):
                    return True
            return False
        
        # 检查线段是否与多边形任何边相交
        return line_intersects_polygon(p1, p2, self.points)


class GridPathPlanner:
    """增强版网格A*路径规划器"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 40  # 增加安全边距到40米
        self.grid_size = 8  # 减小网格到8米，提高精度
        self.max_iterations = 20000  # 增加最大迭代次数
    
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
        """判断是否需要强制绕行（有障碍物高于飞行高度）"""
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
            # 【关键修复】如果障碍物高度 >= 飞行高度，必须绕行
            if obs.height >= flight_alt:
                # 严格检查，包括安全边距
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
            # 如果障碍物低于飞行高度，但飞行高度不够高（低于障碍物+15米），也检查
            elif flight_alt < obs.height + 15:
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
        return False
    
    def line_hits_obstacle(self, p1, p2, flight_alt):
        """精确检测线段是否与任何障碍物相交"""
        for obs in self.obstacles:
            # 【关键修复】如果障碍物高度 >= 飞行高度，必须绕行
            if obs.height >= flight_alt:
                # 检查线段是否与障碍物边界相交
                if obs.line_intersects(p1, p2):
                    return True
                # 检查中点是否在障碍物内
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
        
        # 检查起点和终点
        if self.is_collision(start[0], start[1], flight_alt):
            st.error("起点在障碍物内或安全边界内，请调整起点位置")
            return None
        if self.is_collision(end[0], end[1], flight_alt):
            st.error("终点在障碍物内或安全边界内，请调整终点位置")
            return None
        
        # 计算起点到终点的距离
        direct_distance = self.haversine_distance(start[0], start[1], end[0], end[1])
        
        # 检查直线路径
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [start_wp, end_wp]
        
        # 【关键修复】扩大搜索范围，根据距离动态调整
        margin_deg = max(0.008, min(0.02, direct_distance / 100000))  # 根据距离调整，至少800米
        
        lat_min = min(start[0], end[0]) - margin_deg
        lat_max = max(start[0], end[0]) + margin_deg
        lon_min = min(start[1], end[1]) - margin_deg
        lon_max = max(start[1], end[1]) + margin_deg
        
        base_lat = lat_min
        base_lon = lon_min
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        # 【关键修复】使用16方向搜索（增加斜向和更远距离）
        directions = [
            (0,1), (1,0), (0,-1), (-1,0),  # 4正方向
            (1,1), (1,-1), (-1,1), (-1,-1),  # 4对角方向
            (0,2), (2,0), (0,-2), (-2,0),  # 远距离正方向
            (2,2), (2,-2), (-2,2), (-2,-2),  # 远距离对角方向
        ]
        
        # A*算法
        open_set = [(0, 0, start_grid[0], start_grid[1], [start_grid])]
        visited = {}  # 使用字典存储最佳代价
        
        iteration = 0
        best_path = None
        best_dist = float('inf')
        
        while open_set and iteration < self.max_iterations:
            iteration += 1
            f_cost, g_cost, x, y, path = heapq.heappop(open_set)
            
            # 检查是否到达终点（允许2个网格误差）
            if abs(x - end_grid[0]) <= 2 and abs(y - end_grid[1]) <= 2:
                waypoints = [start_wp]
                for grid in path[1:]:
                    lat, lon = self.grid_to_latlon(grid[0], grid[1], base_lat, base_lon)
                    waypoints.append(Waypoint(lat, lon, flight_alt, 16, len(waypoints)))
                
                waypoints.append(end_wp)
                waypoints[-1].seq = len(waypoints) - 1
                
                # 路径平滑
                waypoints = self.smooth_path(waypoints, flight_alt)
                
                current_dist = sum(self.haversine_distance(
                    waypoints[i].lat, waypoints[i].lon, 
                    waypoints[i+1].lat, waypoints[i+1].lon) for i in range(len(waypoints)-1))
                
                if current_dist < best_dist:
                    best_dist = current_dist
                    best_path = waypoints
                
                # 找到一条路径后继续搜索，看是否有更短的
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
                
                # 检查网格点是否安全
                lat, lon = self.grid_to_latlon(nx, ny, base_lat, base_lon)
                
                # 边界检查
                if not (lat_min <= lat <= lat_max and lon_min <= lon <= lon_max):
                    continue
                
                if self.is_collision(lat, lon, flight_alt):
                    continue
                
                # 检查从当前点到新点的路径是否穿越障碍物
                curr_lat, curr_lon = self.grid_to_latlon(x, y, base_lat, base_lon)
                if self.line_hits_obstacle((curr_lat, curr_lon), (lat, lon), flight_alt):
                    continue
                
                move_cost = math.sqrt(dx**2 + dy**2) * self.grid_size
                new_g_cost = g_cost + move_cost
                
                if new_key in visited and visited[new_key] <= new_g_cost:
                    continue
                
                # 启发式函数：到终点的距离
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
            # 尝试找到最远可以直接到达的点
            j = len(waypoints) - 1
            while j > i + 1:
                p1 = (waypoints[i].lat, waypoints[i].lon)
                p2 = (waypoints[j].lat, waypoints[j].lon)
                if not self.line_hits_obstacle(p1, p2, flight_alt):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j
        
        # 重新编号
        for idx, wp in enumerate(smoothed):
            wp.seq = idx
        
        return smoothed
    
    def plan_climb_over(self, start_wp, end_wp, max_altitude):
        """爬升飞越（仅当没有障碍物高于原始飞行高度时可用）"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        
        # 【关键修复】检查是否有障碍物高于原始飞行高度
        for obs in self.obstacles:
            if obs.height >= start_wp.alt:
                st.error("存在高于飞行高度的障碍物，无法使用爬升飞越策略")
                return None
        
        # 找出路径上的最高障碍物
        max_obs_height = 0
        steps = 50  # 增加采样密度
        
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            
            for obs in self.obstacles:
                if obs.is_inside(lat, lon, 0):
                    max_obs_height = max(max_obs_height, obs.height)
        
        if max_obs_height == 0:
            return [start_wp, end_wp]
        
        # 计算所需飞越高度
        fly_alt = max_obs_height + 25  # 增加安全余量到25米
        if fly_alt > max_altitude:
            st.warning(f"需要飞越高度{fly_alt}m超过最大限制{max_altitude}m，无法执行爬升飞越")
            return None
        
        # 构建爬升路径
        path = [start_wp]
        dist_total = self.haversine_distance(start[0], start[1], end[0], end[1])
        
        # 在障碍物前开始爬升
        if dist_total > 100:
            # 爬升点（距离起点20%处）
            climb_lat = start[0] + (end[0] - start[0]) * 0.2
            climb_lon = start[1] + (end[1] - start[1]) * 0.2
            path.append(Waypoint(climb_lat, climb_lon, fly_alt, 16, 1))
            
            # 保持高度的中点
            mid_lat = (start[0] + end[0]) / 2
            mid_lon = (start[1] + end[1]) / 2
            path.append(Waypoint(mid_lat, mid_lon, fly_alt, 16, len(path)))
            
            # 下降点（距离终点20%处）
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
    
    # 显示最高障碍物
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
    
    with st.expander("📖 使用说明", expanded=True):
        col1, col2 = st.columns(2)
        with col1:
            st.markdown("""
            ### 📋 操作步骤：
            1. **设置A/B点**：选择坐标系，输入起点终点坐标
            2. **添加障碍物**：使用地图绘制或参数化矩形
            3. **规划路径**：系统自动判断绕行或爬升策略
            
            ### ⚠️ 重要提示：
            - 当障碍物高度 ≥ 飞行高度时，**强制水平绕行**
            - 当障碍物高度 < 飞行高度时，可选择爬升飞越
            - 所有内部计算使用WGS-84坐标，确保精度
            """)
        with col2:
            st.markdown(f"""
            ### 🔧 当前配置：
            - **安全边距**: {st.session_state.planner.safety_margin}米
            - **网格精度**: {st.session_state.planner.grid_size}米
            - **最大迭代**: {st.session_state.planner.max_iterations}次
            - **当前坐标系**: {st.session_state.coord_system}
            """)
            
            # 显示障碍物高度对比
            max_obs_h = st.session_state.planner.get_max_obstacle_height()
            if max_obs_h > 0:
                st.markdown("### 📊 高度对比：")
                st.markdown(f"- 飞行高度: **{st.session_state.flight_altitude}m**")
                st.markdown(f"- 最高障碍物: **{max_obs_h}m**")
                if max_obs_h >= st.session_state.flight_altitude:
                    st.error("🔒 强制绕行模式已激活")
                else:
                    st.success("✅ 爬升飞越可用")
    
    col_map, col_ctrl = st.columns([3, 2])
    
    with col_map:
        st.subheader("🗺️ 地图")
        
        if st.session_state.point_a and st.session_state.point_b:
            center = [(st.session_state.point_a[0]+st.session_state.point_b[0])/2,
                     (st.session_state.point_a[1]+st.session_state.point_b[1])/2]
        else:
            center = st.session_state.map_center
        
        # 地图显示坐标转换
        display_center = CoordinateConverter.to_display_coords(center[0], center[1], st.session_state.coord_system)
        
        m = folium.Map(location=display_center, zoom_start=16, tiles="CartoDB positron")
        
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
            pos = CoordinateConverter.to_display_coords(st.session_state.point_a[0], st.session_state.point_a[1], st.session_state.coord_system)
            folium.Marker(pos, 
                         popup=f"起点A<br>WGS84: {st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}",
                         icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
            folium.Circle(pos, radius=8, color='green', fill=True, fillOpacity=0.3).add_to(m)
        
        if st.session_state.point_b:
            pos = CoordinateConverter.to_display_coords(st.session_state.point_b[0], st.session_state.point_b[1], st.session_state.coord_system)
            folium.Marker(pos,
                         popup=f"终点B<br>WGS84: {st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}", 
                         icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
            folium.Circle(pos, radius=8, color='red', fill=True, fillOpacity=0.3).add_to(m)
        
        # 显示障碍物
        for i, obs in enumerate(st.session_state.planner.obstacles):
            color = 'red' if obs.height >= st.session_state.flight_altitude else 'orange'
            
            if obs.type == "polygon":
                display_points = [CoordinateConverter.to_display_coords(p[0], p[1], st.session_state.coord_system) for p in obs.points]
                folium.Polygon(
                    locations=display_points,
                    popup=f"{obs.name}<br>高度:{obs.height}m<br>类型:多边形",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.5,
                    weight=3
                ).add_to(m)
            elif obs.type == "rectangle":
                display_points = [CoordinateConverter.to_display_coords(p[0], p[1], st.session_state.coord_system) for p in obs.points]
                folium.Polygon(
                    locations=display_points,
                    popup=f"{obs.name}<br>高度:{obs.height}m<br>旋转:{obs.rotation}°",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.5,
                    weight=3
                ).add_to(m)
                center_pos = CoordinateConverter.to_display_coords(obs.center_lat, obs.center_lon, st.session_state.coord_system)
                folium.CircleMarker(center_pos, radius=3, color='black', fill=True).add_to(m)
            else:
                center_pos = CoordinateConverter.to_display_coords(obs.center_lat, obs.center_lon, st.session_state.coord_system)
                folium.Circle(
                    center_pos,
                    radius=obs.radius,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillOpacity=0.5
                ).add_to(m)
        
        # 显示选中的路径
        if st.session_state.selected_path_type and st.session_state.waypoints:
            path_coords = [CoordinateConverter.to_display_coords(wp.lat, wp.lon, st.session_state.coord_system) for wp in st.session_state.waypoints]
            
            if st.session_state.selected_path_type == 'horizontal':
                AntPath(path_coords, color='blue', weight=6, opacity=0.9, 
                       dash_array=[15, 10], delay=800).add_to(m)
                for i, wp in enumerate(st.session_state.waypoints):
                    pos = CoordinateConverter.to_display_coords(wp.lat, wp.lon, st.session_state.coord_system)
                    color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints)-1 else 'blue'
                    folium.CircleMarker(pos, radius=4, color=color, fill=True,
                                       popup=f'航点{i}<br>高度:{wp.alt}m').add_to(m)
            
            elif st.session_state.selected_path_type == 'climb':
                AntPath(path_coords, color='green', weight=6, opacity=0.9,
                       dash_array=[15, 10], delay=800).add_to(m)
                for i, wp in enumerate(st.session_state.waypoints):
                    pos = CoordinateConverter.to_display_coords(wp.lat, wp.lon, st.session_state.coord_system)
                    color = 'darkgreen' if wp.alt > st.session_state.flight_altitude + 5 else 'green'
                    folium.CircleMarker(pos, radius=5, color=color, fill=True,
                                       popup=f'航点{i}<br>高度:{wp.alt}m').add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="main_map")
        
        # 处理地图绘制
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            shape_id = f"{drawing.get('type')}_{id(drawing)}"
            
            if st.session_state.get("last_shape_id") != shape_id:
                st.session_state["last_shape_id"] = shape_id
                geom_type = drawing.get("type")
                
                if geom_type == "circle":
                    center = drawing["geometry"]["coordinates"]
                    radius = drawing["properties"]["radius"]
                    # 地图绘制返回的是GCJ-02，转换为WGS84存储
                    lon_wgs, lat_wgs = CoordinateConverter.gcj02_to_wgs84(center[0], center[1])
                    st.session_state.pending_drawing = {
                        'type': 'circle',
                        'center': (lat_wgs, lon_wgs),
                        'radius': radius
                    }
                    st.rerun()
                elif geom_type in ["polygon", "rectangle"]:
                    coords = drawing["geometry"]["coordinates"][0]
                    points = []
                    for c in coords[:-1]:
                        lon_wgs, lat_wgs = CoordinateConverter.gcj02_to_wgs84(c[0], c[1])
                        points.append((lat_wgs, lon_wgs))
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
        
        default_lat_a = 32.0603
        default_lon_a = 118.7969
        if st.session_state.point_a:
            if st.session_state.coord_system == 'GCJ-02':
                lon_gcj, lat_gcj = CoordinateConverter.wgs84_to_gcj02(st.session_state.point_a[1], st.session_state.point_a[0])
                default_lat_a, default_lon_a = lat_gcj, lon_gcj
            else:
                default_lat_a, default_lon_a = st.session_state.point_a
        
        lat_a = c1.number_input("纬度", value=default_lat_a, format="%.6f", key="lat_a")
        lon_a = c2.number_input("经度", value=default_lon_a, format="%.6f", key="lon_a")
        
        if st.button("✅ 设置A点", key="set_a"):
            lat_wgs, lon_wgs = CoordinateConverter.from_input_coords(lat_a, lon_a, st.session_state.coord_system)
            st.session_state.point_a = (lat_wgs, lon_wgs)
            if st.session_state.coord_system == 'GCJ-02':
                st.session_state.point_a_gcj = (lat_a, lon_a)
            st.success(f"A点已设置 (WGS84: {lat_wgs:.6f}, {lon_wgs:.6f})")
            st.rerun()
        
        # B点设置
        st.markdown("**📍 终点 B**")
        c3, c4 = st.columns(2)
        
        default_lat_b = 32.0703
        default_lon_b = 118.8069
        if st.session_state.point_b:
            if st.session_state.coord_system == 'GCJ-02':
                lon_gcj, lat_gcj = CoordinateConverter.wgs84_to_gcj02(st.session_state.point_b[1], st.session_state.point_b[0])
                default_lat_b, default_lon_b = lat_gcj, lon_gcj
            else:
                default_lat_b, default_lon_b = st.session_state.point_b
        
        lat_b = c3.number_input("纬度", value=default_lat_b, format="%.6f", key="lat_b")
        lon_b = c4.number_input("经度", value=default_lon_b, format="%.6f", key="lon_b")
        
        if st.button("✅ 设置B点", key="set_b"):
            lat_wgs, lon_wgs = CoordinateConverter.from_input_coords(lat_b, lon_b, st.session_state.coord_system)
            st.session_state.point_b = (lat_wgs, lon_wgs)
            if st.session_state.coord_system == 'GCJ-02':
                st.session_state.point_b_gcj = (lat_b, lon_b)
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
                        st.session_state.planner.add_circle_obstacle(
                            drawing['center'][0], drawing['center'][1],
                            drawing['radius'], obs_height, f"圆形({obs_height}m)"
                        )
                    else:
                        st.session_state.planner.add_polygon_obstacle(
                            drawing['points'], obs_height, f"多边形({obs_height}m)"
                        )
                    st.session_state.pending_drawing = None
                    st.success("✅ 障碍物已添加到WGS84坐标系")
                    st.rerun()
            
            with col_cancel:
                if st.button("❌ 取消"):
                    st.session_state.pending_drawing = None
                    st.rerun()
            
            st.markdown("---")
        
        # 旋转矩形（参数化输入）
        with st.expander("⬜ 添加旋转矩形（参数化）"):
            default_lat = st.session_state.point_a[0] if st.session_state.point_a else st.session_state.map_center[0]
            default_lon = st.session_state.point_a[1] if st.session_state.point_a else st.session_state.map_center[1]
            
            if st.session_state.coord_system == 'GCJ-02' and st.session_state.point_a_gcj:
                default_lat = st.session_state.point_a_gcj[0]
                default_lon = st.session_state.point_a_gcj[1]
            
            rect_lat = st.number_input("中心纬度", value=default_lat, format="%.6f", key="rect_lat")
            rect_lon = st.number_input("中心经度", value=default_lon, format="%.6f", key="rect_lon")
            rect_width = st.slider("宽度(m)", 10, 300, 60, key="rect_w")
            rect_height = st.slider("长度(m)", 10, 300, 100, key="rect_h")
            rect_rotation = st.slider("旋转角度(°)", 0, 360, 45, key="rect_rot")
            rect_obs_h = st.number_input("矩形高度(m)", 5, 300, 60, key="rect_obs_h")
            
            if st.button("➕ 添加旋转矩形"):
                lat_wgs, lon_wgs = CoordinateConverter.from_input_coords(rect_lat, rect_lon, st.session_state.coord_system)
                
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
        
        # 判断是否需要强制绕行
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
                        
                        # 验证路径
                        is_safe = True
                        for i in range(len(path)-1):
                            p1 = (path[i].lat, path[i].lon)
                            p2 = (path[i+1].lat, path[i+1].lon)
                            if st.session_state.planner.line_hits_obstacle(p1, p2, st.session_state.flight_altitude):
                                is_safe = False
                                break
                        
                        if is_safe:
                            st.success(f"✅ 水平绕行规划成功！{len(path)}个航点, {dist:.0f}m")
                        else:
                            st.error("⚠️ 路径验证失败，请重新规划")
                            st.session_state.waypoints = []
                    else:
                        st.error("❌ 规划失败：无法找到可行的绕行路径")
                        st.session_state.planned_path_horizontal = None
                        st.session_state.waypoints = []
                
                st.rerun()
        
        with col_c:
            # 【关键修复】当障碍物高于飞行高度时，禁用爬升飞越
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
            
            # 安全验证
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
            
            # 地图显示
            center = st.session_state.drone_position if st.session_state.drone_position else [st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]
            display_center = CoordinateConverter.to_display_coords(center[0], center[1], st.session_state.coord_system)
            
            m = folium.Map(location=display_center, zoom_start=17, tiles="CartoDB dark_matter")
            
            # 显示完整航线
            if st.session_state.waypoints:
                full_path = [CoordinateConverter.to_display_coords(wp.lat, wp.lon, st.session_state.coord_system) for wp in st.session_state.waypoints]
                folium.PolyLine(full_path, color='blue', weight=3, opacity=0.6, dash_array='5,10').add_to(m)
                
                for i, wp in enumerate(st.session_state.waypoints):
                    pos = CoordinateConverter.to_display_coords(wp.lat, wp.lon, st.session_state.coord_system)
                    color = 'green' if i == 0 else 'red' if i == len(st.session_state.waypoints)-1 else 'blue'
                    folium.CircleMarker(pos, radius=4, color=color, fill=True, 
                                       popup=f'航点{i}').add_to(m)
            
            # 显示已飞路径
            if len(st.session_state.flight_path_history) > 1:
                history_display = [CoordinateConverter.to_display_coords(p[0], p[1], st.session_state.coord_system) for p in st.session_state.flight_path_history]
                folium.PolyLine(history_display, color='lime', weight=5, opacity=0.9).add_to(m)
            
            # 显示无人机位置
            if st.session_state.drone_position:
                pos = CoordinateConverter.to_display_coords(st.session_state.drone_position[0], st.session_state.drone_position[1], st.session_state.coord_system)
                folium.Marker(pos,
                            icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
                            popup="无人机当前位置").add_to(m)
                folium.Circle(pos, radius=10, color='orange', fill=True, fillOpacity=0.3).add_to(m)
            
            st_folium(m, width=800, height=500)
            
            # 动画模拟
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
st.caption("MAVLink GCS v5.2 | 智能避障 | 坐标系自动转换 | 北京时间 (UTC+8)")
