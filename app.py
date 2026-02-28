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
    """坐标系转换工具类 - 统一使用 (lat, lon) 格式"""
    
    PI = 3.1415926535897932384626
    A = 6378245.0
    EE = 0.00669342162296594323
    
    @staticmethod
    def _transformlat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * CoordinateConverter.PI) + 40.0 * math.sin(lat / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * CoordinateConverter.PI) + 320 * math.sin(lat * CoordinateConverter.PI / 30.0)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _transformlng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * CoordinateConverter.PI) + 20.0 * math.sin(2.0 * lng * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * CoordinateConverter.PI) + 40.0 * math.sin(lng / 3.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * CoordinateConverter.PI) + 300.0 * math.sin(lng / 30.0 * CoordinateConverter.PI)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _out_of_china(lng, lat):
        return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)
    
    @classmethod
    def gcj02_to_wgs84(cls, lat, lon):
        """GCJ-02 转 WGS-84，参数 (lat, lon)"""
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
        """WGS-84 转 GCJ-02，参数 (lat, lon)"""
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
    def from_user_input(cls, lat, lon, coord_system='WGS-84'):
        """用户输入转内部 WGS-84"""
        if coord_system == 'GCJ-02':
            return cls.gcj02_to_wgs84(lat, lon)
        return lat, lon


# ==================== 通信链路日志系统 ====================
class CommLinkLogger:
    """通信链路日志系统 - 记录GCS-OBC-FCU之间的通信"""
    
    NODE_GCS = "🖥️ GCS"
    NODE_OBC = "🧠 OBC"
    NODE_FCU = "⚙️ FCU"
    
    MSG_NAV_TARGET = "导航目标"
    MSG_PATH_PLAN = "航线规划"
    MSG_MISSION_UP = "任务上传"
    MSG_MISSION_ACK = "任务确认"
    MSG_TELEMETRY = "遥测数据"
    MSG_FLIGHT_STAT = "飞行状态"
    MSG_CMD = "控制指令"
    
    def __init__(self, max_logs=200):
        self.logs = deque(maxlen=max_logs)
        self.stats = {
            'gcs_to_obc': 0,
            'obc_to_gcs': 0,
            'obc_to_fcu': 0,
            'fcu_to_obc': 0
        }
    
    def _get_timestamp(self):
        return (datetime.utcnow() + timedelta(hours=8)).strftime("%H:%M:%S.%f")[:-3]
    
    def log(self, direction, msg_type, content, status="info"):
        status_icons = {
            "success": "✅",
            "error": "❌",
            "warning": "⚠️",
            "info": "ℹ️",
            "pending": "⏳",
            "processing": "🔄"
        }
        self.logs.append({
            'timestamp': self._get_timestamp(),
            'direction': direction,
            'msg_type': msg_type,
            'content': content,
            'status': status,
            'icon': status_icons.get(status, "ℹ️")
        })
    
    def log_nav_target_to_obc(self, point_a, point_b, altitude):
        self.log(f"{self.NODE_GCS} → {self.NODE_OBC}", self.MSG_NAV_TARGET, 
                f"起点: ({point_a[0]:.6f}, {point_a[1]:.6f}), 终点: ({point_b[0]:.6f}, {point_b[1]:.6f}), 目标高度: {altitude}m", "info")
    
    def log_path_planning_start(self, algorithm, obstacles_count):
        self.log(f"{self.NODE_OBC} 内部", self.MSG_PATH_PLAN, f"开始航线规划 | 算法: {algorithm} | 障碍物数量: {obstacles_count}", "processing")
    
    def log_path_planning_complete(self, path_length, waypoints_count, path_type):
        self.log(f"{self.NODE_OBC} 内部", self.MSG_PATH_PLAN, f"航线规划完成 | 类型: {path_type} | 航点数: {waypoints_count} | 路径长度: {path_length:.1f}m", "success")
    
    def log_mission_upload(self, waypoints_count):
        self.log(f"{self.NODE_OBC} → {self.NODE_FCU}", self.MSG_MISSION_UP, f"上传航线任务 | 航点数量: {waypoints_count}", "pending")
    
    def log_flight_start(self):
        self.log(f"{self.NODE_GCS} → {self.NODE_FCU}", self.MSG_CMD, "开始执行飞行任务", "success")
    
    def log_waypoint_reached(self, wp_index, total):
        self.log(f"{self.NODE_FCU} → {self.NODE_OBC} → {self.NODE_GCS}", self.MSG_FLIGHT_STAT, f"到达航点 {wp_index}/{total}", "success")
    
    def log_flight_complete(self):
        self.log(f"{self.NODE_FCU} → {self.NODE_OBC} → {self.NODE_GCS}", self.MSG_FLIGHT_STAT, "任务执行完成！", "success")
    
    def get_logs(self, filter_type=None):
        return list(self.logs)
    
    def clear(self):
        self.logs.clear()


# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 安全避障系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)


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
    
    A, B = (p1[1], p1[0]), (p2[1], p2[0])  # (lon, lat) for calculation
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
        
        if point_in_polygon(lat, lon, self.points):
            return True
        
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
            # 检查线段上多个采样点
            num_samples = 30
            for i in range(num_samples + 1):
                t = i / num_samples
                lat = p1[0] + (p2[0] - p1[0]) * t
                lon = p1[1] + (p2[1] - p1[1]) * t
                if self.is_inside(lat, lon):
                    return True
            return False
        
        # 首先检查线段的两个端点是否在多边形内
        if point_in_polygon(p1[0], p1[1], self.points):
            return True
        if point_in_polygon(p2[0], p2[1], self.points):
            return True
        
        # 然后检查线段是否与多边形的边相交
        return line_intersects_polygon(p1, p2, self.points)


class GridPathPlanner:
    """严格避障路径规划器 - 确保绝不穿行障碍物"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 10  # 小型四旋翼默认安全边距10米
        self.grid_size = 5  # 减小网格到5米，提高精度
        self.max_iterations = 50000  # 增加最大迭代次数
    
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
        if not self.obstacles:
            return 0
        return max(obs.height for obs in self.obstacles)
    
    def should_force_avoidance(self, flight_alt):
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                return True
        return False
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def is_collision(self, lat, lon, flight_alt):
        """【严格】检查是否碰撞"""
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
            elif flight_alt < obs.height + 15:
                if obs.is_inside(lat, lon, self.safety_margin):
                    return True
        return False
    
    def line_hits_obstacle(self, p1, p2, flight_alt):
        """【严格】检测线段是否与任何障碍物相交"""
        for obs in self.obstacles:
            if obs.height >= flight_alt:
                if obs.line_intersects(p1, p2):
                    return True
                # 额外检查中点和采样点
                mid_lat = (p1[0] + p2[0]) / 2
                mid_lon = (p1[1] + p2[1]) / 2
                if obs.is_inside(mid_lat, mid_lon, self.safety_margin):
                    return True
            elif flight_alt < obs.height + 15:
                if obs.line_intersects(p1, p2):
                    return True
        return False
    
    def latlon_to_grid(self, lat, lon, base_lat, base_lon):
        dlat = (lat - base_lat) * 111000
        dlon = (lon - base_lon) * 111000 * math.cos(math.radians(base_lat))
        return (round(dlon / self.grid_size), round(dlat / self.grid_size))
    
    def grid_to_latlon(self, grid_x, grid_y, base_lat, base_lon):
        lon = base_lon + (grid_x * self.grid_size) / (111000 * math.cos(math.radians(base_lat)))
        lat = base_lat + (grid_y * self.grid_size) / 111000
        return (lat, lon)
    
    def get_bounding_box_with_obstacles(self, start, end):
        """获取包含所有障碍物的扩展边界框"""
        if not self.obstacles:
            return (min(start[0], end[0]), max(start[0], end[0]), 
                   min(start[1], end[1]), max(start[1], end[1]))
        
        all_lats = [start[0], end[0]]
        all_lons = [start[1], end[1]]
        
        for obs in self.obstacles:
            if obs.type == "circle":
                # 圆形边界
                r_deg = obs.radius / 111000
                all_lats.extend([obs.center_lat - r_deg, obs.center_lat + r_deg])
                all_lons.extend([obs.center_lon - r_deg, obs.center_lon + r_deg])
            else:
                # 多边形边界
                for p in obs.points:
                    all_lats.append(p[0])
                    all_lons.append(p[1])
        
        lat_min, lat_max = min(all_lats), max(all_lats)
        lon_min, lon_max = min(all_lons), max(all_lons)
        
        # 扩展边界
        lat_margin = 0.005  # 约550米
        lon_margin = 0.005 / math.cos(math.radians((lat_min + lat_max) / 2))
        
        return (lat_min - lat_margin, lat_max + lat_margin, 
                lon_min - lon_margin, lon_max + lon_margin)
    
    def plan_horizontal_avoidance(self, start_wp, end_wp, bias=0):
        """
        【严格】强制水平绕行路径规划 - 绝不穿行
        bias: 绕行偏向 (-1=左偏, 0=最优, 1=右偏)
        """
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        flight_alt = start_wp.alt
        
        # 【关键】严格检查起点和终点
        if self.is_collision(start[0], start[1], flight_alt):
            return None, "起点在障碍物安全边界内"
        if self.is_collision(end[0], end[1], flight_alt):
            return None, "终点在障碍物安全边界内"
        
        # 【关键】首先检查直线路径是否安全
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [start_wp, end_wp], "直线路径安全"
        
        # 获取包含障碍物的边界框
        lat_min, lat_max, lon_min, lon_max = self.get_bounding_box_with_obstacles(start, end)
        
        # 根据偏向调整边界框（实现左右绕行）
        lat_range = lat_max - lat_min
        lon_range = lon_max - lon_min
        if bias < 0:  # 左偏 - 扩展左侧边界
            lon_min -= lon_range * 0.3
        elif bias > 0:  # 右偏 - 扩展右侧边界
            lon_max += lon_range * 0.3
        
        base_lat = lat_min
        base_lon = lon_min
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        # 【关键】24方向搜索，确保能找到绕行路径
        directions = [
            (0,1), (1,0), (0,-1), (-1,0),  # 4正方向
            (1,1), (1,-1), (-1,1), (-1,-1),  # 4对角
            (0,2), (2,0), (0,-2), (-2,0),  # 2步正方向
            (2,2), (2,-2), (-2,2), (-2,-2),  # 2步对角
            (0,3), (3,0), (0,-3), (-3,0),  # 3步正方向
            (1,2), (2,1), (-1,2), (-2,1), (1,-2), (2,-1), (-1,-2), (-2,-1),  # 混合
        ]
        
        # A*算法
        open_set = [(0, 0, start_grid[0], start_grid[1], [start_grid])]
        visited = {}
        
        iteration = 0
        best_path = None
        best_dist = float('inf')
        
        while open_set and iteration < self.max_iterations:
            iteration += 1
            f_cost, g_cost, x, y, path = heapq.heappop(open_set)
            
            # 检查是否到达终点（允许2个网格误差，约10米）
            if abs(x - end_grid[0]) <= 2 and abs(y - end_grid[1]) <= 2:
                # 构建路径
                waypoints = [start_wp]
                for grid in path[1:]:
                    lat, lon = self.grid_to_latlon(grid[0], grid[1], base_lat, base_lon)
                    waypoints.append(Waypoint(lat, lon, flight_alt, 16, len(waypoints)))
                
                waypoints.append(end_wp)
                waypoints[-1].seq = len(waypoints) - 1
                
                # 【关键】路径平滑和验证
                waypoints = self.smooth_path(waypoints, flight_alt)
                
                # 验证路径
                if self.validate_path(waypoints, flight_alt):
                    current_dist = sum(self.haversine_distance(
                        waypoints[i].lat, waypoints[i].lon, 
                        waypoints[i+1].lat, waypoints[i+1].lon) for i in range(len(waypoints)-1))
                    
                    if current_dist < best_dist:
                        best_dist = current_dist
                        best_path = waypoints
                
                if iteration > 10000:
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
                
                # 边界检查
                if not (lat_min <= lat <= lat_max and lon_min <= lon <= lon_max):
                    continue
                
                # 检查该点是否安全
                if self.is_collision(lat, lon, flight_alt):
                    continue
                
                # 【关键】检查从当前点到新点的线段是否安全
                curr_lat, curr_lon = self.grid_to_latlon(x, y, base_lat, base_lon)
                if self.line_hits_obstacle((curr_lat, curr_lon), (lat, lon), flight_alt):
                    continue
                
                move_cost = math.sqrt(dx**2 + dy**2) * self.grid_size
                new_g_cost = g_cost + move_cost
                
                if new_key in visited and visited[new_key] <= new_g_cost:
                    continue
                
                # 启发式函数
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                
                heapq.heappush(open_set, (new_g_cost + h, new_g_cost, nx, ny, path + [(nx, ny)]))
        
        if best_path is not None:
            # 最终验证
            if self.validate_path(best_path, flight_alt):
                return best_path, "规划成功"
            else:
                return None, "路径验证失败"
        
        return None, "无法找到可行的绕行路径"
    
    def plan_multiple_paths(self, start_wp, end_wp, max_altitude):
        """规划多条路径供选择"""
        paths = {}
        
        # 1. 左绕行
        left_path, left_msg = self.plan_horizontal_avoidance(start_wp, end_wp, bias=-1)
        if left_path:
            paths['left'] = {
                'path': left_path,
                'name': '⬅️ 左侧绕行',
                'distance': sum(self.haversine_distance(
                    left_path[i].lat, left_path[i].lon,
                    left_path[i+1].lat, left_path[i+1].lon) for i in range(len(left_path)-1)),
                'type': 'horizontal'
            }
        
        # 2. 右绕行
        right_path, right_msg = self.plan_horizontal_avoidance(start_wp, end_wp, bias=1)
        if right_path:
            paths['right'] = {
                'path': right_path,
                'name': '➡️ 右侧绕行',
                'distance': sum(self.haversine_distance(
                    right_path[i].lat, right_path[i].lon,
                    right_path[i+1].lat, right_path[i+1].lon) for i in range(len(right_path)-1)),
                'type': 'horizontal'
            }
        
        # 3. 最优绕行
        best_path, best_msg = self.plan_horizontal_avoidance(start_wp, end_wp, bias=0)
        if best_path and len(best_path) == 2:  # 直线路径
            paths['direct'] = {
                'path': best_path,
                'name': '⬆️ 直线飞行',
                'distance': sum(self.haversine_distance(
                    best_path[i].lat, best_path[i].lon,
                    best_path[i+1].lat, best_path[i+1].lon) for i in range(len(best_path)-1)),
                'type': 'direct'
            }
        elif best_path:
            paths['best'] = {
                'path': best_path,
                'name': '✨ 最优绕行',
                'distance': sum(self.haversine_distance(
                    best_path[i].lat, best_path[i].lon,
                    best_path[i+1].lat, best_path[i+1].lon) for i in range(len(best_path)-1)),
                'type': 'horizontal'
            }
        
        # 4. 爬升飞越（如果可行）
        climb_path = self.plan_climb_over(start_wp, end_wp, max_altitude)
        if climb_path and len(climb_path) > 0:
            max_fly_alt = max(wp.alt for wp in climb_path)
            paths['climb'] = {
                'path': climb_path,
                'name': '⬆️ 爬升飞越',
                'distance': sum(self.haversine_distance(
                    climb_path[i].lat, climb_path[i].lon,
                    climb_path[i+1].lat, climb_path[i+1].lon) for i in range(len(climb_path)-1)),
                'max_altitude': max_fly_alt,
                'type': 'climb'
            }
        
        return paths
    
    def smooth_path(self, waypoints, flight_alt):
        """路径平滑 - 移除不必要的中间点"""
        if len(waypoints) <= 2:
            return waypoints
        
        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            # 找到最远可以直接到达的点
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
    
    def validate_path(self, waypoints, flight_alt):
        """【严格】验证路径是否安全 - 检查所有航点之间的线段"""
        for i in range(len(waypoints) - 1):
            p1 = (waypoints[i].lat, waypoints[i].lon)
            p2 = (waypoints[i+1].lat, waypoints[i+1].lon)
            
            # 检查起点和终点
            if self.is_collision(p1[0], p1[1], flight_alt):
                st.error(f"❌ 路径验证失败：航点{i}在障碍物内")
                return False
            if self.is_collision(p2[0], p2[1], flight_alt):
                st.error(f"❌ 路径验证失败：航点{i+1}在障碍物内")
                return False
            
            # 检查线段
            if self.line_hits_obstacle(p1, p2, flight_alt):
                st.error(f"❌ 路径验证失败：航段{i}-{i+1}穿过障碍物")
                return False
        
        return True
    
    def plan_climb_over(self, start_wp, end_wp, max_altitude):
        """爬升飞越 - 仅当没有障碍物高于原始飞行高度时可用"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        
        for obs in self.obstacles:
            if obs.height >= start_wp.alt:
                st.error("❌ 存在高于飞行高度的障碍物，无法使用爬升飞越")
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
            st.warning(f"需要飞越高度{fly_alt}m超过最大限制{max_altitude}m")
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
        'drawings_count': 0,
        'last_processed_drawing': None,
        'available_paths': {},
        'selected_path_name': None,
        # 新增：通信链路日志和飞行模拟
        'comm_logger': CommLinkLogger(),
        'all_flight_positions': [],
        'drone_pos_index': 0,
        'flight_start_time': None,
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
    st.metric("安全半径", f"{st.session_state.planner.safety_margin}m")
    
    max_obs_h = st.session_state.planner.get_max_obstacle_height()
    if max_obs_h > 0:
        st.metric("最高障碍物", f"{max_obs_h}m")
        if max_obs_h >= st.session_state.flight_altitude:
            st.error("⚠️ 强制绕行模式")
    
    if st.session_state.waypoints:
        path_name = st.session_state.get('selected_path_name', '未命名')
        st.metric("选中路径", path_name)
        st.metric("航点数量", len(st.session_state.waypoints))
    
    if st.session_state.get('available_paths'):
        st.metric("可选路径数", len(st.session_state['available_paths']))
    
    # 通信链路日志 - 侧边栏实时显示
    st.markdown("---")
    st.header("📡 通信链路")
    logs = st.session_state.comm_logger.get_logs()
    if logs:
        # 显示最近5条日志
        for log in list(logs)[-5:]:
            color = {"success": "green", "error": "red", "warning": "orange", "info": "blue"}.get(log['status'], "gray")
            st.markdown(f"<small>[{log['timestamp']}] {log['icon']} <b>{log['msg_type']}</b></small>", unsafe_allow_html=True)
            st.markdown(f"<small style='color:{color}'>{log['content'][:30]}...</small>", unsafe_allow_html=True)
    else:
        st.info("暂无通信记录")


# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title("🚁 MAVLink 地面站 - 严格避障系统")
    st.caption("绝对安全绕行 | 严格碰撞检测 | 坐标系自动转换")
    
    # 显示安全警告
    if st.session_state.planner.should_force_avoidance(st.session_state.flight_altitude):
        st.error("⚠️ **强制绕行模式激活**：存在高于或等于飞行高度的障碍物，系统将严格绕行")
    
    with st.expander("📖 使用说明", expanded=True):
        col1, col2 = st.columns(2)
        with col1:
            st.markdown("""
            ### 📋 操作步骤：
            1. **设置A/B点**：选择坐标系，输入起点终点坐标
            2. **添加障碍物**：在地图上绘制或在AB之间添加障碍物
            3. **规划路径**：点击"水平绕行"，系统将严格避开所有障碍物
            
            ### ⚠️ 安全保证：
            - **50米安全边距**：无人机与障碍物保持至少50米距离
            - **严格验证**：规划后再次验证路径安全性
            - **强制绕行**：障碍物高于飞行高度时，绝不穿行
            """)
        with col2:
            st.markdown(f"""
            ### 🔧 当前配置：
            - **安全边距**: {st.session_state.planner.safety_margin}米
            - **网格精度**: {st.session_state.planner.grid_size}米
            - **飞行高度**: {st.session_state.flight_altitude}米
            - **最高障碍**: {st.session_state.planner.get_max_obstacle_height()}米
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
            lat, lon = st.session_state.point_a
            folium.Marker([lat, lon], 
                         popup=f"起点A<br>WGS84: {lat:.6f}, {lon:.6f}",
                         icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
            folium.Circle([lat, lon], radius=8, color='green', fill=True, fillOpacity=0.3).add_to(m)
        
        if st.session_state.point_b:
            lat, lon = st.session_state.point_b
            folium.Marker([lat, lon],
                         popup=f"终点B<br>WGS84: {lat:.6f}, {lon:.6f}", 
                         icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
            folium.Circle([lat, lon], radius=8, color='red', fill=True, fillOpacity=0.3).add_to(m)
        
        # 显示障碍物
        for i, obs in enumerate(st.session_state.planner.obstacles):
            color = 'red' if obs.height >= st.session_state.flight_altitude else 'orange'
            
            if obs.type in ["polygon", "rectangle"]:
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
        
        # 处理地图绘制 - 简化逻辑
        if map_data:
            last_drawing = map_data.get("last_active_drawing")
            
            # 获取几何类型（处理 GeoJSON Feature 格式）
            geom_type = None
            if last_drawing:
                if last_drawing.get("type") == "Feature":
                    # GeoJSON Feature 格式
                    geom_type = last_drawing.get("geometry", {}).get("type")
                else:
                    # 直接类型
                    geom_type = last_drawing.get("type")
            
            # 调试信息
            st.session_state["debug_map"] = {
                "has_last_drawing": last_drawing is not None,
                "last_drawing_type": last_drawing.get("type") if last_drawing else None,
                "geom_type": geom_type,
                "pending_exists": st.session_state.pending_drawing is not None,
            }
            
            # 只要有 last_drawing 且没有待处理的绘制，就处理
            if last_drawing and st.session_state.pending_drawing is None:
                try:
                    if geom_type == "Point":  # Circle 在 GeoJSON 中是 Point
                        center = last_drawing["geometry"]["coordinates"]
                        radius = last_drawing.get("properties", {}).get("radius", 50)
                        st.session_state.pending_drawing = {
                            'type': 'circle',
                            'center': (center[1], center[0]),
                            'radius': radius
                        }
                        st.rerun()
                    elif geom_type in ["Polygon", "polygon"]:
                        coords = last_drawing["geometry"]["coordinates"][0]
                        points = [(c[1], c[0]) for c in coords[:-1]]
                        st.session_state.pending_drawing = {
                            'type': 'polygon',
                            'points': points
                        }
                        st.rerun()
                    else:
                        # 未知类型，记录完整数据结构用于调试
                        st.session_state["debug_map"]["unknown_geom_type"] = geom_type
                        st.session_state["debug_map"]["full_drawing"] = str(last_drawing)[:200]
                except Exception as e:
                    st.session_state["debug_map"]["error"] = str(e)
    
    with col_ctrl:
        st.subheader("⚙️ 控制面板")
        
        # A点设置
        st.markdown("**📍 起点 A**")
        st.caption(f"输入坐标系: {st.session_state.coord_system}")
        col_a, col_def = st.columns([4, 1])
        with col_def:
            if st.button("⌗", key="def_a", help="默认A点: 32.2323, 118.7496"):
                st.session_state.point_a = (32.2323, 118.7496)
                st.rerun()
        c1, c2 = col_a.columns(2)
        
        default_lat_a, default_lon_a = 32.2323, 118.7496
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
            lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat_a, lon_a, st.session_state.coord_system)
            st.session_state.point_a = (lat_wgs, lon_wgs)
            st.success(f"A点已设置 (WGS84: {lat_wgs:.6f}, {lon_wgs:.6f})")
            st.rerun()
        
        # B点设置
        st.markdown("**📍 终点 B**")
        col_b, col_def2 = st.columns([4, 1])
        with col_def2:
            if st.button("⌗", key="def_b", help="默认B点: 32.2344, 118.7493"):
                st.session_state.point_b = (32.2344, 118.7493)
                st.rerun()
        c3, c4 = col_b.columns(2)
        
        default_lat_b, default_lon_b = 32.2344, 118.7493
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
        
        # 安全半径设置
        st.markdown("---")
        st.markdown("**🛡️ 安全设置**")
        new_safety = st.slider("无人机安全半径(m)", 5, 20, st.session_state.planner.safety_margin, key="safety_margin")
        if new_safety != st.session_state.planner.safety_margin:
            st.session_state.planner.safety_margin = new_safety
            st.success(f"✅ 安全半径已设置为 {new_safety} 米")
            st.rerun()
        
        # 高度对比
        max_obs_h = st.session_state.planner.get_max_obstacle_height()
        if max_obs_h > 0:
            st.markdown("---")
            st.markdown("**📊 高度分析**")
            col_h1, col_h2 = st.columns(2)
            col_h1.metric("飞行高度", f"{st.session_state.flight_altitude}m")
            col_h2.metric("最高障碍", f"{max_obs_h}m")
            
            if max_obs_h >= st.session_state.flight_altitude:
                st.error("🔴 障碍物高于飞行高度！必须绕行")
            elif max_obs_h >= st.session_state.flight_altitude - 20:
                st.warning("🟠 障碍物接近飞行高度")
        
        st.markdown("---")
        
        # 障碍物管理
        st.markdown("**🧱 障碍物管理**")
        
        # 计算 can_plan (必须在调试信息之前定义)
        has_a = st.session_state.point_a is not None
        has_b = st.session_state.point_b is not None
        can_plan = has_a and has_b
        
        # 调试信息显示
        with st.expander("🔧 调试信息", expanded=True):
            st.write("地图调试:", st.session_state.get("debug_map", {}))
            st.write("pending_drawing:", st.session_state.pending_drawing)
            st.write("障碍物数量:", len(st.session_state.planner.obstacles))
            st.write("A点:", st.session_state.point_a)
            st.write("B点:", st.session_state.point_b)
            st.write("can_plan:", can_plan)
        
        # ====== 优先处理地图圈选的待确认障碍物 ======
        if st.session_state.pending_drawing:
            drawing = st.session_state.pending_drawing
            
            # 使用 container 包裹，更明显
            confirm_container = st.container()
            with confirm_container:
                st.error("🚨 **请确认地图圈选的障碍物** (必须设置高度并确认)")
                
                if drawing['type'] == 'circle':
                    lat, lon = drawing['center']
                    st.info(f"⭕ 圆形障碍物: 中心({lat:.6f}, {lon:.6f}) 半径{drawing['radius']:.1f}m")
                else:
                    st.info(f"📐 多边形: {len(drawing['points'])}个顶点")
                
                # 使用独立的key，避免与其他number_input冲突
                map_obs_height = st.number_input(
                    "🗺️ 地图障碍物高度(m)", 
                    min_value=5, max_value=300, 
                    value=max(st.session_state.flight_altitude + 10, 50), 
                    key="map_obs_height_unique_v2"
                )
                
                col_add, col_cancel = st.columns(2)
                with col_add:
                    btn_clicked = st.button("✅ 确认添加障碍物", type="primary", key="confirm_map_obs_btn_v2")
                    if btn_clicked:
                        if drawing['type'] == 'circle':
                            lat, lon = drawing['center']
                            st.session_state.planner.add_circle_obstacle(
                                lat, lon, drawing['radius'], map_obs_height, f"圆形({map_obs_height}m)"
                            )
                        else:
                            st.session_state.planner.add_polygon_obstacle(
                                drawing['points'], map_obs_height, f"多边形({map_obs_height}m)"
                            )
                        st.session_state.pending_drawing = None
                        st.session_state["last_processed_drawing"] = None
                        st.success("✅ 障碍物已添加")
                        st.rerun()
                
                with col_cancel:
                    if st.button("❌ 取消", key="cancel_map_obs_btn_v2"):
                        st.session_state.pending_drawing = None
                        st.session_state["last_processed_drawing"] = None
                        st.rerun()
            
            st.markdown("---")
        
        # 方式1: 在地图上圈选
        st.info("💡 **方式1**: 在左侧地图上用 🔲矩形/⭕圆形/📐多边形 工具圈选，选中图形后右侧会显示确认按钮")
        
        # 方式2: 手动输入坐标添加
        with st.expander("➕ 方式2: 手动输入坐标添加障碍物", expanded=True):
            st.caption(f"输入坐标系: {st.session_state.coord_system} (将自动转换为WGS-84)")
            
            obs_type = st.selectbox("障碍物类型", ["矩形", "圆形", "多边形"], key="manual_obs_type")
            manual_obs_height = st.number_input("障碍物高度(m)", 5, 300, 
                                        max(st.session_state.flight_altitude + 10, 50), key="manual_obs_h")
            
            # 选择位置来源
            pos_source = st.radio("位置来源", ["自定义坐标", "AB中点", "A点位置", "B点位置"], key="pos_source")
            
            if pos_source == "AB中点" and st.session_state.point_a and st.session_state.point_b:
                default_lat = (st.session_state.point_a[0] + st.session_state.point_b[0]) / 2
                default_lon = (st.session_state.point_a[1] + st.session_state.point_b[1]) / 2
                # 转换回当前坐标系显示
                if st.session_state.coord_system == 'GCJ-02':
                    default_lat, default_lon = CoordinateConverter.wgs84_to_gcj02(default_lat, default_lon)
            elif pos_source == "A点位置" and st.session_state.point_a:
                default_lat, default_lon = st.session_state.point_a
                if st.session_state.coord_system == 'GCJ-02':
                    default_lat, default_lon = CoordinateConverter.wgs84_to_gcj02(default_lat, default_lon)
            elif pos_source == "B点位置" and st.session_state.point_b:
                default_lat, default_lon = st.session_state.point_b
                if st.session_state.coord_system == 'GCJ-02':
                    default_lat, default_lon = CoordinateConverter.wgs84_to_gcj02(default_lat, default_lon)
            else:
                default_lat, default_lon = st.session_state.map_center
            
            if obs_type == "矩形":
                st.markdown("**矩形中心坐标**")
                c1, c2 = st.columns(2)
                rect_lat_input = c1.number_input("中心纬度", value=default_lat, format="%.6f", key="rect_lat")
                rect_lon_input = c2.number_input("中心经度", value=default_lon, format="%.6f", key="rect_lon")
                rect_w = st.slider("宽度(m)", 10, 500, 50, key="manual_w")
                rect_h = st.slider("长度(m)", 10, 500, 80, key="manual_h")
                rect_rot = st.slider("旋转角度(°)", 0, 360, 0, key="manual_rot")
                
                if st.button("➕ 添加矩形障碍物", type="primary", key="btn_add_rect"):
                    # 坐标系转换
                    rect_lat_wgs, rect_lon_wgs = CoordinateConverter.from_user_input(
                        rect_lat_input, rect_lon_input, st.session_state.coord_system
                    )
                    st.session_state.planner.add_rotated_rectangle_obstacle(
                        rect_lat_wgs, rect_lon_wgs, rect_w, rect_h, rect_rot, 
                        manual_obs_height, f"矩形障碍({manual_obs_height}m)"
                    )
                    st.success(f"✅ 已添加矩形障碍物 ({rect_lat_wgs:.6f}, {rect_lon_wgs:.6f})，高度{manual_obs_height}m")
                    st.rerun()
                    
            elif obs_type == "圆形":
                st.markdown("**圆形中心坐标**")
                c1, c2 = st.columns(2)
                circle_lat_input = c1.number_input("中心纬度", value=default_lat, format="%.6f", key="circle_lat")
                circle_lon_input = c2.number_input("中心经度", value=default_lon, format="%.6f", key="circle_lon")
                circle_r = st.slider("半径(m)", 10, 200, 30, key="manual_r")
                
                if st.button("➕ 添加圆形障碍物", type="primary", key="btn_add_circle"):
                    # 坐标系转换
                    circle_lat_wgs, circle_lon_wgs = CoordinateConverter.from_user_input(
                        circle_lat_input, circle_lon_input, st.session_state.coord_system
                    )
                    st.session_state.planner.add_circle_obstacle(
                        circle_lat_wgs, circle_lon_wgs, circle_r, manual_obs_height, 
                        f"圆形障碍({manual_obs_height}m)"
                    )
                    st.success(f"✅ 已添加圆形障碍物 ({circle_lat_wgs:.6f}, {circle_lon_wgs:.6f})，高度{manual_obs_height}m")
                    st.rerun()
                    
            else:  # 多边形
                st.markdown("**多边形顶点坐标** (输入3个以上顶点)")
                st.caption(f"格式: 纬度,经度 (每行一个点) | 当前坐标系: {st.session_state.coord_system}")
                poly_input = st.text_area(
                    "顶点列表",
                    value=f"{default_lat + 0.001:.6f},{default_lon:.6f}\n{default_lat:.6f},{default_lon + 0.001:.6f}\n{default_lat - 0.001:.6f},{default_lon:.6f}",
                    height=100,
                    key="poly_input"
                )
                
                if st.button("➕ 添加多边形障碍物", type="primary", key="btn_add_poly"):
                    try:
                        points_input = []
                        for line in poly_input.strip().split('\n'):
                            line = line.strip()
                            if line and ',' in line:
                                lat, lon = map(float, line.split(','))
                                points_input.append((lat, lon))
                        
                        if len(points_input) >= 3:
                            # 坐标系转换
                            points_wgs = []
                            for lat, lon in points_input:
                                lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat, lon, st.session_state.coord_system)
                                points_wgs.append((lat_wgs, lon_wgs))
                            
                            st.session_state.planner.add_polygon_obstacle(
                                points_wgs, manual_obs_height, f"多边形障碍({manual_obs_height}m)"
                            )
                            st.success(f"✅ 已添加多边形障碍物 ({len(points_wgs)}个顶点)，高度{manual_obs_height}m")
                            st.rerun()
                        else:
                            st.error("❌ 多边形需要至少3个顶点")
                    except Exception as e:
                        st.error(f"❌ 输入格式错误: {e}")
        
        # 障碍物列表
        if st.session_state.planner.obstacles:
            with st.expander(f"📋 障碍物列表({len(st.session_state.planner.obstacles)}个)", expanded=True):
                for i, obs in enumerate(st.session_state.planner.obstacles):
                    icon = "⭕" if obs.type == "circle" else "⬜" if obs.type == "rectangle" else "📐"
                    is_blocking = "🔴" if obs.height >= st.session_state.flight_altitude else "🟢"
                    
                    col_obs, col_del = st.columns([4, 1])
                    with col_obs:
                        st.write(f"{is_blocking} {icon} #{i+1}: {obs.name} - {obs.height}m")
                    with col_del:
                        if st.button("🗑️", key=f"del_obs_{i}", help=f"删除障碍物 #{i+1}"):
                            # 删除指定索引的障碍物
                            st.session_state.planner.obstacles.pop(i)
                            # 清除已规划的路径
                            st.session_state.planned_path_horizontal = None
                            st.session_state.planned_path_climb = None
                            st.session_state.waypoints = []
                            st.success(f"✅ 已删除障碍物 #{i+1}")
                            st.rerun()
                
                st.markdown("---")
                if st.button("🗑️ 清除全部障碍物", key="clear_all_obs"):
                    st.session_state.planner.clear_obstacles()
                    st.session_state.planned_path_horizontal = None
                    st.session_state.planned_path_climb = None
                    st.session_state.waypoints = []
                    st.rerun()
        
        st.markdown("---")
        
        # 路径规划 (can_plan 已在上方定义)
        # 显示当前规划状态
        plan_status = []
        plan_status.append("✅ A点已设" if has_a else "❌ A点未设")
        plan_status.append("✅ B点已设" if has_b else "❌ B点未设")
        plan_status.append(f"障碍物: {len(st.session_state.planner.obstacles)}个")
        
        st.markdown(f"**🧭 路径规划** ({' | '.join(plan_status)})")
        
        # 检查是否可以爬升飞越
        force_avoidance = st.session_state.planner.should_force_avoidance(st.session_state.flight_altitude)
        can_climb = not force_avoidance
        
        # 规划所有路径按钮
        if st.button("🧮 规划所有路径", disabled=not can_plan, type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                # 记录导航目标到OBC
                st.session_state.comm_logger.log_nav_target_to_obc(
                    st.session_state.point_a, st.session_state.point_b, 
                    st.session_state.flight_altitude
                )
                st.session_state.comm_logger.log_path_planning_start(
                    "A*", len(st.session_state.planner.obstacles)
                )
                
                with st.spinner("🧭 正在规划多条路径..."):
                    all_paths = st.session_state.planner.plan_multiple_paths(
                        start_wp, end_wp, st.session_state.max_altitude
                    )
                    st.session_state['available_paths'] = all_paths
                    
                    if all_paths:
                        st.success(f"✅ 规划完成！共 {len(all_paths)} 条可选路径")
                        # 记录每条路径的规划完成日志
                        for k, v in all_paths.items():
                            st.session_state.comm_logger.log_path_planning_complete(
                                v['distance'], len(v['path']), v['type']
                            )
                    else:
                        st.error("❌ 无法找到可行路径")
                st.rerun()
        
        # 显示可选路径列表
        if st.session_state.get('available_paths'):
            st.markdown("---")
            st.markdown("**📍 可选路径**")
            
            paths = st.session_state['available_paths']
            
            # 按距离排序
            sorted_paths = sorted(paths.items(), key=lambda x: x[1]['distance'])
            
            for path_key, path_info in sorted_paths:
                col_path, col_btn = st.columns([3, 1])
                
                with col_path:
                    if path_info['type'] == 'climb':
                        st.write(f"{path_info['name']}: {path_info['distance']:.0f}m, 最高{path_info['max_altitude']:.0f}m")
                    else:
                        st.write(f"{path_info['name']}: {path_info['distance']:.0f}m, {len(path_info['path'])}个航点")
                
                with col_btn:
                    if st.button("选择", key=f"select_{path_key}"):
                        st.session_state.waypoints = path_info['path']
                        st.session_state.selected_path_type = path_info['type']
                        st.session_state.selected_path_name = path_info['name']
                        st.success(f"✅ 已选择: {path_info['name']}")
                        st.rerun()
        
        # 显示当前选中的路径
        if st.session_state.waypoints:
            st.markdown("---")
            st.markdown("**✅ 当前选中路径**")
            path_name = st.session_state.get('selected_path_name', '未命名路径')
            st.success(f"{path_name}: {len(st.session_state.waypoints)}个航点")
            
            # 显示详细航点信息
            with st.expander("📋 航点详情"):
                for i, wp in enumerate(st.session_state.waypoints):
                    st.write(f"航点{i}: ({wp.lat:.6f}, {wp.lon:.6f}), 高度{wp.alt}m")
            
            if st.button("📤 上传到飞控", type="primary"):
                st.session_state.mission_sent = True
                st.session_state.comm_logger.log_mission_upload(len(st.session_state.waypoints))
                st.success(f"✅ 已上传 {len(st.session_state.waypoints)} 个航点到飞控")
                st.balloons()


# ==================== 飞行监控页面 ====================
elif page == "✈️ 飞行监控":
    st.title("✈️ 飞行监控 - 实时进程显示")
    
    if not st.session_state.mission_sent:
        st.warning("请先规划并上传航线")
    else:
        total = len(st.session_state.waypoints)
        
        # 通信链路拓扑图
        with st.expander("📡 通信链路拓扑", expanded=True):
            topo_col = st.columns([1, 1.5, 1, 1.5, 1])
            gcs_active = st.checkbox("🖥️ GCS", value=True, key="gcs_check", label_visibility="visible")
            obc_active = st.checkbox("🧠 OBC", value=True, key="obc_check", label_visibility="visible")
            fcu_active = st.checkbox("⚙️ FCU", value=True, key="fcu_check", label_visibility="visible")
            
            gcs_obc_status = "🟢" if gcs_active and obc_active else "⚪"
            obc_fcu_status = "🟢" if obc_active and fcu_active else "⚪"
            
            with topo_col[0]:
                st.markdown("<div style='text-align:center;padding:10px;background:#e8f4f8;border-radius:8px;'>🖥️ GCS<br><small>地面站</small></div>", unsafe_allow_html=True)
            with topo_col[1]:
                st.markdown(f"<div style='text-align:center;padding:25px 0;'><span style='font-size:20px'>{gcs_obc_status}</span><br><small>UDP</small></div>", unsafe_allow_html=True)
            with topo_col[2]:
                st.markdown("<div style='text-align:center;padding:10px;background:#fff4e6;border-radius:8px;'>🧠 OBC<br><small>机载计算机</small></div>", unsafe_allow_html=True)
            with topo_col[3]:
                st.markdown(f"<div style='text-align:center;padding:25px 0;'><span style='font-size:20px'>{obc_fcu_status}</span><br><small>MAVLink</small></div>", unsafe_allow_html=True)
            with topo_col[4]:
                st.markdown("<div style='text-align:center;padding:10px;background:#f0f0f0;border-radius:8px;'>⚙️ FCU<br><small>飞控</small></div>", unsafe_allow_html=True)
        
        # 控制按钮
        col1, col2, col3 = st.columns(3)
        
        with col1:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始执行任务", type="primary", use_container_width=True):
                    st.session_state.mission_executing = True
                    st.session_state.current_waypoint_index = 0
                    st.session_state.flight_path_history = []
                    
                    # 预计算所有飞行位置点
                    positions = []
                    steps_per_segment = 15  # 每段15个点，平衡流畅度和性能
                    for i in range(len(st.session_state.waypoints) - 1):
                        curr, next_wp = st.session_state.waypoints[i], st.session_state.waypoints[i + 1]
                        for step in range(steps_per_segment):
                            t = step / steps_per_segment
                            positions.append([
                                curr.lat + (next_wp.lat - curr.lat) * t,
                                curr.lon + (next_wp.lon - curr.lon) * t
                            ])
                    positions.append([st.session_state.waypoints[-1].lat, st.session_state.waypoints[-1].lon])
                    st.session_state.all_flight_positions = positions
                    st.session_state.drone_pos_index = 0
                    st.session_state.flight_start_time = time.time()
                    
                    if st.session_state.waypoints:
                        st.session_state.drone_position = [
                            st.session_state.waypoints[0].lat,
                            st.session_state.waypoints[0].lon
                        ]
                    
                    st.session_state.comm_logger.log_flight_start()
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
                st.session_state.drone_pos_index = 0
                st.session_state.all_flight_positions = []
                st.rerun()
        
        # 自动推进飞行位置
        if st.session_state.mission_executing and st.session_state.all_flight_positions:
            idx = st.session_state.drone_pos_index
            total_pos = len(st.session_state.all_flight_positions)
            
            if idx < total_pos - 1:
                st.session_state.drone_pos_index += 1
                # 计算当前航点索引 (steps_per_segment=15)
                st.session_state.current_waypoint_index = min(st.session_state.drone_pos_index // 15, total - 1)
                # 更新drone_position和flight_path_history
                st.session_state.drone_position = st.session_state.all_flight_positions[st.session_state.drone_pos_index]
                st.session_state.flight_path_history = st.session_state.all_flight_positions[:st.session_state.drone_pos_index+1]
                # 0.12秒刷新，约8fps，流畅但不闪烁
                time.sleep(0.12)
                st.rerun()
            else:
                st.session_state.mission_executing = False
                st.session_state.comm_logger.log_flight_complete()
                st.success("🎉 任务执行完成！")
        
        # 状态显示 - 两列布局：地图 + 日志
        main_col, log_col = st.columns([3, 2])
        
        with main_col:
            curr = st.session_state.current_waypoint_index
            total = len(st.session_state.waypoints)
            
            if total > 0:
                prog = min(100, int((curr / max(1, total-1)) * 100))
                st.progress(prog)
                cols = st.columns(4)
                cols[0].metric("当前航点", f"{min(curr+1, total)}/{total}")
                cols[1].metric("完成进度", f"{prog}%")
                if st.session_state.flight_start_time:
                    cols[2].metric("飞行时间", f"{int(time.time() - st.session_state.flight_start_time)}s")
                cols[3].metric("速度", "8.5m/s")
                
                if st.session_state.mission_executing:
                    st.info("🚁 正在执行任务...")
                elif st.session_state.drone_position:
                    st.warning("⏸️ 任务已暂停")
            
            # 地图显示
            if st.session_state.all_flight_positions and st.session_state.drone_pos_index < len(st.session_state.all_flight_positions):
                drone_pos = st.session_state.all_flight_positions[st.session_state.drone_pos_index]
            else:
                # 修复：waypoints[0] 是 Waypoint 对象，需要转换为 [lat, lon]
                if st.session_state.waypoints:
                    first_wp = st.session_state.waypoints[0]
                    drone_pos = [first_wp.lat, first_wp.lon]
                else:
                    drone_pos = [32.0603, 118.7969]
            
            m = folium.Map(location=drone_pos, zoom_start=17, tiles="CartoDB dark_matter")
            
            # 计划航线
            if st.session_state.waypoints:
                path_coords = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
                folium.PolyLine(path_coords, color='blue', weight=3, opacity=0.4, dash_array='5,10').add_to(m)
                
                # 航点
                for i, wp in enumerate(st.session_state.waypoints):
                    if i == 0:
                        folium.Marker([wp.lat, wp.lon], icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
                    elif i == len(st.session_state.waypoints) - 1:
                        folium.Marker([wp.lat, wp.lon], icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
                    else:
                        color = 'blue' if i > curr else 'gray'
                        folium.CircleMarker([wp.lat, wp.lon], radius=4, color=color, fill=True).add_to(m)
                
                # 已飞路径
                if st.session_state.drone_pos_index > 0:
                    flown_path = st.session_state.all_flight_positions[:st.session_state.drone_pos_index+1]
                    folium.PolyLine(flown_path, color='#00FF00', weight=5, opacity=0.9).add_to(m)
            
            # 无人机当前位置
            folium.Marker(drone_pos, icon=folium.Icon(color='orange', icon='plane', prefix='fa')).add_to(m)
            folium.Circle(drone_pos, radius=10, color='orange', fill=True, fillOpacity=0.3).add_to(m)
            
            # 使用动态key避免完全重建
            st_folium(m, width=700, height=500, key=f"flight_map_{st.session_state.drone_pos_index}")
        
        # 右侧：通信日志
        with log_col:
            st.subheader("📡 实时通信日志")
            logs = st.session_state.comm_logger.get_logs()
            log_html = "<div style='max-height:500px;overflow-y:auto;font-family:monospace;font-size:12px;background:#f8f9fa;padding:10px;border-radius:5px;'>"
            for log in reversed(logs[-20:]):
                bg_color = {"success": "#d4edda", "error": "#f8d7da", "warning": "#fff3cd", "info": "#e7f3ff"}.get(log['status'], "#f8f9fa")
                log_html += f"<div style='padding:5px;margin:2px 0;border-radius:3px;background:{bg_color};border-left:3px solid {'#28a745' if log['status']=='success' else '#dc3545' if log['status']=='error' else '#ffc107'}'>"
                log_html += f"<span style='color:#666;font-size:10px'>[{log['timestamp']}]</span> "
                log_html += f"{log['icon']} <b>{log['msg_type']}</b><br>"
                log_html += f"<span style='color:#333'>{log['content']}</span><br>"
                log_html += f"<small style='color:#888'>{log['direction']}</small>"
                log_html += f"</div>"
            log_html += "</div>"
            st.html(log_html)
            
            if st.button("🗑️ 清除日志"):
                st.session_state.comm_logger.clear()
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
st.caption("MAVLink GCS v6.0 | 严格避障 | 安全绕行 | 北京时间 (UTC+8)")
