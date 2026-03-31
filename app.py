import streamlit as st
import streamlit.components.v1 as components
import time
import math
import heapq
import json
import os
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, AntPath
from streamlit_folium import st_folium

# ==================== 版本信息 ====================
VERSION = "v12.3"
VERSION_NAME = "紧凑布局版"
# 配置文件保存路径
OBSTACLE_CONFIG_FILE = r"C:\Users\77463\obstacle_config.json"

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
    page_title=f"MAVLink 地面站 - 安全避障系统 {VERSION}",
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
    
    def plan_takeoff_escape(self, start_wp, flight_alt):
        """
        【新增】规划起飞避让路径 - 当起点在障碍物内时，先飞出来
        尝试向四个正方向寻找最近的出口
        """
        start = (start_wp.lat, start_wp.lon)
        
        # 检查是否真的在障碍物内
        if not self.is_collision(start[0], start[1], flight_alt):
            return None, "起点不在障碍物内，无需避让起飞"
        
        # 寻找最近的出口方向
        directions = [
            (0, 1),    # 北
            (0, -1),   # 南
            (1, 0),    # 东
            (-1, 0),   # 西
            (1, 1),    # 东北
            (1, -1),   # 东南
            (-1, 1),   # 西北
            (-1, -1),  # 西南
        ]
        
        best_exit = None
        best_dist = float('inf')
        
        for dlat, dlon in directions:
            # 沿该方向逐步搜索，直到找到安全点
            for step in range(1, 50):  # 最多搜索50步
                test_lat = start[0] + dlat * step * 0.0001  # 约11米/步
                test_lon = start[1] + dlon * step * 0.0001 / math.cos(math.radians(start[0]))
                
                if not self.is_collision(test_lat, test_lon, flight_alt):
                    # 找到出口，计算距离
                    dist = self.haversine_distance(start[0], start[1], test_lat, test_lon)
                    if dist < best_dist:
                        best_dist = dist
                        best_exit = (test_lat, test_lon)
                    break
        
        if best_exit:
            # 构建起飞避让路径：起点 → 出口点 → 终点
            escape_wp = Waypoint(best_exit[0], best_exit[1], flight_alt, 16, 1)
            return escape_wp, f"起飞避让路径: 先向安全区域飞行{best_dist:.1f}米"
        
        return None, "无法找到起飞避让出口"
    
    def plan_landing_approach(self, end_wp, flight_alt):
        """
        【新增】规划终点悬停路径 - 当终点在障碍物内时，找最近的安全悬停点
        尝试向四个正方向寻找最近的入口点（离终点最近的安全点）
        """
        end = (end_wp.lat, end_wp.lon)
        
        # 检查是否真的在障碍物内
        if not self.is_collision(end[0], end[1], flight_alt):
            return None, "终点不在障碍物内，无需悬停接近"
        
        # 寻找最近的入口方向（从终点向外）
        directions = [
            (0, 1),    # 北
            (0, -1),   # 南
            (1, 0),    # 东
            (-1, 0),   # 西
            (1, 1),    # 东北
            (1, -1),   # 东南
            (-1, 1),   # 西北
            (-1, -1),  # 西南
        ]
        
        best_entry = None
        best_dist = float('inf')
        
        for dlat, dlon in directions:
            # 沿该方向逐步搜索，直到找到安全点
            for step in range(1, 50):  # 最多搜索50步
                test_lat = end[0] + dlat * step * 0.0001  # 约11米/步
                test_lon = end[1] + dlon * step * 0.0001 / math.cos(math.radians(end[0]))
                
                if not self.is_collision(test_lat, test_lon, flight_alt):
                    # 找到入口，计算到终点的距离
                    dist = self.haversine_distance(test_lat, test_lon, end[0], end[1])
                    if dist < best_dist:
                        best_dist = dist
                        best_entry = (test_lat, test_lon)
                    break
        
        if best_entry:
            # 构建悬停点
            hover_wp = Waypoint(best_entry[0], best_entry[1], flight_alt, 16, 0)
            return hover_wp, f"终点悬停: 在距离终点{best_dist:.1f}米处悬停"
        
        return None, "无法找到合适的终点悬停点"
    
    def plan_horizontal_avoidance(self, start_wp, end_wp, bias=0):
        """
        【严格】强制水平绕行路径规划 - 绝不穿行
        bias: 绕行偏向 (-1=左偏/西侧, 0=最优, 1=右偏/东侧)
        左偏(bias<0): 鼓励向西绕行 (x减小方向)
        右偏(bias>0): 鼓励向东绕行 (x增大方向)
        """
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        flight_alt = start_wp.alt
        
        # 【修改】处理起点在障碍物内的情况 - 先规划起飞避让
        if self.is_collision(start[0], start[1], flight_alt):
            escape_wp, escape_msg = self.plan_takeoff_escape(start_wp, flight_alt)
            if escape_wp is None:
                return None, f"起点在障碍物安全边界内，且{escape_msg}"
            # 成功找到起飞避让点，继续规划从避让点到终点的路径
            # 【修改】处理终点在障碍物内的情况
            if self.is_collision(end[0], end[1], flight_alt):
                hover_wp, hover_msg = self.plan_landing_approach(end_wp, flight_alt)
                if hover_wp is None:
                    return None, f"起点需避让，但{hover_msg}"
                # 规划到悬停点的路径
                result, msg = self._plan_path_from_escape(escape_wp, hover_wp, flight_alt, bias)
                if result:
                    full_path = [start_wp, escape_wp] + result[1:]
                    return full_path, f"【起飞避让+终点悬停】{escape_msg}，飞行至目标附近后{hover_msg}"
                else:
                    return None, f"起飞避让成功，但到悬停点路径规划失败"
            # 终点安全，正常规划
            result, msg = self._plan_path_from_escape(escape_wp, end_wp, flight_alt, bias)
            if result:
                full_path = [start_wp, escape_wp] + result[1:]
                return full_path, f"【起飞避让】{escape_msg}，然后{msg}"
            else:
                return None, f"起飞避让成功，但后续路径规划失败: {msg}"
        
        # 【修改】处理终点在障碍物内的情况 - 寻找悬停点
        if self.is_collision(end[0], end[1], flight_alt):
            hover_wp, hover_msg = self.plan_landing_approach(end_wp, flight_alt)
            if hover_wp is None:
                return None, f"终点在障碍物安全边界内，且{hover_msg}"
            # 规划到悬停点的路径
            result, msg = self._plan_path_from_start(start_wp, hover_wp, flight_alt, bias)
            if result:
                return result, f"【终点悬停】{msg}，到达后{hover_msg}"
            else:
                return None, f"到悬停点路径规划失败: {msg}"
        
        # 【关键】首先检查直线路径是否安全
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [start_wp, end_wp], "直线路径安全"
        
        # 获取包含障碍物的边界框
        lat_min, lat_max, lon_min, lon_max = self.get_bounding_box_with_obstacles(start, end)
        
        # 根据偏向调整边界框（实现左右绕行）
        lat_range = lat_max - lat_min
        lon_range = lon_max - lon_min
        if bias < 0:  # 左偏 - 大幅扩展西侧边界
            lon_min -= lon_range * 0.5
            lon_max -= lon_range * 0.1  # 收缩东侧
        elif bias > 0:  # 右偏 - 大幅扩展东侧边界
            lon_max += lon_range * 0.5
            lon_min += lon_range * 0.1  # 收缩西侧
        
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
                
                # 启发式函数 - 基础距离
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                
                # 【关键】添加绕行偏向到启发函数
                # bias < 0 (左偏): 鼓励向西(nx较小)，惩罚向东(nx较大)
                # bias > 0 (右偏): 鼓励向东(nx较大)，惩罚向西(nx较小)
                dx_from_start = nx - start_grid[0]
                if bias < 0:  # 左绕行 - 优先向西
                    # 向东移动(nx大)时增加成本，向西移动(nx小)时减少成本
                    h += dx_from_start * self.grid_size * 0.8
                elif bias > 0:  # 右绕行 - 优先向东
                    # 向西移动(nx小)时增加成本，向东移动(nx大)时减少成本
                    h -= dx_from_start * self.grid_size * 0.8
                
                heapq.heappush(open_set, (new_g_cost + h, new_g_cost, nx, ny, path + [(nx, ny)]))
        
        if best_path is not None:
            # 最终验证
            if self.validate_path(best_path, flight_alt):
                return best_path, "规划成功"
            else:
                return None, "路径验证失败"
        
        return None, "无法找到可行的绕行路径"
    
    def _plan_path_from_start(self, start_wp, hover_wp, flight_alt, bias=0):
        """
        【辅助方法】从起点规划到终点悬停点的路径
        这是 plan_horizontal_avoidance 的简化版本，终点是悬停点
        """
        start = (start_wp.lat, start_wp.lon)
        end = (hover_wp.lat, hover_wp.lon)
        
        # 检查直线路径
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [start_wp, hover_wp], "直线路径安全"
        
        # 复用主规划逻辑
        lat_min, lat_max, lon_min, lon_max = self.get_bounding_box_with_obstacles(start, end)
        
        lat_range = lat_max - lat_min
        lon_range = lon_max - lon_min
        if bias < 0:
            lon_min -= lon_range * 0.5
            lon_max -= lon_range * 0.1
        elif bias > 0:
            lon_max += lon_range * 0.5
            lon_min += lon_range * 0.1
        
        base_lat = lat_min
        base_lon = lon_min
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        directions = [
            (0,1), (1,0), (0,-1), (-1,0),
            (1,1), (1,-1), (-1,1), (-1,-1),
            (0,2), (2,0), (0,-2), (-2,0),
            (2,2), (2,-2), (-2,2), (-2,-2),
            (0,3), (3,0), (0,-3), (-3,0),
            (1,2), (2,1), (-1,2), (-2,1), (1,-2), (2,-1), (-1,-2), (-2,-1),
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
                waypoints.append(hover_wp)
                waypoints[-1].seq = len(waypoints) - 1
                waypoints = self.smooth_path(waypoints, flight_alt)
                
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
                if (nx, ny) in visited and visited[(nx, ny)] <= new_g_cost:
                    continue
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                if bias < 0:
                    h += (nx - start_grid[0]) * self.grid_size * 0.8
                elif bias > 0:
                    h -= (nx - start_grid[0]) * self.grid_size * 0.8
                heapq.heappush(open_set, (new_g_cost + h, new_g_cost, nx, ny, path + [(nx, ny)]))
        
        if best_path is not None:
            if self.validate_path(best_path, flight_alt):
                return best_path, "规划成功"
        return None, "无法找到可行路径"
    
    def _plan_path_from_escape(self, escape_wp, end_wp, flight_alt, bias=0):
        """
        【辅助方法】从起飞避让点规划到终点的路径
        这是 plan_horizontal_avoidance 的简化版本，起点已经确保安全
        """
        start = (escape_wp.lat, escape_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        
        # 检查直线路径
        if not self.line_hits_obstacle(start, end, flight_alt):
            return [escape_wp, end_wp], "直线路径安全"
        
        # 复用主规划逻辑
        lat_min, lat_max, lon_min, lon_max = self.get_bounding_box_with_obstacles(start, end)
        
        lat_range = lat_max - lat_min
        lon_range = lon_max - lon_min
        if bias < 0:
            lon_min -= lon_range * 0.5
            lon_max -= lon_range * 0.1
        elif bias > 0:
            lon_max += lon_range * 0.5
            lon_min += lon_range * 0.1
        
        base_lat = lat_min
        base_lon = lon_min
        
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        directions = [
            (0,1), (1,0), (0,-1), (-1,0),
            (1,1), (1,-1), (-1,1), (-1,-1),
            (0,2), (2,0), (0,-2), (-2,0),
            (2,2), (2,-2), (-2,2), (-2,-2),
            (0,3), (3,0), (0,-3), (-3,0),
            (1,2), (2,1), (-1,2), (-2,1), (1,-2), (2,-1), (-1,-2), (-2,-1),
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
                waypoints = [escape_wp]
                for grid in path[1:]:
                    lat, lon = self.grid_to_latlon(grid[0], grid[1], base_lat, base_lon)
                    waypoints.append(Waypoint(lat, lon, flight_alt, 16, len(waypoints)))
                waypoints.append(end_wp)
                waypoints[-1].seq = len(waypoints) - 1
                waypoints = self.smooth_path(waypoints, flight_alt)
                
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
                if (nx, ny) in visited and visited[(nx, ny)] <= new_g_cost:
                    continue
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                if bias < 0:
                    h += (nx - start_grid[0]) * self.grid_size * 0.8
                elif bias > 0:
                    h -= (nx - start_grid[0]) * self.grid_size * 0.8
                heapq.heappush(open_set, (new_g_cost + h, new_g_cost, nx, ny, path + [(nx, ny)]))
        
        if best_path is not None:
            if self.validate_path(best_path, flight_alt):
                return best_path, "规划成功"
        return None, "无法找到可行路径"
    
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


# ==================== 障碍物配置持久化 ====================
def save_obstacles_to_file(obstacles, filename=OBSTACLE_CONFIG_FILE):
    """保存障碍物配置到JSON文件"""
    config = {
        'version': VERSION,
        'save_time': (datetime.utcnow() + timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S'),
        'obstacles': []
    }
    for obs in obstacles:
        obs_data = {
            'type': obs.type,
            'points': obs.points,
            'height': obs.height,
            'name': obs.name,
            'rotation': obs.rotation,
            'width': obs.width,
            'height_m': obs.height_m,
            'radius': getattr(obs, 'radius', 30)
        }
        config['obstacles'].append(obs_data)
    
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
        return True, len(config['obstacles'])
    except Exception as e:
        return False, str(e)

def load_obstacles_from_file(filename=OBSTACLE_CONFIG_FILE):
    """从JSON文件加载障碍物配置"""
    if not os.path.exists(filename):
        return None, "配置文件不存在"
    
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            config = json.load(f)
        return config, None
    except Exception as e:
        return None, str(e)

def apply_obstacles_config(planner, config):
    """应用障碍物配置到规划器"""
    planner.clear_obstacles()
    count = 0
    for o in config.get('obstacles', []):
        if o['type'] == 'circle':
            planner.add_circle_obstacle(
                o['points'][0][0], o['points'][0][1], o['radius'], o['height'], o['name'])
        elif o['type'] == 'rectangle':
            planner.add_rotated_rectangle_obstacle(
                o['points'][0][0], o['points'][0][1], o['width'], o['height_m'], 
                o['rotation'], o['height'], o['name'])
        else:
            planner.add_polygon_obstacle(o['points'], o['height'], o['name'])
        count += 1
    return count

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
        # 障碍物记忆
        'saved_obstacles': [],
        # 起飞避让和终点悬停标记
        'has_takeoff_escape': False,
        'has_landing_hover': False,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 启动时自动加载障碍物配置 ====================
if 'auto_load_done' not in st.session_state:
    st.session_state.auto_load_done = True
    config, error = load_obstacles_from_file()
    if config:
        count = apply_obstacles_config(st.session_state.planner, config)
        st.session_state.saved_obstacles = config.get('obstacles', [])
        st.session_state.last_loaded_config = config
        if count > 0:
            st.toast(f"🎯 已自动加载 {count} 个障碍物配置", icon="✅")


# ==================== 侧边栏 ====================
with st.sidebar:
    # 导航
    st.markdown("**🧭 导航**")
    page = st.radio("功能", ["🗺️ 航线规划", "✈️ 飞行监控"], label_visibility="collapsed")
    
    # 坐标系设置
    st.markdown("**⚙️ 坐标系**")
    coord_opt = ["WGS-84", "GCJ-02(高德/百度)"]
    sel = st.radio("坐标系", coord_opt, 
                   index=0 if st.session_state.coord_system=='WGS-84' else 1,
                   label_visibility="collapsed")
    st.session_state.coord_system = 'WGS-84' if 'WGS' in sel else 'GCJ-02'
    
    # 系统状态 - 紧凑布局
    st.markdown("**📊 系统状态**")
    c1, c2 = st.columns(2)
    with c1:
        st.caption("A点" + ("✅" if st.session_state.point_a else "❌"))
    with c2:
        st.caption("B点" + ("✅" if st.session_state.point_b else "❌"))
    
    # 关键指标 - 横向排列
    m1, m2 = st.columns(2)
    with m1:
        st.metric("障碍物", len(st.session_state.planner.obstacles), label_visibility="collapsed")
    with m2:
        st.metric("安全半径", f"{st.session_state.planner.safety_margin}m", label_visibility="collapsed")
    
    max_obs_h = st.session_state.planner.get_max_obstacle_height()
    if max_obs_h > 0:
        st.caption(f"最高障碍: {max_obs_h}m")
        if max_obs_h >= st.session_state.flight_altitude:
            st.error("⚠️ 强制绕行", icon=None)
    
    if st.session_state.waypoints:
        st.caption(f"航点: {len(st.session_state.waypoints)}个")
        if st.session_state.get('has_takeoff_escape'):
            st.warning("🚨 起飞需避让", icon=None)
        if st.session_state.get('has_landing_hover'):
            st.info("🚁 终点悬停", icon=None)
    
    # 通信链路 - 紧凑显示
    st.markdown("**📡 通信**")
    logs = st.session_state.comm_logger.get_logs()
    if logs:
        for log in list(logs)[-3:]:
            status_emoji = {"success": "🟢", "error": "🔴", "warning": "🟡", "info": "🔵"}.get(log['status'], "⚪")
            st.caption(f"{status_emoji} {log['msg_type']}: {log['content'][:20]}...")
    else:
        st.caption("暂无记录")
    
    # 版本信息
    st.divider()
    st.caption(f"📦 {VERSION} {VERSION_NAME}")


# ==================== 航线规划页面 ====================
if page == "🗺️ 航线规划":
    st.title(f"🚁 MAVLink 地面站 - 严格避障系统 {VERSION}")
    st.caption(f"{VERSION_NAME} | 绝对安全绕行 | 严格碰撞检测 | 坐标系自动转换 | 障碍物持久化")
    
    # 显示安全警告
    if st.session_state.planner.should_force_avoidance(st.session_state.flight_altitude):
        st.error("⚠️ **强制绕行模式激活**：存在高于或等于飞行高度的障碍物，系统将严格绕行")
    
    # ========== 第一行：地图 + 紧凑控制面板 ==========
    col_map, col_ctrl = st.columns([2.5, 1])
    
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
        
        map_data = st_folium(m, width=700, height=350, key="main_map")
        
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
        st.markdown("**⚙️ 控制面板**")
        
        # A/B点设置 - 紧凑横向布局
        st.caption("📍 起点 A")
        ca1, ca2, ca3 = st.columns([3, 3, 1])
        with ca1:
            lat_a = st.number_input(" lat", value=32.2323 if not st.session_state.point_a else st.session_state.point_a[0], format="%.6f", key="lat_a", label_visibility="collapsed")
        with ca2:
            lon_a = st.number_input(" lon", value=118.7496 if not st.session_state.point_a else st.session_state.point_a[1], format="%.6f", key="lon_a", label_visibility="collapsed")
        with ca3:
            if st.button("⌗", key="def_a"):
                st.session_state.point_a = (32.2323, 118.7496)
                st.rerun()
        
        if st.button("✅ 设A点", key="set_a", use_container_width=True):
            lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat_a, lon_a, st.session_state.coord_system)
            st.session_state.point_a = (lat_wgs, lon_wgs)
            st.rerun()
        
        st.caption("📍 终点 B")
        cb1, cb2, cb3 = st.columns([3, 3, 1])
        with cb1:
            lat_b = st.number_input(" lat", value=32.2344 if not st.session_state.point_b else st.session_state.point_b[0], format="%.6f", key="lat_b", label_visibility="collapsed")
        with cb2:
            lon_b = st.number_input(" lon", value=118.7493 if not st.session_state.point_b else st.session_state.point_b[1], format="%.6f", key="lon_b", label_visibility="collapsed")
        with cb3:
            if st.button("⌗", key="def_b"):
                st.session_state.point_b = (32.2344, 118.7493)
                st.rerun()
        
        if st.button("✅ 设B点", key="set_b", use_container_width=True):
            lat_wgs, lon_wgs = CoordinateConverter.from_user_input(lat_b, lon_b, st.session_state.coord_system)
            st.session_state.point_b = (lat_wgs, lon_wgs)
            st.rerun()
        
        st.divider()
        
        # 飞行参数 - 紧凑布局
        st.caption("✈️ 飞行参数")
        c1, c2 = st.columns(2)
        with c1:
            new_alt = st.number_input("高度(m)", 10, 200, st.session_state.flight_altitude, key="flight_alt")
        with c2:
            new_safety = st.number_input("安全半径(m)", 5, 20, st.session_state.planner.safety_margin, key="safety_margin")
        
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
        if new_safety != st.session_state.planner.safety_margin:
            st.session_state.planner.safety_margin = new_safety
        
        # 高度警告 - 紧凑显示
        max_obs_h = st.session_state.planner.get_max_obstacle_height()
        if max_obs_h > 0:
            if max_obs_h >= st.session_state.flight_altitude:
                st.error(f"⚠️ 障碍物({max_obs_h}m)高于飞行高度！", icon=None)
            elif max_obs_h >= st.session_state.flight_altitude - 20:
                st.warning(f"障碍物({max_obs_h}m)接近飞行高度", icon=None)
        
        st.divider()
        
        # 障碍物管理 - 简化标题
        st.caption("🧱 障碍物管理")
        
        # 计算 can_plan (必须在调试信息之前定义)
        has_a = st.session_state.point_a is not None
        has_b = st.session_state.point_b is not None
        can_plan = has_a and has_b
        
        # ====== 优先处理地图圈选的待确认障碍物 ======
        # 方式1: 在地图上圈选提示
        st.caption("💡 在左侧地图圈选障碍物")
        
        if st.session_state.pending_drawing:
            drawing = st.session_state.pending_drawing
            st.error("🚨 确认地图圈选的障碍物")
            
            if drawing['type'] == 'circle':
                lat, lon = drawing['center']
                st.caption(f"⭕ 圆形: ({lat:.5f}, {lon:.5f}) R{drawing['radius']:.0f}m")
            else:
                st.caption(f"📐 多边形: {len(drawing['points'])}顶点")
            
            map_obs_height = st.number_input("高度(m)", 5, 300, 
                max(st.session_state.flight_altitude + 10, 50), key="map_obs_height_v3")
            
            c1, c2 = st.columns(2)
            with c1:
                if st.button("✅ 确认", type="primary", key="confirm_map_obs"):
                    if drawing['type'] == 'circle':
                        lat, lon = drawing['center']
                        st.session_state.planner.add_circle_obstacle(
                            lat, lon, drawing['radius'], map_obs_height, f"圆形({map_obs_height}m)")
                    else:
                        st.session_state.planner.add_polygon_obstacle(
                            drawing['points'], map_obs_height, f"多边形({map_obs_height}m)")
                    st.session_state.pending_drawing = None
                    st.rerun()
            with c2:
                if st.button("❌ 取消", key="cancel_map_obs"):
                    st.session_state.pending_drawing = None
                    st.rerun()
            st.divider()
        
        # 方式2: 手动输入坐标添加
        with st.expander("➕ 方式2: 手动输入坐标添加", expanded=False):
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
        
        st.divider()
        
        # 路径规划 - 紧凑布局
        st.caption("🧭 路径规划")
        
        # 状态提示
        if st.session_state.point_a and st.session_state.planner.is_collision(
            st.session_state.point_a[0], st.session_state.point_a[1], st.session_state.flight_altitude):
            st.warning("🚨 起点在障碍物内", icon=None)
        if st.session_state.point_b and st.session_state.planner.is_collision(
            st.session_state.point_b[0], st.session_state.point_b[1], st.session_state.flight_altitude):
            st.info("🚁 终点在障碍物内", icon=None)
        
        # 规划按钮
        if st.button("🧮 规划路径", disabled=not can_plan, type="primary", use_container_width=True):
            if st.session_state.point_a and st.session_state.point_b:
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], 
                                   st.session_state.flight_altitude, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], 
                                 st.session_state.flight_altitude, 16)
                
                st.session_state.comm_logger.log_nav_target_to_obc(
                    st.session_state.point_a, st.session_state.point_b, st.session_state.flight_altitude)
                st.session_state.comm_logger.log_path_planning_start("A*", len(st.session_state.planner.obstacles))
                
                with st.spinner("规划中..."):
                    all_paths = st.session_state.planner.plan_multiple_paths(start_wp, end_wp, st.session_state.max_altitude)
                    st.session_state['available_paths'] = all_paths
                    
                    if all_paths:
                        st.success(f"✅ {len(all_paths)}条路径")
                        for k, v in all_paths.items():
                            st.session_state.comm_logger.log_path_planning_complete(v['distance'], len(v['path']), v['type'])
                    else:
                        st.error("❌ 无可用路径")
                st.rerun()
        
        # 显示当前选中路径
        if st.session_state.waypoints:
            st.caption(f"✅ {st.session_state.get('selected_path_name', '路径')}: {len(st.session_state.waypoints)}航点")
            if st.button("📤 上传飞控", type="primary", use_container_width=True):
                st.session_state.mission_sent = True
                st.session_state.comm_logger.log_mission_upload(len(st.session_state.waypoints))
                st.success(f"✅ 已上传{len(st.session_state.waypoints)}航点")
                st.balloons()

    # ========== 第二行：地图下方的功能区 ==========
    st.divider()
    
    # 功能标签页：快速指南 | 障碍物管理 | 路径选择 | 航点详情
    tab_guide, tab1, tab2, tab3 = st.tabs(["📖 指南", f"🧱 障碍物({len(st.session_state.planner.obstacles)})", "📍 路径选择", "📋 航点详情"])
    
    with tab_guide:
        # 快速指南内容
        g1, g2, g3, g4 = st.columns(4)
        with g1:
            st.markdown("**📋 操作步骤**")
            st.caption("1. 设置A/B点坐标\n2. 添加障碍物\n3. 点击规划路径")
        with g2:
            st.markdown("**⚠️ 安全保证**")
            st.caption(f"• {st.session_state.planner.safety_margin}米安全边距\n• 强制绕行高障碍物")
        with g3:
            st.markdown("**🔧 当前配置**")
            st.caption(f"• 网格: {st.session_state.planner.grid_size}米\n• 飞行高度: {st.session_state.flight_altitude}米")
        with g4:
            max_h = st.session_state.planner.get_max_obstacle_height()
            st.markdown("**📊 状态**")
            st.caption(f"• 障碍物: {len(st.session_state.planner.obstacles)}个\n• 最高: {max_h}米")
    
    with tab1:
        # 障碍物列表和持久化
        if st.session_state.planner.obstacles:
            cols = st.columns([1, 1, 1, 1])
            with cols[0]:
                if st.button("💾 保存", key="save_to_file"):
                    success, result = save_obstacles_to_file(st.session_state.planner.obstacles)
                    st.success(f"✅ 已保存{result}个") if success else st.error(f"❌ {result}")
            with cols[1]:
                if st.button("📂 加载", key="load_from_file"):
                    config, error = load_obstacles_from_file()
                    if config:
                        count = apply_obstacles_config(st.session_state.planner, config)
                        st.success(f"✅ 加载{count}个")
                        st.rerun()
                    else:
                        st.error(f"❌ {error}")
            with cols[2]:
                if st.button("🗑️ 清除", key="clear_all_obs"):
                    st.session_state.planner.clear_obstacles()
                    st.rerun()
            with cols[3]:
                if st.button("🚀 部署", key="one_click_deploy"):
                    config, error = load_obstacles_from_file()
                    if config:
                        count = apply_obstacles_config(st.session_state.planner, config)
                        st.success(f"🚀 部署{count}个")
                        st.rerun()
            
            # 下载配置
            if os.path.exists(OBSTACLE_CONFIG_FILE):
                with open(OBSTACLE_CONFIG_FILE, 'r', encoding='utf-8') as f:
                    st.download_button("⬇️ 下载配置", f.read(), "obstacle_config.json", 
                                     mime="application/json", key="download_json")
            
            # 障碍物列表
            st.caption("障碍物列表")
            obs_cols = st.columns(4)
            for i, obs in enumerate(st.session_state.planner.obstacles):
                with obs_cols[i % 4]:
                    icon = "⭕" if obs.type == "circle" else "⬜" if obs.type == "rectangle" else "📐"
                    color = "🔴" if obs.height >= st.session_state.flight_altitude else "🟢"
                    st.caption(f"{color} {icon} #{i+1}: {obs.height}m")
                    if st.button("🗑️", key=f"del_obs_{i}"):
                        st.session_state.planner.obstacles.pop(i)
                        st.rerun()
        else:
            st.info("暂无障碍物")
    
    with tab2:
        # 可选路径列表
        if st.session_state.get('available_paths'):
            paths = st.session_state['available_paths']
            sorted_paths = sorted(paths.items(), key=lambda x: x[1]['distance'])
            
            for path_key, path_info in sorted_paths:
                col_path, col_btn = st.columns([4, 1])
                with col_path:
                    first_wp = path_info['path'][0]
                    last_wp = path_info['path'][-1]
                    is_escape = len(path_info['path']) > 2 and (
                        abs(first_wp.lat - st.session_state.point_a[0]) > 0.00001 or
                        abs(first_wp.lon - st.session_state.point_a[1]) > 0.00001)
                    is_hover = len(path_info['path']) > 2 and (
                        abs(last_wp.lat - st.session_state.point_b[0]) > 0.00001 or
                        abs(last_wp.lon - st.session_state.point_b[1]) > 0.00001)
                    
                    badge = "🚨" if is_escape else ""
                    badge += "🚁" if is_hover else ""
                    
                    if path_info['type'] == 'climb':
                        st.write(f"{badge} {path_info['name']}: {path_info['distance']:.0f}m, 最高{path_info['max_altitude']:.0f}m")
                    else:
                        st.write(f"{badge} {path_info['name']}: {path_info['distance']:.0f}m, {len(path_info['path'])}航点")
                
                with col_btn:
                    if st.button("选择", key=f"select_{path_key}"):
                        st.session_state.waypoints = path_info['path']
                        st.session_state.selected_path_type = path_info['type']
                        st.session_state.selected_path_name = path_info['name']
                        st.session_state.has_takeoff_escape = is_escape
                        st.session_state.has_landing_hover = is_hover
                        st.rerun()
        else:
            st.info("请先规划路径")
    
    with tab3:
        # 航点详情
        if st.session_state.waypoints:
            path_name = st.session_state.get('selected_path_name', '未命名')
            st.caption(f"当前路径: {path_name}")
            wp_data = []
            for i, wp in enumerate(st.session_state.waypoints):
                wp_data.append({"序号": i, "纬度": f"{wp.lat:.6f}", "经度": f"{wp.lon:.6f}", "高度": f"{wp.alt}m"})
            st.dataframe(wp_data, use_container_width=True, hide_index=True)
        else:
            st.info("未选择路径")


# ==================== 飞行监控页面 ====================
elif page == "✈️ 飞行监控":
    st.title("🚁 飞行实时画面 - 任务执行监控")
    
    if not st.session_state.mission_sent:
        st.warning("⚠️ 请先规划并上传航线")
    else:
        total_wp = len(st.session_state.waypoints)
        
        # ==========================================
        # 顶部控制栏 - 简化版本
        # ==========================================
        ctrl_cols = st.columns([2, 2, 2, 2, 2])
        
        with ctrl_cols[0]:
            if not st.session_state.mission_executing:
                if st.button("▶️ 开始任务", type="primary", use_container_width=True, key="start_btn"):
                    st.session_state.mission_executing = True
                    st.session_state.current_waypoint_index = 0
                    st.session_state.flight_start_time = time.time()
                    st.session_state.logged_waypoints = set([0])  # 起点算已完成
                    
                    # 【关键】清除之前的路径时间缓存，重新计算
                    if 'waypoint_cumulative_times' in st.session_state:
                        del st.session_state['waypoint_cumulative_times']
                    if 'total_flight_distance' in st.session_state:
                        del st.session_state['total_flight_distance']
                    
                    # 简化的位置计算 - 直接记录当前目标航点
                    st.session_state.drone_pos_index = 0
                    st.session_state.drone_position = [
                        st.session_state.waypoints[0].lat,
                        st.session_state.waypoints[0].lon
                    ]
                    
                    st.session_state.comm_logger.log_flight_start()
                    timestamp = (datetime.utcnow() + timedelta(hours=8)).strftime("%H:%M:%S")
                    st.session_state.send_log.append(f"[{timestamp}] GCS→OBC: MISSION_START")
                    st.session_state.recv_log.append(f"[{timestamp}] FCU→OBC→GCS: ACK | Mode: AUTO")
                    # 不rerun，让下面的显示逻辑处理
            else:
                st.button("⏳ 执行中...", disabled=True, use_container_width=True)
        
        with ctrl_cols[1]:
            if st.button("⏸️ 暂停", use_container_width=True, key="pause_btn"):
                st.session_state.mission_executing = False
                timestamp = (datetime.utcnow() + timedelta(hours=8)).strftime("%H:%M:%S")
                st.session_state.send_log.append(f"[{timestamp}] GCS→OBC: PAUSE_MISSION")
        
        with ctrl_cols[2]:
            if st.button("⏹️ 停止", use_container_width=True, key="stop_btn"):
                st.session_state.mission_executing = False
                st.session_state.current_waypoint_index = 0
                st.session_state.logged_waypoints = set()
        
        with ctrl_cols[3]:
            if st.button("🔄 重置", use_container_width=True, key="reset_btn"):
                st.session_state.mission_executing = False
                st.session_state.current_waypoint_index = 0
                st.session_state.drone_position = None
                st.session_state.flight_start_time = None
                st.session_state.flight_complete_time = None
                st.session_state.logged_waypoints = set()
                
                # 【关键】清除路径时间缓存
                if 'waypoint_cumulative_times' in st.session_state:
                    del st.session_state['waypoint_cumulative_times']
                if 'total_flight_distance' in st.session_state:
                    del st.session_state['total_flight_distance']
        
        with ctrl_cols[4]:
            status_text = "🟢 飞行中" if st.session_state.mission_executing else ("🟡 已暂停" if st.session_state.drone_position else "⚪ 就绪")
            st.markdown(f"**{status_text}**")
        
        # 【修复】确保curr_idx始终有值
        curr_idx = st.session_state.current_waypoint_index
        
        # ==========================================
        # 【核心修复】基于距离的飞行推进逻辑
        # ==========================================
        # 计算总距离和每段航段时间
        if 'waypoint_cumulative_times' not in st.session_state:
            total_dist = 0
            seg_times = [0]  # 起点时间为0
            for i in range(total_wp - 1):
                dist = st.session_state.planner.haversine_distance(
                    st.session_state.waypoints[i].lat, st.session_state.waypoints[i].lon,
                    st.session_state.waypoints[i+1].lat, st.session_state.waypoints[i+1].lon
                )
                total_dist += dist
                seg_times.append(total_dist / 8.5)  # 累计时间（秒）
            st.session_state.waypoint_cumulative_times = seg_times
            st.session_state.total_flight_distance = total_dist
        
        if st.session_state.mission_executing and st.session_state.flight_start_time:
            elapsed = time.time() - st.session_state.flight_start_time
            
            # 基于累计时间计算当前航点
            target_wp_idx = 0
            for i, cum_time in enumerate(st.session_state.waypoint_cumulative_times):
                if elapsed >= cum_time:
                    target_wp_idx = i
                else:
                    break
            target_wp_idx = min(target_wp_idx, total_wp - 1)
            
            # 航点有变化
            if target_wp_idx > curr_idx:
                # 记录所有经过的航点
                for wp_idx in range(curr_idx + 1, target_wp_idx + 1):
                    if wp_idx not in st.session_state.logged_waypoints:
                        st.session_state.logged_waypoints.add(wp_idx)
                        timestamp = (datetime.utcnow() + timedelta(hours=8)).strftime("%H:%M:%S")
                        st.session_state.comm_logger.log_waypoint_reached(wp_idx, total_wp)
                        st.session_state.recv_log.append(f"[{timestamp}] FCU→OBC→GCS: WP_REACHED #{wp_idx}")
                
                st.session_state.current_waypoint_index = target_wp_idx
                curr_idx = target_wp_idx
                
                # 到达终点
                if curr_idx >= total_wp - 1:
                    st.session_state.mission_executing = False
                    # 【修复】记录任务完成时间，用于停止计时
                    st.session_state.flight_complete_time = time.time()
                    st.session_state.comm_logger.log_flight_complete()
                    timestamp = (datetime.utcnow() + timedelta(hours=8)).strftime("%H:%M:%S")
                    st.session_state.recv_log.append(f"[{timestamp}] FCU→OBC→GCS: MISSION_COMPLETE")
        
        # ==========================================
        # 实时状态显示
        # ==========================================
        st.markdown("---")
        
        # 【修复】飞行时间计算 - 任务完成后停止计时
        flight_time = 0
        if st.session_state.flight_start_time:
            if hasattr(st.session_state, 'flight_complete_time') and st.session_state.flight_complete_time:
                # 任务已完成，使用完成时的时间
                flight_time = int(st.session_state.flight_complete_time - st.session_state.flight_start_time)
            else:
                # 任务进行中
                flight_time = int(time.time() - st.session_state.flight_start_time)
        
        flight_speed = 8.5
        if st.session_state.mission_executing:
            flight_speed = 8.5 + (0.3 if int(time.time()) % 4 < 2 else -0.2)
        
        # 计算剩余距离
        remaining_dist = 0
        for i in range(curr_idx, total_wp - 1):
            remaining_dist += st.session_state.planner.haversine_distance(
                st.session_state.waypoints[i].lat, st.session_state.waypoints[i].lon,
                st.session_state.waypoints[i+1].lat, st.session_state.waypoints[i+1].lon
            )
        
        eta_str = "00:00" if curr_idx >= total_wp - 1 else f"{int(remaining_dist/8.5//60):02d}:{int(remaining_dist/8.5%60):02d}"
        progress_pct = min(100, int((curr_idx / max(1, total_wp-1)) * 100))
        
        # 状态卡片
        status_cols = st.columns(6)
        status_cols[0].metric("📍 当前航点", f"{min(curr_idx+1, total_wp)}/{total_wp}")
        status_cols[1].metric("⚡ 飞行速度", f"{flight_speed:.1f} m/s")
        status_cols[2].metric("⏱️ 已用时间", f"{flight_time//60:02d}:{flight_time%60:02d}")
        status_cols[3].metric("📏 剩余距离", f"{remaining_dist:.0f} m")
        status_cols[4].metric("🏁 预计到达", eta_str)
        status_cols[5].metric("🔋 电量模拟", f"{max(0, 100 - flight_time//10)}%")
        
        st.progress(progress_pct / 100, text=f"任务进度: {progress_pct}%")
        st.markdown("---")
        
        # ==========================================
        # 主显示区域：地图 + 通信链路
        # ==========================================
        main_col, right_col = st.columns([3, 2])
        
        with main_col:
            st.subheader("🗺️ 实时飞行地图")
            
            # 【修复】基于当前航点索引计算无人机位置，确保与航点推进逻辑一致
            if st.session_state.mission_executing and st.session_state.flight_start_time and curr_idx < total_wp - 1:
                # 飞行中：在当前航段内插值
                elapsed = time.time() - st.session_state.flight_start_time
                
                # 计算当前航段进度
                if curr_idx < len(st.session_state.waypoint_cumulative_times) - 1:
                    seg_start_time = st.session_state.waypoint_cumulative_times[curr_idx]
                    seg_end_time = st.session_state.waypoint_cumulative_times[curr_idx + 1]
                    seg_duration = seg_end_time - seg_start_time
                    seg_elapsed = elapsed - seg_start_time
                    seg_progress = max(0, min(1, seg_elapsed / seg_duration)) if seg_duration > 0 else 0
                else:
                    seg_progress = 0
                
                # 插值计算当前位置
                curr_wp = st.session_state.waypoints[curr_idx]
                next_wp = st.session_state.waypoints[curr_idx + 1]
                drone_lat = curr_wp.lat + (next_wp.lat - curr_wp.lat) * seg_progress
                drone_lon = curr_wp.lon + (next_wp.lon - curr_wp.lon) * seg_progress
                drone_pos = [drone_lat, drone_lon]
                
                # 保存位置
                st.session_state.drone_position = drone_pos
            elif curr_idx >= total_wp - 1 and st.session_state.waypoints:
                # 到达终点或任务完成
                drone_pos = [st.session_state.waypoints[-1].lat, st.session_state.waypoints[-1].lon]
                st.session_state.drone_position = drone_pos
            elif st.session_state.drone_position:
                drone_pos = st.session_state.drone_position
            elif st.session_state.waypoints:
                drone_pos = [st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]
            else:
                drone_pos = [32.0603, 118.7969]
            
            # 创建地图
            m = folium.Map(location=drone_pos, zoom_start=17, tiles="OpenStreetMap")
            
            # 添加卫星图层
            folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='🛰️ 卫星影像',
                overlay=False,
                control=True
            ).add_to(m)
            folium.LayerControl(position='topright').add_to(m)
            
            # 显示障碍物
            for obs in st.session_state.planner.obstacles:
                color = 'red' if obs.height >= st.session_state.flight_altitude else 'orange'
                if obs.type == "circle":
                    folium.Circle(
                        [obs.center_lat, obs.center_lon],
                        radius=obs.radius,
                        popup=f"{obs.name}<br>高度:{obs.height}m",
                        color=color, fill=True, fillOpacity=0.3
                    ).add_to(m)
                else:
                    folium.Polygon(
                        locations=obs.points,
                        popup=f"{obs.name}<br>高度:{obs.height}m",
                        color=color, fill=True, fillOpacity=0.3
                    ).add_to(m)
            
            # 绘制计划航线（灰色虚线）- 任务完成后隐藏，避免与已飞路径重叠
            if st.session_state.waypoints:
                # 只在任务执行中或未开始时显示计划航线
                show_plan = st.session_state.mission_executing or not st.session_state.flight_start_time
                if show_plan:
                    path_coords = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
                    folium.PolyLine(
                        path_coords, 
                        color='gray', 
                        weight=2, 
                        opacity=0.5, 
                        dash_array='5,10',
                        popup="计划航线"
                    ).add_to(m)
                
                # 【修复】绘制航点 - 使用logged_waypoints判断完成状态
                logged = getattr(st.session_state, 'logged_waypoints', set())
                for i, wp in enumerate(st.session_state.waypoints):
                    is_completed = i in logged or i < curr_idx
                    is_current = i == curr_idx and st.session_state.mission_executing
                    
                    if i == 0:
                        folium.Marker(
                            [wp.lat, wp.lon],
                            icon=folium.Icon(color='green', icon='play', prefix='glyphicon'),
                            popup=f"🚁 起点 WP{i}<br>高度: {wp.alt}m"
                        ).add_to(m)
                    elif i == len(st.session_state.waypoints) - 1:
                        folium.Marker(
                            [wp.lat, wp.lon],
                            icon=folium.Icon(color='red', icon='stop', prefix='glyphicon'),
                            popup=f"🎯 终点 WP{i}<br>高度: {wp.alt}m<br>状态: {'已完成' if is_completed else '待执行'}"
                        ).add_to(m)
                    else:
                        color = '#4caf50' if is_completed else ('#2196f3' if is_current else '#9e9e9e')
                        radius = 7 if is_current else 5
                        
                        folium.CircleMarker(
                            [wp.lat, wp.lon],
                            radius=radius,
                            color=color,
                            fill=True,
                            fillOpacity=0.9,
                            popup=f"航点 WP{i}<br>高度: {wp.alt}m<br>状态: {'✅已完成' if is_completed else ('🚁当前' if is_current else '⏳待执行')}"
                        ).add_to(m)
                
                # 【修复】绘制飞行路径
                # 飞行中：显示从起点到当前位置的实时路径
                # 任务完成后：显示完整路径（起点到终点）
                is_flight_completed = (not st.session_state.mission_executing) and st.session_state.flight_start_time and curr_idx >= total_wp - 1
                is_flight_in_progress = st.session_state.mission_executing and st.session_state.flight_start_time
                
                if is_flight_completed:
                    # 任务完成后：显示完整路径（起点到终点）
                    full_path_coords = [[wp.lat, wp.lon] for wp in st.session_state.waypoints]
                    folium.PolyLine(
                        full_path_coords,
                        color='#00FF00',
                        weight=5,
                        opacity=0.9,
                        popup="飞行路径"
                    ).add_to(m)
                elif is_flight_in_progress and curr_idx > 0:
                    # 飞行中：显示从起点到当前位置的实时路径
                    flown_coords = [[st.session_state.waypoints[0].lat, st.session_state.waypoints[0].lon]]
                    for i in range(1, curr_idx + 1):
                        flown_coords.append([st.session_state.waypoints[i].lat, st.session_state.waypoints[i].lon])
                    # 添加当前无人机位置
                    flown_coords.append(drone_pos)
                    
                    folium.PolyLine(
                        flown_coords,
                        color='#00FF00',
                        weight=5,
                        opacity=0.9,
                        popup="已飞路径"
                    ).add_to(m)
            
            # 无人机当前位置标记
            if st.session_state.drone_position or st.session_state.all_flight_positions:
                # 无人机图标
                folium.Marker(
                    drone_pos,
                    icon=folium.DivIcon(
                        html='<div style="background:#ff6b00;color:white;width:36px;height:36px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:18px;border:3px solid white;box-shadow:0 2px 8px rgba(0,0,0,0.4);transform:rotate(45deg);">✈</div>',
                        icon_size=[36, 36],
                        icon_anchor=[18, 18]
                    ),
                    popup=f"🚁 无人机<br>位置: ({drone_pos[0]:.6f}, {drone_pos[1]:.6f})<br>速度: {flight_speed:.1f} m/s"
                ).add_to(m)
                
                # 安全半径圆圈
                safety_m = st.session_state.planner.safety_margin
                folium.Circle(
                    drone_pos,
                    radius=safety_m,
                    color='orange',
                    fill=True,
                    fillOpacity=0.15,
                    popup=f"🛡️ 安全半径: {safety_m}m"
                ).add_to(m)
            
            # 渲染地图
            st_folium(m, width=750, height=500, key="flight_monitor_map_v2")
        
        with right_col:
            # ==========================================
            # 通信链路拓扑与数据流
            # ==========================================
            st.subheader("📡 通信链路拓扑与数据流")
            
            # 节点在线状态
            node_cols = st.columns(3)
            with node_cols[0]:
                gcs_online = st.checkbox("🖥️ GCS 在线", value=True, key="gcs_node_v9")
            with node_cols[1]:
                obc_online = st.checkbox("🧠 OBC 在线", value=True, key="obc_node_v9")
            with node_cols[2]:
                fcu_online = st.checkbox("⚙️ FCU 在线", value=True, key="fcu_node_v9")
            
            st.markdown("---")
            
            # 链路状态计算
            gcs_obc_ok = gcs_online and obc_online
            obc_fcu_ok = obc_online and fcu_online
            gcs_fcu_ok = gcs_online and fcu_online
            
            gcs_obc_status = "🟢 已连接" if gcs_obc_ok else "🔴 断开"
            obc_fcu_status = "🟢 已连接" if obc_fcu_ok else "🔴 断开"
            gcs_fcu_status = "🟢 直连" if gcs_fcu_ok else "⚪ 未直连"
            
            # 拓扑图可视化
            topo_html = """
            <div style="background:#f8f9fa;padding:15px;border-radius:10px;margin:10px 0;">
                <table style="width:100%;text-align:center;">
                    <tr>
                        <td style="width:20%;">
                            <div style="background:#e3f2fd;padding:15px;border-radius:8px;border:2px solid #2196f3;">
                                <div style="font-size:24px;">🖥️</div>
                                <div style="font-weight:bold;">GCS</div>
                                <div style="font-size:11px;color:#666;">地面站<br>192.168.1.100</div>
                            </div>
                        </td>
                        <td style="width:15%;vertical-align:middle;">
                            <div style="font-size:14px;color:#0066cc;">⬆⬇<br>UDP:14550</div>
                            <div style="font-size:12px;padding:3px 8px;background:#e8f5e9;border-radius:10px;display:inline-block;">""" + gcs_obc_status + """</div>
                        </td>
                        <td style="width:20%;">
                            <div style="background:#fff3e0;padding:15px;border-radius:8px;border:2px solid #ff9800;">
                                <div style="font-size:24px;">🧠</div>
                                <div style="font-weight:bold;">OBC</div>
                                <div style="font-size:11px;color:#666;">机载计算机<br>Raspberry Pi 4</div>
                            </div>
                        </td>
                        <td style="width:15%;vertical-align:middle;">
                            <div style="font-size:14px;color:#e65100;">⬆⬇<br>MAVLink</div>
                            <div style="font-size:12px;padding:3px 8px;background:#e8f5e9;border-radius:10px;display:inline-block;">""" + obc_fcu_status + """</div>
                        </td>
                        <td style="width:20%;">
                            <div style="background:#f3e5f5;padding:15px;border-radius:8px;border:2px solid #9c27b0;">
                                <div style="font-size:24px;">⚙️</div>
                                <div style="font-weight:bold;">FCU</div>
                                <div style="font-size:11px;color:#666;">飞控<br>PX4 / ArduPilot</div>
                            </div>
                        </td>
                    </tr>
                </table>
                
                <div style="margin-top:15px;padding:10px;background:#fff;border-radius:5px;font-size:12px;">
                    <b>📊 链路统计：</b>
                    <span style="margin-left:15px;">GCS↔OBC: """ + ("正常" if gcs_obc_ok else "异常") + """</span>
                    <span style="margin-left:15px;">OBC↔FCU: """ + ("正常" if obc_fcu_ok else "异常") + """</span>
                    <span style="margin-left:15px;">延迟: ~25ms</span>
                    <span style="margin-left:15px;">丢包率: 0.1%</span>
                </div>
            </div>
            """
            st.html(topo_html)
            
            # ==========================================
            # 通信日志 Tab
            # ==========================================
            st.subheader("📋 通信日志")
            
            log_tab1, log_tab2, log_tab3 = st.tabs(["🔄 业务流程", "📤 GCS→OBC→FCU", "📥 FCU→OBC→GCS"])
            
            with log_tab1:
                logs = st.session_state.comm_logger.get_logs()
                if logs:
                    log_html = "<div style='max-height:280px;overflow-y:auto;font-family:monospace;font-size:10px;background:#f8f9fa;padding:8px;border-radius:5px;'>"
                    for log in reversed(logs[-20:]):
                        bg_color = {"success": "#d4edda", "error": "#f8d7da", "warning": "#fff3cd", "info": "#e7f3ff", "processing": "#e2e3e5"}.get(log['status'], "#f8f9fa")
                        border_color = {"success": "#28a745", "error": "#dc3545", "warning": "#ffc107", "info": "#17a2b8", "processing": "#6c757d"}.get(log['status'], "#6c757d")
                        log_html += f"<div style='padding:4px;margin:2px 0;border-radius:3px;background:{bg_color};border-left:3px solid {border_color};'>"
                        log_html += f"<span style='color:#666;font-size:9px'>[{log['timestamp']}]</span> "
                        log_html += f"{log['icon']} <b>{log['msg_type']}</b><br>"
                        log_html += f"<span style='color:#333'>{log['content']}</span><br>"
                        log_html += f"<small style='color:#666'>{log['direction']}</small>"
                        log_html += f"</div>"
                    log_html += "</div>"
                    st.html(log_html)
                else:
                    st.info("暂无业务流程日志")
                
                if st.button("🗑️ 清除日志", key="clear_biz_log"):
                    st.session_state.comm_logger.clear()
                    st.rerun()
            
            with log_tab2:
                # GCS → OBC → FCU 发送日志
                send_html = "<div style='max-height:280px;overflow-y:auto;font-family:monospace;font-size:10px;background:#e7f3ff;padding:8px;border-radius:5px;'>"
                send_html += "<div style='color:#0066cc;font-weight:bold;margin-bottom:5px;'>📤 GCS → OBC</div>"
                if st.session_state.send_log:
                    for log in list(st.session_state.send_log)[-15:]:
                        if '→OBC' in log or 'GCS→' in log:
                            send_html += f"<div style='padding:2px 0;border-bottom:1px dashed #ccc;'>{log}</div>"
                send_html += "<div style='color:#e65100;font-weight:bold;margin:10px 0 5px 0;'>📤 OBC → FCU</div>"
                if st.session_state.send_log:
                    for log in list(st.session_state.send_log)[-15:]:
                        if 'OBC→FCU' in log or '→FCU' in log:
                            send_html += f"<div style='padding:2px 0;border-bottom:1px dashed #ccc;'>{log}</div>"
                if not st.session_state.send_log:
                    send_html += "<div style='color:#999'>暂无发送记录</div>"
                send_html += "</div>"
                st.html(send_html)
            
            with log_tab3:
                # FCU → OBC → GCS 接收日志
                recv_html = "<div style='max-height:280px;overflow-y:auto;font-family:monospace;font-size:10px;background:#fff8e7;padding:8px;border-radius:5px;'>"
                recv_html += "<div style='color:#e65100;font-weight:bold;margin-bottom:5px;'>📥 FCU → OBC</div>"
                if st.session_state.recv_log:
                    for log in list(st.session_state.recv_log)[-15:]:
                        if 'FCU→' in log or '→OBC' in log:
                            recv_html += f"<div style='padding:2px 0;border-bottom:1px dashed #ccc;'>{log}</div>"
                recv_html += "<div style='color:#0066cc;font-weight:bold;margin:10px 0 5px 0;'>📥 OBC → GCS</div>"
                if st.session_state.recv_log:
                    for log in list(st.session_state.recv_log)[-15:]:
                        if 'OBC→GCS' in log or '→GCS' in log:
                            recv_html += f"<div style='padding:2px 0;border-bottom:1px dashed #ccc;'>{log}</div>"
                if not st.session_state.recv_log:
                    recv_html += "<div style='color:#999'>暂无接收记录</div>"
                recv_html += "</div>"
                st.html(recv_html)
        
        # ==========================================
        # 底部：航点详细进程表
        # ==========================================
        st.markdown("---")
        st.subheader("📍 航点详细进程")
        
        if st.session_state.waypoints:
            # 【修复】创建航点表格数据 - 使用logged_waypoints判断完成
            logged = getattr(st.session_state, 'logged_waypoints', set())
            wp_data = []
            for i, wp in enumerate(st.session_state.waypoints):
                dist_to_next = 0
                if i < len(st.session_state.waypoints) - 1:
                    dist_to_next = st.session_state.planner.haversine_distance(
                        wp.lat, wp.lon,
                        st.session_state.waypoints[i+1].lat, st.session_state.waypoints[i+1].lon
                    )
                
                # 【关键修复】使用logged_waypoints判断完成状态
                is_completed = i in logged
                is_current = (i == curr_idx and st.session_state.mission_executing)
                
                if is_completed:
                    status = "✅ 已完成"
                elif is_current:
                    status = "🚁 当前"
                else:
                    status = "⏳ 待执行"
                
                # 计算ETA
                eta = "--:--"
                if i > curr_idx and flight_speed > 0:
                    dist_accum = 0
                    for j in range(curr_idx, i):
                        if j < len(st.session_state.waypoints) - 1:
                            dist_accum += st.session_state.planner.haversine_distance(
                                st.session_state.waypoints[j].lat, st.session_state.waypoints[j].lon,
                                st.session_state.waypoints[j+1].lat, st.session_state.waypoints[j+1].lon
                            )
                    eta_seconds = dist_accum / flight_speed
                    eta = f"{int(eta_seconds//60):02d}:{int(eta_seconds%60):02d}"
                
                wp_data.append({
                    "序号": f"WP{i}",
                    "坐标": f"{wp.lat:.5f}, {wp.lon:.5f}",
                    "高度": f"{wp.alt}m",
                    "距下点": f"{dist_to_next:.0f}m" if dist_to_next > 0 else "--",
                    "状态": status,
                    "预计": eta
                })
            
            # 显示表格
            wp_df_cols = st.columns(len(wp_data))
            for i, wp_info in enumerate(wp_data):
                with wp_df_cols[i]:
                    card_style = "background:#e8f5e9;border:2px solid #4caf50;" if "已完成" in wp_info['状态'] else \
                                "background:#fff3e0;border:2px solid #ff9800;" if "当前" in wp_info['状态'] else \
                                "background:#f5f5f5;border:1px solid #ddd;"
                    
                    # 状态卡片样式
                    is_completed = "已完成" in wp_info['状态']
                    is_current = "当前" in wp_info['状态']
                    card_style = "background:#e8f5e9;border:2px solid #4caf50;" if is_completed else \
                                "background:#fff3e0;border:2px solid #ff9800;" if is_current else \
                                "background:#f5f5f5;border:1px solid #ddd;"
                    
                    st.markdown(f"""
                    <div style="{card_style}padding:8px;border-radius:6px;text-align:center;font-size:11px;">
                        <div style="font-weight:bold;font-size:13px;margin-bottom:4px;">{wp_info['序号']}</div>
                        <div style="color:#666;margin-bottom:2px;">{wp_info['坐标']}</div>
                        <div style="color:#2196f3;font-weight:bold;">{wp_info['高度']}</div>
                        <div style="margin-top:4px;padding-top:4px;border-top:1px dashed #ccc;">
                            <span style="font-size:10px;">{wp_info['状态']}</span>
                        </div>
                        <div style="font-size:9px;color:#666;margin-top:2px;">{wp_info['预计']}</div>
                    </div>
                    """, unsafe_allow_html=True)
        
        # ==========================================
        # 【已移除】自动推进逻辑已移到页面顶部
        # 页面通过自然刷新（按钮点击等）更新状态
        # ==========================================




st.markdown("---")
st.caption(f"MAVLink GCS {VERSION} {VERSION_NAME} | 严格避障 | 安全绕行 | 障碍物持久化 | 北京时间 (UTC+8)")
