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
    page_title="MAVLink 地面站 - 3D避障规划系统",
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
    """支持多边形、圆形、旋转矩形的障碍物"""
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
        """判断点是否在障碍物水平投影内"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center_lat)**2 + (lon-self.center_lon)**2) * 111000
            return dist < self.radius
        return point_in_polygon(lat, lon, self.points)
    
    def get_horizontal_distance(self, lat, lon):
        """获取点到障碍物的水平距离"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center_lat)**2 + (lon-self.center_lon)**2) * 111000
            return max(0, dist - self.radius)
        
        # 多边形/矩形：检查是否在内部或到边界的距离
        if self.contains_point(lat, lon):
            return 0
        
        min_dist = float('inf')
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i+1) % n]
            dist = point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1])
            min_dist = min(min_dist, dist)
        return min_dist

class Node3D:
    """3D路径规划节点"""
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
                abs(self.lon - other.lon) < 1e-8 and
                abs(self.alt - other.alt) < 0.1)
    
    def __hash__(self):
        return hash((round(self.lat, 8), round(self.lon, 8), round(self.alt, 1)))

class PathPlanner3D:
    """真正的3D路径规划器 - 支持水平绕行和高度变化"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 20  # 水平安全边距（米）
        self.vertical_margin = 5  # 垂直安全边距（米）
        self.min_altitude = 10
        self.max_altitude = 120
    
    def add_polygon_obstacle(self, points, height, name="多边形障碍物"):
        self.obstacles.append(Obstacle(points, height, name, "polygon"))
    
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
    
    def set_altitude_limits(self, min_alt, max_alt):
        self.min_altitude = min_alt
        self.max_altitude = max_alt
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def check_collision_3d(self, lat, lon, alt):
        """
        3D碰撞检测 - 检查点是否在障碍物的3D空间内
        返回: (是否碰撞, 碰撞的障碍物)
        """
        for obs in self.obstacles:
            # 检查水平距离
            h_dist = obs.get_horizontal_distance(lat, lon)
            
            if h_dist < self.safety_margin:
                # 水平方向在安全边距内，检查垂直方向
                # 碰撞条件：飞行高度 < 障碍物高度 + 垂直安全边距
                if alt < obs.height + self.vertical_margin:
                    return True, obs
        
        return False, None
    
    def can_pass_over(self, lat, lon, alt):
        """检查是否可以从障碍物上方飞越"""
        for obs in self.obstacles:
            h_dist = obs.get_horizontal_distance(lat, lon)
            if h_dist < self.safety_margin:
                # 在障碍物上方，检查是否足够高
                if alt >= obs.height + self.vertical_margin:
                    return True
                else:
                    return False
        return True
    
    def get_neighbors_3d(self, node, end_node, step_size=20):
        """
        生成3D邻居节点 - 包括水平8个方向和垂直2个方向（上升/下降）
        """
        neighbors = []
        
        # 基础水平步长
        lat_step = step_size / 111000.0
        lon_step = step_size / (111000.0 * math.cos(math.radians(node.lat)))
        
        # 垂直步长（米）
        alt_step = 10
        
        # 8个水平方向 + 2个垂直方向（上升/下降）的组合
        directions = []
        
        # 水平方向（8个）
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            rad = math.radians(angle)
            directions.append((math.cos(rad), math.sin(rad), 0))  # (dlon, dlat, dalt)
        
        # 纯垂直方向（2个）
        directions.append((0, 0, 1))   # 上升
        directions.append((0, 0, -1))  # 下降
        
        # 斜向垂直（16个）- 水平移动同时改变高度
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            rad = math.radians(angle)
            # 上升
            directions.append((math.cos(rad), math.sin(rad), 1))
            # 下降
            directions.append((math.cos(rad), math.sin(rad), -1))
        
        for dlon_dir, dlat_dir, dalt_dir in directions:
            new_lat = node.lat + dlat_dir * lat_step
            new_lon = node.lon + dlon_dir * lon_step
            new_alt = node.alt + dalt_dir * alt_step
            
            # 高度限制
            if new_alt < self.min_altitude or new_alt > self.max_altitude:
                continue
            
            # 3D碰撞检测
            collision, obs = self.check_collision_3d(new_lat, new_lon, new_alt)
            
            if not collision:
                # 计算代价
                h_dist = math.sqrt((dlon_dir * step_size)**2 + (dlat_dir * step_size)**2)
                v_dist = abs(dalt_dir * alt_step)
                move_cost = math.sqrt(h_dist**2 + v_dist**2)
                
                g_cost = node.g_cost + move_cost
                h_cost = self.haversine_distance(new_lat, new_lon, end_node.lat, end_node.lon)
                
                # 高度变化惩罚（鼓励水平飞行）
                if dalt_dir != 0:
                    h_cost += abs(new_alt - end_node.alt) * 2  # 高度偏离惩罚
                
                neighbors.append(Node3D(new_lat, new_lon, new_alt, g_cost, h_cost, node))
        
        return neighbors
    
    def plan_path_3d(self, start_wp, end_wp, step_size=20):
        """
        3D A*路径规划 - 真正实现避障绕行
        """
        start_node = Node3D(
            start_wp.lat, start_wp.lon, start_wp.alt, 
            0,
            self.haversine_distance(start_wp.lat, start_wp.lon, end_wp.lat, end_wp.lon)
        )
        end_node = Node3D(end_wp.lat, end_wp.lon, end_wp.alt)
        
        # 检查起点和终点
        if self.check_collision_3d(start_node.lat, start_node.lon, start_node.alt)[0]:
            st.error("❌ 起点在障碍物内或过于接近障碍物")
            return [start_wp, end_wp]
        
        if self.check_collision_3d(end_node.lat, end_node.lon, end_node.alt)[0]:
            st.error("❌ 终点在障碍物内或过于接近障碍物")
            return [start_wp, end_wp]
        
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        
        max_iter = 8000
        iteration = 0
        
        while open_list and iteration < max_iter:
            iteration += 1
            
            current = heapq.heappop(open_list)
            
            # 检查是否到达目标（水平距离和高度都接近）
            h_dist = self.haversine_distance(current.lat, current.lon, end_node.lat, end_node.lon)
            v_dist = abs(current.alt - end_node.alt)
            
            if h_dist < step_size * 2 and v_dist < 5:
                # 重建路径
                path = []
                node = current
                while node:
                    path.append(node)
                    node = node.parent
                path.reverse()
                
                # 确保终点精确
                if path[-1].lat != end_node.lat or path[-1].lon != end_node.lon:
                    path.append(end_node)
                
                waypoints = [Waypoint(n.lat, n.lon, n.alt, seq=i) for i, n in enumerate(path)]
                waypoints[0].cmd = 22  # TAKEOFF
                waypoints[-1].cmd = 16  # NAV_WAYPOINT
                return waypoints
            
            # 生成唯一键
            node_key = (round(current.lat, 7), round(current.lon, 7), round(current.alt, 0))
            if node_key in closed_set:
                continue
            closed_set.add(node_key)
            
            # 生成邻居
            for neighbor in self.get_neighbors_3d(current, end_node, step_size):
                neighbor_key = (round(neighbor.lat, 7), round(neighbor.lon, 7), round(neighbor.alt, 0))
                if neighbor_key in closed_set:
                    continue
                
                # 检查open_list中是否已有更优路径
                existing_idx = -1
                for idx, existing in enumerate(open_list):
                    if (abs(existing.lat - neighbor.lat) < 1e-8 and 
                        abs(existing.lon - neighbor.lon) < 1e-8 and
                        abs(existing.alt - neighbor.alt) < 0.1):
                        existing_idx = idx
                        break
                
                if existing_idx >= 0:
                    if open_list[existing_idx].g_cost <= neighbor.g_cost:
                        continue
                    else:
                        # 移除较差的现有节点
                        open_list.pop(existing_idx)
                        heapq.heapify(open_list)
                
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
        'path_planner': PathPlanner3D(),  # 使用新的3D规划器
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
        'pending_drawing': None,
        'rect_width': 50,
        'rect_height': 80,
        'rect_rotation': 0,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面 ====================
st.title("🚁 MAVLink 地面站 - 3D避障规划系统")
st.caption("支持水平绕行 | 高度变化 | 真正的3D A*避障 | 北京时间 (UTC+8)")

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
    st.header("🗺️ 航线规划与3D避障")
    
    with st.expander("📖 使用说明", expanded=True):
        st.markdown("""
        ### 🎯 操作步骤：
        
        1. **设置A/B点**：在右侧输入起点和终点坐标
        2. **添加障碍物**：在地图上绘制多边形/矩形/圆形，设置高度
        3. **规划路径**：点击"规划3D避障路径"
        
        ### 🚫 避障策略（按优先级）：
        1. **优先水平绕行**：当障碍物高于飞行高度时，强制水平绕行
        
