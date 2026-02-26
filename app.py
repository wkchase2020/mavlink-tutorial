import streamlit as st
import math
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw, AntPath
from streamlit_folium import st_folium

# ==================== 坐标转换 ====================
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

    if not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55):
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

    if not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55):
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

# ==================== 几何函数 ====================
def point_in_polygon(lat, lon, points):
    n = len(points)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = points[i][0], points[i][1]
        yj, xj = points[j][0], points[j][1]
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
        
        # 检查是否在多边形内
        if point_in_polygon(lat, lon, self.points):
            return True
        
        # 检查到边界的距离
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i+1) % n]
            if point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1]) < margin:
                return True
        return False

class GridPathPlanner:
    """基于网格的A*路径规划器 - 确保可靠绕行"""
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 25  # 安全边距（米）
        self.grid_size = 15  # 网格大小（米）
    
    def add_obstacle(self, obs):
        self.obstacles.append(obs)
    
    def clear(self):
        self.obstacles = []
    
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def is_collision(self, lat, lon, flight_alt):
        """检查是否碰撞（考虑高度）"""
        for obs in self.obstacles:
            # 如果飞行高度足够高，可以飞越
            if flight_alt >= obs.height + 10:
                continue
            
            # 否则检查水平碰撞
            if obs.is_inside(lat, lon, self.safety_margin):
                return True
        return False
    
    def latlon_to_grid(self, lat, lon, base_lat, base_lon):
        """将经纬度转换为网格坐标"""
        dlat = (lat - base_lat) * 111000  # 米
        dlon = (lon - base_lon) * 111000 * math.cos(math.radians(base_lat))
        grid_x = int(dlon / self.grid_size)
        grid_y = int(dlat / self.grid_size)
        return (grid_x, grid_y)
    
    def grid_to_latlon(self, grid_x, grid_y, base_lat, base_lon):
        """将网格坐标转换为经纬度"""
        lon = base_lon + (grid_x * self.grid_size) / (111000 * math.cos(math.radians(base_lat)))
        lat = base_lat + (grid_y * self.grid_size) / 111000
        return (lat, lon)
    
    def plan_path(self, start_wp, end_wp):
        """
        使用网格A*算法规划路径
        """
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        flight_alt = start_wp.alt
        
        # 检查起点终点是否合法
        if self.is_collision(start[0], start[1], flight_alt):
            st.error("❌ 起点在障碍物内")
            return [start_wp, end_wp]
        if self.is_collision(end[0], end[1], flight_alt):
            st.error("❌ 终点在障碍物内")
            return [start_wp, end_wp]
        
        # 计算网格基准点
        base_lat = min(start[0], end[0]) - 0.001
        base_lon = min(start[1], end[1]) - 0.001
        
        # 起点终点网格坐标
        start_grid = self.latlon_to_grid(start[0], start[1], base_lat, base_lon)
        end_grid = self.latlon_to_grid(end[0], end[1], base_lat, base_lon)
        
        # A*搜索
        open_set = []
        heapq.heappush(open_set, (0, start_grid[0], start_grid[1], [start_grid]))
        visited = set()
        
        # 8个方向
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        max_iter = 5000
        iteration = 0
        
        while open_set and iteration < max_iter:
            iteration += 1
            
            cost, x, y, path = heapq.heappop(open_set)
            
            # 检查是否到达终点
            if abs(x - end_grid[0]) <= 2 and abs(y - end_grid[1]) <= 2:
                # 重建路径
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
            
            # 探索邻居
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if (nx, ny) in visited:
                    continue
                
                # 检查新位置是否碰撞
                lat, lon = self.grid_to_latlon(nx, ny, base_lat, base_lon)
                if self.is_collision(lat, lon, flight_alt):
                    continue
                
                # 计算代价
                move_cost = math.sqrt(dx**2 + dy**2) * self.grid_size
                new_cost = cost + move_cost
                
                # 启发式（到终点的距离）
                h = math.sqrt((nx - end_grid[0])**2 + (ny - end_grid[1])**2) * self.grid_size
                
                heapq.heappush(open_set, (new_cost + h, nx, ny, path + [(nx, ny)]))
        
        st.warning("⚠️ 未找到完整路径，返回直线路径")
        return [start_wp, end_wp]
    
    def plan_climb_path(self, start_wp, end_wp, max_alt):
        """爬升飞越路径"""
        start = (start_wp.lat, start_wp.lon)
        end = (end_wp.lat, end_wp.lon)
        base_alt = start_wp.alt
        
        # 找出路径上的最大障碍物高度
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
        if fly_alt > max_alt:
            st.warning(f"需要飞越高度{fly_alt}m超过最大限制{max_alt}m")
            return None
        
        # 生成爬升-飞越-下降路径
        path = [start_wp]
        
        # 爬升点（起点附近）
        climb_lat = start[0] + (end[0] - start[0]) * 0.2
        climb_lon = start[1] + (end[1] - start[1]) * 0.2
        path.append(Waypoint(climb_lat, climb_lon, fly_alt, 16, 1))
        
        # 中点（保持高度）
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        path.append(Waypoint(mid_lat, mid_lon, fly_alt, 16, 2))
        
        # 下降点（终点附近）
        descend_lat = start[0] + (end[0] - start[0]) * 0.8
        descend_lon = start[1] + (end[1] - start[1]) * 0.8
        path.append(Waypoint(descend_lat, descend_lon, fly_alt, 16, 3))
        
        path.append(end_wp)
        path[-1].seq = 4
        
        return path

# ==================== 初始化 ====================
if 'planner' not in st.session_state:
    st.session_state.planner = GridPathPlanner()
if 'point_a' not in st.session_state:
    st.session_state.point_a = None
if 'point_b' not in st.session_state:
    st.session_state.point_b = None
if 'path' not in st.session_state:
    st.session_state.path = None
if 'path_type' not in st.session_state:
    st.session_state.path_type = None
if 'pending' not in st.session_state:
    st.session_state.pending = None
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = 'WGS-84'

# ==================== 页面 ====================
st.set_page_config(page_title="无人机避障规划", layout="wide")
st.title("🚁 无人机网格A*避障规划")

col1, col2 = st.columns([3, 2])

with col1:
    st.subheader("🗺️ 地图")
    
    center = [32.0603, 118.7969]
    if st.session_state.point_a and st.session_state.point_b:
        center = [(st.session_state.point_a[0] + st.session_state.point_b[0])/2,
                 (st.session_state.point_a[1] + st.session_state.point_b[1])/2]
    
    m = folium.Map(location=center, zoom_start=16, tiles="CartoDB positron")
    
    Draw(
        draw_options={'polyline': False, 'rectangle': True, 'polygon': True, 
                     'circle': True, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True}
    ).add_to(m)
    
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri', name='卫星影像', overlay=False, control=True
    ).add_to(m)
    
    # 显示A/B点
    if st.session_state.point_a:
        folium.Marker(st.session_state.point_a, icon=folium.Icon(color='green', icon='play')).add_to(m)
    if st.session_state.point_b:
        folium.Marker(st.session_state.point_b, icon=folium.Icon(color='red', icon='stop')).add_to(m)
    
    # 显示障碍物
    for obs in st.session_state.planner.obstacles:
        color = 'red' if obs.height >= 50 else 'orange'
        if obs.type == 'circle':
            folium.Circle([obs.center_lat, obs.center_lon], radius=obs.radius, 
                         color=color, fill=True, fill_opacity=0.4).add_to(m)
        else:
            folium.Polygon(obs.points, color=color, fill=True, fill_opacity=0.4).add_to(m)
    
    # 显示路径
    if st.session_state.path:
        coords = [[wp.lat, wp.lon] for wp in st.session_state.path]
        color = 'blue' if st.session_state.path_type == 'horizontal' else 'green'
        AntPath(coords, color=color, weight=5, opacity=0.9, dash_array=[10, 20], delay=500).add_to(m)
        for wp in st.session_state.path:
            folium.CircleMarker([wp.lat, wp.lon], radius=4, color=color, fill=True,
                               popup=f'{wp.alt}m').add_to(m)
    
    result = st_folium(m, width=800, height=600, key="map")
    
    if result and result.get("last_active_drawing"):
        drawing = result["last_active_drawing"]
        shape_id = f"{drawing.get('type')}_{id(drawing)}"
        
        if st.session_state.get("last_id") != shape_id:
            st.session_state["last_id"] = shape_id
            geom_type = drawing.get("type")
            
            if geom_type == "circle":
                center = drawing["geometry"]["coordinates"]
                radius = drawing["properties"]["radius"]
                st.session_state.pending = {
                    'type': 'circle',
                    'center': (center[1], center[0]),
                    'radius': radius
                }
                st.rerun()
            elif geom_type in ["polygon", "rectangle"]:
                coords = drawing["geometry"]["coordinates"][0]
                points = [(c[1], c[0]) for c in coords[:-1]]
                st.session_state.pending = {
                    'type': 'polygon',
                    'points': points
                }
                st.rerun()

with col2:
    st.subheader("⚙️ 控制面板")
    
    # 坐标系选择
    st.session_state.coord_system = st.radio("坐标系", ["WGS-84", "GCJ-02"], 
                                            index=0 if st.session_state.coord_system=='WGS-84' else 1)
    
    st.markdown("---")
    
    # A点
    st.markdown("**起点 A**")
    c1, c2 = st.columns(2)
    
    def_lat_a, def_lon_a = 32.0603, 118.7969
    if st.session_state.point_a:
        if st.session_state.coord_system == 'GCJ-02':
            lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_a[1], st.session_state.point_a[0])
            def_lat_a, def_lon_a = lat_gcj, lon_gcj
        else:
            def_lat_a, def_lon_a = st.session_state.point_a
    
    lat_a = c1.number_input("纬度", value=def_lat_a, format="%.6f", key="a_lat")
    lon_a = c2.number_input("经度", value=def_lon_a, format="%.6f", key="a_lon")
    if st.button("设置A点"):
        if st.session_state.coord_system == 'GCJ-02':
            lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
            st.session_state.point_a = (lat_wgs, lon_wgs)
        else:
            st.session_state.point_a = (lat_a, lon_a)
        st.rerun()
    
    # B点
    st.markdown("**终点 B**")
    c3, c4 = st.columns(2)
    
    def_lat_b, def_lon_b = 32.0703, 118.8069
    if st.session_state.point_b:
        if st.session_state.coord_system == 'GCJ-02':
            lon_gcj, lat_gcj = wgs84_to_gcj02(st.session_state.point_b[1], st.session_state.point_b[0])
            def_lat_b, def_lon_b = lat_gcj, lon_gcj
        else:
            def_lat_b, def_lon_b = st.session_state.point_b
    
    lat_b = c3.number_input("纬度", value=def_lat_b, format="%.6f", key="b_lat")
    lon_b = c4.number_input("经度", value=def_lon_b, format="%.6f", key="b_lon")
    if st.button("设置B点"):
        if st.session_state.coord_system == 'GCJ-02':
            lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
            st.session_state.point_b = (lat_wgs, lon_wgs)
        else:
            st.session_state.point_b = (lat_b, lon_b)
        st.rerun()
    
    st.markdown("---")
    
    # 飞行高度
    flight_alt = st.slider("飞行高度(m)", 10, 100, 50)
    max_alt = st.slider("最大高度(m)", flight_alt + 10, 200, 120)
    
    # 障碍物处理
    if st.session_state.pending:
        st.info(f"待添加: {st.session_state.pending['type']}")
        obs_h = st.number_input("障碍物高度(m)", 5, 200, 60)
        
        col_a, col_c = st.columns(2)
        with col_a:
            if st.button("确认添加", type="primary"):
                p = st.session_state.pending
                if p['type'] == 'circle':
                    obs = Obstacle([p['center']], obs_h, f"圆形({obs_h}m)", "circle")
                    obs.radius = p['radius']
                else:
                    obs = Obstacle(p['points'], obs_h, f"多边形({obs_h}m)", "polygon")
                st.session_state.planner.add_obstacle(obs)
                st.session_state.pending = None
                st.rerun()
        with col_c:
            if st.button("取消"):
                st.session_state.pending = None
                st.rerun()
    
    # 旋转矩形
    with st.expander("添加旋转矩形"):
        r_lat = st.number_input("中心纬度", value=def_lat_a, format="%.6f", key="r_lat")
        r_lon = st.number_input("中心经度", value=def_lon_a, format="%.6f", key="r_lon")
        r_w = st.slider("宽度(m)", 10, 200, 50, key="r_w")
        r_h = st.slider("长度(m)", 10, 200, 80, key="r_h")
        r_rot = st.slider("旋转(°)", 0, 360, 0, key="r_rot")
        r_obs_h = st.number_input("矩形高度(m)", 5, 200, 40, key="r_obs_h")
        
        if st.button("添加矩形"):
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(r_lon, r_lat)
            else:
                lat_wgs, lon_wgs = r_lat, r_lon
            
            points = create_rotated_rectangle(lat_wgs, lon_wgs, r_w, r_h, r_rot)
            obs = Obstacle(points, r_obs_h, f"矩形({r_obs_h}m)", "rectangle", r_rot, r_w, r_h)
            st.session_state.planner.add_obstacle(obs)
            st.rerun()
    
    # 障碍物列表
    if st.session_state.planner.obstacles:
        st.write(f"**障碍物: {len(st.session_state.planner.obstacles)}个**")
        for i, obs in enumerate(st.session_state.planner.obstacles):
            st.write(f"#{i+1}: {obs.name}")
        if st.button("清除全部"):
            st.session_state.planner.clear()
            st.rerun()
    
    st.markdown("---")
    
    # 路径规划
    if st.session_state.point_a and st.session_state.point_b:
        col_h, col_c = st.columns(2)
        
        with col_h:
            if st.button("🔵 水平绕行", use_container_width=True):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], flight_alt, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], flight_alt, 16)
                
                with st.spinner("规划中..."):
                    path = st.session_state.planner.plan_path(start_wp, end_wp)
                    st.session_state.path = path
                    st.session_state.path_type = 'horizontal'
                    
                    dist = sum(st.session_state.planner.haversine(
                        path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon)
                        for i in range(len(path)-1))
                    st.success(f"水平绕行: {len(path)}航点, {dist:.0f}m")
                    st.rerun()
        
        with col_c:
            if st.button("🟢 爬升飞越", use_container_width=True):
                start_wp = Waypoint(st.session_state.point_a[0], st.session_state.point_a[1], flight_alt, 22)
                end_wp = Waypoint(st.session_state.point_b[0], st.session_state.point_b[1], flight_alt, 16)
                
                with st.spinner("规划中..."):
                    path = st.session_state.planner.plan_climb_path(start_wp, end_wp, max_alt)
                    if path:
                        st.session_state.path = path
                        st.session_state.path_type = 'climb'
                        
                        max_fly = max(wp.alt for wp in path)
                        dist = sum(st.session_state.planner.haversine(
                            path[i].lat, path[i].lon, path[i+1].lat, path[i+1].lon)
                            for i in range(len(path)-1))
                        st.success(f"爬升飞越: 最高{max_fly}m, {dist:.0f}m")
                        st.rerun()
    else:
        st.warning("请先设置A点和B点")

st.markdown("---")
st.caption("网格A*避障规划器 | 15米精度网格 | 强制绕行")
