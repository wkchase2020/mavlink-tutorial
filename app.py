import streamlit as st
import time
import math
import heapq
from datetime import datetime, timedelta
from collections import deque
import folium
from folium.plugins import Draw
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

# ==================== 几何函数 ====================
def point_in_polygon(lat, lon, points):
    """射线法判断点是否在多边形内"""
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
    """点到线段距离（米）"""
    lat_diff = lat2 - lat1
    lon_diff = lon2 - lon1
    if abs(lat_diff) < 1e-10 and abs(lon_diff) < 1e-10:
        return math.sqrt((lat - lat1)**2 + (lon - lon1)**2) * 111000
    t = max(0, min(1, ((lat - lat1) * lat_diff + (lon - lon1) * lon_diff) / (lat_diff**2 + lon_diff**2)))
    proj_lat = lat1 + t * lat_diff
    proj_lon = lon1 + t * lon_diff
    return math.sqrt((lat - proj_lat)**2 + (lon - proj_lon)**2) * 111000

# ==================== 障碍物类 ====================
class Obstacle:
    def __init__(self, points, height, obs_type="polygon", radius=0, rotation=0):
        self.points = points
        self.height = height
        self.type = obs_type
        self.radius = radius
        self.rotation = rotation
        
        if obs_type == "circle":
            self.center = points[0]
        else:
            self.center = (sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points))
    
    def check_collision(self, lat, lon, safety_margin=20):
        """检查水平碰撞"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center[0])**2 + (lon-self.center[1])**2) * 111000
            return dist < (self.radius + safety_margin)
        
        # 多边形
        if point_in_polygon(lat, lon, self.points):
            return True
        
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i+1) % n]
            if point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1]) < safety_margin:
                return True
        return False
    
    def get_distance(self, lat, lon):
        """获取到障碍物的最短距离"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center[0])**2 + (lon-self.center[1])**2) * 111000
            return max(0, dist - self.radius)
        
        if point_in_polygon(lat, lon, self.points):
            return 0
        
        min_dist = float('inf')
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i+1) % n]
            dist = point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1])
            min_dist = min(min_dist, dist)
        return min_dist

# ==================== 3D航点类 ====================
class Waypoint3D:
    def __init__(self, lat, lon, alt, seq=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.seq = seq

# ==================== 路径规划器 ====================
class PathPlanner3D:
    def __init__(self):
        self.obstacles = []
        self.safety_margin = 20
        self.vertical_margin = 10  # 垂直安全边距
    
    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
    
    def clear(self):
        self.obstacles = []
    
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def check_collision_at_alt(self, lat, lon, alt):
        """在指定高度检查碰撞"""
        for obs in self.obstacles:
            # 如果飞行高度 >= 障碍物高度 + 安全边距，可以飞越
            if alt >= obs.height + self.vertical_margin:
                continue
            
            # 否则检查水平碰撞
            if obs.check_collision(lat, lon, self.safety_margin):
                return True, obs
        return False, None
    
    def get_max_obstacle_height_on_path(self, start, end):
        """获取路径上的最大障碍物高度"""
        max_height = 0
        dist = self.haversine(start[0], start[1], end[0], end[1])
        steps = max(int(dist / 5), 20)  # 每5米检查
        
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            
            for obs in self.obstacles:
                if obs.check_collision(lat, lon, 0):  # 检查是否在障碍物水平范围内
                    max_height = max(max_height, obs.height)
        
        return max_height
    
    def plan_path_horizontal(self, start, end, flight_alt):
        """策略1: 水平绕行 - 保持高度，绕过障碍物"""
        path = []
        
        # 检查直线路径
        if self.is_path_clear(start, end, flight_alt):
            return [Waypoint3D(start[0], start[1], flight_alt, 0),
                   Waypoint3D(end[0], end[1], flight_alt, 1)]
        
        # 需要绕行，使用A*搜索
        return self.astar_2d(start, end, flight_alt)
    
    def plan_path_climb(self, start, end, flight_alt, max_altitude=120):
        """策略2: 爬升飞越 - 爬升到障碍物上方，直线飞过"""
        # 获取路径上的最大障碍物高度
        max_obs_height = self.get_max_obstacle_height_on_path(start, end)
        
        if max_obs_height == 0:
            # 无障碍物，直接飞行
            return [Waypoint3D(start[0], start[1], flight_alt, 0),
                   Waypoint3D(end[0], end[1], flight_alt, 1)]
        
        # 计算需要的飞越高度
        fly_over_alt = max_obs_height + self.vertical_margin
        
        if fly_over_alt > max_altitude:
            st.warning(f"⚠️ 需要飞越高度{fly_over_alt}m超过最大限制{max_altitude}m，建议水平绕行")
            return None
        
        # 生成爬升-飞越-下降路径
        path = []
        
        # 起点
        path.append(Waypoint3D(start[0], start[1], flight_alt, 0))
        
        # 计算爬升点（距离起点50米处）
        dist_total = self.haversine(start[0], start[1], end[0], end[1])
        if dist_total > 100:
            climb_ratio = 50 / dist_total
            climb_lat = start[0] + (end[0] - start[0]) * climb_ratio
            climb_lon = start[1] + (end[1] - start[1]) * climb_ratio
            path.append(Waypoint3D(climb_lat, climb_lon, fly_over_alt, 1))
        
        # 飞越点（路径中点，保持高度）
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        path.append(Waypoint3D(mid_lat, mid_lon, fly_over_alt, 2))
        
        # 下降点（距离终点50米处）
        if dist_total > 100:
            descend_ratio = (dist_total - 50) / dist_total
            descend_lat = start[0] + (end[0] - start[0]) * descend_ratio
            descend_lon = start[1] + (end[1] - start[1]) * descend_ratio
            path.append(Waypoint3D(descend_lat, descend_lon, fly_over_alt, 3))
        
        # 终点
        path.append(Waypoint3D(end[0], end[1], flight_alt, len(path)))
        
        return path
    
    def is_path_clear(self, start, end, alt):
        """检查路径是否畅通"""
        dist = self.haversine(start[0], start[1], end[0], end[1])
        steps = max(int(dist / 5), 10)
        
        for i in range(steps + 1):
            t = i / steps
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            
            collision, _ = self.check_collision_at_alt(lat, lon, alt)
            if collision:
                return False
        return True
    
    def astar_2d(self, start, end, flight_alt):
        """2D A*路径规划（固定高度）"""
        # 简化的A*实现
        open_set = [(0, start, [start])]  # (cost, pos, path)
        visited = set()
        
        # 生成候选方向（8个方向 + 不同距离）
        directions = []
        for angle in range(0, 360, 22):
            rad = math.radians(angle)
            for dist in [30, 60, 90]:  # 不同步长
                dlat = math.cos(rad) * dist / 111000
                dlon = math.sin(rad) * dist / (111000 * math.cos(math.radians(start[0])))
                directions.append((dlat, dlon))
        
        max_iter = 2000
        iteration = 0
        
        while open_set and iteration < max_iter:
            iteration += 1
            cost, current, path = heapq.heappop(open_set)
            
            # 检查是否到达终点
            if self.haversine(current[0], current[1], end[0], end[1]) < 30:
                # 构建航点
                waypoints = [Waypoint3D(p[0], p[1], flight_alt, i) for i, p in enumerate(path + [end])]
                return waypoints
            
            key = (round(current[0], 6), round(current[1], 6))
            if key in visited:
                continue
            visited.add(key)
            
            # 生成邻居
            for dlat, dlon in directions:
                new_lat = current[0] + dlat
                new_lon = current[1] + dlon
                new_pos = (new_lat, new_lon)
                
                # 检查新位置是否有效
                if not self.is_path_clear(current, new_pos, flight_alt):
                    continue
                
                new_cost = cost + self.haversine(current[0], current[1], new_lat, new_lon)
                new_cost += self.haversine(new_lat, new_lon, end[0], end[1])  # 启发式
                
                heapq.heappush(open_set, (new_cost, new_pos, path + [current]))
        
        # 失败，返回直线路径
        st.warning("⚠️ 绕行规划失败，返回直线路径")
        return [Waypoint3D(start[0], start[1], flight_alt, 0),
                Waypoint3D(end[0], end[1], flight_alt, 1)]

# ==================== 初始化 ====================
if 'planner' not in st.session_state:
    st.session_state.planner = PathPlanner3D()
if 'point_a' not in st.session_state:
    st.session_state.point_a = None
if 'point_b' not in st.session_state:
    st.session_state.point_b = None
if 'path_horizontal' not in st.session_state:
    st.session_state.path_horizontal = None
if 'path_climb' not in st.session_state:
    st.session_state.path_climb = None
if 'selected_path' not in st.session_state:
    st.session_state.selected_path = None
if 'temp_shape' not in st.session_state:
    st.session_state.temp_shape = None

# ==================== 页面 ====================
st.set_page_config(page_title="无人机双策略避障规划", layout="wide")
st.title("🚁 无人机双策略避障路径规划")
st.caption("支持水平绕行 | 爬升飞越 | 3D路径规划")

col1, col2 = st.columns([3, 2])

with col1:
    st.subheader("🗺️ 地图（绘制障碍物）")
    
    # 地图中心
    center = [32.0603, 118.7969]
    if st.session_state.point_a and st.session_state.point_b:
        center = [(st.session_state.point_a[0] + st.session_state.point_b[0])/2,
                 (st.session_state.point_a[1] + st.session_state.point_b[1])/2]
    
    m = folium.Map(location=center, zoom_start=16, tiles="CartoDB positron")
    
    # 绘制工具
    Draw(
        draw_options={'polyline': False, 'rectangle': True, 'polygon': True, 
                     'circle': True, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True}
    ).add_to(m)
    
    # 显示A/B点
    if st.session_state.point_a:
        folium.Marker(st.session_state.point_a, 
                     popup="起点A",
                     icon=folium.Icon(color='green', icon='play', prefix='glyphicon')).add_to(m)
    if st.session_state.point_b:
        folium.Marker(st.session_state.point_b,
                     popup="终点B", 
                     icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')).add_to(m)
    
    # 显示障碍物
    for i, obs in enumerate(st.session_state.planner.obstacles):
        color = 'red' if obs.height > 50 else 'orange'
        if obs.type == 'circle':
            folium.Circle(obs.center, radius=obs.radius, color=color, fill=True, 
                         fill_opacity=0.4, popup=f"障碍{i+1} 高度{obs.height}m").add_to(m)
        else:
            folium.Polygon(obs.points, color=color, fill=True, fill_opacity=0.4,
                          popup=f"障碍{i+1} 高度{obs.height}m").add_to(m)
    
    # 显示路径
    if st.session_state.selected_path == 'horizontal' and st.session_state.path_horizontal:
        coords = [[wp.lat, wp.lon] for wp in st.session_state.path_horizontal]
        folium.PolyLine(coords, color='blue', weight=5, opacity=0.9, 
                       popup='水平绕行路径').add_to(m)
        # 显示高度标记
        for wp in st.session_state.path_horizontal[::2]:  # 每隔一个点显示
            folium.CircleMarker([wp.lat, wp.lon], radius=4, color='blue', fill=True,
                               popup=f'高度: {wp.alt}m').add_to(m)
    
    elif st.session_state.selected_path == 'climb' and st.session_state.path_climb:
        coords = [[wp.lat, wp.lon] for wp in st.session_state.path_climb]
        folium.PolyLine(coords, color='green', weight=5, opacity=0.9,
                       popup='爬升飞越路径').add_to(m)
        # 显示高度变化
        for wp in st.session_state.path_climb:
            color = 'green' if wp.alt > 60 else 'blue'
            folium.CircleMarker([wp.lat, wp.lon], radius=5, color=color, fill=True,
                               popup=f'高度: {wp.alt}m').add_to(m)
    
    # 捕获绘制
    result = st_folium(m, width=800, height=600, key="map")
    
    if result and result.get("last_active_drawing"):
        drawing = result["last_active_drawing"]
        geom_type = drawing.get("type")
        shape_id = f"{geom_type}_{str(drawing)}"
        
        if st.session_state.get("last_shape_id") != shape_id:
            st.session_state["last_shape_id"] = shape_id
            
            if geom_type == "circle":
                center = drawing["geometry"]["coordinates"]
                radius = drawing["properties"]["radius"]
                st.session_state.temp_shape = {
                    "type": "circle",
                    "center": (center[1], center[0]),
                    "radius": radius
                }
                st.rerun()
                
            elif geom_type in ["polygon", "rectangle"]:
                coords = drawing["geometry"]["coordinates"][0]
                points = [(c[1], c[0]) for c in coords[:-1]]
                st.session_state.temp_shape = {
                    "type": "polygon",
                    "points": points
                }
                st.rerun()

with col2:
    st.subheader("⚙️ 控制面板")
    
    # 起点终点设置
    st.markdown("**🟢 起点 A**")
    c1, c2 = st.columns(2)
    a_lat = c1.number_input("纬度", value=32.0603, format="%.6f", key="a_lat")
    a_lon = c2.number_input("经度", value=118.7969, format="%.6f", key="a_lon")
    if st.button("✅ 设置A点", key="set_a"):
        st.session_state.point_a = (a_lat, a_lon)
        st.rerun()
    
    st.markdown("**🔴 终点 B**")
    c3, c4 = st.columns(2)
    b_lat = c3.number_input("纬度", value=32.0703, format="%.6f", key="b_lat")
    b_lon = c4.number_input("经度", value=118.8069, format="%.6f", key="b_lon")
    if st.button("✅ 设置B点", key="set_b"):
        st.session_state.point_b = (b_lat, b_lon)
        st.rerun()
    
    st.markdown("---")
    
    # 障碍物处理
    if st.session_state.temp_shape:
        st.info("📐 检测到新绘制的障碍物")
        shape = st.session_state.temp_shape
        
        if shape["type"] == "circle":
            st.write(f"⭕ 圆形：半径{shape['radius']:.1f}m")
        else:
            st.write(f"📐 多边形：{len(shape['points'])}个顶点")
        
        obs_height = st.number_input("障碍物高度(m)", 10, 200, 60, key="obs_h")
        
        col_add, col_cancel = st.columns(2)
        with col_add:
            if st.button("✅ 确认添加", type="primary"):
                if shape["type"] == "circle":
                    obs = Obstacle([shape["center"]], obs_height, "circle", shape["radius"])
                else:
                    obs = Obstacle(shape["points"], obs_height, "polygon")
                
                st.session_state.planner.add_obstacle(obs)
                st.session_state.temp_shape = None
                st.success("✅ 障碍物已添加！")
                st.rerun()
        
        with col_cancel:
            if st.button("❌ 取消"):
                st.session_state.temp_shape = None
                st.rerun()
        
        st.markdown("---")
    
    # 障碍物列表
    if st.session_state.planner.obstacles:
        st.markdown(f"**🚧 已有障碍物：{len(st.session_state.planner.obstacles)}个**")
        for i, obs in enumerate(st.session_state.planner.obstacles):
            icon = "⭕" if obs.type == "circle" else "📐"
            st.write(f"{icon} #{i+1}: 高度{obs.height}m")
        
        if st.button("🗑️ 清除所有障碍物"):
            st.session_state.planner.clear()
            st.session_state.path_horizontal = None
            st.session_state.path_climb = None
            st.session_state.selected_path = None
            st.rerun()
        st.markdown("---")
    
    # 飞行参数
    st.markdown("**✈️ 飞行参数**")
    flight_alt = st.slider("设定飞行高度(m)", 10, 120, 50, key="flight_alt")
    max_alt = st.slider("最大允许高度(m)", flight_alt + 10, 200, 120, key="max_alt")
    
    # 路径规划
    if st.session_state.point_a and st.session_state.point_b:
        st.markdown("---")
        st.markdown("**🧮 路径规划策略**")
        
        col_h, col_c = st.columns(2)
        
        with col_h:
            if st.button("🔵 规划水平绕行", use_container_width=True):
                with st.spinner("规划水平绕行路径..."):
                    path = st.session_state.planner.plan_path_horizontal(
                        st.session_state.point_a,
                        st.session_state.point_b,
                        flight_alt
                    )
                    st.session_state.path_horizontal = path
                    st.session_state.selected_path = 'horizontal'
                    st.success(f"✅ 水平绕行：{len(path)}个航点")
                    st.rerun()
        
        with col_c:
            if st.button("🟢 规划爬升飞越", use_container_width=True):
                with st.spinner("规划爬升飞越路径..."):
                    path = st.session_state.planner.plan_path_climb(
                        st.session_state.point_a,
                        st.session_state.point_b,
                        flight_alt,
                        max_alt
                    )
                    if path:
                        st.session_state.path_climb = path
                        st.session_state.selected_path = 'climb'
                        max_fly_alt = max(wp.alt for wp in path)
                        st.success(f"✅ 爬升飞越：最高{max_fly_alt}m，{len(path)}个航点")
                        st.rerun()
        
        # 显示路径对比
        if st.session_state.path_horizontal or st.session_state.path_climb:
            st.markdown("---")
            st.markdown("**📊 路径对比**")
            
            # 计算距离
            if st.session_state.path_horizontal:
                dist_h = sum(st.session_state.planner.haversine(
                    st.session_state.path_horizontal[i].lat, st.session_state.path_horizontal[i].lon,
                    st.session_state.path_horizontal[i+1].lat, st.session_state.path_horizontal[i+1].lon)
                    for i in range(len(st.session_state.path_horizontal)-1))
                st.write(f"🔵 水平绕行: {dist_h:.0f}m")
            
            if st.session_state.path_climb:
                dist_c = sum(st.session_state.planner.haversine(
                    st.session_state.path_climb[i].lat, st.session_state.path_climb[i].lon,
                    st.session_state.path_climb[i+1].lat, st.session_state.path_climb[i+1].lon)
                    for i in range(len(st.session_state.path_climb)-1))
                max_alt_c = max(wp.alt for wp in st.session_state.path_climb)
                st.write(f"🟢 爬升飞越: {dist_c:.0f}m (最高{max_alt_c}m)")
            
            # 路径选择
            st.markdown("**🎯 选择显示路径**")
            path_options = []
            if st.session_state.path_horizontal:
                path_options.append("水平绕行")
            if st.session_state.path_climb:
                path_options.append("爬升飞越")
            
            selected = st.radio("显示路径", path_options, 
                              index=0 if st.session_state.selected_path == 'horizontal' else 1 if st.session_state.selected_path == 'climb' else 0,
                              horizontal=True)
            
            new_selection = 'horizontal' if selected == "水平绕行" else 'climb'
            if new_selection != st.session_state.selected_path:
                st.session_state.selected_path = new_selection
                st.rerun()
    else:
        st.warning("⚠️ 请先设置起点A和终点B")

st.markdown("---")
st.caption("双策略3D避障规划器 | 水平绕行 vs 爬升飞越")
