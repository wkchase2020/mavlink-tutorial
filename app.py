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

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 地面站 - 多边形避障规划系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

def get_local_time():
    return datetime.utcnow() + timedelta(hours=8)

# ==================== 核心类 ====================
class Waypoint:
    def __init__(self, lat, lon, alt=50, cmd=16, seq=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.cmd = cmd
        self.seq = seq

class Obstacle:
    """支持多边形的障碍物"""
    def __init__(self, points, height, name="障碍物", obs_type="polygon"):
        """
        points: 多边形顶点列表 [(lat1,lon1), (lat2,lon2), ...] 或 圆心(lat,lon)
        height: 高度
        obs_type: "polygon"(多边形) 或 "circle"(圆形)
        """
        self.points = points if isinstance(points, list) else [points]
        self.height = height
        self.name = name
        self.type = obs_type
        
        # 计算中心点和边界
        if obs_type == "polygon" and len(self.points) > 0:
            self.center_lat = sum(p[0] for p in self.points) / len(self.points)
            self.center_lon = sum(p[1] for p in self.points) / len(self.points)
            # 计算近似半径（用于路径规划）
            self.radius = max(
                math.sqrt((p[0]-self.center_lat)**2 + (p[1]-self.center_lon)**2) * 111000 
                for p in self.points
            )
        else:
            self.center_lat = self.points[0][0]
            self.center_lon = self.points[0][1]
            self.radius = 30  # 默认半径
    
    def contains_point(self, lat, lon):
        """判断点是否在多边形内（射线法）"""
        if self.type == "circle":
            dist = math.sqrt((lat-self.center_lat)**2 + (lon-self.center_lon)**2) * 111000
            return dist < self.radius
        
        # 多边形判断
        n = len(self.points)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = self.points[i][1], self.points[i][0]
            xj, yj = self.points[j][1], self.points[j][0]
            
            if ((yi > lon) != (yj > lon)) and (lat < (xj - xi) * (lon - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

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
            if alt < obs.height:  # 高度低于障碍物顶部
                if obs.type == "polygon":
                    # 检查点是否在多边形内
                    if obs.contains_point(lat, lon):
                        return True, obs
                    # 检查点是否在多边形边界附近（安全边距）
                    for i in range(len(obs.points)):
                        p1 = obs.points[i]
                        p2 = obs.points[(i+1) % len(obs.points)]
                        dist = self.point_to_segment_distance(lat, lon, p1[0], p1[1], p2[0], p2[1])
                        if dist < self.safety_margin:
                            return True, obs
                else:  # circle
                    dist = self.haversine_distance(lat, lon, obs.center_lat, obs.center_lon)
                    if dist < (obs.radius + self.safety_margin):
                        return True, obs
        return False, None
    
    def point_to_segment_distance(self, lat, lon, lat1, lon1, lat2, lon2):
        """计算点到线段的距离"""
        # 简化的平面距离计算
        A = lon - lon1
        B = lat - lat1
        C = lon2 - lon1
        D = lat2 - lat1
        
        dot = A * C + B * D
        len_sq = C * C + D * D
        
        if len_sq == 0:
            return math.sqrt(A * A + B * B) * 111000
        
        t = max(0, min(1, dot / len_sq))
        proj_lat = lat1 + t * D
        proj_lon = lon1 + t * C
        
        return self.haversine_distance(lat, lon, proj_lat, proj_lon)
    
    def get_neighbors(self, node, end_node, step_size=20):
        """获取邻居节点"""
        neighbors = []
        
        dlat = end_node.lat - node.lat
        dlon = end_node.lon - node.lon
        dist = math.sqrt(dlat**2 + dlon**2)
        
        if dist > 0:
            dlat_norm = dlat / dist
            dlon_norm = dlon / dist
            
            # 16个方向
            directions = []
            for angle in range(0, 360, 22.5):  # 每22.5度一个方向
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
        'map_draw_data': None,  # 存储地图绘制数据
        'temp_obstacle_points': [],  # 临时存储多边形点
        'obstacle_height_input': 40,  # 障碍物高度输入
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 页面 ====================
st.title("🚁 MAVLink 地面站 - 多边形避障规划系统")
st.caption("支持多边形框选障碍物 | A*水平绕行 | 北京时间 (UTC+8)")

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
        
        1. **设置A/B点**：在右侧输入起点和终点坐标
        2. **框选障碍物**：
           - 在地图上点击 🔵 按钮（绘制多边形）
           - 依次点击地图画出建筑物轮廓
           - 双击完成绘制
           - 在右侧设置高度并点击"添加多边形障碍物"
        3. **规划路径**：点击"规划避障路径"
        
        ### 🚫 避障规则：
        - 障碍物高度 ≥ 飞行高度：**强制水平绕行**
        - 障碍物高度 < 飞行高度：**可以飞越**
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
                # 多边形障碍物
                folium.Polygon(
                    locations=obs.points,
                    popup=f"{obs.name}<br>高度:{obs.height}m",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.4,
                    weight=2
                ).add_to(m)
                # 添加标签
                folium.Marker(
                    [obs.center_lat, obs.center_lon],
                    icon=folium.DivIcon(
                        html=f'<div style="background:{color};color:white;padding:2px 6px;border-radius:3px;font-size:11px;">{obs.height}m</div>'
                    )
                ).add_to(m)
            else:
                # 圆形障碍物
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
        
        # 处理绘制数据
        if map_data['all_drawings']:
            st.session_state.map_draw_data = map_data['all_drawings']
            
            # 获取最后一个绘制的图形
            last_drawing = map_data['all_drawings'][-1]
            
            if last_drawing['type'] == 'polygon':
                # 提取多边形坐标
                coords = last_drawing['geometry']['coordinates'][0]  # [lon, lat] 列表
                # 转换为 (lat, lon) 元组列表
                points = [(coord[1], coord[0]) for coord in coords[:-1]]  # 去掉最后一个重复点
                st.session_state.temp_obstacle_points = points
                st.info(f"📐 已绘制多边形，{len(points)}个顶点，请在右侧设置高度并添加")
                
            elif last_drawing['type'] == 'rectangle':
                # 矩形转多边形
                coords = last_drawing['geometry']['coordinates'][0]
                points = [(coord[1], coord[0]) for coord in coords[:-1]]
                st.session_state.temp_obstacle_points = points
                st.info(f"📐 已绘制矩形，请在右侧设置高度并添加")
                
            elif last_drawing['type'] == 'circle':
                # 圆形
                center = last_drawing['geometry']['coordinates']
                radius = last_drawing['properties']['radius']
                st.session_state.temp_obstacle_points = [(center[1], center[0])]  # (lat, lon)
                st.session_state.temp_circle_radius = radius
                st.info(f"⭕ 已绘制圆形，半径{radius:.1f}m，请在右侧设置高度并添加")
    
    with col_ctrl:
        st.subheader("⚙️ 控制面板")
        
        # A点设置
        st.markdown("**🟢 起点 A**")
        c1, c2 = st.columns(2)
        lat_a = c1.number_input("纬度", value=32.0603, format="%.6f", key="lat_a")
        lon_a = c2.number_input("经度", value=118.7969, format="%.6f", key="lon_a")
        
        if st.button("✅ 设置A点", key="set_a"):
            st.session_state.point_a_gcj = (lat_a, lon_a)
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_a, lat_a)
                st.session_state.point_a = (lat_wgs, lon_wgs)
            else:
                st.session_state.point_a = (lat_a, lon_a)
            st.success(f"A点已设置")
            st.rerun()
        
        # B点设置
        st.markdown("**🔴 终点 B**")
        c3, c4 = st.columns(2)
        lat_b = c3.number_input("纬度", value=32.0703, format="%.6f", key="lat_b")
        lon_b = c4.number_input("经度", value=118.8069, format="%.6f", key="lon_b")
        
        if st.button("✅ 设置B点", key="set_b"):
            st.session_state.point_b_gcj = (lat_b, lon_b)
            if st.session_state.coord_system == 'GCJ-02':
                lon_wgs, lat_wgs = gcj02_to_wgs84(lon_b, lat_b)
                st.session_state.point_b = (lat_wgs, lon_wgs)
            else:
                st.session_state.point_b = (lat_b, lon_b)
            st.success(f"B点已设置")
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
        st.markdown("**🚧 障碍物设置**")
        
        # 显示当前绘制的图形
        if st.session_state.temp_obstacle_points:
            shape_type = "圆形" if len(st.session_state.temp_obstacle_points) == 1 else f"多边形({len(st.session_state.temp_obstacle_points)}顶点)"
            st.success(f"📐 已绘制: {shape_type}")
            
            obs_height = st.number_input("障碍物高度(m)", 5, 200, 40, key="obs_h")
            
            c5, c6 = st.columns(2)
            with c5:
                if st.button("➕ 添加障碍物", key="add_obs"):
                    if len(st.session_state.temp_obstacle_points) == 1:
                        # 圆形
                        center = st.session_state.temp_obstacle_points[0]
                        radius = getattr(st.session_state, 'temp_circle_radius', 30)
                        st.session_state.path_planner.add_circle_obstacle(
                            center[0], center[1], radius, obs_height,
                            f"圆形障碍({obs_height}m)"
                        )
                    else:
                        # 多边形
                        st.session_state.path_planner.add_polygon_obstacle(
                            st.session_state.temp_obstacle_points, obs_height,
                            f"多边形障碍({obs_height}m)"
                        )
                    
                    # 清空临时数据
                    st.session_state.temp_obstacle_points = []
                    if hasattr(st.session_state, 'temp_circle_radius'):
                        delattr(st.session_state, 'temp_circle_radius')
                    
                    st.success("✅ 障碍物已添加！")
                    st.rerun()
            
            with c6:
                if st.button("❌ 取消绘制", key="cancel_obs"):
                    st.session_state.temp_obstacle_points = []
                    st.rerun()
        else:
            st.info("💡 请在地图上绘制多边形/矩形/圆形障碍物")
        
        # 显示障碍物列表
        if st.session_state.path_planner.obstacles:
            with st.expander(f"📋 障碍物列表({len(st.session_state.path_planner.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.path_planner.obstacles):
                    need_detour = "🔴" if obs.height >= st.session_state.flight_altitude else "🟢"
                    st.write(f"{need_detour} #{i+1}: {obs.name} - {obs.type}")
                
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
            
            # 地图
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
            
            # 动画
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
st.caption("MAVLink GCS v4.0 | 多边形避障 | A*算法 | 北京时间 (UTC+8)")
