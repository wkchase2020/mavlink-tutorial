import streamlit as st
import time
from datetime import datetime
import pytz
import math
import random
import folium
from folium import plugins
from streamlit_folium import st_folium
import numpy as np

# 页面配置
st.set_page_config(
    page_title="无人机地面站经纬规划系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# 自定义CSS样式
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: bold;
        color: #1f77b4;
        text-align: center;
        margin-bottom: 1rem;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
    }
    .sub-header {
        font-size: 1.2rem;
        color: #555;
        text-align: center;
        margin-bottom: 2rem;
    }
    .metric-card {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        padding: 1rem;
        border-radius: 10px;
        color: white;
        text-align: center;
        box-shadow: 0 4px 6px rgba(0,0,0,0.1);
    }
    .control-panel {
        background-color: #f8f9fa;
        padding: 1.5rem;
        border-radius: 10px;
        border: 1px solid #dee2e6;
        margin-bottom: 1rem;
    }
    .info-box {
        background-color: #e7f3ff;
        border-left: 4px solid #2196F3;
        padding: 1rem;
        margin: 1rem 0;
        border-radius: 0 5px 5px 0;
    }
    .warning-box {
        background-color: #fff3cd;
        border-left: 4px solid #ffc107;
        padding: 1rem;
        margin: 1rem 0;
        border-radius: 0 5px 5px 0;
    }
    .success-box {
        background-color: #d4edda;
        border-left: 4px solid #28a745;
        padding: 1rem;
        margin: 1rem 0;
        border-radius: 0 5px 5px 0;
    }
    .stButton>button {
        width: 100%;
        border-radius: 5px;
        font-weight: bold;
        transition: all 0.3s;
    }
    .stButton>button:hover {
        transform: translateY(-2px);
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
    }
</style>
""", unsafe_allow_html=True)

# ==================== 初始化Session State ====================

def init_session_state():
    """初始化所有session state变量"""
    defaults = {
        'obstacles': [],
        'flight_path': [],
        'path_planned': False,
        'drone_pos': None,
        'simulating': False,
        'map_center': [39.9042, 116.4074],  # 北京默认中心
        'zoom': 13,
        'point_a': None,
        'point_b': None,
        'planning_method': 'astar',
        'flight_height': 50,
        'safety_margin': 10,
        'show_grid': False,
        'obstacle_counter': 0,
        'last_click': None,
        'map_key': 0,
        'path_key': 0,
    }
    
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# ==================== 工具函数 ====================

def get_beijing_time():
    """获取北京时间"""
    utc_now = datetime.now(pytz.utc)
    beijing_tz = pytz.timezone('Asia/Shanghai')
    return utc_now.astimezone(beijing_tz)

def haversine_distance(lat1, lon1, lat2, lon2):
    """计算两点间距离（米）"""
    R = 6371000  # 地球半径（米）
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def coordinate_offset(lat, lon, distance_north, distance_east):
    """计算坐标偏移（米转经纬度）"""
    lat_offset = distance_north / 111000
    lon_offset = distance_east / (111000 * math.cos(math.radians(lat)))
    return lat + lat_offset, lon + lon_offset

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

def create_rotated_rectangle(center_lat, center_lon, width, height, rotation):
    """创建旋转矩形（返回4个角点坐标）"""
    lat_offset = width / 2 / 111000
    lon_offset = height / 2 / (111000 * math.cos(math.radians(center_lat)))
    
    corners = [
        (center_lat + lat_offset, center_lon + lon_offset),
        (center_lat + lat_offset, center_lon - lon_offset),
        (center_lat - lat_offset, center_lon - lon_offset),
        (center_lat - lat_offset, center_lon + lon_offset),
    ]
    
    rotated_corners = []
    for lat, lon in corners:
        new_lat, new_lon = rotate_point(center_lat, center_lon, lat, lon, rotation)
        rotated_corners.append((new_lat, new_lon))
    
    return rotated_corners

def point_in_polygon(lat, lon, polygon_points):
    """射线法判断点是否在多边形内（纯Python实现，替代shapely）"""
    n = len(polygon_points)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = polygon_points[i][1], polygon_points[i][0]  # lon, lat
        xj, yj = polygon_points[j][1], polygon_points[j][0]
        
        if ((yi > lon) != (yj > lon)) and (lat < (xj - xi) * (lon - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside

def check_collision(lat, lon, height, obstacles, safety_margin=10):
    """检查点是否与障碍物碰撞（3D检查）"""
    for obs in obstacles:
        obs_type = obs.get('type', 'circle')
        obs_height = obs.get('height', 100)
        
        # 如果飞行高度高于障碍物高度+安全余量，不碰撞
        if height > obs_height + safety_margin:
            continue
            
        if obs_type == 'circle':
            center = obs['center']
            radius = obs.get('radius', 50)
            dist = haversine_distance(lat, lon, center[0], center[1])
            if dist < radius + safety_margin:
                return True
                
        elif obs_type in ['rectangle', 'polygon']:
            points = obs.get('points', [])
            if len(points) >= 3:
                # 使用射线法检测点是否在多边形内
                # 为了考虑安全余量，我们稍微扩大检测范围（简化处理）
                if point_in_polygon(lat, lon, points):
                    return True
                # 检查距离多边形边界是否太近（简化：检查距离各顶点）
                for p in points:
                    if haversine_distance(lat, lon, p[0], p[1]) < safety_margin:
                        return True
    
    return False

# ==================== 路径规划算法 ====================

class Node3D:
    """3D路径规划节点"""
    def __init__(self, lat, lon, alt, g=0, h=0, parent=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return (abs(self.lat - other.lat) < 1e-6 and 
                abs(self.lon - other.lon) < 1e-6 and 
                abs(self.alt - other.alt) < 0.1)

def heuristic_3d(node, goal):
    """3D启发函数"""
    h_dist = haversine_distance(node.lat, node.lon, goal.lat, goal.lon)
    v_dist = abs(node.alt - goal.alt)
    return math.sqrt(h_dist**2 + v_dist**2)

def astar_3d(start_lat, start_lon, start_alt, goal_lat, goal_lon, goal_alt, 
             obstacles, safety_margin=10, max_iter=2000):
    """
    3D A*路径规划算法 - 优先水平绕行
    """
    start = Node3D(start_lat, start_lon, start_alt)
    goal = Node3D(goal_lat, goal_lon, goal_alt)
    
    open_list = [start]
    closed_set = set()
    
    # 定义26个方向（3D邻居）
    directions = []
    step_dist = 25  # 步长25米
    
    for dl in [-1, 0, 1]:
        for dn in [-1, 0, 1]:
            for da in [-1, 0, 1]:
                if dl == 0 and dn == 0 and da == 0:
                    continue
                directions.append((dl, dn, da))
    
    iteration = 0
    while open_list and iteration < max_iter:
        iteration += 1
        
        # 获取f值最小的节点
        current = min(open_list, key=lambda x: x.f)
        open_list.remove(current)
        
        # 检查是否到达目标
        dist_to_goal = haversine_distance(current.lat, current.lon, goal.lat, goal.lon)
        if dist_to_goal < 15 and abs(current.alt - goal.alt) < 5:
            # 重建路径
            path = []
            node = current
            while node:
                path.append((node.lat, node.lon, node.alt))
                node = node.parent
            return path[::-1]
        
        closed_key = (round(current.lat, 6), round(current.lon, 6), round(current.alt, 1))
        if closed_key in closed_set:
            continue
        closed_set.add(closed_key)
        
        # 生成邻居
        for dl, dn, da in directions:
            new_lat, new_lon = coordinate_offset(current.lat, current.lon, 
                                                 dn * step_dist, dl * step_dist)
            new_alt = current.alt + da * 5
            
            # 边界检查
            if new_alt < 10 or new_alt > 120:
                continue
            
            # 碰撞检查
            if check_collision(new_lat, new_lon, new_alt, obstacles, safety_margin):
                continue
            
            # 创建新节点
            g_new = current.g + math.sqrt((dl*step_dist)**2 + (dn*step_dist)**2 + (da*5)**2)
            neighbor = Node3D(new_lat, new_lon, new_alt, g_new, 0, current)
            neighbor.h = heuristic_3d(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h
            
            neighbor_key = (round(new_lat, 6), round(new_lon, 6), round(new_alt, 1))
            if neighbor_key in closed_set:
                continue
            
            # 检查open_list中是否已有更优路径
            existing = next((n for n in open_list if 
                           abs(n.lat - new_lat) < 1e-6 and 
                           abs(n.lon - new_lon) < 1e-6 and 
                           abs(n.alt - new_alt) < 0.1), None)
            
            if existing and existing.g <= g_new:
                continue
            
            if existing:
                open_list.remove(existing)
            
            open_list.append(neighbor)
    
    return None

# ==================== 地图创建函数 ====================

def create_base_map():
    """创建基础地图"""
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=st.session_state.zoom,
        tiles='OpenStreetMap'
    )
    
    # 添加不同图层
    folium.TileLayer('CartoDB positron', name='浅色地图').add_to(m)
    folium.TileLayer('CartoDB dark_matter', name='深色地图').add_to(m)
    folium.TileLayer(
        tiles='http://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
        attr='高德地图',
        name='高德地图',
        subdomains=['1', '2', '3', '4']
    ).add_to(m)
    
    plugins.MousePosition().add_to(m)
    plugins.MeasureControl(position='topright').add_to(m)
    
    draw = plugins.Draw(
        export=True,
        filename='drone_plan.geojson',
        position='topleft',
        draw_options={
            'polyline': False,
            'rectangle': True,
            'polygon': True,
            'circle': True,
            'marker': False,
            'circlemarker': False
        },
        edit_options={'edit': True}
    )
    draw.add_to(m)
    
    return m

def add_obstacles_to_map(m):
    """在地图上添加障碍物"""
    for i, obs in enumerate(st.session_state.obstacles):
        is_blocking = obs.get('height', 0) > st.session_state.flight_height
        color = '#FF4444' if is_blocking else '#FFAA00'
        
        if obs['type'] == 'circle':
            folium.Circle(
                location=obs['center'],
                radius=obs.get('radius', 50),
                popup=f"障碍物 {i+1}<br>高度: {obs.get('height', 100)}m<br>类型: 圆形",
                color=color,
                fill=True,
                fillColor=color,
                fillOpacity=0.4,
                weight=2
            ).add_to(m)
            
        elif obs['type'] == 'rectangle':
            points = obs.get('points', [])
            if len(points) >= 3:
                folium.Polygon(
                    locations=points,
                    popup=f"障碍物 {i+1}<br>高度: {obs.get('height', 100)}m<br>类型: 矩形<br>旋转: {obs.get('rotation', 0)}°",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.4,
                    weight=2
                ).add_to(m)
                
        elif obs['type'] == 'polygon':
            points = obs.get('points', [])
            if len(points) >= 3:
                folium.Polygon(
                    locations=points,
                    popup=f"障碍物 {i+1}<br>高度: {obs.get('height', 100)}m<br>类型: 多边形",
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.4,
                    weight=2
                ).add_to(m)

def add_path_to_map(m, path, color='#00FF00', weight=4):
    """在地图上添加路径"""
    if not path or len(path) < 2:
        return
    
    points = [(p[0], p[1]) for p in path]
    
    folium.PolyLine(
        locations=points,
        color=color,
        weight=weight,
        opacity=0.8,
        popup='规划路径'
    ).add_to(m)
    
    # 添加起点和终点标记
    folium.CircleMarker(
        location=points[0],
        radius=8,
        color='#00AA00',
        fill=True,
        fillOpacity=0.8,
        popup='起点'
    ).add_to(m)
    
    folium.CircleMarker(
        location=points[-1],
        radius=8,
        color='#AA0000',
        fill=True,
        fillOpacity=0.8,
        popup='终点'
    ).add_to(m)
    
    # 添加高度变化标记
    for i in range(0, len(path), max(1, len(path)//10)):
        p = path[i]
        folium.CircleMarker(
            location=(p[0], p[1]),
            radius=3,
            color='#0066CC',
            fill=True,
            fillColor='#0066CC',
            fillOpacity=0.6,
            popup=f'高度: {p[2]:.1f}m'
        ).add_to(m)

def add_markers_to_map(m):
    """添加起点终点标记"""
    if st.session_state.point_a:
        folium.Marker(
            location=st.session_state.point_a,
            popup='起点 A',
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(m)
    
    if st.session_state.point_b:
        folium.Marker(
            location=st.session_state.point_b,
            popup='终点 B',
            icon=folium.Icon(color='red', icon='stop', prefix='fa')
        ).add_to(m)

# ==================== 侧边栏控制 ====================

with st.sidebar:
    st.markdown("## 🚁 无人机地面站控制面板")
    
    beijing_time = get_beijing_time()
    st.markdown(f"**北京时间:** {beijing_time.strftime('%Y-%m-%d %H:%M:%S')}")
    st.markdown("---")
    
    st.markdown("### 🎯 规划模式")
    mode = st.radio(
        "选择操作模式",
        ["地图选点", "坐标输入", "路径规划", "飞行模拟"],
        index=0
    )
    
    st.markdown("---")
    
    st.markdown("### ⚙️ 飞行参数")
    
    flight_height = st.slider(
        "飞行高度 (米)",
        min_value=10,
        max_value=100,
        value=st.session_state.flight_height,
        step=5,
        key='flight_height_slider'
    )
    st.session_state.flight_height = flight_height
    
    safety_margin = st.slider(
        "安全余量 (米)",
        min_value=5,
        max_value=30,
        value=st.session_state.safety_margin,
        step=5,
        key='safety_margin_slider'
    )
    st.session_state.safety_margin = safety_margin
    
    st.markdown("---")
    
    st.markdown("### 🛠️ 快捷操作")
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🗑️ 清空障碍", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.path_planned = False
            st.session_state.flight_path = []
            st.success("已清空所有障碍物")
            st.rerun()
    
    with col2:
        if st.button("🔄 重置视图", use_container_width=True):
            st.session_state.map_center = [39.9042, 116.4074]
            st.session_state.zoom = 13
            st.rerun()
    
    if st.button("📍 定位到北京", use_container_width=True):
        st.session_state.map_center = [39.9042, 116.4074]
        st.session_state.zoom = 13
        st.rerun()
    
    st.markdown("---")
    st.markdown("### 📊 系统状态")
    st.markdown(f"**障碍物数量:** {len(st.session_state.obstacles)}")
    st.markdown(f"**路径状态:** {'已规划' if st.session_state.path_planned else '未规划'}")
    if st.session_state.flight_path:
        st.markdown(f"**航点数量:** {len(st.session_state.flight_path)}")

# ==================== 主界面 ====================

st.markdown('<div class="main-header">🚁 无人机地面站经纬规划系统</div>', unsafe_allow_html=True)
st.markdown('<div class="sub-header">支持地图选点、障碍物绘制、3D路径规划与飞行模拟</div>', unsafe_allow_html=True)

if mode == "地图选点":
    st.markdown('<div class="info-box">💡 <b>使用说明:</b> 在地图上点击选择起点A和终点B，或使用右侧输入框直接输入经纬度坐标</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.markdown("#### 📍 起点 A")
        if st.session_state.point_a:
            st.markdown(f"**已设置:** {st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}")
        lat_a = st.number_input("纬度 A", value=39.9042, format="%.6f", key='lat_a')
        lon_a = st.number_input("经度 A", value=116.4074, format="%.6f", key='lon_a')
        
        if st.button("✅ 设置起点A", key='set_a'):
            st.session_state.point_a = [lat_a, lon_a]
            st.session_state.map_center = [lat_a, lon_a]
            st.success(f"起点A已设置: ({lat_a:.6f}, {lon_a:.6f})")
            st.rerun()
    
    with col2:
        st.markdown("#### 🎯 终点 B")
        if st.session_state.point_b:
            st.markdown(f"**已设置:** {st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}")
        lat_b = st.number_input("纬度 B", value=39.9142, format="%.6f", key='lat_b')
        lon_b = st.number_input("经度 B", value=116.4174, format="%.6f", key='lon_b')
        
        if st.button("✅ 设置终点B", key='set_b'):
            st.session_state.point_b = [lat_b, lon_b]
            st.success(f"终点B已设置: ({lat_b:.6f}, {lon_b:.6f})")
            st.rerun()

elif mode == "坐标输入":
    st.markdown('<div class="info-box">💡 <b>批量输入:</b> 支持直接输入经纬度坐标，格式：纬度,经度</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns(2)
    
    with col1:
        coord_input_a = st.text_input("起点A坐标 (格式: 39.9042,116.4074)", "39.9042,116.4074")
        if st.button("📍 解析起点A"):
            try:
                lat, lon = map(float, coord_input_a.split(','))
                st.session_state.point_a = [lat, lon]
                st.success(f"起点A已设置: ({lat}, {lon})")
                st.rerun()
            except:
                st.error("格式错误！请使用: 纬度,经度")
    
    with col2:
        coord_input_b = st.text_input("终点B坐标 (格式: 39.9142,116.4174)", "39.9142,116.4174")
        if st.button("🎯 解析终点B"):
            try:
                lat, lon = map(float, coord_input_b.split(','))
                st.session_state.point_b = [lat, lon]
                st.success(f"终点B已设置: ({lat}, {lon})")
                st.rerun()
            except:
                st.error("格式错误！请使用: 纬度,经度")

elif mode == "路径规划":
    st.markdown('<div class="info-box">🧭 <b>路径规划:</b> 系统将基于A*算法规划避障路径，优先水平绕行</div>', unsafe_allow_html=True)
    
    if not st.session_state.point_a or not st.session_state.point_b:
        st.warning("⚠️ 请先设置起点A和终点B！")
    else:
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.markdown(f"**起点A:** {st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}")
        
        with col2:
            st.markdown(f"**终点B:** {st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}")
        
        with col3:
            dist = haversine_distance(
                st.session_state.point_a[0], st.session_state.point_a[1],
                st.session_state.point_b[0], st.session_state.point_b[1]
            )
            st.markdown(f"**直线距离:** {dist:.1f}m")
        
        if st.button("🚀 开始规划路径", type="primary", use_container_width=True):
            with st.spinner("正在规划路径..."):
                path = astar_3d(
                    st.session_state.point_a[0], st.session_state.point_a[1], st.session_state.flight_height,
                    st.session_state.point_b[0], st.session_state.point_b[1], st.session_state.flight_height,
                    st.session_state.obstacles,
                    st.session_state.safety_margin
                )
                
                if path:
                    st.session_state.flight_path = path
                    st.session_state.path_planned = True
                    st.success(f"✅ 路径规划成功！共{len(path)}个航点")
                else:
                    st.error("❌ 未找到可行路径，请调整障碍物或飞行高度")
                    st.session_state.path_planned = False
        
        if st.session_state.path_planned and st.session_state.flight_path:
            st.markdown('<div class="success-box">✅ <b>路径已生成</b> - 可在地图上查看绿色航线</div>', unsafe_allow_html=True)
            
            path = st.session_state.flight_path
            total_dist = 0
            for i in range(len(path)-1):
                total_dist += haversine_distance(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
            
            col1, col2, col3 = st.columns(3)
            col1.metric("总距离", f"{total_dist:.1f}m")
            col2.metric("航点数量", len(path))
            col3.metric("预计时间", f"{total_dist/15/60:.1f}min")

elif mode == "飞行模拟":
    st.markdown('<div class="info-box">🎮 <b>飞行模拟:</b> 模拟无人机沿规划路径飞行，实时显示位置和高度</div>', unsafe_allow_html=True)
    
    if not st.session_state.path_planned:
        st.warning("⚠️ 请先规划路径！")
    else:
        if st.button("▶️ 开始飞行模拟", type="primary", use_container_width=True):
            st.session_state.simulating = True
            st.session_state.path_key += 1
        
        if st.session_state.simulating:
            progress_bar = st.progress(0)
            status_text = st.empty()
            
            path = st.session_state.flight_path
            for i, point in enumerate(path):
                progress = (i + 1) / len(path)
                progress_bar.progress(min(progress, 1.0))
                
                status_text.markdown(f"""
                **当前位置:** {point[0]:.6f}, {point[1]:.6f}  
                **当前高度:** {point[2]:.1f}m  
                **进度:** {i+1}/{len(path)}
                """)
                
                st.session_state.drone_pos = point
                time.sleep(0.05)
            
            st.session_state.simulating = False
            st.success("✅ 飞行模拟完成！")

# ==================== 障碍物管理 ====================

st.markdown("---")
st.markdown("### 🏢 障碍物管理")

obs_col1, obs_col2, obs_col3 = st.columns(3)

with obs_col1:
    st.markdown("#### ⭕ 圆形障碍物")
    obs_lat_c = st.number_input("中心纬度", value=st.session_state.map_center[0], format="%.6f", key='obs_lat_c')
    obs_lon_c = st.number_input("中心经度", value=st.session_state.map_center[1], format="%.6f", key='obs_lon_c')
    obs_radius = st.slider("半径 (米)", 10, 200, 50, key='obs_radius')
    obs_height_c = st.slider("障碍物高度 (米)", 20, 150, 80, key='obs_height_c')
    
    if st.button("➕ 添加圆形障碍", key='add_circle'):
        st.session_state.obstacles.append({
            'type': 'circle',
            'center': [obs_lat_c, obs_lon_c],
            'radius': obs_radius,
            'height': obs_height_c,
            'id': st.session_state.obstacle_counter
        })
        st.session_state.obstacle_counter += 1
        st.success(f"已添加圆形障碍物 #{st.session_state.obstacle_counter}")
        st.rerun()

with obs_col2:
    st.markdown("#### ⬜ 矩形障碍物（可旋转）")
    obs_lat_r = st.number_input("中心纬度", value=st.session_state.map_center[0], format="%.6f", key='obs_lat_r')
    obs_lon_r = st.number_input("中心经度", value=st.session_state.map_center[1], format="%.6f", key='obs_lon_r')
    obs_width = st.slider("宽度 (米)", 10, 200, 80, key='obs_width')
    obs_length = st.slider("长度 (米)", 10, 200, 120, key='obs_length')
    obs_rotation = st.slider("旋转角度 (度)", 0, 360, 0, key='obs_rotation')
    obs_height_r = st.slider("障碍物高度 (米)", 20, 150, 80, key='obs_height_r')
    
    if st.button("➕ 添加矩形障碍", key='add_rect'):
        points = create_rotated_rectangle(obs_lat_r, obs_lon_r, obs_width, obs_length, obs_rotation)
        st.session_state.obstacles.append({
            'type': 'rectangle',
            'center': [obs_lat_r, obs_lon_r],
            'points': points,
            'width': obs_width,
            'length': obs_length,
            'rotation': obs_rotation,
            'height': obs_height_r,
            'id': st.session_state.obstacle_counter
        })
        st.session_state.obstacle_counter += 1
        st.success(f"已添加矩形障碍物 #{st.session_state.obstacle_counter} (旋转{obs_rotation}°)")
        st.rerun()

with obs_col3:
    st.markdown("#### 📋 障碍物列表")
    if st.session_state.obstacles:
        for i, obs in enumerate(st.session_state.obstacles):
            col_del, col_info = st.columns([1, 4])
            with col_del:
                if st.button("🗑️", key=f'del_{i}'):
                    st.session_state.obstacles.pop(i)
                    st.rerun()
            with col_info:
                obs_type = "⭕" if obs['type'] == 'circle' else "⬜" if obs['type'] == 'rectangle' else "🔺"
                st.markdown(f"{obs_type} #{i+1} 高度:{obs.get('height', 100)}m")
    else:
        st.info("暂无障碍物")

# ==================== 地图显示 ====================

st.markdown("---")
st.markdown("### 🗺️ 实时地图")

m = create_base_map()
add_obstacles_to_map(m)
add_markers_to_map(m)

if st.session_state.flight_path:
    add_path_to_map(m, st.session_state.flight_path)

if st.session_state.drone_pos:
    folium.Marker(
        location=[st.session_state.drone_pos[0], st.session_state.drone_pos[1]],
        popup=f"无人机<br>高度: {st.session_state.drone_pos[2]:.1f}m",
        icon=folium.Icon(color='blue', icon='plane', prefix='fa')
    ).add_to(m)

map_data = st_folium(m, width=1200, height=600, key=f"folium_map_{st.session_state.map_key}")

if map_data and map_data.get('last_clicked'):
    clicked_lat = map_data['last_clicked']['lat']
    clicked_lng = map_data['last_clicked']['lng']
    st.markdown(f"**最后点击位置:** {clicked_lat:.6f}, {clicked_lng:.6f}")

# ==================== 页脚 ====================

st.markdown("---")
st.markdown("""
<div style="text-align: center; color: #666; padding: 1rem;">
    <p>🚁 无人机地面站经纬规划系统 | 基于Streamlit + Folium开发</p>
    <p>支持功能：地图选点、3D路径规划、障碍物管理、飞行模拟</p>
</div>
""", unsafe_allow_html=True)
