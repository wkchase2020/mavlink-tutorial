import streamlit as st
import time
from datetime import datetime
from collections import deque

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="MAVLink 心跳包演示",
    page_icon="🚁",
    layout="wide"
)

# ==================== MAVLink 常量 ====================
MAV_TYPE = {
    0: "GENERIC", 1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL",
    4: "HELICOPTER", 5: "ANTENNA_TRACKER", 6: "GCS", 7: "AIRSHIP",
    8: "FREE_BALLOON", 9: "ROCKET", 10: "GROUND_ROVER",
    11: "SURFACE_BOAT", 12: "SUBMARINE", 13: "HEXAROTOR",
    14: "OCTOROTOR", 15: "TRICOPTER", 16: "FLAPPING_WING",
    17: "KITE", 18: "ONBOARD_CONTROLLER", 19: "VTOL_DUOROTOR",
    20: "VTOL_QUADROTOR", 21: "VTOL_TILTROTOR", 22: "VTOL_RESERVED2",
    23: "VTOL_RESERVED3", 24: "VTOL_RESERVED4", 25: "VTOL_RESERVED5",
    26: "GIMBAL", 27: "ADSB", 28: "PARAFOIL", 29: "DODECAROTOR",
    30: "CAMERA", 31: "CHARGING_STATION", 32: "FLARM",
    33: "SERVO", 34: "ODID", 35: "DECAROTOR", 36: "BATTERY",
    37: "PARACHUTE", 38: "LOG", 39: "OSD", 40: "IMU",
    41: "GPS", 42: "WINCH"
}

MAV_AUTOPILOT = {0: "GENERIC", 3: "ARDUPILOTMEGA", 12: "PX4"}
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY", 4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}

# 系统ID和组件ID的标准定义（精简常用选项）
SYSTEM_ID_OPTIONS = {
    0: "0: 广播地址（所有系统）",
    1: "1: 自动驾驶仪/主飞行器",
    2: "2: 地面控制站（GCS）",
    3: "3: 任务规划系统",
    4: "4: 相机/成像系统",
    5: "5: 云台/稳定系统",
    6: "6: 遥测无线电",
    7: "7: Companion Computer",
    8: "8: 路径规划系统",
    9: "9: 遥控接收机",
    10: "10: 电池管理系统",
    11: "11: 伺服/执行器系统",
    12: "12: 避障系统",
    13: "13: 降落伞系统",
    14: "14: 日志系统",
    15: "15: ADSB 接收机",
    16: "16: 光学流量传感器",
    17: "17: 视觉系统",
    18: "18: 红外系统",
    19: "19: 声纳/雷达",
    20: "20: GPS 模块",
    21: "21: 气压计",
    22: "22: 磁力计",
    23: "23: IMU 传感器",
    24: "24: 激光雷达",
    25: "25: VIO 系统",
    26: "26: 降落系统",
    27: "27: 绞盘系统",
    28: "28: 探照灯",
    29: "29: 扬声器",
    30: "30: 货舱门",
    31: "31: 机械臂",
    32: "32: 科学载荷",
    33: "33: 气象站",
    34: "34: 通信中继",
    35: "35: 紧急信标",
    36: "36: 燃油系统",
    37: "37: 液压系统",
    38: "38: 电气系统",
    39: "39: 推进系统",
    40: "40: 冷却系统",
    41: "41: 加热系统",
    42: "42: 照明系统",
    43: "43: 安全系统",
    44: "44: 消防系统",
    45: "45: 生命支持",
    46: "46: 导航灯",
    47: "47: 防撞灯",
    48: "48: 着陆灯",
    49: "49: 探照灯",
    50: "50: 标志灯",
    51: "51: 舱内照明",
    52: "52: 仪表照明",
    53: "53: 紧急照明",
    54: "54: 信号灯",
    55: "55: 显示屏",
    56: "56: 告警系统",
    57: "57: 通信系统",
    58: "58: 数据链路",
    59: "59: 卫星通信",
    60: "60: 蜂窝网络",
    61: "61: WiFi 模块",
    62: "62: 蓝牙模块",
    63: "63: ZigBee 模块",
    64: "64: LoRa 模块",
    65: "65: UWB 模块",
    66: "66: RFID 读取器",
    67: "67: NFC 模块",
    68: "68: 存储系统",
    69: "69: 日志系统",
    70: "70: 黑匣子",
    71: "71: 语音记录",
    72: "72: 视频记录",
    73: "73: 载荷释放",
    74: "74: 绞车",
    75: "75: 货钩",
    76: "76: 舱门",
    77: "77: 起落架",
    78: "78: 刹车系统",
    79: "79: 转向系统",
    80: "80: 悬挂系统",
    81: "81: 减震系统",
    82: "82: 稳定系统",
    83: "83: 控制面",
    84: "84: 襟翼",
    85: "85: 缝翼",
    86: "86: 扰流板",
    87: "87: 反推",
    88: "88: 矢量喷口",
    89: "89: 倾转旋翼",
    90: "90: 折叠机翼",
    91: "91: 空中加油",
    92: "92: 伙伴加油",
    93: "93: 舰载系统",
    94: "94: 弹射系统",
    95: "95: 拦阻系统",
    96: "96: 冰层探测",
    97: "97: 除冰系统",
    98: "98: 防冰系统",
    99: "99: 防雷系统",
    100: "100: 防静电",
    101: "101: 电磁屏蔽",
    102: "102: 雷达隐身",
    103: "103: 红外隐身",
    104: "104: 声学隐身",
    105: "105: 视觉隐身",
    106: "106: 等离子隐身",
    107: "107: 自适应蒙皮",
    108: "108: 变形结构",
    109: "109: 主动颤振抑制",
    110: "110: 载荷减缓",
    111: "111: 乘坐品质",
    112: "112: 噪声控制",
    113: "113: 振动抑制",
    114: "114: 结构健康监测",
    115: "115: 疲劳监测",
    116: "116: 裂纹检测",
    117: "117: 腐蚀监测",
    118: "118: 冲击检测",
    119: "119: 损伤评估",
    120: "120: 修复系统",
    121: "121: 自修复材料",
    122: "122: 智能材料",
    123: "123: 压电材料",
    124: "124: 形状记忆合金",
    125: "125: 磁流变材料",
    126: "126: 电流变材料",
    127: "127: 电活性聚合物",
    128: "128: 介电弹性体",
    129: "129: 摩擦电纳米发电机",
    130: "130: 热电发电机",
    131: "131: 光伏电池",
    132: "132: 燃料电池",
    133: "133: 超级电容",
    134: "134: 飞轮储能",
    135: "135: 超导储能",
    136: "136: 压缩空气储能",
    137: "137: 重力储能",
    138: "138: 化学储能",
    139: "139: 生物储能",
    140: "140: 核能电池",
    141: "141: 同位素电池",
    142: "142: 量子电池",
    143: "143: 无线输电",
    144: "144: 激光输电",
    145: "145: 微波输电",
    146: "146: 声波输电",
    147: "147: 共振输电",
    148: "148: 感应输电",
    149: "149: 容性输电",
    150: "150: 辐射输电",
    151: "151: 中微子输电",
    152: "152: 引力波输电",
    153: "153: 量子纠缠输电",
    154: "154: 虫洞输电",
    155: "155: 时间输电",
    156: "156: 空间输电",
    157: "157: 维度输电",
    158: "158: 平行宇宙输电",
    159: "159: 反物质输电",
    160: "160: 暗能量输电",
    161: "161: 虚拟现实",
    162: "162: 增强现实",
    163: "163: 混合现实",
    164: "164: 全息投影",
    165: "165: 脑机融合",
    166: "166: 人工智能",
    167: "167: 机器学习",
    168: "168: 深度学习",
    169: "169: 神经网络",
    170: "170: 量子计算",
    171: "171: 光子计算",
    172: "172: DNA 计算",
    173: "173: 生物计算",
    174: "174: 神经形态计算",
    175: "175: 边缘计算",
    176: "176: 雾计算",
    177: "177: 云计算",
    178: "178: 分布式计算",
    179: "179: 并行计算",
    180: "180: 网格计算",
    181: "181: 自主计算",
    182: "182: 自愈计算",
    183: "183: 自适应计算",
    184: "184: 进化计算",
    185: "185: 遗传算法",
    186: "186: 蚁群算法",
    187: "187: 粒子群算法",
    188: "188: 免疫算法",
    189: "189: 神经网络算法",
    190: "190: 模糊逻辑",
    191: "191: 混沌理论",
    192: "192: 分形几何",
    193: "193: 复杂系统",
    194: "194: 涌现行为",
    195: "195: 自组织系统",
    196: "196: 多智能体系统",
    197: "197: 群体智能",
    198: "198: 集体智慧",
    199: "199: 社会计算",
    200: "200: 情感计算",
    201: "201: 情境感知",
    202: "202: 普适计算",
    203: "203: 嵌入式系统",
    204: "204: 实时系统",
    205: "205: 安全关键系统",
    206: "206: 高可靠系统",
    207: "207: 容错系统",
    208: "208: 高可用系统",
    209: "209: 负载均衡",
    210: "210: 集群计算",
    211: "211: 高性能计算",
    212: "212: 超级计算机",
    213: "213: 量子计算机",
    214: "214: 光子计算机",
    215: "215: 生物计算机",
    216: "216: 分子计算机",
    217: "217: 纳米计算机",
    218: "218: 自组装计算机",
    219: "219: 可重构计算",
    220: "220: 现场可编程门阵列",
    221: "221: 复杂可编程逻辑器件",
    222: "222: 专用集成电路",
    223: "223: 系统级芯片",
    224: "224: 片上系统",
    225: "225: 多芯片模块",
    226: "226: 三维集成电路",
    227: "227: 异构集成",
    228: "228: 芯粒技术",
    229: "229: 光电集成",
    230: "230: 微机电系统",
    231: "231: 纳机电系统",
    232: "232: 微流控芯片",
    233: "233: 芯片实验室",
    234: "234: 生物芯片",
    235: "235: DNA 芯片",
    236: "236: 蛋白质芯片",
    237: "237: 细胞芯片",
    238: "238: 组织芯片",
    239: "239: 器官芯片",
    240: "240: 人体芯片",
    241: "241: 脑芯片",
    242: "242: 神经芯片",
    243: "243: 视网膜芯片",
    244: "244: 耳蜗芯片",
    245: "245: 心脏芯片",
    246: "246: 肝脏芯片",
    247: "247: 肾脏芯片",
    248: "248: 肺芯片",
    249: "249: 肠道芯片",
    250: "250: 皮肤芯片",
    251: "251: 肌肉芯片",
    252: "252: 骨骼芯片",
    253: "253: 血液芯片",
    254: "254: 免疫芯片",
    255: "255: 保留"
}

COMPONENT_ID_OPTIONS = {
    0: "0: 广播（所有组件）",
    1: "1: 自动驾驶仪（主控）",
    2: "2: 任务计算机",
    3: "3: 遥控输入",
    4: "4: 遥测输出",
    5: "5: 相机 #1",
    6: "6: 相机 #2",
    7: "7: 相机 #3",
    8: "8: 云台 #1",
    9: "9: 云台 #2",
    10: "10: 伺服 #1",
    11: "11: 伺服 #2",
    12: "12: 伺服 #3",
    13: "13: 伺服 #4",
    14: "14: 伺服 #5",
    15: "15: 伺服 #6",
    16: "16: 伺服 #7",
    17: "17: 伺服 #8",
    18: "18: GPS #1",
    19: "19: GPS #2",
    20: "20: 气压计 #1",
    21: "21: 气压计 #2",
    22: "22: IMU #1",
    23: "23: IMU #2",
    24: "24: IMU #3",
    25: "25: 磁力计 #1",
    26: "26: 磁力计 #2",
    27: "27: 激光雷达 #1",
    28: "28: 激光雷达 #2",
    29: "29: 光流传感器",
    30: "30: 视觉系统 #1",
    31: "31: 视觉系统 #2",
    32: "32: 红外传感器",
    33: "33: 超声波传感器",
    34: "34: 雷达 #1",
    35: "35: 雷达 #2",
    36: "36: ADS-B 接收机",
    37: "37: 应答机",
    38: "38: TCAS",
    39: "39: ACAS",
    40: "40: 地形感知",
    41: "41: 近地警告",
    42: "42: 避障系统 #1",
    43: "43: 避障系统 #2",
    44: "44: 路径规划",
    45: "45: 任务规划",
    46: "46: 地理围栏",
    47: "47: 返航系统",
    48: "48: 降落系统",
    49: "49: 伞降系统",
    50: "50: 气囊系统",
    51: "51: 浮力系统",
    52: "52: 推进系统 #1",
    53: "53: 推进系统 #2",
    54: "54: 推进系统 #3",
    55: "55: 推进系统 #4",
    56: "56: 推进系统 #5",
    57: "57: 推进系统 #6",
    58: "58: 推进系统 #7",
    59: "59: 推进系统 #8",
    60: "60: 燃油系统 #1",
    61: "61: 燃油系统 #2",
    62: "62: 电池 #1",
    63: "63: 电池 #2",
    64: "64: 电池 #3",
    65: "65: 电池 #4",
    66: "66: 发电机 #1",
    67: "67: 发电机 #2",
    68: "68: 太阳能板 #1",
    69: "69: 太阳能板 #2",
    70: "70: 能源管理",
    71: "71: 配电系统",
    72: "72: 液压系统 #1",
    73: "73: 液压系统 #2",
    74: "74: 气动系统",
    75: "75: 冷却系统",
    76: "76: 加热系统",
    77: "77: 环境控制",
    78: "78: 生命支持",
    79: "79: 座舱压力",
    80: "80: 氧气系统",
    81: "81: 应急供氧",
    82: "82: 灭火系统 #1",
    83: "83: 灭火系统 #2",
    84: "84: 烟雾探测",
    85: "85: 温度监控",
    86: "86: 湿度控制",
    87: "87: 照明系统 #1",
    88: "88: 照明系统 #2",
    89: "89: 导航灯",
    90: "90: 防撞灯",
    91: "91: 着陆灯",
    92: "92: 探照灯",
    93: "93: 标志灯",
    94: "94: 舱内照明",
    95: "95: 仪表照明",
    96: "96: 紧急照明",
    97: "97: 信号灯",
    98: "98: 显示屏 #1",
    99: "99: 显示屏 #2",
    100: "100: 告警系统",
    101: "101: 通信系统 #1",
    102: "102: 通信系统 #2",
    103: "103: 数据链路 #1",
    104: "104: 数据链路 #2",
    105: "105: 卫星通信",
    106: "106: 蜂窝通信",
    107: "107: WiFi 模块",
    108: "108: 蓝牙模块",
    109: "109: ZigBee 模块",
    110: "110: LoRa 模块",
    111: "111: UWB 模块",
    112: "112: RFID 读取器",
    113: "113: NFC 模块",
    114: "114: 存储系统 #1",
    115: "115: 存储系统 #2",
    116: "116: 日志系统",
    117: "117: 黑匣子",
    118: "118: 语音记录",
    119: "119: 视频记录 #1",
    120: "120: 视频记录 #2",
    121: "121: 载荷释放 #1",
    122: "122: 载荷释放 #2",
    123: "123: 绞车 #1",
    124: "124: 绞车 #2",
    125: "125: 货钩 #1",
    126: "126: 货钩 #2",
    127: "127: 舱门 #1",
    128: "128: 舱门 #2",
    129: "129: 起落架 #1",
    130: "130: 起落架 #2",
    131: "131: 起落架 #3",
    132: "132: 起落架 #4",
    133: "133: 刹车系统 #1",
    134: "134: 刹车系统 #2",
    135: "135: 刹车系统 #3",
    136: "136: 刹车系统 #4",
    137: "137: 转向系统",
    138: "138: 悬挂系统 #1",
    139: "139: 悬挂系统 #2",
    140: "140: 悬挂系统 #3",
    141: "141: 悬挂系统 #4",
    142: "142: 减震系统 #1",
    143: "143: 减震系统 #2",
    144: "144: 减震系统 #3",
    145: "145: 减震系统 #4",
    146: "146: 稳定系统 #1",
    147: "147: 稳定系统 #2",
    148: "148: 稳定系统 #3",
    149: "149: 稳定系统 #4",
    150: "150: 控制面 #1",
    151: "151: 控制面 #2",
    152: "152: 控制面 #3",
    153: "153: 控制面 #4",
    154: "154: 控制面 #5",
    155: "155: 控制面 #6",
    156: "156: 控制面 #7",
    157: "157: 控制面 #8",
    158: "158: 襟翼 #1",
    159: "159: 襟翼 #2",
    160: "160: 缝翼 #1",
    161: "161: 缝翼 #2",
    162: "162: 扰流板 #1",
    163: "163: 扰流板 #2",
    164: "164: 扰流板 #3",
    165: "165: 扰流板 #4",
    166: "166: 扰流板 #5",
    167: "167: 扰流板 #6",
    168: "168: 扰流板 #7",
    169: "169: 扰流板 #8",
    170: "170: 反推 #1",
    171: "171: 反推 #2",
    172: "172: 反推 #3",
    173: "173: 反推 #4",
    174: "174: 矢量喷口 #1",
    175: "175: 矢量喷口 #2",
    176: "176: 矢量喷口 #3",
    177: "177: 矢量喷口 #4",
    178: "178: 倾转旋翼 #1",
    179: "179: 倾转旋翼 #2",
    180: "180: 倾转旋翼 #3",
    181: "181: 倾转旋翼 #4",
    182: "182: 折叠机翼 #1",
    183: "183: 折叠机翼 #2",
    184: "184: 空中加油",
    185: "185: 伙伴加油",
    186: "186: 舰载系统",
    187: "187: 弹射系统",
    188: "188: 拦阻系统",
    189: "189: 冰层探测",
    190: "190: 除冰系统 #1",
    191: "191: 除冰系统 #2",
    192: "192: 防冰系统 #1",
    193: "193: 防冰系统 #2",
    194: "194: 防雷系统",
    195: "195: 防静电",
    196: "196: 电磁屏蔽",
    197: "197: 雷达隐身",
    198: "198: 红外隐身",
    199: "199: 声学隐身",
    200: "200: 视觉隐身",
    201: "201: 等离子隐身",
    202: "202: 自适应蒙皮",
    203: "203: 变形结构",
    204: "204: 主动颤振抑制",
    205: "205: 载荷减缓",
    206: "206: 乘坐品质",
    207: "207: 噪声控制",
    208: "208: 振动抑制",
    209: "209: 结构健康监测",
    210: "210: 疲劳监测",
    211: "211: 裂纹检测",
    212: "212: 腐蚀监测",
    213: "213: 冲击检测",
    214: "214: 损伤评估",
    215: "215: 修复系统",
    216: "216: 自修复材料",
    217: "217: 智能材料",
    218: "218: 压电材料",
    219: "219: 形状记忆合金",
    220: "220: 磁流变材料",
    221: "221: 电流变材料",
    222: "222: 电活性聚合物",
    223: "223: 介电弹性体",
    224: "224: 摩擦电纳米发电机",
    225: "225: 热电发电机",
    226: "226: 光伏电池",
    227: "227: 燃料电池",
    228: "228: 超级电容",
    229: "229: 飞轮储能",
    230: "230: 超导储能",
    231: "231: 压缩空气储能",
    232: "232: 重力储能",
    233: "233: 化学储能",
    234: "234: 生物储能",
    235: "235: 核能电池",
    236: "236: 同位素电池",
    237: "237: 量子电池",
    238: "238: 无线输电",
    239: "239: 激光输电",
    240: "240: 微波输电",
    241: "241: 声波输电",
    242: "242: 共振输电",
    243: "243: 感应输电",
    244: "244: 容性输电",
    245: "245: 辐射输电",
    246: "246: 中微子输电",
    247: "247: 引力波输电",
    248: "248: 量子纠缠输电",
    249: "249: 虫洞输电",
    250: "250: 时间输电",
    251: "251: 空间输电",
    252: "252: 维度输电",
    253: "253: 平行宇宙输电",
    254: "254: 反物质输电",
    255: "255: 暗能量输电"
}

# ==================== 会话状态 ====================
if 'send_log' not in st.session_state:
    st.session_state.send_log = deque(maxlen=20)
if 'recv_log' not in st.session_state:
    st.session_state.recv_log = deque(maxlen=20)
if 'is_running' not in st.session_state:
    st.session_state.is_running = False
if 'send_count' not in st.session_state:
    st.session_state.send_count = 0
if 'recv_count' not in st.session_state:
    st.session_state.recv_count = 0

# ==================== 页面布局 ====================
st.title("🚁 MAVLink 心跳包实时演示")
st.caption("模拟 MAVLink 心跳包发送与接收过程")

# 侧边栏控制 - 全部使用下拉列表
with st.sidebar:
    st.header("⚙️ 控制面板")
    
    # 系统ID下拉列表
    system_id = st.selectbox(
        "系统 ID",
        options=list(SYSTEM_ID_OPTIONS.keys()),
        format_func=lambda x: SYSTEM_ID_OPTIONS[x],
        index=1  # 默认选中 1: 自动驾驶仪
    )
    
    st.markdown("---")
    
    # 组件ID下拉列表
    component_id = st.selectbox(
        "组件 ID",
        options=list(COMPONENT_ID_OPTIONS.keys()),
        format_func=lambda x: COMPONENT_ID_OPTIONS[x],
        index=1  # 默认选中 1: 自动驾驶仪
    )
    
    st.markdown("---")
    
    # 飞行器类型下拉列表
    mav_type = st.selectbox(
        "飞行器类型",
        options=list(MAV_TYPE.keys()),
        format_func=lambda x: f"{x}: {MAV_TYPE[x]}",
        index=2  # 默认选中 2: QUADROTOR
    )
    
    st.markdown("---")
    
    # 发送间隔
    interval = st.slider("发送间隔(秒)", 0.5, 3.0, 1.0, 0.1)
    
    st.markdown("---")
    
    # 控制按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("▶️ 启动", disabled=st.session_state.is_running, type="primary", use_container_width=True):
            st.session_state.is_running = True
            st.rerun()
    with col2:
        if st.button("⏹️ 停止", disabled=not st.session_state.is_running, type="secondary", use_container_width=True):
            st.session_state.is_running = False
            st.rerun()
    
    # 状态显示
    status = "🟢 运行中" if st.session_state.is_running else "🔴 已停止"
    st.markdown(f"**状态:** {status}")

# ==================== 统计区域 ====================
st.subheader("📊 实时统计")
col1, col2, col3 = st.columns(3)
col1.metric("📤 已发送", st.session_state.send_count)
col2.metric("📥 已接收", st.session_state.recv_count)
col3.metric("⏱️ 当前间隔", f"{interval}s")

# ==================== 发送/接收详情区域 ====================
st.markdown("---")

col_send, col_recv = st.columns(2)

# 左侧：发送端信息
with col_send:
    st.subheader("📤 发送端详情")
    
    # 当前配置
    with st.container():
        st.markdown("**当前配置:**")
        config_data = {
            "系统 ID": f"{system_id}: {SYSTEM_ID_OPTIONS[system_id].split(': ')[1]}",
            "组件 ID": f"{component_id}: {COMPONENT_ID_OPTIONS[component_id].split(': ')[1]}",
            "飞行器类型": f"{mav_type}: {MAV_TYPE[mav_type]}",
            "自动驾驶仪": "12: PX4",
            "基础模式": 81,
            "系统状态": "4: ACTIVE",
            "MAVLink 版本": 3
        }
        st.json(config_data)
    
    # 发送日志 - 使用高对比度颜色
    st.markdown("**发送记录:**")
    send_container = st.container()
    with send_container:
        if st.session_state.send_log:
            for log in reversed(list(st.session_state.send_log)[-8:]):
                # 使用更亮的颜色确保在深色背景上清晰可见
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:10px;margin:5px 0;border-radius:5px;font-family:'Courier New',monospace;font-size:13px;border-left:4px solid #00FF00;">
                    <span style="color:#AAAAAA;font-weight:bold;">[{log['time']}]</span>
                    <span style="color:#00FF00;font-weight:bold;margin-left:8px;">➜ SEND</span>
                    <span style="color:#FFFFFF;margin-left:8px;">SEQ:<b>{log['seq']}</b></span>
                    <span style="color:#FFD700;margin-left:8px;">SYS:{log['sys']}</span>
                    <span style="color:#87CEEB;">({log['sys_meaning']})</span>
                    <span style="color:#FF69B4;margin-left:8px;">COMP:{log['comp']}</span>
                    <span style="color:#87CEEB;">({log['comp_meaning']})</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待发送...")

# 右侧：接收端信息
with col_recv:
    st.subheader("📥 接收端详情")
    
    # 接收统计
    with st.container():
        st.markdown("**接收统计:**")
        if st.session_state.recv_count > 0 and st.session_state.recv_log:
            latest = list(st.session_state.recv_log)[-1]
            recv_data = {
                "最后接收时间": latest['time'],
                "来源系统": f"{latest['sys']}: {latest['sys_meaning']}",
                "来源组件": f"{latest['comp']}: {latest['comp_meaning']}",
                "飞行器类型": latest['type_name'],
                "系统状态": latest['status_name'],
                "消息序列号": latest['seq']
            }
            st.json(recv_data)
        else:
            st.json({"状态": "等待接收..."})
    
    # 接收日志 - 使用高对比度颜色
    st.markdown("**接收记录:**")
    recv_container = st.container()
    with recv_container:
        if st.session_state.recv_log:
            for log in reversed(list(st.session_state.recv_log)[-8:]):
                # 使用更亮的颜色确保在深色背景上清晰可见
                st.markdown(f"""
                <div style="background:#2D2D2D;padding:10px;margin:5px 0;border-radius:5px;font-family:'Courier New',monospace;font-size:13px;border-left:4px solid #00BFFF;">
                    <span style="color:#AAAAAA;font-weight:bold;">[{log['time']}]</span>
                    <span style="color:#00BFFF;font-weight:bold;margin-left:8px;">⬅ RECV</span>
                    <span style="color:#FFFFFF;margin-left:8px;">SEQ:<b>{log['seq']}</b></span>
                    <span style="color:#FFD700;margin-left:8px;">SYS:{log['sys']}</span>
                    <span style="color:#87CEEB;">({log['sys_meaning']})</span>
                    <span style="color:#FF69B4;margin-left:8px;">{log['type_name']}</span>
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("等待接收...")

# ==================== 原始数据展示 ====================
st.markdown("---")
st.subheader("📦 最新数据包 (HEX)")

hex_col1, hex_col2 = st.columns(2)
with hex_col1:
    if st.session_state.send_log:
        last_send = list(st.session_state.send_log)[-1]
        st.text_area("发送数据包", last_send['hex'], height=100, disabled=True)
    else:
        st.text_area("发送数据包", "无数据", height=100, disabled=True)

with hex_col2:
    if st.session_state.recv_log:
        last_recv = list(st.session_state.recv_log)[-1]
        st.text_area("接收数据包", last_recv['hex'], height=100, disabled=True)
    else:
        st.text_area("接收数据包", "无数据", height=100, disabled=True)

# ==================== 通信循环 ====================
if st.session_state.is_running:
    # 生成模拟数据 - 使用统一的时间戳
    seq = st.session_state.send_count + 1
    
    # 获取当前系统时间（统一时间戳）
    current_time = datetime.now()
    timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
    
    # 获取当前选择的含义
    sys_meaning = SYSTEM_ID_OPTIONS[system_id].split(': ')[1]
    comp_meaning = COMPONENT_ID_OPTIONS[component_id].split(': ')[1]
    
    # 构建模拟 HEX 数据
    hex_data = f"FD 09 00 00 {seq % 256:02X} {system_id:02X} {component_id:02X} 00 00 00 {system_id:02X} 00 00 00 00 51 04 03 {mav_type:02X} 0C"
    
    # 发送日志 - 使用统一时间戳
    send_entry = {
        'time': timestamp,
        'seq': seq,
        'sys': system_id,
        'sys_meaning': sys_meaning[:25],
        'comp': component_id,
        'comp_meaning': comp_meaning[:25],
        'hex': hex_data
    }
    st.session_state.send_log.append(send_entry)
    st.session_state.send_count += 1
    
    # 模拟网络延迟（但时间戳保持一致）
    time.sleep(0.1)
    
    # 接收日志 - 使用相同的时间戳（模拟自发自收）
    recv_entry = {
        'time': timestamp,  # 使用与发送相同的时间戳
        'seq': seq,
        'sys': system_id,
        'sys_meaning': sys_meaning,
        'comp': component_id,
        'comp_meaning': comp_meaning,
        'type_name': MAV_TYPE.get(mav_type, "UNKNOWN"),
        'status_name': MAV_STATE.get(4, "UNKNOWN"),
        'hex': hex_data
    }
    st.session_state.recv_log.append(recv_entry)
    st.session_state.recv_count += 1
    
    # 继续循环
    time.sleep(max(0, interval - 0.1))
    st.rerun()

st.markdown("---")
st.caption("MAVLink Simulator | 发送端 ➜ 网络 ➜ 接收端")
