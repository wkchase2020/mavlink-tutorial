# MAVLink 入门常见问题解答 (FAQ)

## 一、环境安装问题

### Q1: 安装 pymavlink 时报错，提示缺少编译器？

**问题现象:**
```
error: Microsoft Visual C++ 14.0 is required
```

**解决方案:**
1. 安装预编译版本（推荐）：
```bash
pip install pymavlink --only-binary :all:
```

2. 如果仍有问题，尝试使用 conda：
```bash
conda install -c conda-forge pymavlink
```

---

### Q2: 提示 "No module named 'pymavlink'"？

**问题现象:**
```
ModuleNotFoundError: No module named 'pymavlink'
```

**解决方案:**
1. 确认安装成功：
```bash
pip list | findstr pymavlink
```

2. 检查 Python 版本：
```bash
python --version
```
确保使用的是安装 pymavlink 的同一个 Python。

3. 如果使用虚拟环境，确保已激活：
```bash
# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

---

### Q3: pip 命令找不到？

**解决方案:**
使用 Python 模块方式运行 pip：
```bash
python -m pip install pymavlink
```

---

## 二、程序运行问题

### Q4: 接收端收不到心跳消息？

**排查步骤:**

1. **检查运行顺序**
   - 必须先运行 `receiver.py`（接收端）
   - 再运行 `sender.py`（发送端）

2. **检查防火墙设置**
   - Windows 可能会阻止 UDP 通信
   - 暂时关闭防火墙测试

3. **检查端口占用**
   ```bash
   # Windows 查看端口占用
   netstat -ano | findstr 14550
   ```

4. **检查 IP 地址**
   - 确保 `common_setup.py` 中的 IP 设置正确
   - 同一台电脑使用 `127.0.0.1`

5. **增加调试输出**
   在 `receiver.py` 中添加：
   ```python
   # 在 wait_for_heartbeat 中添加调试
   print("正在等待消息...")
   ```

---

### Q5: 程序卡在 "等待第一条心跳消息"？

**问题原因:**
- 发送端没有启动
- 网络连接有问题

**解决方案:**
1. 确认发送端已在另一个窗口运行
2. 检查两台电脑是否在同一个网络（如果使用两台电脑）
3. 暂时关闭防火墙/杀毒软件测试

---

### Q6: 按 Ctrl+C 无法停止程序？

**解决方案:**
1. 按多次 Ctrl+C
2. 或者关闭命令行窗口
3. Windows 上也可以按 Ctrl+Break

---

## 三、理解 MAVLink 消息

### Q7: 心跳消息里的字段是什么意思？

| 字段 | 含义 | 常见值 |
|------|------|--------|
| type | 飞行器类型 | 0=通用, 1=固定翼, 2=四旋翼 |
| autopilot | 自动驾驶仪 | 0=通用, 3=ArduPilot, 12=PX4 |
| base_mode | 基础模式标志 | 位标志，如 81=预解锁+已解锁 |
| custom_mode | 自定义模式 | 飞行模式编号 |
| system_status | 系统状态 | 1=初始化, 3=待机, 4=激活 |

---

### Q8: 系统 ID 和组件 ID 有什么区别？

**系统 ID (system_id):**
- 标识网络上的独立设备
- 范围: 1-255
- 例如：无人机 #1、无人机 #2、地面站

**组件 ID (component_id):**
- 标识设备内部的功能模块
- 例如：飞控、相机、GPS、Gimbal（云台）
- 同一设备的不同组件共享同一个系统 ID

**举例:**
```
无人机 A:
  - 系统 ID: 1
  - 组件: 飞控(1), 相机(100)

无人机 B:
  - 系统 ID: 2
  - 组件: 飞控(1), GPS(220)

地面站:
  - 系统 ID: 255
  - 组件: 地面站(0)
```

---

## 四、网络连接问题

### Q9: 两台电脑之间如何通信？

**步骤:**

1. 修改 `common_setup.py`：
```python
# 在接收端电脑上，使用 0.0.0.0 监听所有接口
RECEIVER_ADDRESS = "udpin:0.0.0.0:14550"

# 在发送端电脑上，使用接收端电脑的 IP
# 例如接收端 IP 是 192.168.1.100
SENDER_ADDRESS = "udpout:192.168.1.100:14550"
```

2. 确保两台电脑在同一网络
3. 关闭防火墙或开放 14550 端口

---

### Q10: udpout 和 udpin 有什么区别？

| 模式 | 含义 | 用途 |
|------|------|------|
| udpout | UDP 输出 | 发送数据到指定地址 |
| udpin | UDP 输入 | 在指定地址监听数据 |
| tcp | TCP 连接 | 可靠的连接（较少用）|
| serial | 串口 | 连接真实飞控 |

**记忆技巧:**
- `out` = 向外发送
- `in` = 向内接收

---

### Q11: 如何连接真实的飞控（如 Pixhawk）？

**USB 连接:**
```python
# Windows
connection = mavutil.mavlink_connection('COM3', baud=57600)

# Linux
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
```

**串口连接:**
```python
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
```

**注意:**
- 需要安装串口驱动
- 波特率要与飞控设置匹配（通常是 57600 或 115200）

---

## 五、编程相关问题

### Q12: 如何发送其他类型的消息？

**示例：发送 GPS 位置**
```python
# 创建 GPS 消息
# 参数说明：
#   time_usec - 时间戳（微秒）
#   fix_type - 定位类型（0-8）
#   lat - 纬度（度 * 1e7）
#   lon - 经度（度 * 1e7）
#   alt - 海拔（毫米）
#   ...其他参数

gps_msg = mavutil.mavlink.MAVLink_global_position_int_message(
    time_boot_ms=0,      # 时间戳
    lat=399999999,       # 纬度 39.9999999 度
    lon=1160000000,      # 经度 116.0000000 度
    alt=100000,          # 海拔 100 米（毫米）
    relative_alt=100000, # 相对高度 100 米
    vx=0, vy=0, vz=0,    # 速度
    hdg=0                # 航向
)

connection.mav.send(gps_msg)
```

---

### Q13: 如何接收其他类型的消息？

**示例：接收 GPS 位置**
```python
# 等待 GPS 消息
message = connection.recv_match(
    type='GLOBAL_POSITION_INT',
    blocking=True,
    timeout=5.0
)

if message:
    print(f"纬度: {message.lat / 1e7}")
    print(f"经度: {message.lon / 1e7}")
    print(f"高度: {message.alt / 1000} 米")
```

**接收所有消息：**
```python
# 接收任何类型的消息
message = connection.recv_match(blocking=True)
print(f"收到消息类型: {message.get_type()}")
```

---

### Q14: 如何设置不同的发送频率？

**方法 1：修改间隔时间**
```python
# common_setup.py
HEARTBEAT_INTERVAL = 0.5  # 每 0.5 秒发送一次（2Hz）
```

**方法 2：使用 MAVLink 消息间隔命令**
```python
# 请求以特定频率发送消息
connection.mav.command_long_send(
    target_system=1,
    target_component=1,
    command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    confirmation=0,
    param1=33,    # 消息 ID（33 = GLOBAL_POSITION_INT）
    param2=100000, # 间隔（微秒），100000 = 10Hz
    param3=0, param4=0, param5=0, param6=0, param7=0
)
```

---

## 六、调试技巧

### Q15: 如何查看所有收到的原始数据？

**开启调试模式：**
```python
import mavutil

# 设置调试级别
mavutil.set_dialect("common")
```

**打印原始字节：**
```python
# 在接收循环中添加
print(f"原始数据: {message.to_dict()}")
```

---

### Q16: 如何保存收到的消息到文件？

```python
# 打开文件
with open('mavlink_log.txt', 'w') as f:
    while True:
        message = connection.recv_match(blocking=True)
        
        # 写入文件
        log_line = f"{time.time()}: {message}\n"
        f.write(log_line)
        f.flush()  # 立即写入磁盘
```

---

## 七、扩展学习

### 推荐学习资源

1. **MAVLink 官方文档**
   - https://mavlink.io/

2. **消息定义文件**
   - 查看 `pymavlink/mavutil.py` 中的消息定义

3. **ArduPilot 文档**
   - https://ardupilot.org/

4. **QGroundControl 源码**
   - 学习完整地面站如何实现 MAVLink

---

### 下一步练习

1. **修改飞行器类型**
   - 在 `common_setup.py` 中修改 `VEHICLE_TYPE`
   - 尝试 1（固定翼）、13（六旋翼）等

2. **添加消息计数**
   - 在心跳中添加序列号
   - 接收端检查是否有丢失

3. **实现双向通信**
   - 接收端也发送心跳
   - 发送端也接收心跳

4. **添加时间戳**
   - 测量消息延迟

---

## 八、寻求帮助

如果以上问题都无法解决：

1. **检查 pymavlink 版本**
```bash
python -c "from pymavlink import mavutil; print(mavutil.__file__)"
```

2. **查看详细错误信息**
```python
import traceback
try:
    # 你的代码
    pass
except Exception as e:
    traceback.print_exc()
```

3. **搜索 GitHub Issues**
   - https://github.com/ArduPilot/pymavlink/issues

4. **ArduPilot 论坛**
   - https://discuss.ardupilot.org/
