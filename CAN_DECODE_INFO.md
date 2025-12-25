# IMU CAN 數據格式編碼/解碼指南

## 概述
此文件說明 IMU_TEST6.py 中 5 個 CAN 頻道的數據格式、編碼方式和解碼方式。

---

## 📡 CAN 頻道定義

| 頻道 ID | 十進位 | 傳感器 | 數據類型 | 大小 |
|---------|--------|--------|---------|------|
| `0x425` | 1061 | LSM6DSOX | 加速度 (X, Y, Z) | 6 bytes |
| `0x426` | 1062 | LSM303AGR | 加速度 (X, Y, Z) | 6 bytes |
| `0x427` | 1063 | LSM6DSOX | 陀螺儀/角速度 (X, Y, Z) | 6 bytes |
| `0x428` | 1064 | 融合數據 | 歐拉角 (Roll, Pitch, Yaw) | 6 bytes |
| `0x429` | 1065 | LIS2MDL | 磁力計 (X, Y, Z) | 6 bytes |

---

## 🔄 編碼格式詳解

### 1️⃣ 通道 0x425 - LSM6DSOX 加速度計

**原始數據格式：** 
- 單位: m/s²
- 範圍: float32 (浮點數)

**編碼方式：**
```python
# 編碼 (float → int16)
scale_factor = 1000          # 1 LSB = 1 mg = 0.001 m/s²
ax_int = int(ax_float * 1000)
ay_int = int(ay_float * 1000)
az_int = int(az_float * 1000)

# 結構: 小端序 (Little-Endian) 3個16位有符號整數
# 位元組排列: [ax_low, ax_high, ay_low, ay_high, az_low, az_high]
struct.pack('<hhh', ax_int, ay_int, az_int)
```

**解碼方式：**
```python
# 解碼 (int16 → float)
ax_int, ay_int, az_int = struct.unpack('<hhh', data_bytes)
ax_float = ax_int / 1000      # 結果單位: m/s²
ay_float = ay_int / 1000
az_float = az_int / 1000
```

**值域範圍：**
- int16 範圍: -32768 ~ 32767
- 對應加速度: -32.768 ~ 32.767 m/s²

---

### 2️⃣ 通道 0x426 - LSM303AGR 加速度計

**與 0x425 相同的編碼格式**

```python
# 編碼方式相同
scale_factor = 1000
struct.pack('<hhh', ax_int, ay_int, az_int)

# 解碼方式相同
ax_int, ay_int, az_int = struct.unpack('<hhh', data_bytes)
ax_float = ax_int / 1000  # m/s²
```

**用途：** 雙加速度計融合（提高準確性）

---

### 3️⃣ 通道 0x427 - 陀螺儀角速度

**原始數據格式：**
- 單位: rad/s (弧度/秒)
- 範圍: float32

**編碼方式：**
```python
# 編碼 (rad/s → int16)
# 先轉換單位: rad/s → deg/s → 0.1 deg/s
scale_factor = 10 * 57.2958  # ≈ 572.958
gx_int = int(gx_rad * scale_factor)
gy_int = int(gy_rad * scale_factor)
gz_int = int(gz_rad * scale_factor)

# 1 LSB = 0.1 deg/s = 0.001745 rad/s
struct.pack('<hhh', gx_int, gy_int, gz_int)
```

**解碼方式：**
```python
# 解碼 (int16 → rad/s)
gx_int, gy_int, gz_int = struct.unpack('<hhh', data_bytes)

# 方法1: 轉換為 deg/s
gx_deg = gx_int * 0.1        # deg/s
gy_deg = gy_int * 0.1
gz_deg = gz_int * 0.1

# 方法2: 轉換為 rad/s
gx_rad = gx_int / 572.958     # rad/s
gy_rad = gy_int / 572.958
gz_rad = gz_int / 572.958
```

**值域範圍：**
- int16 範圍: -32768 ~ 32767
- 對應角速度: -3276.8 ~ 3276.8 deg/s (-57.2 ~ 57.2 rad/s)

---

### 4️⃣ 通道 0x428 - 歐拉角 (融合傳感器)

**原始數據格式：**
- 單位: 度數 (degrees)
- 排列: Roll, Pitch, Yaw
- 計算方式: 使用加速度計 + 磁力計融合算法

**編碼方式：**
```python
# 編碼 (float → int16)
# Roll, Pitch: 範圍 -180 ~ 180 度
# Yaw: 範圍 0 ~ 360 度
scale_factor = 100           # 1 LSB = 0.01 度

roll_int = int(roll_deg * 100)
pitch_int = int(pitch_deg * 100)
yaw_int = int(yaw_deg * 100)

struct.pack('<hhh', roll_int, pitch_int, yaw_int)
```

**解碼方式：**
```python
# 解碼 (int16 → float)
roll_int, pitch_int, yaw_int = struct.unpack('<hhh', data_bytes)

roll_deg = roll_int / 100      # 度數
pitch_deg = pitch_int / 100
yaw_deg = yaw_int / 100
```

**計算公式：**
```
Roll  = atan2(ay, az) × (180/π)
Pitch = atan2(-ax, √(ay²+az²)) × (180/π)
Yaw   = atan2(-mag_y_tilt, mag_x_tilt) × (180/π)  // 帶傾斜補償
```

**值域範圍：**
- int16 範圍: -32768 ~ 32767
- 對應角度: -327.68 ~ 327.67 度

---

### 5️⃣ 通道 0x429 - 磁力計 (LIS2MDL)

**原始數據格式：**
- 單位: μT (微特斯拉)
- 範圍: float32

**編碼方式：**
```python
# 編碼 (float → int16)
# 典型地球磁場: 25-65 μT
scale_factor = 10            # 1 LSB = 0.1 μT

mx_int = int(mx_ut * 10)
my_int = int(my_ut * 10)
mz_int = int(mz_ut * 10)

struct.pack('<hhh', mx_int, my_int, mz_int)
```

**解碼方式：**
```python
# 解碼 (int16 → float)
mx_int, my_int, mz_int = struct.unpack('<hhh', data_bytes)

mx_ut = mx_int / 10          # μT (微特斯拉)
my_ut = my_int / 10
mz_ut = mz_int / 10
```

**值域範圍：**
- int16 範圍: -32768 ~ 32767
- 對應磁場: -3276.8 ~ 3276.8 μT

---

## 📝 通用解碼 Python 代碼

### 完整解碼函數

```python
import struct

def decode_can_message(can_id, data_bytes):
    """
    通用 CAN 訊息解碼函數
    
    Args:
        can_id: CAN 仲裁 ID (0x425-0x429)
        data_bytes: 6 字節的數據 (bytes)
    
    Returns:
        tuple: (x, y, z, unit_str)
    """
    
    if len(data_bytes) < 6:
        raise ValueError(f"期望 6 字節，但收到 {len(data_bytes)} 字節")
    
    # 解包為 3 個 int16
    x_int, y_int, z_int = struct.unpack('<hhh', data_bytes[:6])
    
    if can_id == 0x425:  # LSM6DSOX 加速度
        return x_int / 1000, y_int / 1000, z_int / 1000, "m/s²"
    
    elif can_id == 0x426:  # LSM303 加速度
        return x_int / 1000, y_int / 1000, z_int / 1000, "m/s²"
    
    elif can_id == 0x427:  # 陀螺儀角速度
        # 轉換為 deg/s
        x_deg = x_int * 0.1
        y_deg = y_int * 0.1
        z_deg = z_int * 0.1
        return x_deg, y_deg, z_deg, "deg/s"
    
    elif can_id == 0x428:  # 歐拉角
        return x_int / 100, y_int / 100, z_int / 100, "degree (Roll, Pitch, Yaw)"
    
    elif can_id == 0x429:  # 磁力計
        return x_int / 10, y_int / 10, z_int / 10, "μT"
    
    else:
        raise ValueError(f"未知的 CAN ID: 0x{can_id:03X}")

# 使用範例
can_data_map = {
    0x425: "LSM6DSOX Accel",
    0x426: "LSM303 Accel",
    0x427: "Gyro (Angular Vel)",
    0x428: "Euler Angles",
    0x429: "Magnetometer"
}

# 假設從 CAN 接收到數據
received_can_id = 0x425
received_data = b'\x10\x27\xF0\xD8\x20\x03'  # 範例數據

x, y, z, unit = decode_can_message(received_can_id, received_data)
sensor_name = can_data_map.get(received_can_id, "Unknown")

print(f"{sensor_name}: X={x:.3f} {unit}, Y={y:.3f} {unit}, Z={z:.3f} {unit}")
```

### 編碼函數

```python
def encode_can_message(can_id, x, y, z):
    """
    通用 CAN 訊息編碼函數
    
    Args:
        can_id: CAN 仲裁 ID (0x425-0x429)
        x, y, z: 原始數據值
    
    Returns:
        bytes: 6 字節的編碼數據
    """
    
    if can_id in (0x425, 0x426):  # 加速度
        scale = 1000
        x_int = int(x * scale)
        y_int = int(y * scale)
        z_int = int(z * scale)
    
    elif can_id == 0x427:  # 陀螺儀 (輸入為 deg/s)
        scale = 10  # 0.1 deg/s per LSB
        x_int = int(x * scale)
        y_int = int(y * scale)
        z_int = int(z * scale)
    
    elif can_id == 0x428:  # 歐拉角
        scale = 100
        x_int = int(x * scale)
        y_int = int(y * scale)
        z_int = int(z * scale)
    
    elif can_id == 0x429:  # 磁力計
        scale = 10
        x_int = int(x * scale)
        y_int = int(y * scale)
        z_int = int(z * scale)
    
    else:
        raise ValueError(f"未知的 CAN ID: 0x{can_id:03X}")
    
    # 限制在 int16 範圍內
    x_int = max(-32768, min(32767, x_int))
    y_int = max(-32768, min(32767, y_int))
    z_int = max(-32768, min(32767, z_int))
    
    return struct.pack('<hhh', x_int, y_int, z_int)

# 使用範例
accel_x, accel_y, accel_z = 9.81, 0.0, 0.0  # m/s²
encoded = encode_can_message(0x425, accel_x, accel_y, accel_z)
print(f"編碼後的字節: {encoded.hex()}")

# 解碼驗證
x, y, z, unit = decode_can_message(0x425, encoded)
print(f"解碼結果: X={x:.3f} {unit}, Y={y:.3f} {unit}, Z={z:.3f} {unit}")
```

---

## 📊 數據精度與分辨率

| CAN ID | 傳感器 | 解析度 | 範圍 | 精度損失 |
|--------|--------|--------|------|---------|
| 0x425 | LSM6DSOX Accel | 0.001 m/s² (1 mg) | ±32.768 m/s² | 0.1% |
| 0x426 | LSM303 Accel | 0.001 m/s² (1 mg) | ±32.768 m/s² | 0.1% |
| 0x427 | Gyro | 0.1 deg/s | ±3276.8 deg/s | 0.01° |
| 0x428 | Euler | 0.01° | ±327.68° | 良好 |
| 0x429 | Mag | 0.1 μT | ±3276.8 μT | < 0.5% |

---

## 🔍 CSV 日誌格式

CSV 檔案中每一行的格式：
```
Timestamp(μs), CAN_ID(HEX), Extended(bool), Direction, Bus, DLC, D1, D2, D3, D4, D5, D6, D7, D8, ...
```

**範例：**
```
1731709200000000, 00000425, false, Tx, 0, 6, 10, 27, F0, D8, 20, 03, 00, 00
```

其中前 6 個字節 (D1-D6) 對應編碼的 X、Y、Z 數據。

---

## 💡 實際應用範例

### 從 CSV 中解碼

```python
import csv
import struct

def decode_imu_csv(csv_filename):
    """讀取並解碼 IMU CSV 日誌"""
    
    can_map = {
        0x425: "LSM6DSOX Accel",
        0x426: "LSM303 Accel", 
        0x427: "Gyro",
        0x428: "Euler",
        0x429: "Mag"
    }
    
    with open(csv_filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            can_id = int(row['ID'], 16)
            
            # 提取 6 個數據字節
            data_bytes = bytes([
                int(row[f'D{i}'], 16) for i in range(1, 7)
            ])
            
            # 解碼
            x_int, y_int, z_int = struct.unpack('<hhh', data_bytes)
            
            if can_id == 0x425 or can_id == 0x426:
                x, y, z = x_int/1000, y_int/1000, z_int/1000
                unit = "m/s²"
            elif can_id == 0x427:
                x, y, z = x_int*0.1, y_int*0.1, z_int*0.1
                unit = "deg/s"
            elif can_id == 0x428:
                x, y, z = x_int/100, y_int/100, z_int/100
                unit = "deg"
            else:  # 0x429
                x, y, z = x_int/10, y_int/10, z_int/10
                unit = "μT"
            
            print(f"{can_map[can_id]}: X={x:.3f} {unit}, Y={y:.3f} {unit}, Z={z:.3f} {unit}")

# 使用
decode_imu_csv('/home/pi/Desktop/IMU_LOGS/imu_can_log_20251115_184533.csv')
```

---

## 📚 參考資料

- **LSM6DSOX**：6軸 IMU (加速度計+陀螺儀)
- **LSM303AGR**：3軸加速度計
- **LIS2MDL**：3軸磁力計
- **小端序 (Little-Endian)**：低位字節在前

