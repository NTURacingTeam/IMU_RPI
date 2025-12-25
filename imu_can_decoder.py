"""
IMU CAN 解碼工具
================
用於解碼和編碼 IMU_TEST6.py 中 5 個 CAN 頻道的數據

使用方式：
    from imu_can_decoder import CANDecoder
    
    decoder = CANDecoder()
    x, y, z, unit = decoder.decode(0x425, data_bytes)
    data_bytes = decoder.encode(0x425, x, y, z)
"""

import struct
from enum import IntEnum

class CANID(IntEnum):
    """CAN 頻道定義"""
    LSM6_ACCEL = 0x425      # LSM6DSOX 加速度計
    LSM303_ACCEL = 0x426    # LSM303AGR 加速度計
    GYRO = 0x427            # 陀螺儀 (角速度)
    EULER = 0x428           # 歐拉角
    MAG = 0x429             # 磁力計

class CanDecoder:
    """CAN 訊息解碼器"""
    
    # 傳感器信息
    SENSOR_INFO = {
        CANID.LSM6_ACCEL: {
            'name': 'LSM6DSOX Accelerometer',
            'unit': 'm/s²',
            'scale': 1000,
            'description': '6軸 IMU 加速度計'
        },
        CANID.LSM303_ACCEL: {
            'name': 'LSM303AGR Accelerometer',
            'unit': 'm/s²',
            'scale': 1000,
            'description': '3軸加速度計'
        },
        CANID.GYRO: {
            'name': 'Gyroscope (Angular Velocity)',
            'unit': 'deg/s',
            'scale': 0.1,  # 原始單位: rad/s → deg/s
            'description': '角速度傳感器'
        },
        CANID.EULER: {
            'name': 'Euler Angles',
            'unit': 'degree',
            'scale': 100,
            'components': ['Roll', 'Pitch', 'Yaw'],
            'description': '歐拉角 (融合傳感器)'
        },
        CANID.MAG: {
            'name': 'Magnetometer',
            'unit': 'μT',
            'scale': 10,
            'description': '磁力計'
        }
    }
    
    @classmethod
    def validate_can_id(cls, can_id):
        """驗證 CAN ID 是否有效"""
        valid_ids = [CANID.LSM6_ACCEL, CANID.LSM303_ACCEL, 
                     CANID.GYRO, CANID.EULER, CANID.MAG]
        if can_id not in valid_ids:
            raise ValueError(f"無效的 CAN ID: 0x{can_id:03X}. "
                           f"必須是 {[f'0x{x:03X}' for x in valid_ids]} 之一")
    
    @classmethod
    def decode(cls, can_id, data_bytes):
        """
        解碼 CAN 訊息
        
        Args:
            can_id: CAN 仲裁 ID (0x425-0x429)
            data_bytes: 6 字節的數據 (bytes)
        
        Returns:
            tuple: (x, y, z, unit)
        
        Raises:
            ValueError: 如果 CAN ID 無效或數據長度不正確
        """
        cls.validate_can_id(can_id)
        
        if len(data_bytes) < 6:
            raise ValueError(f"期望至少 6 字節，但收到 {len(data_bytes)} 字節")
        
        # 解包為 3 個 int16 (小端序)
        x_int, y_int, z_int = struct.unpack('<hhh', data_bytes[:6])
        
        if can_id == CANID.LSM6_ACCEL:  # LSM6DSOX 加速度
            return x_int / 1000, y_int / 1000, z_int / 1000, "m/s²"
        
        elif can_id == CANID.LSM303_ACCEL:  # LSM303 加速度
            return x_int / 1000, y_int / 1000, z_int / 1000, "m/s²"
        
        elif can_id == CANID.GYRO:  # 陀螺儀 (轉換為 deg/s)
            return x_int * 0.1, y_int * 0.1, z_int * 0.1, "deg/s"
        
        elif can_id == CANID.EULER:  # 歐拉角
            return x_int / 100, y_int / 100, z_int / 100, "degree (Roll, Pitch, Yaw)"
        
        elif can_id == CANID.MAG:  # 磁力計
            return x_int / 10, y_int / 10, z_int / 10, "μT"
    
    @classmethod
    def decode_with_info(cls, can_id, data_bytes):
        """
        解碼 CAN 訊息並返回詳細信息
        
        Returns:
            dict: {
                'can_id': 0x425,
                'sensor': 'LSM6DSOX Accelerometer',
                'values': {'x': 9.81, 'y': 0.0, 'z': 0.0},
                'unit': 'm/s²',
                'components': ['X', 'Y', 'Z']
            }
        """
        cls.validate_can_id(can_id)
        
        x, y, z, unit = cls.decode(can_id, data_bytes)
        sensor_info = cls.SENSOR_INFO[can_id]
        
        components = sensor_info.get('components', ['X', 'Y', 'Z'])
        
        return {
            'can_id': f'0x{can_id:03X}',
            'sensor': sensor_info['name'],
            'description': sensor_info['description'],
            'values': {components[0]: x, components[1]: y, components[2]: z},
            'unit': unit
        }
    
    @classmethod
    def encode(cls, can_id, x, y, z):
        """
        編碼 CAN 訊息
        
        Args:
            can_id: CAN 仲裁 ID (0x425-0x429)
            x, y, z: 原始數據值
        
        Returns:
            bytes: 6 字節的編碼數據
        
        Raises:
            ValueError: 如果 CAN ID 無效或數據超出範圍
        """
        cls.validate_can_id(can_id)
        
        if can_id in (CANID.LSM6_ACCEL, CANID.LSM303_ACCEL):  # 加速度
            scale = 1000
        elif can_id == CANID.GYRO:  # 陀螺儀 (輸入為 deg/s)
            scale = 10  # 0.1 deg/s per LSB
        elif can_id == CANID.EULER:  # 歐拉角
            scale = 100
        elif can_id == CANID.MAG:  # 磁力計
            scale = 10
        
        x_int = int(x * scale)
        y_int = int(y * scale)
        z_int = int(z * scale)
        
        # 限制在 int16 範圍內
        x_int = max(-32768, min(32767, x_int))
        y_int = max(-32768, min(32767, y_int))
        z_int = max(-32768, min(32767, z_int))
        
        return struct.pack('<hhh', x_int, y_int, z_int)
    
    @classmethod
    def get_sensor_info(cls, can_id):
        """獲取傳感器信息"""
        cls.validate_can_id(can_id)
        info = cls.SENSOR_INFO[can_id].copy()
        info['can_id'] = f'0x{can_id:03X}'
        return info
    
    @classmethod
    def get_all_sensors(cls):
        """獲取所有傳感器列表"""
        return {
            f'0x{cid:03X}': info['name'] 
            for cid, info in cls.SENSOR_INFO.items()
        }
    
    @classmethod
    def print_decode(cls, can_id, data_bytes):
        """打印解碼結果"""
        info = cls.decode_with_info(can_id, data_bytes)
        print(f"\n{'='*60}")
        print(f"CAN ID: {info['can_id']}")
        print(f"傳感器: {info['sensor']}")
        print(f"說明: {info['description']}")
        print(f"單位: {info['unit']}")
        print("-" * 60)
        for component, value in info['values'].items():
            print(f"  {component}: {value:.4f} {info['unit']}")
        print(f"{'='*60}\n")


class CSVDecoder:
    """CSV 日誌解碼器"""
    
    @staticmethod
    def decode_csv_row(row):
        """
        解碼 CSV 的一行
        
        Args:
            row: CSV DictReader 的一行
        
        Returns:
            dict: 解碼後的數據
        """
        can_id = int(row['ID'], 16)
        dlc = int(row['LEN'])
        
        # 提取數據字節
        data_bytes = bytes([
            int(row.get(f'D{i}', '0'), 16) for i in range(1, dlc + 1)
        ])
        
        # 解碼
        x, y, z, unit = CanDecoder.decode(can_id, data_bytes)
        
        return {
            'timestamp': int(row['Time Stamp']),
            'can_id': f'0x{can_id:03X}',
            'sensor': CanDecoder.SENSOR_INFO[can_id]['name'],
            'x': x,
            'y': y,
            'z': z,
            'unit': unit,
            'raw_data': data_bytes.hex().upper()
        }
    
    @staticmethod
    def decode_csv_file(csv_filename, filter_can_id=None, max_rows=None):
        """
        解碼整個 CSV 檔案
        
        Args:
            csv_filename: CSV 檔案路徑
            filter_can_id: 只解碼特定的 CAN ID (可選)
            max_rows: 最多解碼的行數 (可選)
        
        Yields:
            dict: 解碼後的每一行數據
        """
        import csv
        
        row_count = 0
        with open(csv_filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if max_rows and row_count >= max_rows:
                    break
                
                can_id = int(row['ID'], 16)
                
                if filter_can_id is None or can_id == filter_can_id:
                    try:
                        decoded = CSVDecoder.decode_csv_row(row)
                        yield decoded
                        row_count += 1
                    except Exception as e:
                        print(f"解碼錯誤: {e}")


if __name__ == "__main__":
    # 使用範例
    print("=" * 60)
    print("IMU CAN 解碼工具使用範例")
    print("=" * 60)
    
    # 範例 1: 解碼加速度數據
    print("\n[例1] 解碼 LSM6DSOX 加速度")
    accel_raw = b'\x10\x27\xF0\xD8\x20\x03'  # 範例字節
    x, y, z, unit = CanDecoder.decode(0x425, accel_raw)
    print(f"X={x:.3f} {unit}, Y={y:.3f} {unit}, Z={z:.3f} {unit}")
    
    # 範例 2: 詳細信息
    print("\n[例2] 獲取詳細信息")
    info = CanDecoder.decode_with_info(0x425, accel_raw)
    print(f"傳感器: {info['sensor']}")
    print(f"值: {info['values']}")
    
    # 範例 3: 編碼與解碼
    print("\n[例3] 編碼與解碼")
    original = (9.81, 0.0, 0.0)
    encoded = CanDecoder.encode(0x425, *original)
    print(f"編碼後: {encoded.hex().upper()}")
    decoded = CanDecoder.decode(0x425, encoded)
    print(f"解碼後: X={decoded[0]:.3f}, Y={decoded[1]:.3f}, Z={decoded[2]:.3f}")
    
    # 範例 4: 所有傳感器列表
    print("\n[例4] 所有可用傳感器")
    for can_id, sensor_name in CanDecoder.get_all_sensors().items():
        print(f"  {can_id}: {sensor_name}")
    
    # 範例 5: 打印解碼結果
    print("\n[例5] 格式化輸出")
    CanDecoder.print_decode(0x425, accel_raw)
