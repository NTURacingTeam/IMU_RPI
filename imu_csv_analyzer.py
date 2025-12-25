#!/usr/bin/env python3
"""
IMU CSV 日誌分析工具
====================
用於分析和解碼 IMU_TEST6.py 生成的 CSV 日誌檔案

使用方式：
    python3 imu_csv_analyzer.py <csv_file>
    python3 imu_csv_analyzer.py <csv_file> --filter 0x425
    python3 imu_csv_analyzer.py <csv_file> --stats
"""

import csv
import sys
import argparse
from datetime import datetime
from imu_can_decoder import CanDecoder, CANID

class IMUCSVAnalyzer:
    """IMU CSV 分析器"""
    
    def __init__(self, csv_filename):
        self.csv_filename = csv_filename
        self.data = []
        self.load_csv()
    
    def load_csv(self):
        """載入 CSV 檔案"""
        with open(self.csv_filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.data.append(row)
        print(f"✓ 已載入 {len(self.data)} 條記錄")
    
    def decode_all(self, filter_can_id=None, max_rows=None):
        """解碼所有數據"""
        count = 0
        for row in self.data:
            if max_rows and count >= max_rows:
                break
            
            can_id = int(row['ID'], 16)
            
            if filter_can_id is None or can_id == filter_can_id:
                dlc = int(row['LEN'])
                data_bytes = bytes([
                    int(row.get(f'D{i}', '0'), 16) for i in range(1, dlc + 1)
                ])
                
                try:
                    x, y, z, unit = CanDecoder.decode(can_id, data_bytes)
                    timestamp = int(row['Time Stamp'])
                    
                    info = {
                        'timestamp': timestamp,
                        'can_id': f'0x{can_id:03X}',
                        'sensor': CanDecoder.SENSOR_INFO[can_id]['name'],
                        'x': x,
                        'y': y,
                        'z': z,
                        'unit': unit
                    }
                    yield info
                    count += 1
                except Exception as e:
                    print(f"⚠ 解碼錯誤 (第 {count+1} 行): {e}")
    
    def print_table(self, filter_can_id=None, max_rows=50):
        """以表格形式打印數據"""
        print(f"\n{'-'*100}")
        print(f"{'CAN ID':<10} {'傳感器':<30} {'X':<15} {'Y':<15} {'Z':<15} {'單位':<10}")
        print(f"{'-'*100}")
        
        count = 0
        for decoded in self.decode_all(filter_can_id, max_rows):
            can_id = decoded['can_id']
            sensor = decoded['sensor'][:28]
            x = f"{decoded['x']:>13.4f}"
            y = f"{decoded['y']:>13.4f}"
            z = f"{decoded['z']:>13.4f}"
            unit = decoded['unit']
            
            print(f"{can_id:<10} {sensor:<30} {x:<15} {y:<15} {z:<15} {unit:<10}")
            count += 1
        
        print(f"{'-'*100}")
        print(f"共顯示 {count} 條記錄\n")
    
    def get_statistics(self, filter_can_id=None):
        """計算統計信息"""
        stats = {}
        
        for decoded in self.decode_all(filter_can_id):
            can_id = decoded['can_id']
            
            if can_id not in stats:
                stats[can_id] = {
                    'sensor': decoded['sensor'],
                    'unit': decoded['unit'],
                    'x_values': [],
                    'y_values': [],
                    'z_values': [],
                    'count': 0
                }
            
            stats[can_id]['x_values'].append(decoded['x'])
            stats[can_id]['y_values'].append(decoded['y'])
            stats[can_id]['z_values'].append(decoded['z'])
            stats[can_id]['count'] += 1
        
        # 計算統計值
        result = {}
        for can_id, data in stats.items():
            if data['count'] > 0:
                result[can_id] = {
                    'sensor': data['sensor'],
                    'unit': data['unit'],
                    'count': data['count'],
                    'x': {
                        'min': min(data['x_values']),
                        'max': max(data['x_values']),
                        'avg': sum(data['x_values']) / len(data['x_values'])
                    },
                    'y': {
                        'min': min(data['y_values']),
                        'max': max(data['y_values']),
                        'avg': sum(data['y_values']) / len(data['y_values'])
                    },
                    'z': {
                        'min': min(data['z_values']),
                        'max': max(data['z_values']),
                        'avg': sum(data['z_values']) / len(data['z_values'])
                    }
                }
        
        return result
    
    def print_statistics(self, filter_can_id=None):
        """打印統計信息"""
        stats = self.get_statistics(filter_can_id)
        
        print(f"\n{'='*100}")
        print(f"統計信息")
        print(f"{'='*100}\n")
        
        for can_id in sorted(stats.keys()):
            data = stats[can_id]
            sensor = data['sensor']
            unit = data['unit']
            count = data['count']
            
            print(f"CAN ID: {can_id} | {sensor}")
            print(f"  記錄數: {count}")
            print(f"  X ({unit}): min={data['x']['min']:>10.4f}, max={data['x']['max']:>10.4f}, avg={data['x']['avg']:>10.4f}")
            print(f"  Y ({unit}): min={data['y']['min']:>10.4f}, max={data['y']['max']:>10.4f}, avg={data['y']['avg']:>10.4f}")
            print(f"  Z ({unit}): min={data['z']['min']:>10.4f}, max={data['z']['max']:>10.4f}, avg={data['z']['avg']:>10.4f}")
            print()
        
        print(f"{'='*100}\n")
    
    def export_decoded_csv(self, output_filename, filter_can_id=None):
        """導出解碼後的 CSV"""
        decoded_data = list(self.decode_all(filter_can_id))
        
        if not decoded_data:
            print("⚠ 沒有數據可導出")
            return
        
        with open(output_filename, 'w', newline='') as f:
            fieldnames = ['timestamp', 'can_id', 'sensor', 'x', 'y', 'z', 'unit']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(decoded_data)
        
        print(f"✓ 已導出到: {output_filename}")


def main():
    parser = argparse.ArgumentParser(
        description='IMU CSV 日誌分析工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
範例:
  python3 imu_csv_analyzer.py imu_can_log_20251115_184533.csv
  python3 imu_csv_analyzer.py imu_can_log_20251115_184533.csv --filter 0x425
  python3 imu_csv_analyzer.py imu_can_log_20251115_184533.csv --stats
  python3 imu_csv_analyzer.py imu_can_log_20251115_184533.csv --export decoded.csv
        '''
    )
    
    parser.add_argument('csv_file', help='CSV 日誌檔案路徑')
    parser.add_argument('--filter', help='只顯示特定的 CAN ID (如 0x425)', default=None)
    parser.add_argument('--rows', type=int, help='最多顯示的行數', default=50)
    parser.add_argument('--stats', action='store_true', help='顯示統計信息')
    parser.add_argument('--export', help='導出解碼後的 CSV 檔案')
    
    args = parser.parse_args()
    
    try:
        analyzer = IMUCSVAnalyzer(args.csv_file)
        
        # 解析 CAN ID
        filter_can_id = None
        if args.filter:
            try:
                filter_can_id = int(args.filter, 16)
            except ValueError:
                print(f"❌ 無效的 CAN ID: {args.filter}")
                sys.exit(1)
        
        # 執行操作
        if args.stats:
            analyzer.print_statistics(filter_can_id)
        elif args.export:
            analyzer.export_decoded_csv(args.export, filter_can_id)
        else:
            analyzer.print_table(filter_can_id, args.rows)
    
    except FileNotFoundError:
        print(f"❌ 找不到檔案: {args.csv_file}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 錯誤: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
