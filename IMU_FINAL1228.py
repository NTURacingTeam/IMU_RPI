import time
import board
import busio
import struct
import math
import can
from datetime import datetime

# LSM6DSOX (IMU)
import adafruit_lsm6ds.lsm6dsox

# LSM303AGR (Accel + Mag)
import adafruit_lsm303_accel
import adafruit_lis2mdl

lsm6_can_label = 0x185
lsm303_can_label = 0x426
gyro_can_label = 0x285
euler_can_label = 0x385
mag_can_label = 0x429

def init_can_bus():
    """Initialize CAN bus interface"""
    try:
        # Configure CAN bus - adjust interface name as needed
        # For SocketCAN on Linux: interface='can0'
        # For PCAN: interface='PCAN_USBBUS1'
        # For Vector: interface='vector'
        bus = can.Bus("can0", interface="socketcan")

        print("CAN bus initialized successfully")
        return bus
    except Exception as e:
        print(f"Failed to initialize CAN bus: {e}")
        return None

def float_to_int16_scaled(value, scale_factor):
    """Convert float to 16-bit signed integer with scaling"""
    scaled_value = int(value * scale_factor)
    # Clamp to 16-bit signed range
    scaled_value = max(-32768, min(32767, scaled_value))
    return scaled_value

def pack_acceleration_data(accel_data):
    """Pack acceleration data into CAN message format"""
    # Scale factor: 0.001 g (1 LSB = 0.001g)
    scale_factor = 1000/9.81  # Convert mm/s2 to mg
    
    # Swap X and Y axes, and invert new X axis
    # test = -accel_data[0]
    # accel_data[0] = accel_data[1]
    # accel_data[1] = test


    ax_int = float_to_int16_scaled(accel_data[0], scale_factor)
    ay_int = float_to_int16_scaled(accel_data[1], scale_factor)
    az_int = float_to_int16_scaled(accel_data[2], scale_factor)
    
    # Pack as 3 x 16-bit signed integers (6 bytes total)
    return struct.pack('<hhh', ax_int, ay_int, az_int)

def pack_angular_velocity_data(gyro_data):
    """Pack angular velocity data into CAN message format"""
    # Scale factor: 0.1 DPS (1 LSB = 0.1 degrees per second)
    # Convert rad/s to deg/s first, then scale
    scale_factor = 10 * 57.2958  # Convert rad/s to 0.1 deg/s

    # Swap X and Y axes, and invert new X axis
    # testg = -gyro_data[0]
    # gyro_data[0] = gyro_data[1]
    # gyro_data[1] = testg

    gx_int = float_to_int16_scaled(gyro_data[0], scale_factor)
    gy_int = float_to_int16_scaled(gyro_data[1], scale_factor)
    gz_int = float_to_int16_scaled(gyro_data[2], scale_factor)
    
    # Pack as 3 x 16-bit signed integers (6 bytes total)
    return struct.pack('<hhh', gx_int, gy_int, gz_int)

def pack_euler_angle_data(angles):
    """Pack Euler angle data into CAN message format"""
    # Scale factor: 0.01 degrees (1 LSB = 0.01 degrees)
    scale_factor = 100
    
    roll_int = float_to_int16_scaled(angles[0], scale_factor)
    pitch_int = float_to_int16_scaled(angles[1], scale_factor)
    yaw_int = float_to_int16_scaled(angles[2], scale_factor)
    
    # Pack as 3 x 16-bit signed integers (6 bytes total)
    return struct.pack('<hhh', roll_int, pitch_int, yaw_int)

def pack_magnetometer_data(mag_data):
    """Pack magnetometer data into CAN message format"""
    # Scale factor: assume 1 LSB = 0.1 uT
    scale_factor = 10
    
    mx_int = float_to_int16_scaled(mag_data[0], scale_factor)
    my_int = float_to_int16_scaled(mag_data[1], scale_factor)
    mz_int = float_to_int16_scaled(mag_data[2], scale_factor)
    
    # Pack as 3 x 16-bit signed integers (6 bytes total)
    return struct.pack('<hhh', mx_int, my_int, mz_int)

def calculate_euler_angles(accel_data, mag_data):
    """Calculate Euler angles from accelerometer and magnetometer data"""
    ax, ay, az = accel_data
    mx, my, mz = mag_data
    
    # Calculate Roll and Pitch from accelerometer
    roll = math.atan2(ay, az) * 57.2958
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958
    
    # Calculate Yaw from magnetometer (simplified tilt compensation)
    # This is a basic implementation - for better accuracy, full tilt compensation should be used
    cos_roll = math.cos(roll * math.pi / 180)
    sin_roll = math.sin(roll * math.pi / 180)
    cos_pitch = math.cos(pitch * math.pi / 180)
    sin_pitch = math.sin(pitch * math.pi / 180)
    
    # Tilt compensated magnetic field components
    mag_x = mx * cos_pitch + mz * sin_pitch
    mag_y = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch
    
    yaw = math.atan2(-mag_y, mag_x) * 57.2958
    if yaw < 0:
        yaw += 360
    
    return roll, pitch, yaw

def send_can_messages(bus, accel_lsm6, accel_lsm303, gyro_data, mag_data, angles):
    """Send all IMU data via CAN bus"""
    try:
        # Send LSM6DSOX Accelerometer data
        accel_lsm6_data = pack_acceleration_data(accel_lsm6)
        msg1 = can.Message(arbitration_id=lsm6_can_label, data=accel_lsm6_data, is_extended_id=False)
        bus.send(msg1)
        
        # Send LSM303AGR Accelerometer data
        accel_lsm303_data = pack_acceleration_data(accel_lsm303)
        msg2 = can.Message(arbitration_id=lsm303_can_label, data=accel_lsm303_data, is_extended_id=False)
        bus.send(msg2)
        
        # Send Angular Velocity data
        gyro_data_packed = pack_angular_velocity_data(gyro_data)
        msg3 = can.Message(arbitration_id=gyro_can_label, data=gyro_data_packed, is_extended_id=False)
        bus.send(msg3)
        
        # Send Euler Angles data
        angle_data = pack_euler_angle_data(angles)
        msg4 = can.Message(arbitration_id=euler_can_label, data=angle_data, is_extended_id=False)
        bus.send(msg4)
        
        # Send Magnetometer data
        mag_data_packed = pack_magnetometer_data(mag_data)
        msg5 = can.Message(arbitration_id=mag_can_label, data=mag_data_packed, is_extended_id=False)
        bus.send(msg5)
        
        return True
        
    except Exception as e:
        print(f"Failed to send CAN messages: {e}")
        return False

def print_can_data_info(accel_lsm6, accel_lsm303, gyro_data, mag_data, angles):
    """Print information about the data being sent"""
    print(f"\nCAN Message Data (100Hz):")
    print("-" * 50)
    print(f"0x185 - LSM6DSOX Accel: X={accel_lsm6[0]:6.3f}, Y={accel_lsm6[1]:6.3f}, Z={accel_lsm6[2]:6.3f} m/s2")
    print(f"0x426 - LSM303 Accel:   X={accel_lsm303[0]:6.3f}, Y={accel_lsm303[1]:6.3f}, Z={accel_lsm303[2]:6.3f} m/s2")
    print(f"0x285 - Angular Vel:    X={gyro_data[0]*57.3:6.1f}, Y={gyro_data[1]*57.3:6.1f}, Z={gyro_data[2]*57.3:6.1f} deg/s")
    print(f"0x385 - Euler Angles:   Roll={angles[0]:6.1f}, Pitch={angles[1]:6.1f}, Yaw={angles[2]:6.1f} deg")
    print(f"0x429 - Magnetometer:   X={mag_data[0]:6.1f}, Y={mag_data[1]:6.1f}, Z={mag_data[2]:6.1f} uT")

import csv
import os

def listen_for_commands(bus, timeout=0.001):
    """Listen for CAN commands on 0x420"""
    try:
        message = bus.recv(timeout=timeout)
        if message and message.arbitration_id == 0x420:
            if len(message.data) >= 2:
                command = message.data[1]
                return command
    except can.CanTimeoutError:
        pass
    except Exception as e:
        print(f"Command listening error: {e}")
    return None

# CSV 相關函數已移除

def main():
    # Initialize I2C
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialize sensors
    print("Initializing sensors...")
    try:
        imu = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c, address=0x6A)
        accel303 = adafruit_lsm303_accel.LSM303_Accel(i2c)
        mag303 = adafruit_lis2mdl.LIS2MDL(i2c)
        print("✓ All sensors initialized successfully")
    except Exception as e:
        print(f"✗ Sensor initialization failed: {e}")
        return
    
    # Initialize CAN bus
    can_bus = init_can_bus()
    if not can_bus:
        print("✗ CAN bus initialization failed, exiting...")
        return
    
    print("\n" + "="*60)
    print("IMU CAN Gateway Started - No Commands Needed")
    print("="*60)
    print("CAN Data Channels:")
    print("  • 0x185 (LSM6 Accel)")
    print("  • 0x426 (LSM303 Accel)")
    print("  • 0x285 (Gyro/Angular Vel)")
    print("  • 0x385 (Euler Angles)")
    print("  • 0x429 (Magnetometer)")
    print("Frequency: 100 Hz")
    print("CSV Logging: Disabled")
    print("="*60 + "\n")
    
    message_count = 0
    start_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            try:
                # Read sensor data
                accel_lsm6 = imu.acceleration
                gyro_lsm6 = imu.gyro
                accel_lsm303 = accel303.acceleration
                mag_lsm303 = mag303.magnetic
                
                # Calculate Euler angles
                euler_angles = calculate_euler_angles(accel_lsm6, mag_lsm303)
                
                # Pack data
                accelrefresh1 = (accel_lsm6[1], -accel_lsm6[0], accel_lsm6[2])
                gyro_lsm6refresh = (gyro_lsm6[1], -gyro_lsm6[0], gyro_lsm6[2])
                accelrefresh2 = (accel_lsm303[1], -accel_lsm303[0], accel_lsm303[2])

                accel_lsm6_data = pack_acceleration_data(accelrefresh1)
                accel_lsm303_data = pack_acceleration_data(accelrefresh2)
                gyro_data_packed = pack_angular_velocity_data(gyro_lsm6refresh)
                angle_data = pack_euler_angle_data(euler_angles)
                mag_data_packed = pack_magnetometer_data(mag_lsm303)
                
                # Send all CAN messages immediately
                try:
                    msg1 = can.Message(arbitration_id=lsm6_can_label, data=accel_lsm6_data, is_extended_id=False)
                    can_bus.send(msg1)

                    msg2 = can.Message(arbitration_id=lsm303_can_label, data=accel_lsm303_data, is_extended_id=False)
                    can_bus.send(msg2)

                    msg3 = can.Message(arbitration_id=gyro_can_label, data=gyro_data_packed, is_extended_id=False)
                    can_bus.send(msg3)

                    msg4 = can.Message(arbitration_id=euler_can_label, data=angle_data, is_extended_id=False)
                    can_bus.send(msg4)

                    msg5 = can.Message(arbitration_id=mag_can_label, data=mag_data_packed, is_extended_id=False)
                    can_bus.send(msg5)
                    
                    message_count += 5
                    
                except Exception as e:
                    print(f"✗ Failed to send CAN messages: {e}")
                
                # Print status every second
                if message_count % 500 == 0 and message_count > 0:  # Every 100 loops (1 second)
                    elapsed_time = time.time() - start_time
                    can_freq = message_count / elapsed_time if elapsed_time > 0 else 0
                    
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Status:")
                    print(f"  CAN Messages: {message_count}, Frequency: {can_freq:.1f} Hz")
                    print_can_data_info(accel_lsm6, accel_lsm303, gyro_lsm6, mag_lsm303, euler_angles)
                
            except Exception as e:
                print(f"✗ Data processing error: {e}")
            
            # Maintain 100Hz frequency
            loop_time = time.time() - loop_start
            sleep_time = max(0, 0.01 - loop_time)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print(f"Program terminated by user")
        print(f"Total CAN messages sent: {message_count}")
        print(f"{'='*60}")
        
    finally:
        print("Shutting down...")


if __name__ == "__main__":
    main()