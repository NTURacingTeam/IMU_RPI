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

def init_can_bus():
    """Initialize CAN bus interface"""
    try:
        # Configure CAN bus - adjust interface name as needed
        # For SocketCAN on Linux: interface='can0'
        # For PCAN: interface='PCAN_USBBUS1'
        # For Vector: interface='vector'
        bus = can.interface.Bus(interface='can0', bustype='socketcan')
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
    scale_factor = 1000  # Convert mm/s2 to mg
    
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
        # Send LSM6DSOX Accelerometer data (0x180)
        accel_lsm6_data = pack_acceleration_data(accel_lsm6)
        msg1 = can.Message(arbitration_id=0x180, data=accel_lsm6_data, is_extended_id=False)
        bus.send(msg1)
        
        # Send LSM303AGR Accelerometer data (0x181)
        accel_lsm303_data = pack_acceleration_data(accel_lsm303)
        msg2 = can.Message(arbitration_id=0x181, data=accel_lsm303_data, is_extended_id=False)
        bus.send(msg2)
        
        # Send Angular Velocity data (0x280)
        gyro_data_packed = pack_angular_velocity_data(gyro_data)
        msg3 = can.Message(arbitration_id=0x280, data=gyro_data_packed, is_extended_id=False)
        bus.send(msg3)
        
        # Send Euler Angles data (0x380)
        angle_data = pack_euler_angle_data(angles)
        msg4 = can.Message(arbitration_id=0x380, data=angle_data, is_extended_id=False)
        bus.send(msg4)
        
        # Send Magnetometer data (0x430)
        mag_data_packed = pack_magnetometer_data(mag_data)
        msg5 = can.Message(arbitration_id=0x430, data=mag_data_packed, is_extended_id=False)
        bus.send(msg5)
        
        return True
        
    except Exception as e:
        print(f"Failed to send CAN messages: {e}")
        return False

def print_can_data_info(accel_lsm6, accel_lsm303, gyro_data, mag_data, angles):
    """Print information about the data being sent"""
    print(f"\nCAN Message Data (100Hz):")
    print("-" * 50)
    print(f"0x180 - LSM6DSOX Accel: X={accel_lsm6[0]:6.3f}, Y={accel_lsm6[1]:6.3f}, Z={accel_lsm6[2]:6.3f} m/s2")
    print(f"0x181 - LSM303 Accel:   X={accel_lsm303[0]:6.3f}, Y={accel_lsm303[1]:6.3f}, Z={accel_lsm303[2]:6.3f} m/s2")
    print(f"0x280 - Angular Vel:    X={gyro_data[0]*57.3:6.1f}, Y={gyro_data[1]*57.3:6.1f}, Z={gyro_data[2]*57.3:6.1f} deg/s")
    print(f"0x380 - Euler Angles:   Roll={angles[0]:6.1f}, Pitch={angles[1]:6.1f}, Yaw={angles[2]:6.1f} deg")
    print(f"0x430 - Magnetometer:   X={mag_data[0]:6.1f}, Y={mag_data[1]:6.1f}, Z={mag_data[2]:6.1f} uT")

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

def new_csv_writer(base_dir, base_name):
    """Create new CSV file with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(base_dir, f"{base_name}_{timestamp}.csv")
    f = open(filename, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(["Time Stamp", "ID", "Extended", "Dir", "Bus", "LEN", "D1",
                     "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", "D10",
                     "D11", "D12"])
    return f, writer

def write_can_message_to_csv(writer, can_id, data_bytes):
    """Write CAN message to CSV in the same format as canlogging.py"""
    timestamp = int(time.time() * 1000000)  # microseconds
    can_id_str = f"{can_id:08X}"
    extended = 'false'  # Standard CAN IDs
    direction = 'Tx'  # We're transmitting
    bus_num = 0
    dlc = len(data_bytes)
    
    # Format data bytes as hex strings
    data_hex = [f"{byte:02X}" for byte in data_bytes]
    data_hex += ['00'] * (8 - len(data_hex))  # Pad to 8 bytes
    
    writer.writerow([timestamp, can_id_str, extended, direction, bus_num, dlc] + data_hex)

def main():
    # Initialize I2C
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Initialize sensors
    print("Initializing sensors...")
    try:
        imu = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c, address=0x6B)
        accel303 = adafruit_lsm303_accel.LSM303_Accel(i2c)
        mag303 = adafruit_lis2mdl.LIS2MDL(i2c)
        print("All sensors initialized successfully")
    except Exception as e:
        print(f"Sensor initialization failed: {e}")
        return
    
    # Initialize CAN bus
    can_bus = init_can_bus()
    if not can_bus:
        print("CAN bus initialization failed, exiting...")
        return
    
    # Setup CSV logging
    base_dir = "/home/pi/Desktop/IMU_LOGS"
    os.makedirs(base_dir, exist_ok=True)
    csv_file, csv_writer = new_csv_writer(base_dir, "imu_can_log")
    
    print("IMU CAN Gateway with CSV Logging Ready")
    print("Command Protocol:")
    print("  - Send to 0x420: [XX, 0x01] to START CAN transmission")
    print("  - Send to 0x420: [XX, 0x02] to STOP CAN transmission")
    print("Data IDs: 0x180 (LSM6 Accel), 0x181 (LSM303 Accel), 0x280 (Gyro), 0x380 (Angles), 0x430 (Mag)")
    print("CSV Logging: Always active, saving all IMU data in CAN format")
    print("Frequency: 100 Hz")
    print("Listening for commands on 0x420...")
    
    is_transmitting = False
    message_count = 0
    csv_count = 0
    start_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            # Listen for commands
            command = listen_for_commands(can_bus)
            if command is not None:
                if command == 0x01 and not is_transmitting:
                    is_transmitting = True
                    message_count = 0
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] COMMAND RECEIVED: START CAN transmission (0x01)")
                elif command == 0x02 and is_transmitting:
                    is_transmitting = False
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] COMMAND RECEIVED: STOP CAN transmission (0x02)")
                    print(f"Total CAN messages sent in this session: {message_count}")
            
            # Always read sensor data and write to CSV
            try:
                # Read sensor data
                accel_lsm6 = imu.acceleration
                gyro_lsm6 = imu.gyro
                accel_lsm303 = accel303.acceleration
                mag_lsm303 = mag303.magnetic
                
                # Calculate Euler angles
                euler_angles = calculate_euler_angles(accel_lsm6, mag_lsm303)
                
                # Pack data for CSV logging (always)
                accel_lsm6_data = pack_acceleration_data(accel_lsm6)
                accel_lsm303_data = pack_acceleration_data(accel_lsm303)
                gyro_data_packed = pack_angular_velocity_data(gyro_lsm6)
                angle_data = pack_euler_angle_data(euler_angles)
                mag_data_packed = pack_magnetometer_data(mag_lsm303)
                
                # Write all messages to CSV (always active)
                write_can_message_to_csv(csv_writer, 0x180, accel_lsm6_data)
                write_can_message_to_csv(csv_writer, 0x181, accel_lsm303_data)
                write_can_message_to_csv(csv_writer, 0x280, gyro_data_packed)
                write_can_message_to_csv(csv_writer, 0x380, angle_data)
                write_can_message_to_csv(csv_writer, 0x430, mag_data_packed)
                csv_count += 5
                
                # Send CAN messages only if transmission is enabled
                if is_transmitting:
                    try:
                        msg1 = can.Message(arbitration_id=0x180, data=accel_lsm6_data, is_extended_id=False)
                        can_bus.send(msg1)
                        
                        msg2 = can.Message(arbitration_id=0x181, data=accel_lsm303_data, is_extended_id=False)
                        can_bus.send(msg2)
                        
                        msg3 = can.Message(arbitration_id=0x280, data=gyro_data_packed, is_extended_id=False)
                        can_bus.send(msg3)
                        
                        msg4 = can.Message(arbitration_id=0x380, data=angle_data, is_extended_id=False)
                        can_bus.send(msg4)
                        
                        msg5 = can.Message(arbitration_id=0x430, data=mag_data_packed, is_extended_id=False)
                        can_bus.send(msg5)
                        
                        message_count += 5
                        
                    except Exception as e:
                        print(f"Failed to send CAN messages: {e}")
                
                # Print status every second
                if csv_count % 500 == 0:  # Every 100 loops (1 second)
                    elapsed_time = time.time() - start_time
                    csv_freq = csv_count / elapsed_time if elapsed_time > 0 else 0
                    can_freq = message_count / elapsed_time if elapsed_time > 0 and is_transmitting else 0
                    
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Status:")
                    print(f"  CSV Records: {csv_count}, Freq: {csv_freq:.1f} Hz")
                    if is_transmitting:
                        print(f"  CAN Messages: {message_count}, Freq: {can_freq:.1f} Hz")
                        print_can_data_info(accel_lsm6, accel_lsm303, gyro_lsm6, mag_lsm303, euler_angles)
                    else:
                        print(f"  CAN Transmission: STOPPED (data still logged to CSV)")
                
            except Exception as e:
                print(f"Data processing error: {e}")
            
            # Maintain 100Hz frequency
            loop_time = time.time() - loop_start
            sleep_time = max(0, 0.01 - loop_time)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print(f"\nProgram terminated")
        print(f"Total CSV records written: {csv_count}")
        if is_transmitting:
            print(f"Total CAN messages sent: {message_count}")
        else:
            print("CAN transmission was stopped")
        
    finally:
        # Close CSV file
        if csv_file:
            csv_file.close()
            print("CSV file closed")

if __name__ == "__main__":
    main()