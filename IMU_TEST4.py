import time
import board
import busio
import os
import math
from datetime import datetime

# LSM6DSOX (IMU)
import adafruit_lsm6ds.lsm6dsox

# LSM303AGR (Accel + Mag)
import adafruit_lsm303_accel
import adafruit_lis2mdl

def clear_screen():
    """Clear the screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_header():
    """Print header"""
    print("=" * 80)
    print("                        IMU Data Monitoring Dashboard")
    print("=" * 80)
    print(f"Update Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 80)

def print_sensor_data(title, accel_data, gyro_data=None, mag_data=None):
    """Print formatted sensor data"""
    print(f"\n{title}")
    print("-" * 40)
    
    # Accelerometer data
    if accel_data:
        print(f"Accelerometer:")
        print(f"   X-axis: {accel_data[0]:8.3f} m/s2")
        print(f"   Y-axis: {accel_data[1]:8.3f} m/s2") 
        print(f"   Z-axis: {accel_data[2]:8.3f} m/s2")
        print(f"   Magnitude: {(accel_data[0]**2 + accel_data[1]**2 + accel_data[2]**2)**0.5:8.3f} m/s2")
    
    # Gyroscope data
    if gyro_data:
        print(f"\nGyroscope:")
        print(f"   X-axis: {gyro_data[0]:8.3f} rad/s  ({gyro_data[0]*57.2958:6.1f} deg/s)")
        print(f"   Y-axis: {gyro_data[1]:8.3f} rad/s  ({gyro_data[1]*57.2958:6.1f} deg/s)")
        print(f"   Z-axis: {gyro_data[2]:8.3f} rad/s  ({gyro_data[2]*57.2958:6.1f} deg/s)")
    
    # Magnetometer data
    if mag_data:
        print(f"\nMagnetometer:")
        print(f"   X-axis: {mag_data[0]:8.1f} uT")
        print(f"   Y-axis: {mag_data[1]:8.1f} uT")
        print(f"   Z-axis: {mag_data[2]:8.1f} uT")
        print(f"   Magnitude: {(mag_data[0]**2 + mag_data[1]**2 + mag_data[2]**2)**0.5:8.1f} uT")

def calculate_tilt_angles(accel_data):
    """Calculate tilt angles"""
    ax, ay, az = accel_data
    
    # Calculate Roll and Pitch angles (in degrees)
    roll = math.atan2(ay, az) * 57.2958
    pitch = math.atan2(-ax, (ay**2 + az**2)**0.5) * 57.2958
    
    return roll, pitch

def print_calculated_values(accel_lsm6, accel_lsm303, gyro_data, mag_data):
    """Print calculated values"""
    print(f"\nCalculated Data")
    print("-" * 40)
    
    # Calculate angles using LSM6DSOX accelerometer
    roll, pitch = calculate_tilt_angles(accel_lsm6)
    print(f"Attitude Angles (from LSM6DSOX):")
    print(f"   Roll:  {roll:6.1f} deg")
    print(f"   Pitch: {pitch:6.1f} deg")
    
    # Magnetometer heading (simplified calculation)
    if mag_data:
        heading = math.atan2(mag_data[1], mag_data[0]) * 57.2958
        if heading < 0:
            heading += 360
        print(f"   Yaw:   {heading:6.1f} deg")
    
    # Difference between two accelerometers
    diff_x = abs(accel_lsm6[0] - accel_lsm303[0])
    diff_y = abs(accel_lsm6[1] - accel_lsm303[1])
    diff_z = abs(accel_lsm6[2] - accel_lsm303[2])
    
    print(f"\nDual Accelerometer Difference:")
    print(f"   Delta-X: {diff_x:6.3f} m/s2")
    print(f"   Delta-Y: {diff_y:6.3f} m/s2")
    print(f"   Delta-Z: {diff_z:6.3f} m/s2")

def print_status_indicators(accel_lsm6, gyro_data, mag_data):
    """Print status indicators"""
    print(f"\nSensor Status")
    print("-" * 40)
    
    # Accelerometer status (check if close to gravity)
    total_accel = (accel_lsm6[0]**2 + accel_lsm6[1]**2 + accel_lsm6[2]**2)**0.5
    if 9.5 < total_accel < 10.1:
        accel_status = "NORMAL"
    else:
        accel_status = "ABNORMAL"
    
    # Gyroscope status (check if stationary)
    total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
    if total_gyro < 0.1:
        gyro_status = "STATIONARY"
    else:
        gyro_status = "MOVING"
    
    # Magnetometer status (check variation range)
    total_mag = (mag_data[0]**2 + mag_data[1]**2 + mag_data[2]**2)**0.5
    if 20 < total_mag < 100:
        mag_status = "NORMAL"
    else:
        mag_status = "NEEDS_CALIBRATION"
    
    print(f"Accelerometer: {accel_status} (Magnitude: {total_accel:.2f} m/s2)")
    print(f"Gyroscope:     {gyro_status} (Total: {total_gyro:.3f} rad/s)")
    print(f"Magnetometer:  {mag_status} (Magnitude: {total_mag:.1f} uT)")

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
    
    # Configure sensors for faster sampling (optional)
    try:
        # Set higher data rates
        # imu.accelerometer_data_rate = adafruit_lsm6ds.Rate.RATE_208_HZ
        # imu.gyro_data_rate = adafruit_lsm6ds.Rate.RATE_208_HZ
        print("Sensor configuration completed")
    except:
        print("Using default configuration")
    
    print("\nStarting monitoring... (Press Ctrl+C to exit)")
    time.sleep(2)
    
    try:
        while True:
            # Clear screen and print header
            clear_screen()
            print_header()
            
            # Read data
            try:
                accel_lsm6 = imu.acceleration
                gyro_lsm6 = imu.gyro
                accel_lsm303 = accel303.acceleration
                mag_lsm303 = mag303.magnetic
                
                # Print sensor data
                print_sensor_data("LSM6DSOX (6-axis IMU)", accel_lsm6, gyro_lsm6)
                print_sensor_data("LSM303AGR Accelerometer", accel_lsm303)
                print_sensor_data("LIS2MDL Magnetometer", None, None, mag_lsm303)
                
                # Print calculated values
                print_calculated_values(accel_lsm6, accel_lsm303, gyro_lsm6, mag_lsm303)
                
                # Print status indicators
                print_status_indicators(accel_lsm6, gyro_lsm6, mag_lsm303)
                
            except Exception as e:
                print(f"Data reading error: {e}")
            
            print("\n" + "=" * 80)
            print("Updating every 0.2 seconds | Press Ctrl+C to exit")
            
            time.sleep(0.2)  # Update every 200ms for faster display
            
    except KeyboardInterrupt:
        clear_screen()
        print("Monitoring stopped")

if __name__ == "__main__":
    main()