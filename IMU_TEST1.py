import time
import board
import busio

# LSM6DSOX (IMU)
import adafruit_lsm6ds.lsm6dsox

# LSM303AGR (Accel + Mag)
import adafruit_lsm303_accel
import adafruit_lis2mdl

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize sensors with correct addresses
imu = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c, address=0x6B)
accel303 = adafruit_lsm303_accel.LSM303_Accel(i2c)  # default 0x19
mag303 = adafruit_lis2mdl.LIS2MDL(i2c)                     # default 0x1E

# Continuous readout
while True:
    # LSM6DSOX
    accel_6ds = imu.acceleration
    gyro_6ds = imu.gyro

    # LSM303AGR
    accel_303 = accel303.acceleration
    mag_303 = mag303.magnetic

    print("LSM6DSOX Accel:", accel_6ds)
    print("LSM6DSOX Gyro :", gyro_6ds)
    print("LSM303AGR Accel:", accel_303)
    print("LSM303AGR Mag  :", mag_303)
    print("-" * 40)

    time.sleep(0.1)  # 10 Hz
