import time
import board
import busio
import adafruit_lsm6ds.lsm6dsox
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Sensor setup
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c, address=0x6B)

# Data buffers with fixed maxlen
maxlen = 100
timestamps = deque(maxlen=maxlen)
accel_data = [deque(maxlen=maxlen) for _ in range(3)]
gyro_data  = [deque(maxlen=maxlen) for _ in range(3)]

# Plot setup
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
lines_acc = [ax1.plot([], [], label=axis)[0] for axis in ['X','Y','Z']]
lines_gyro= [ax2.plot([], [], label=axis)[0] for axis in ['X','Y','Z']]
ax1.set_title("Acceleration (m/s²)"); ax1.legend(); ax1.grid(True)
ax2.set_title("Gyroscope (rad/s)");    ax2.legend(); ax2.grid(True)

def init():
    for ln in lines_acc + lines_gyro:
        ln.set_data([], [])
    return lines_acc + lines_gyro

def update(frame):
    t = time.time()
    acc = imu.acceleration
    gyr = imu.gyro

    timestamps.append(t)
    for i in range(3):
        accel_data[i].append(acc[i])
        gyro_data[i].append(gyr[i])

    # Update line data
    for i, ln in enumerate(lines_acc):
        ln.set_data(timestamps, accel_data[i])
    for i, ln in enumerate(lines_gyro):
        ln.set_data(timestamps, gyro_data[i])

    ax1.relim(); ax1.autoscale_view()
    ax2.relim(); ax2.autoscale_view()
    return lines_acc + lines_gyro

# Keep `ani` alive, disable frame caching, and specify save_count
ani = FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=100,
    blit=True,
    cache_frame_data=False,
    save_count=maxlen
)

plt.tight_layout()
plt.show()
