import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import random

# Global buffers
timestamps = []
data = []

# Plot setup
fig, ax = plt.subplots()
line, = ax.plot([], [])
ax.set_title("Live Random Data")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.grid(True)

def update(frame):
    t = time.time()
    val = random.uniform(-1, 1)
    timestamps.append(t)
    data.append(val)
    timestamps[:] = timestamps[-100:]
    data[:] = data[-100:]
    line.set_data(timestamps, data)
    ax.relim(); ax.autoscale_view()
    return [line]

# ✅ Keep animation object alive globally
anim = FuncAnimation(
    fig,
    update,
    interval=100,
    cache_frame_data=False,
    save_count=100
)

plt.tight_layout()
plt.show()  # ✅ Must be last line
