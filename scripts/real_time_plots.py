import re
import serial
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === Configuration ===
SERIAL_PORT = '/dev/ttyACM0'  # Change to your port (e.g., COM3 on Windows)
BAUD_RATE = 115200
MAX_POINTS = 200  # Max number of points to display on the plot

# === Regular Expression for Matching Sensor Lines ===
line_re = re.compile(
    r"(\d+)\s*\|\s*Roll:\s*([-\d.]+),\s*Pitch:\s*([-\d.]+),\s*Yaw:\s*([-\d.]+),\s*Raw yaw:\s*([-\d.]+)"
)

# === Data Buffers ===
timestamps = deque(maxlen=MAX_POINTS)
rolls = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)
yaws = deque(maxlen=MAX_POINTS)

# === Open Serial Port ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Let the serial port settle

def read_serial_line():
    try:
        line = ser.readline().decode('utf-8').strip()
        match = line_re.match(line)
        if match:
            ts, roll, pitch, yaw, raw_yaw = match.groups()
            print(f"Yaw: {yaw}, Raw jaw: {raw_yaw}")
            return int(ts), float(roll), float(pitch), float(yaw)
    except Exception as e:
        print(f"Read error: {e}")
    return None

# === Plotting Setup ===
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
fig.suptitle("Sensor Data from UART")

line_roll, = ax1.plot([], [], label="Roll")
line_pitch, = ax2.plot([], [], label="Pitch")
line_yaw, = ax3.plot([], [], label="Yaw")

for ax in (ax1, ax2, ax3):
    ax.legend()
    ax.grid(True)

def update_plot(frame):
    data = read_serial_line()
    if data:
        ts, roll, pitch, yaw = data
        timestamps.append(ts / 1e6)  # Convert us to seconds
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)

        line_roll.set_data(timestamps, rolls)
        line_pitch.set_data(timestamps, pitches)
        line_yaw.set_data(timestamps, yaws)

        ax1.relim()
        ax2.relim()
        ax3.relim()
        ax1.autoscale_view()
        ax2.autoscale_view()
        ax3.autoscale_view()

    return line_roll, line_pitch, line_yaw

ani = animation.FuncAnimation(fig, update_plot, interval=50)

plt.xlabel("Time (s)")
ax1.set_ylabel("Roll (rad)")
ax2.set_ylabel("Pitch (rad)")
ax3.set_ylabel("Yaw (rad)")
plt.tight_layout()
plt.show()
