import re
import serial
from collections import deque
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# === Configuration ===
SERIAL_PORT = '/dev/ttyACM0'  # Change to your port
BAUD_RATE = 115200
MAX_POINTS = 200
ALPHA = 0.98  # complementary filter coefficient
WINDOW_SECONDS = 10  # time window for x-axis in seconds

# === Regex for parsing lines with timestamp and sensor data ===
line_re = re.compile(
    r"(\d+)\s*\|\s*(acc|gyro|mag) data: X = (-?\d+), Y = (-?\d+), Z = (-?\d+)"
)

# === Data buffers for plotting ===
timestamps = deque(maxlen=MAX_POINTS)
rolls = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)
yaws = deque(maxlen=MAX_POINTS)

# === Open serial port with short timeout for non-blocking reads ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
time.sleep(2)  # wait for serial port to settle

# === State variables ===
last_time = None
roll = 0.0
pitch = 0.0
yaw = 0.0

# Latest sensor readings
acc = {'x': 0, 'y': 0, 'z': 0}
gyro = {'x': 0, 'y': 0, 'z': 0}
mag = {'x': 0, 'y': 0, 'z': 0}

def complementary_filter(dt, acc, gyro, mag, prev_roll, prev_pitch, prev_yaw):
    # Gyro sensitivity (example MPU6050 250 dps)
    GYRO_SENSITIVITY = 131.0

    gx = gyro['x'] / GYRO_SENSITIVITY * (math.pi / 180.0)
    gy = gyro['y'] / GYRO_SENSITIVITY * (math.pi / 180.0)
    gz = gyro['z'] / GYRO_SENSITIVITY * (math.pi / 180.0)

    # Acc angles
    acc_roll = math.atan2(acc['y'], acc['z'])
    acc_pitch = math.atan2(-acc['x'], math.sqrt(acc['y']**2 + acc['z']**2))

    # Integrate gyro
    gyro_roll = prev_roll + gx * dt
    gyro_pitch = prev_pitch + gy * dt
    gyro_yaw = prev_yaw + gz * dt

    # Complementary filter for roll and pitch
    roll = ALPHA * gyro_roll + (1 - ALPHA) * acc_roll
    pitch = ALPHA * gyro_pitch + (1 - ALPHA) * acc_pitch

    # Normalize magnetometer
    norm = math.sqrt(mag['x']**2 + mag['y']**2 + mag['z']**2)
    if norm == 0:
        norm = 1
    mag_x = mag['x'] / norm
    mag_y = mag['y'] / norm
    mag_z = mag['z'] / norm

    # Tilt compensated mag yaw
    mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
    mag_y_comp = (mag_x * math.sin(roll) * math.sin(pitch) +
                  mag_y * math.cos(roll) -
                  mag_z * math.sin(roll) * math.cos(pitch))

    mag_yaw = math.atan2(-mag_y_comp, mag_x_comp)

    # Complementary filter for yaw
    yaw = ALPHA * gyro_yaw + (1 - ALPHA) * mag_yaw

    return roll, pitch, yaw

def read_serial_line():
    global acc, gyro, mag, last_time, roll, pitch, yaw

    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return None

        m = line_re.match(line)
        if m:
            ts_raw, sensor_type, x, y, z = m.groups()
            ts = int(ts_raw) / 1e6  # Convert microseconds to seconds
            x, y, z = int(x), int(y), int(z)

            # Update sensor readings
            if sensor_type == 'acc':
                acc['x'], acc['y'], acc['z'] = x, y, z
            elif sensor_type == 'gyro':
                gyro['x'], gyro['y'], gyro['z'] = x, y, z
            elif sensor_type == 'mag':
                mag['x'], mag['y'], mag['z'] = x, y, z
            else:
                return None

            # Compute dt for filter only if last_time is set
            if last_time is None:
                last_time = ts
                return None

            dt = ts - last_time
            if dt <= 0:
                dt = 1e-3  # minimal positive dt fallback

            last_time = ts

            # Run complementary filter with current sensor values
            roll, pitch, yaw = complementary_filter(dt, acc, gyro, mag, roll, pitch, yaw)

            return ts, roll, pitch, yaw

    except Exception as e:
        print(f"Error reading/parsing line: {e}")
    return None

# === Plotting setup ===
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
fig.suptitle("Roll, Pitch, Yaw from Complementary Filter")

line_roll, = ax1.plot([], [], label="Roll (rad)")
line_pitch, = ax2.plot([], [], label="Pitch (rad)")
line_yaw, = ax3.plot([], [], label="Yaw (rad)")

for ax in (ax1, ax2, ax3):
    ax.legend()
    ax.grid(True)

ax1.set_ylim(-math.pi, math.pi)
ax2.set_ylim(-math.pi, math.pi)
ax3.set_ylim(-math.pi, math.pi)

def update_plot(frame):
    data = None

    # Read all available lines without blocking
    while True:
        d = read_serial_line()
        if d:
            data = d  # keep latest valid data
        else:
            break

    if data:
        ts, r, p, y = data
        timestamps.append(ts)
        rolls.append(r)
        pitches.append(p)
        yaws.append(y)

        line_roll.set_data(timestamps, rolls)
        line_pitch.set_data(timestamps, pitches)
        line_yaw.set_data(timestamps, yaws)

        # Set x-axis to show last WINDOW_SECONDS seconds
        if len(timestamps) > 0:
            ax1.set_xlim(max(timestamps[-1] - WINDOW_SECONDS, 0), timestamps[-1])

    return line_roll, line_pitch, line_yaw

ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)

plt.xlabel("Time (s)")
ax1.set_ylabel("Roll (rad)")
ax2.set_ylabel("Pitch (rad)")
ax3.set_ylabel("Yaw (rad)")
plt.tight_layout()
plt.show()
