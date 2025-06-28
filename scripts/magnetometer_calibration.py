import re
import queue
import signal
import threading
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MAX_POINTS = 2000
#[Hard Iron Bias] X: 23.0385, Y: 5.1832, Z: 104.6073
# Regex for float values like: X = -26.4975, Y = 0.3840, Z = -25.3470
MAG_PATTERN = re.compile(
    r'X\s*=\s*(-?\d+(?:\.\d+)?),\s*Y\s*=\s*(-?\d+(?:\.\d+)?),\s*Z\s*=\s*(-?\d+(?:\.\d+)?)'
)

data_q = queue.Queue()
stop_event = threading.Event()
paused = threading.Event()  # Used to pause/resume data acquisition
paused.clear()  # Start in "running" mode

xs, ys, zs = [], [], []


def correct_overflow(axis_idx, value, yes):
    if not yes:
        return value
    
    global last_mag, overflow_correction
    if last_mag[axis_idx] is not None:
        delta = value - last_mag[axis_idx]
        if delta > 80:  # Wrapped from -49 to +51
            overflow_correction[axis_idx] -= 100.0
            print(f"[Wrap Detected] Axis {axis_idx}: Negative wrap → Correction -= 100")
        elif delta < -80:  # Wrapped from +49 to -51
            overflow_correction[axis_idx] += 100.0
            print(f"[Wrap Detected] Axis {axis_idx}: Positive wrap → Correction += 100")
    last_mag[axis_idx] = value
    return value + overflow_correction[axis_idx]

def read_serial():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            while not stop_event.is_set():
                if paused.is_set():
                    continue  # Pause reading
                line = ser.readline().decode(errors='ignore').strip()
                match = MAG_PATTERN.search(line)
                if match:
                    x, y, z = map(float, match.groups())

                    data_q.put((x, y, z))
    except serial.SerialException as e:
        print(f"[Serial] {e}")
        stop_event.set()

def compute_bias(arr):
    if not arr:
        return 0.0
    return (max(arr) + min(arr)) / 2.0

def update_plot(frame, ax, scatter):
    if paused.is_set():
        return scatter,

    updated = False
    while not data_q.empty():
        x, y, z = data_q.get()
        xs.append(x)
        ys.append(y)
        zs.append(z)
        if len(xs) > MAX_POINTS:
            xs.pop(0)
            ys.pop(0)
            zs.pop(0)
        updated = True

    if updated:
        x_bias = compute_bias(xs)
        y_bias = compute_bias(ys)
        z_bias = compute_bias(zs)

        print(f"[Hard Iron Bias] X: {x_bias:.4f}, Y: {y_bias:.4f}, Z: {z_bias:.4f}")

        scatter._offsets3d = (xs, ys, zs)
        ax.set_xlim(min(xs) - 1, max(xs) + 1)
        ax.set_ylim(min(ys) - 1, max(ys) + 1)
        ax.set_zlim(min(zs) - 1, max(zs) + 1)
        ax.set_xlabel('X (Gauss)')
        ax.set_ylabel('Y (Gauss)')
        ax.set_zlabel('Z (Gauss)')
        ax.set_title('Live Magnetometer Data with Overflow Correction')

    return scatter,

def signal_handler(sig, frame):
    stop_event.set()
    plt.close('all')

def on_key(event):
    if event.key == ' ':
        if paused.is_set():
            print("[Paused] → Resuming data collection.")
            paused.clear()
        else:
            print("[Running] → Pausing data collection.")
            paused.set()

def main():
    signal.signal(signal.SIGINT, signal_handler)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter([], [], [], c='blue')

    fig.canvas.mpl_connect('close_event', lambda event: stop_event.set())
    fig.canvas.mpl_connect('key_press_event', on_key)

    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    ani = FuncAnimation(fig, update_plot, fargs=(ax, scatter), interval=50)
    plt.show()

    stop_event.set()
    serial_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
