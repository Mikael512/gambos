import re
import queue
import signal
import threading
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation
import numpy as np
import os

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MAX_POINTS = 2000

"""
Ellipsoid center (hard iron offset):
[-0.30340633  0.0233795   0.02243486]

Soft iron correction matrix (Q):
[[ 5.44318446  0.07167194  0.06680002]
 [ 0.07167194  5.67524458 -0.10382773]
 [ 0.06680002 -0.10382773  5.72381179]]
Eigenvalues: [5.39487035 5.64102152 5.80634896]
Soft iron correction matrix to apply:
[[ 0.42867194 -0.00277072 -0.00257098]
 [-0.00277072  0.41984551  0.00384091]
 [-0.00257098  0.00384091  0.41805672]]
 """

SOFT = [[0.42867194, -0.00277072, -0.00257098], 
        [-0.00277072, 0.41984551, 0.00384091], 
        [-0.00257098, 0.00384091, 0.41805672]]

HARD = [-0.30340633, 0.0233795, 0.02243486]

MAG_PATTERN = re.compile(
    r'X\s*=\s*(-?\d+(?:\.\d+)?),\s*Y\s*=\s*(-?\d+(?:\.\d+)?),\s*Z\s*=\s*(-?\d+(?:\.\d+)?)'
)

data_q = queue.Queue()
stop_event = threading.Event()
paused = threading.Event()
paused.clear()

xs, ys, zs = [], [], []

# Output file for logging magnetometer data
DATA_FILE = "mag_data.txt"
data_file = open(DATA_FILE, 'w')  # File will be closed on exit

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

                    # Write data to file
                    data_file.write(f"{x:.6f},{y:.6f},{z:.6f}\n")
                    data_file.flush()

                    data_q.put((x, y, z))
    except serial.SerialException as e:
        print(f"[Serial] {e}")
        stop_event.set()

def update_plot(frame, ax, scatter):
    if paused.is_set():
        return scatter,

    updated = False
    while not data_q.empty():
        x, y, z = data_q.get()

        # x = SOFT[0][0] * (xm - HARD[0]) + SOFT[0][1] * (ym - HARD[1]) + SOFT[0][2] * (zm - HARD[2])
        # y = SOFT[1][0] * (xm - HARD[0]) + SOFT[1][1] * (ym - HARD[1]) + SOFT[1][2] * (zm - HARD[2])
        # z = SOFT[2][0] * (xm - HARD[0]) + SOFT[2][1] * (ym - HARD[1]) + SOFT[2][2] * (zm - HARD[2])

        xs.append(x)
        ys.append(y)
        zs.append(z)
        if len(xs) > MAX_POINTS:
            xs.pop(0)
            ys.pop(0)
            zs.pop(0)
        updated = True

    if updated:
        scatter._offsets3d = (xs, ys, zs)
        ax.set_xlim(min(xs) - 1, max(xs) + 1)
        ax.set_ylim(min(ys) - 1, max(ys) + 1)
        ax.set_zlim(min(zs) - 1, max(zs) + 1)
        ax.set_xlabel('X (Gauss)')
        ax.set_ylabel('Y (Gauss)')
        ax.set_zlabel('Z (Gauss)')
        ax.set_title('Live Magnetometer Data')

    return scatter,

def signal_handler(sig, frame):
    stop_event.set()
    data_file.close()
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
    data_file.close()  # Ensure it's closed on normal exit

if __name__ == "__main__":
    main()