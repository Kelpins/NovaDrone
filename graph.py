import socket
import re
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# —— CONFIG ——
HOST = "192.168.76.215"
PORT = 23

# —— SETUP SOCKET ——
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
stream = sock.makefile("r")

# —— DATA & PLOT —— 
xs, ys = [], []
fig, ax = plt.subplots()
ln, = ax.plot([], [], lw=2)
ax.set_xlabel("s"); ax.set_ylabel("yaw (°)")
ax.set_ylim(0, 360)
start = time.time()

def init():
    ax.set_xlim(0, 10)
    return ln,

def update(frame):
    line = stream.readline()
    if not line:
        return ln,
    m = re.search(r"Iy:\s*([0-9]+\.[0-9]+)", line)
    if m:
        t = time.time() - start
        y = float(m.group(1))
        xs.append(t); ys.append(y)
        ln.set_data(xs, ys)
        ax.set_xlim(max(0, t-10), t+1)
    return ln,

ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=100)
plt.show()
