import threading
import serial
import quaternion as qt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

BAUD_RATE = 115200

latest_quaternion = qt.Quaternion(1, 0, 0, 0)
lock = threading.Lock()

def open_com_port():
    global s
    port = "COM"
    port += input("Which serial port is being used? : ")

    try:
        s = serial.Serial(port, BAUD_RATE)
    except serial.SerialException:
        print("Error: Could not open serial port. Please check the connection and try again.")
        open_com_port()

def read_line():
    line = s.readline().decode().strip().split(',')
    if is_valid_line(line):
        return [float(x) for x in line]
    return None

def is_valid_line(line):
    try:
        values = [float(x) for x in line]
        return len(values) == 4
    except ValueError:
        return False

def serial_loop():
    global latest_quaternion
    while True:
        line = read_line()
        if line is not None:
            w, x, y, z = line
            with lock:
                latest_quaternion = qt.Quaternion(w, x, y, z)

def get_latest_quaternion():
    with lock:
        return latest_quaternion

# --- visualization ---

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def to_display_coords(v):
    # v = [x, y, z] in the IMU's native frame (X=yaw axis, Y=roll axis, Z=pitch axis)
    # remap so the yaw axis (IMU X) is drawn as the plot's vertical (Z) axis
    return [v[1], v[2], v[0]]

def draw_triad(q):
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    x_axis = qt.apply_rotation(q, [1, 0, 0])
    y_axis = qt.apply_rotation(q, [0, 1, 0])
    z_axis = qt.apply_rotation(q, [0, 0, 1])

    dx = to_display_coords([x_axis.x, x_axis.y, x_axis.z])
    dy = to_display_coords([y_axis.x, y_axis.y, y_axis.z])
    dz = to_display_coords([z_axis.x, z_axis.y, z_axis.z])

    ax.plot([0, dx[0]], [0, dx[1]], [0, dx[2]], color='r')  # yaw axis (was IMU X)
    ax.plot([0, dy[0]], [0, dy[1]], [0, dy[2]], color='g')  # roll axis (was IMU Y)
    ax.plot([0, dz[0]], [0, dz[1]], [0, dz[2]], color='b')  # pitch axis (was IMU Z)

    # text overlays
    pitch, yaw, roll = qt.quaternion_to_euler(q)

    quat_text = f"w: {q.w:+.3f}\nx: {q.x:+.3f}\ny: {q.y:+.3f}\nz: {q.z:+.3f}"
    euler_text = f"Yaw:   {yaw:+7.2f}°\nRoll:  {roll:+7.2f}°\nPitch: {pitch:+7.2f}°"

    ax.text2D(0.02, 0.95, quat_text, transform=ax.transAxes,
              fontsize=9, family='monospace', verticalalignment='top')
    ax.text2D(0.98, 0.95, euler_text, transform=ax.transAxes,
              fontsize=9, family='monospace', verticalalignment='top',
              horizontalalignment='right')

def update(frame):
    q = get_latest_quaternion()
    draw_triad(q)

if __name__ == "__main__":
    open_com_port()

    reader_thread = threading.Thread(target=serial_loop, daemon=True)
    reader_thread.start()

    ani = animation.FuncAnimation(fig, update, interval=50)
    plt.show()