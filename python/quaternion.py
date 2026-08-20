import math

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Quaternion(w={self.w}, x={self.x}, y={self.y}, z={self.z})"

def mult(q1, q2): # multiply 2 quaternions
    q3 = Quaternion(0, 0, 0, 0)
    q3.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    q3.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    q3.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    q3.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    return q3

def axis_angle_to_quaternion(axis, angle): # expects axis as a unit vector

    half_angle = angle / 2

    return Quaternion(
        math.cos(half_angle),
        axis[0] * math.sin(half_angle),
        axis[1] * math.sin(half_angle),
        axis[2] * math.sin(half_angle)
    )

def conj(q): # conjugate
    return Quaternion(q.w, -q.x, -q.y, -q.z)

def rotate_vector(q, v): # v' = qvq*; v expected as a 3d vector [x,y,z]
    v_q = Quaternion(0, v[0], v[1], v[2])
    return mult(mult(q,v_q), conj(q))

def mag(v): # return magnitude of a 3d vector
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def normalise(v): # normalise a 3d vector
    magnitude = mag(v)

    if magnitude == 0: return v

    vn = [0, 0, 0]

    for i in range(len(v)):
        vn[i] = v[i] / magnitude
    return vn

def quaternion_to_euler(q): ### CONVENTIONS FOR THIS PROJECT : Rotation about X = yaw, rotation about Y = roll, rotation about Z = pitch
    yaw = -math.atan2(
        2 * (q.w * q.x + q.y * q.z),
        1 - 2 * (q.x**2 + q.y**2)
    )

    value = 2 * (q.w * q.y - q.z * q.x)
    value = max(-1.0, min(1.0, value))

    roll = math.asin(value)

    pitch = -math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y**2 + q.z**2)
    )

    return [
        math.degrees(pitch),
        math.degrees(yaw),
        math.degrees(roll)
    ]