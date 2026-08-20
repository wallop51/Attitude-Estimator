import serial, time, math
import quaternion as qt

BAUD_RATE = 115200
params = []
current_q = qt.Quaternion(1,0,0,0) # current estimated rotation
pitch_accel = 0
roll_accel = 0
gravity_vector = [1,0,0] # measured gravity direction from accelerometer
KP = 0.02


def get_calibration_parameters():
    try:
        with open("calibration_results.csv", "r") as f:
            string_params = f.read().strip().split(',')
            string_params.remove('')
            return [float(x) for x in string_params] # [gbiasx, gbiasy, gbiasz, aoffsetx, aoffsety, aoffsetz, ascalex, ascaley, ascelez]

    except FileNotFoundError:
        print("You must first calibrate the IMU. Run calibration.py")
        exit()

def open_com_port():
    global s
    port = "COM"
    port += input("Which serial port is being used? : ") #### TODO : make this safe

    try:
        s = serial.Serial(port, BAUD_RATE) 
    except serial.SerialException:
        print("Error: Could not open serial port. Please check the connection and try again.")
        open_com_port()

def is_valid_line(line):
    try:
        values = [int(x) for x in line]
        return len(values) == 6
    except ValueError:
        return False

def read_sample():
    #s.reset_input_buffer()
    line = s.readline().decode().strip().split(',')
    if (is_valid_line(line)):
        return [int(x) for x in line] # [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
    return None

def calibrate_sample(sample):
    accels = sample[:3] # [accel_x, accel_y, accel_z]
    gyros = sample[3:] # [gyro_x, gyro_y, gryo_z]
    for i in range(3):
        # apply offset and scale to each accel value
        accels[i] = ((accels[i] - params[3 + i]) * params[6 + i]) / 16384 # params[3+i] is offset, params[6+i] is scale
        gyros[i] = (gyros[i] - params[i]) / 131 # params[i] is gyro bias, 131 is conversion factor from raw to dps

    return accels + gyros


def display_sample(sample):
    current_angle = qt.quaternion_to_euler(current_q)
    print(f"ax = {sample[0]:2.3f} ay = {sample[1]:2.3f} az = {sample[2]:2.3f} gx = {sample[3]:2.3f} gy = {sample[4]:6.0f} gz = {sample[5]:6.0f}")
    print(f"gyro angles: PITCH {current_angle[0]:4.0f}, YAW {current_angle[1]:4.0f}, ROLL {current_angle[2]:4.0f}, q = {current_q}")
    print("\033[2A", end="")

def calculate_angle_delta(gyros, dt):
    delta = [0,0,0]
    for i in range(3):
        delta[i] += math.radians(gyros[i] * dt)

    return delta

def get_delta_quaternion(delta):
    angle = qt.mag(delta)
    axis = qt.normalise(delta)

    return qt.axis_angle_to_quaternion(axis, angle)

def get_predicted_gravity(current): # takes a current orientation and returns where gravity "should" according to the current orientation
    world_gravity = [1,0,0]
    predicted_gravity = qt.apply_rotation(current, world_gravity)
    return [predicted_gravity.x, predicted_gravity.y, predicted_gravity.z]

if (__name__ == "__main__"):
    open_com_port()
    params = get_calibration_parameters()
    t0 = time.monotonic()

    while True:
        sample = read_sample()
        if (sample is not None):
            t1 = time.monotonic()
            dt = t1 - t0
            t0 = t1

            calibrated_sample = calibrate_sample(sample)

            accels = calibrated_sample[:3]
            gyros = calibrated_sample[3:]

            gravity_vector = qt.normalise(accels)
            predicted_gravity = get_predicted_gravity(current_q)

            error = qt.cross(predicted_gravity, gravity_vector)
            corrected_gyros = [gyros[i] + KP * error[i] for i in range(3)]


            delta = calculate_angle_delta(corrected_gyros, dt)
            delta_q = get_delta_quaternion(delta)
            current_q = qt.mult(current_q, delta_q)
            display_sample(calibrated_sample)