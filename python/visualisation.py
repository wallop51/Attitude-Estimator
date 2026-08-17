import serial, time

BAUD_RATE = 115200
params = []
current_angle = [0,0,0]
dt = 0.01 # dt @ 100Hz

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
        accels[i] = (accels[i] - params[3 + i]) * params[6 + i] # params[3+i] is offset, params[6+i] is scale
        gyros[i] = (gyros[i] - params[i]) / 131 # params[i] is gyro bias, 131 is conversion factor from raw to dps

    return accels + gyros

def display_sample(sample):
    print(f"ax = {sample[0]:6.0f} ay = {sample[1]:6.0f} az = {sample[2]:6.0f} gx = {sample[3]:6.0f} gy = {sample[4]:6.0f} gz = {sample[5]:6.0f}")
    print(f"angles: {current_angle[0]:4.0f}, {current_angle[1]:4.0f}, {current_angle[2]:4.0f}, ")
    print("\033[2A", end="")

def calculate_angle_delta(sample, dt): # takes a calibrated and converted sample (units dps)
    omega = sample[3:]
    for i in range(3):
        current_angle[i] += omega[i] * dt



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
            calculate_angle_delta(calibrated_sample, dt)
            display_sample(calibrated_sample)