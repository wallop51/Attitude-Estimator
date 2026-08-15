import serial, time

try:
    s = serial.Serial('COM3', 9600) 
except serial.SerialException:
    print("Error: Could not open serial port. Please check the connection and try again.")
    exit()

ORIENTATIONS = ['X', 'Y', 'Z']

gyro_bias = [0, 0, 0] # Initialize gyro bias to zero
accel_averages = [[0, 0,], [0, 0], [0, 0]] # average accel when each axis is facing up (should be 1g) [[+x, -x], [+y, -y], [+z, -z]]

accel_offsets = [0, 0, 0] # Initialize accel offsets to zero
accel_scales = [1, 1, 1] # Initialize accel scales to one

def is_valid_line(line):
    try:
        values = [int(x) for x in line]
        return len(values) == 6
    except ValueError:
        return False

def collect_samples(n):
    
    s.reset_input_buffer()
    line = s.readline().decode().strip().split(',')
    while not is_valid_line(line):
        print(f"Invalid sample received: {line}. Waiting for valid data...\n")
        line = s.readline().decode().strip().split(',')

    sample = [int(x) for x in line] # sample = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
    
    samples = []
    for _ in range(n):
        print(f"\rCollecting samples {len(samples)}/{n}", end="")
        line = s.readline().decode().strip().split(',') # returns a list of values as strings (i.e. ['16856', '-216', '-938', '-419', '158', '11'])
        sample = [int(x) for x in line] # sample = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        if not is_valid_line(line):
            print(f"\rInvalid sample received: {line}. Waiting for valid data...", end="")
            continue
        samples.append(sample)

    print("\n")
    return samples

def calculate_average(samples, index):
    total = 0
    for sample in samples:
        total += sample[index]
    return total / len(samples)

def calculate_offset_and_scale(accel_averages):
    for i in range(3): # for each axis
        accel_offsets[i] = (accel_averages[i][0] + accel_averages[i][1]) / 2
        accel_scales[i] = 32768 / (accel_averages[i][0] - accel_averages[i][1])
    print("Accel Offsets:", accel_offsets)
    print("Accel Scales:", accel_scales)

if __name__ == "__main__":

    # 1 calculate gyro bias for each axis
    print("Keep the IMU stationary")
    input("Press Enter to start collecting samples for gyro bias calculation")
    print("Collecting samples...")
    samples = collect_samples(1000)

    # Calculate gyro bias for each axis (gyro_x, gyro_y, gyro_z)
    for i in range(3, 6):  # Indices 3, 4, 5 correspond to gyro_x, gyro_y, gyro_z
        gyro_bias[i - 3] = calculate_average(samples, i)

    print("Gyro Bias:", gyro_bias)

    for orientation in ORIENTATIONS:
        for i in range(2): # 0 for +ve, 1 for -ve
            input(f"Place the IMU with {"+" if i == 0 else "-"}{orientation} axis facing up and press Enter to start collecting samples")
            print("Collecting samples...")
            samples = collect_samples(1000)

            # calculate average accel for current orientation
            if orientation == 'X':
                accel_averages[0][i] = calculate_average(samples, 0) # accel_x
            elif orientation == 'Y':
                accel_averages[1][i] = calculate_average(samples, 1) # accel_y
            elif orientation == 'Z':
                accel_averages[2][i] = calculate_average(samples, 2) # accel_z

    calculate_offset_and_scale(accel_averages)

    print("Acceleration offsets: ", accel_offsets)
    print("Acceleration scales: ", accel_scales)

# FIRST SET OF RESULTS:
#Gyro Bias: [-405.025, 159.359, 18.535]
#Acceleration offsets:  [612.1850000000004, -331.4806766766769, 644.6109999999999]
#Acceleration scales:  [1.0058664286247092, 1.0072465113750626, 0.987304662489624]