"""
IMU calibration tool.

Basic calibration accounting for gyroscope bias and
accelerometer offset and scaling

Expects raw MPU6050 samples over UART in the format:
ax,ay,az,gx,gy,gz

This tool is intended for sensor calibration only.
The STM32 runtime telemetry format will change once
attitude estimation is moved onto the STM32.
"""

import serial

SAMPLES_TO_COLLECT = 500

BAUD_RATE = 115200

def open_com_port():
    global s
    port = "COM"
    port += input("Which serial port is being used? : ") #### TODO : make this safe

    try:
        s = serial.Serial(port, BAUD_RATE) 
    except serial.SerialException:
        print("Error: Could not open serial port. Please check the connection and try again.")
        open_com_port()

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

    
    samples = []
    while len(samples) != n:

        line = s.readline().decode().strip().split(',') # returns a list of values as strings (i.e. ['16856', '-216', '-938', '-419', '158', '11'])

        if not is_valid_line(line): # check the line is in the format we expect, otherwise skip it.
            continue

        sample = [int(x) for x in line] # sample = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
                
        samples.append(sample)
        print(f"\rCollecting samples {len(samples)}/{n}   |   Current sample: {sample}                               ", end="")
        
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

def construct_output_string(biases, offsets, scales):
    output = ""
    for bias in biases:
        output += str(bias) + ","
    for offset in offsets:
        output += str(offset) + ","
    for scale in scales:
        output += str(scale) + ","

    return output


if __name__ == "__main__":
    open_com_port()

    # 1 calculate gyro bias for each axis
    print("Keep the IMU stationary")
    input("Press Enter to start collecting samples for gyro bias calculation")
    print("Collecting samples...")
    samples = collect_samples(SAMPLES_TO_COLLECT)

    # Calculate gyro bias for each axis (gyro_x, gyro_y, gyro_z)
    for i in range(3, 6):  # Indices 3, 4, 5 correspond to gyro_x, gyro_y, gyro_z
        gyro_bias[i - 3] = calculate_average(samples, i)

    print("Gyro Bias:", gyro_bias)

    for orientation in ORIENTATIONS:
        for i in range(2): # 0 for +ve, 1 for -ve
            input(f"Place the IMU with {"+" if i == 0 else "-"}{orientation} axis facing up and press Enter to start collecting samples")
            print("Collecting samples...")
            samples = collect_samples(SAMPLES_TO_COLLECT)

            # calculate average accel for current orientation
            if orientation == 'X':
                accel_averages[0][i] = calculate_average(samples, 0) # accel_x
            elif orientation == 'Y':
                accel_averages[1][i] = calculate_average(samples, 1) # accel_y
            elif orientation == 'Z':
                accel_averages[2][i] = calculate_average(samples, 2) # accel_z

    calculate_offset_and_scale(accel_averages)

    with open("calibration_results.csv", "w") as f:
        f.write(construct_output_string(gyro_bias, accel_offsets, accel_scales))

# FIRST SET OF RESULTS:
#Gyro Bias: [-405.025, 159.359, 18.535]
#Acceleration offsets:  [612.1850000000004, -331.4806766766769, 644.6109999999999]
#Acceleration scales:  [1.0058664286247092, 1.0072465113750626, 0.987304662489624]