import numpy as np
from scipy.optimize import fsolve
import serial
import time

# Define the reference positions of each actuator and the initial "home" position
base_points = np.array([[x1_base, y1_base, z1_base],
                        [x2_base, y2_base, z2_base],
                        [x3_base, y3_base, z3_base],
                        [x4_base, y4_base, z4_base],
                        [x5_base, y5_base, z5_base],
                        [x6_base, y6_base, z6_base]])

top_points_ref = np.array([[x1_top, y1_top, z1_top],
                           [x2_top, y2_top, z2_top],
                           [x3_top, y3_top, z3_top],
                           [x4_top, y4_top, z4_top],
                           [x5_top, y5_top, z5_top],
                           [x6_top, y6_top, z6_top]])


# Define the function to read lengths (from previous code)
def read_actuator_lengths():
    try:
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize
        ser.write(b'R\n')
        line = ser.readline().decode('utf-8').strip()

        actuator_lengths = line.split(',')
        if len(actuator_lengths) == 6:
            actuator_1_length = int(actuator_lengths[0])
            actuator_2_length = int(actuator_lengths[1])
            actuator_3_length = int(actuator_lengths[2])
            actuator_4_length = int(actuator_lengths[3])
            actuator_5_length = int(actuator_lengths[4])
            actuator_6_length = int(actuator_lengths[5])
            return np.array([actuator_1_length, actuator_2_length, actuator_3_length,
                             actuator_4_length, actuator_5_length, actuator_6_length])
        else:
            print("Error: Unexpected number of readings from Arduino.")
            return None

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None
    finally:
        ser.close()


# Function to calculate error based on lengths
def error_function(angles, actuator_lengths):
    # Extract roll, pitch, and yaw from angles array (X,Y,Z angeles)
    roll, pitch, yaw = angles

    # Rotation matrix for roll-pitch-yaw angles
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    # Total rotation matrix
    R = Rz @ Ry @ Rx

    # Calculate the rotated top points
    top_points_rotated = np.dot(top_points_ref, R.T)

    # Calculate the length errors for each actuator
    errors = []
    for i in range(6):
        base_point = base_points[i]
        top_point = top_points_rotated[i]
        distance = np.linalg.norm(base_point - top_point)
        error = distance - actuator_lengths[i]
        errors.append(error)

    return errors


# Main function to compute roll, pitch, yaw
def compute_angles():
    # Get the actuator lengths
    actuator_lengths = read_actuator_lengths()

    if actuator_lengths is None:
        print("Failed to read actuator lengths.")
        return None

    # Initial guess for roll, pitch, and yaw (in radians)
    initial_guess = [0, 0, 0]

    # Solve the system of equations to find the angles
    result = fsolve(error_function, initial_guess, args=(actuator_lengths,))

    roll, pitch, yaw = result

    return {
        "roll": np.degrees(roll),
        "pitch": np.degrees(pitch),
        "yaw": np.degrees(yaw)
    }


# Example usage
angles = compute_angles()
if angles:
    print("Calculated angles:", angles)