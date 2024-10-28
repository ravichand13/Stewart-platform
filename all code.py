import numpy as np
from scipy.optimize import fsolve
import serial
import time

# Constants for Calibration
REFERENCE_LENGTHS = None

# Define global base points and top points for the neutral position
base_points = None
top_points_ref = None


# Function to set base points and top points reference
def set_base_and_top_points(base_pts, top_pts):
    global base_points, top_points_ref
    base_points = np.array(base_pts)
    top_points_ref = np.array(top_pts)


# Function to calibrate the actuator lengths in the reference position
def calibrate_actuators():
    global REFERENCE_LENGTHS
    REFERENCE_LENGTHS = read_actuator_lengths()
    if REFERENCE_LENGTHS is not None:
        print("Calibration complete. Reference lengths set.")
    else:
        print("Calibration failed.")


# Define the function to read actuator lengths
def read_actuator_lengths():
    try:
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize
        ser.write(b'R\n')
        line = ser.readline().decode('utf-8').strip()

        actuator_lengths = line.split(',')
        if len(actuator_lengths) == 6:
            return np.array([int(actuator_lengths[i]) for i in range(6)])
        else:
            print("Error: Unexpected number of readings from Arduino.")
            return None

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None
    finally:
        ser.close()


# Forward Kinematics: Calculate angles (roll, pitch, yaw) from actuator lengths
def calculate_angles_from_lengths(actuator_lengths):
    def error_function(angles):
        roll, pitch, yaw = angles
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        R = Rz @ Ry @ Rx
        top_points_rotated = np.dot(top_points_ref, R.T)

        errors = []
        for i in range(6):
            base_point = base_points[i]
            top_point = top_points_rotated[i]
            distance = np.linalg.norm(base_point - top_point)
            error = distance - actuator_lengths[i]
            errors.append(error)

        return errors[:3]

    initial_guess = [0, 0, 0]
    result = fsolve(error_function, initial_guess)
    roll, pitch, yaw = result
    return {"roll": np.degrees(roll), "pitch": np.degrees(pitch), "yaw": np.degrees(yaw)}


# Inverse Kinematics: Calculate actuator lengths from given angles
def calculate_actuator_lengths(roll, pitch, yaw):
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    top_points_rotated = np.dot(top_points_ref, R.T)

    actuator_lengths = []
    for i in range(6):
        base_point = base_points[i]
        top_point = top_points_rotated[i]
        length = np.linalg.norm(base_point - top_point)
        actuator_lengths.append(length)

    return actuator_lengths


# Verification function
def verify_angles_with_lengths(actuator_lengths, calculated_angles):
    roll, pitch, yaw = calculated_angles["roll"], calculated_angles["pitch"], calculated_angles["yaw"]
    recalculated_lengths = calculate_actuator_lengths(roll, pitch, yaw)
    print("Original lengths:", actuator_lengths)
    print("Recalculated lengths:", recalculated_lengths)
    if np.allclose(actuator_lengths, recalculated_lengths, atol=1e-3):
        print("Verification successful: Calculated angles match original actuator lengths.")
    else:
        print("Verification failed: Calculated angles do not match original actuator lengths.")


# Main function to perform calibration, calculate angles, and verify
def main():
    # Set your base and top points here
    base_pts = [[x1_base, y1_base, z1_base], [x2_base, y2_base, z2_base], [x3_base, y3_base, z3_base],
                [x4_base, y4_base, z4_base], [x5_base, y5_base, z5_base], [x6_base, y6_base, z6_base]]
    top_pts = [[x1_top, y1_top, z1_top], [x2_top, y2_top, z2_top], [x3_top, y3_top, z3_top],
               [x4_top, y4_top, z4_top], [x5_top, y5_top, z5_top], [x6_top, y6_top, z6_top]]

    # Step 1: Set base and top points
    set_base_and_top_points(base_pts, top_pts)
    print("Base and top points set.")

    # Step 2: Calibrate actuator lengths in neutral position
    calibrate_actuators()
    if REFERENCE_LENGTHS is None:
        print("Exiting due to calibration failure.")
        return

    # Step 3: Calculate angles from current actuator lengths (forward kinematics)
    calculated_angles = calculate_angles_from_lengths(REFERENCE_LENGTHS)
    print("Calculated angles (roll, pitch, yaw):", calculated_angles)

    # Step 4: Verify angles by recalculating lengths (inverse kinematics)
    verify_angles_with_lengths(REFERENCE_LENGTHS, calculated_angles)


# Run the main function
main()
