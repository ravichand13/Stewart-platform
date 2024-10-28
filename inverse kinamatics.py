import numpy as np

# Define the base points and top points in the neutral (home) position
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


# Function to calculate the actuator lengths given roll, pitch, and yaw
def calculate_actuator_lengths(roll, pitch, yaw):
    # Convert angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Rotation matrices for roll, pitch, and yaw
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    # Combined rotation matrix
    R = Rz @ Ry @ Rx

    # Rotate top points by the rotation matrix
    top_points_rotated = np.dot(top_points_ref, R.T)

    # Calculate actuator lengths by finding distances between each base and rotated top point
    actuator_lengths = []
    for i in range(6):
        base_point = base_points[i]
        top_point = top_points_rotated[i]
        length = np.linalg.norm(base_point - top_point)
        actuator_lengths.append(length)

    return actuator_lengths


# Example usage
roll = 10  # example roll angle in degrees
pitch = 5  # example pitch angle in degrees
yaw = 15  # example yaw angle in degrees

actuator_lengths = calculate_actuator_lengths(roll, pitch, yaw)
print("Actuator lengths:", actuator_lengths)