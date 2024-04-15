import numpy as np

def euler_to_quat(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.

    Args:
        roll: Roll angle in radians.
        pitch: Pitch angle in radians.
        yaw: Yaw angle in radians.

    Returns:
        Quaternion [x, y, z, w].
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qx, qy, qz, qw])

def quat_to_euler(quaternion):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.

    Args:
        quaternion: Quaternion [x, y, z, w].

    Returns:
        Roll, pitch, yaw angles in radians.
    """
    qx, qy, qz, qw = quaternion

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))  # Use np.clip to handle edge cases

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # Convert pitch from array to scalar (float)
    pitch = float(pitch)  # Convert to float if needed

    return roll, pitch, yaw