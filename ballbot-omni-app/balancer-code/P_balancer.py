"""
Stability Controller for Ball-Bot

This module implements the core logic for stabilizing a ball-bot using proportional control.
The controller calculates the required torques to minimize the tilt (roll and pitch) and
maintain balance.

Features:
- Computes stabilization torques (Tx, Ty) based on proportional gains (Kp).
- Uses `compute_motor_torques` from `ball_kinematics.py` to compute motor torques (T1, T2, T3).
- Provides a clear structure for students to experiment with gains and logic.

Standalone Mode:
- Run this file directly to test the balance controller with real IMU readings.
- Outputs calculated torques (Tx, Ty, T1, T2, T3) for debugging.

Author: Anuhea Tao, Reina Mezher, Adam Hung, Joseph Fedoronko
Date: May 2025
"""

import numpy as np
import threading
import time
from ballbot_kinematics import compute_motor_torques  # Import from ball_kinematics
from MBot.SerialProtocol.protocol import SerialProtocol
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from loop import SoftRealtimeLoop

# Default Parameters
KP_THETA_X = 12.0  # Proportional gain for roll stability
KP_THETA_Y = 12.0  # Proportional gain for pitch stability
MAX_PLANAR_DUTY = 0.8  # Maximum allowable torque to prevent oversaturation


def compute_stabilization_torques(theta_x, theta_y, desired_theta_x=0.0, desired_theta_y=0.0):
    """
    Compute planar stabilization torques (Tx, Ty) to balance the robot.

    Parameters:
    ----------
    theta_x : float
        Current roll angle (rad).
    theta_y : float
        Current pitch angle (rad).
    desired_theta_x : float, optional
        Desired roll angle (rad), by default 0.0.
    desired_theta_y : float, optional
        Desired pitch angle (rad), by default 0.0.

    Returns:
    --------
    Tx, Ty : tuple of floats
        Stabilization torques along x-axis (roll) and y-axis (pitch).
    """
    # Proportional control
    error_x = desired_theta_x - theta_x
    error_y = desired_theta_y - theta_y

    Tx = KP_THETA_X * error_x
    Ty = KP_THETA_Y * error_y

    Tz = 0.0  # No yaw control for now

    # Saturate torques to prevent excessive output
    Tx = np.clip(Tx, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
    Ty = np.clip(Ty, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)

    return Tx, Ty


# Standalone testing mode
if __name__ == "__main__":
    print("=== Balance Controller Standalone Test with IMU Readings ===")

    # Initialize serial communication for IMU readings
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    
    # Start serial read thread
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands['start'] = 1.0  # Activate motors

    # Initialize state structure for reading sensors
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Synchronize with IMU
    print("Syncing with IMU...")
    time.sleep(1.0)

    # Run control loop
    for t in SoftRealtimeLoop(dt=0.005, report=True):  # 200 Hz loop
        try:
            # Fetch IMU data
            states = ser_dev.get_cur_topic_data(121)[0]
            theta_x = states['theta_roll']  # Roll angle
            theta_y = states['theta_pitch']  # Pitch angle

            # Compute stabilization torques
            Tx, Ty = compute_stabilization_torques(theta_x, theta_y)

            # Compute motor torques
            T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz=0)

            # Print results
            print(
                f"Roll: {theta_x:.2f}, Pitch: {theta_y:.2f} | "
                f"Stabilization Torques: Tx = {Tx:.2f}, Ty = {Ty:.2f} | "
                f"Motor Torques: T1 = {T1:.2f}, T2 = {T2:.2f}, T3 = {T3:.2f}"
            )

            commands['motor_1_duty'] = T1
            commands['motor_2_duty'] = T2
            commands['motor_3_duty'] = T3
            ser_dev.send_topic_data(101, commands)

            # === Data Logging ===
            if i == 0:
                t_start = time.time()  # Start time of the loop
            t_now = time.time() - t_start  # Elapsed time
            i += 1

        except KeyError:
            #print("Waiting for IMU data...")
            continue
