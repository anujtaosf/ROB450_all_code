"""
Steering Controller for Ball-Bot on the Floor

This script integrates:
1. PS4 controller inputs to dynamically adjust the torques applied to the motors of a ball-bot.
2. Real-time sensor readings from the robot to monitor the system's state.
3. Data logging to store relevant information (e.g., sensor readings, torque commands) for analysis.

Key Features:
- PS4 controller interface to adjust torques dynamically.
- Computes motor torques using inputs and the `compute_motor_torques` function from `ball_kinematics.py`.
- Reads sensor data (e.g., IMU angles, motor rotations) and logs them in a structured file for debugging and analysis.

Requirements:
- PS4 controller connected via `/dev/input/js0`.
- Serial communication with the ball-bot for state updates and command transmission.

Author: Yilin Ma
Date: January 2025
"""

import threading
import time
from ps4_controller_api import PS4InputHandler
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
import numpy as np

# Constants for the control loop
FREQ = 200  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds
JOYSTICK_SCALE = 32767  # Scale factor for normalizing joystick values

# 00:E5:F6:B2:C9:8F

TX_MAX = 1.0  # Maximum torque along x-axis
TY_MAX = 1.0  # Maximum torque along y-axis
TZ_MAX = 1.0  # Maximum torque along z-axis (yaw)

# ---------------------------------------------------------------------------


if __name__ == "__main__":
    # === Controller Initialization ===
    # Create an instance of the PS4 controller handler
    controller = PS4InputHandler(interface="/dev/input/js0")

    # Start a separate thread to listen for controller inputs
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Ensures the thread stops with the main program
    controller_thread.start()

    print("PS4 Controller is active. Use thumbsticks and triggers to control torque.")

    # === Serial Communication Initialization ===
    # Initialize the serial communication protocol
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    
    # Start a separate thread for reading serial data
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # # === Data Logging Initialization ===
    # # Prompt user for trial number and create a data logger
    # trial_num = int(input("Trial Number? "))
    # filename = f"steering_controller_trial_{trial_num}.txt"
    # dl = dataLogger(filename)

    # === Command and State Structures ===
    # Define command structure for controlling motors
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands['start'] = 1.0  # Activate motors

    # Initialize state structure for reading sensors
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Allow communication to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)

    print("Starting steering control loop...")

    # === Main Control Loop ===
    i = 0  # Iteration counter
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Read sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]

            # === Controller Input ===
            # Fetch signals from the PS4 controller
            signals = controller.get_signals()

            # Map joystick and trigger inputs to torques
            # ---------------------------------------------------------------
            # This is a sample implementation of mapping controller inputs to 
            # torque commands (Tx, Ty, Tz):
            # - Tx: Controlled by the horizontal movement of the left thumbstick (x-axis).
            # - Ty: Controlled by the vertical movement of the left thumbstick (y-axis).
            # - Tz: Controlled by the difference between R2 and L2 trigger values.
            #
            # Students can modify this mapping to suit their project goals. 
            # For example, they might use different scaling factors, include 
            # inputs from other buttons, or create more complex control logic.
            # ---------------------------------------------------------------
            thumbstick_x = signals["left_thumbstick_x"]
            thumbstick_y = signals["left_thumbstick_y"]
            print(f"thumbstick_x: {thumbstick_x:.2f} | thumbstick_y: {thumbstick_y:.2f}")
            Tx = signals["left_thumbstick_x"] * TX_MAX
            Ty = signals["left_thumbstick_y"] * TY_MAX
            #Tz = (signals["trigger_R2"] - signals["trigger_L2"]) * TZ_MAX
            Tz = 0
            # ---------------------------------------------------------------

            # === Torque Computation ===
            # Compute motor torques using imported function
            T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz) # From previous labs!
            T2= .3
            T3 = .3
            

            # Send computed torques to the robot
            commands['motor_1_duty'] = T1
            commands['motor_2_duty'] = T2
            commands['motor_3_duty'] = T3
            ser_dev.send_topic_data(101, commands)

            # === Data Logging ===
            if i == 0:
                t_start = time.time()  # Start time of the loop
            t_now = time.time() - t_start  # Elapsed time
            i += 1

            # Extract sensor readings
            theta_x = states['theta_roll']  # Roll angle (x-axis)
            theta_y = states['theta_pitch']  # Pitch angle (y-axis)
            psi_1 = states['psi_1']  # Motor 1 rotation
            psi_2 = states['psi_2']  # Motor 2 rotation
            psi_3 = states['psi_3']  # Motor 3 rotation

            # Log data to the file
            # data = [i, t_now, theta_x, theta_y, Tx, Ty, Tz, T1, T2, T3, psi_1, psi_2, psi_3]
            # dl.appendData(data)

            # Print debug information
            # print(
            #     f"Time: {t_now:.2f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
            #     f"T1: {T1:.2f}, T2: {T2:.2f}, T3: {T3:.2f} | "
            #     f"Roll: {theta_x:.2f}, Pitch: {theta_y:.2f}"
            # )
        except KeyError:
            print("Waiting for sensor data...")

    # === Shutdown ===
    # print("Saving data...")
    # dl.writeOut()  # Write logged data to the file

    print("Shutting down motors...")
    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)