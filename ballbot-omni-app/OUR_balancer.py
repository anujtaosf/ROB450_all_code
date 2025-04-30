import threading
import time
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
from DataLogger import dataLogger  # Import data logger for logging data
import numpy as np


# Constants for the control loop
FREQ = 50  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds

KP_THETA_X = 28.0  # Proportional gain for roll stability
KP_THETA_Y = 28.0  # Proportional gain for pitch stability

MAX_PLANAR_DUTY = 1.0



def compute_stabilization_torques(theta_x, theta_y, desired_theta_x=0.0, desired_theta_y=0.0):
    # Proportional control
    error_x = desired_theta_x - theta_x
    error_y = desired_theta_y - theta_y

    Tx = KP_THETA_X * error_x
    Ty = KP_THETA_Y * error_y

    # Saturate torques to prevent excessive output
    Tx = np.clip(Tx, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
    Ty = np.clip(Ty, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)

    return Tx, Ty


if __name__ == "__main__":

    # === Serial Communication Initialization ===
    # Initialize the serial communication protocol
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    
    # Start a separate thread for reading serial data
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # === Command and State Structures ===
    # Define command structure for controlling motors
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands['start'] = 1.0  # Activate motors

    # Initialize state structure for reading sensors
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Allow communication to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)

    print("calibrating...")

    # === Main Control Loop ===
    i = 0  # Iteration counter
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Read sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]

        except:
            #print("waiting")
            continue
        

        theta_x = -(states['theta_roll'])  
        theta_y = (states['theta_pitch'])
        # print(theta_y)

        # ---------------------------------------------------------------


        # === Torque Computation ===
        # Compute motor torques using imported function
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz) # From previous labs!

        # Send computed torques to the robot
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands)


    print("Shutting down motors...")
    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)
