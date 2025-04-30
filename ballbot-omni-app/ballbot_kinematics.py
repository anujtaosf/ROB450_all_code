"""
Ball-bot kinetics

This program uses a soft realtime loop to enforce loop timing. Soft real time loop is a  class
designed to allow clean exits from infinite loops with the potential for post-loop cleanup operations executing.

Authors: Senthur Raj, Gray Thomas, Yves Nazon and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab
"""

import sys
import threading
import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from loop import SoftRealtimeLoop
from DataLogger import dataLogger

def register_topics(ser_dev:SerialProtocol):
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

# ---------------------------------------------------------------------------

# Constants
FREQ = 200  # Frequency in Hz
DT = 1 / FREQ  # Time step
RK = 0.1192 # Ball radius (m)
RW = 0.0488  # Wheel radius (m)
ALPHA = np.deg2rad(45)  # Motor inclination angle (radians)
cos_alpha = np.cos(ALPHA)
# cos_alpha = 1.0

# ---------------------------------------------------------------------------

def compute_motor_torques(Tx, Ty, Tz):
    '''
    Parameters:
    ----------
    Tx: Torque along x-axis
    Ty: Torque along y-axis
    Tz: Torque along z-axis

    Returns:
    --------
            Ty
            T1
            |
            |
            |
            . _ _ _ _ Tx
           / \
          /   \
         /     \
        /       \
       T2       T3

    T1: Motor Torque 1
    T2: Motor Torque 2
    T3: Motor Torque 3
    '''

    T1 = (1 / 3) * (Tz - (2 * Ty) / cos_alpha)
    T2 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (-np.sqrt(3) * Tx + Ty))
    T3 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (np.sqrt(3) * Tx + Ty))

    return T1, T2, T3

# ---------------------------------------------------------------------------

def compute_ball_rotation(psi1, psi2, psi3):
    """
    Computes the ball's rotational velocities (phi_x, phi_y, phi_z)
    given the wheel velocities (psi1, psi2, psi3).
    
    Parameters:
    -----------
    psi1, psi2, psi3 : float
        Wheel angular velocities
    
    Returns:
    --------
    phi_x, phi_y, phi_z : float
        Ball rotational velocities
    """
    phi_x = (2 / 3) * (RW / RK) * (psi2 - psi3)
    phi_y = (np.sqrt(2) / 3) * (RW / RK) * (-2 * psi1 + psi2 + psi3)
    phi_z = (np.sqrt(2) / 3) * (RW / RK) * (psi1 + psi2 + psi3)
    
    return phi_x, phi_y, phi_z


def compute_phi(psi_1_counts, psi_2_counts, psi_3_counts):
    '''
    Parameters:
    ----------
    psi_1_counts: Encoder counts [MOTOR 1]
    psi_2_counts: Encoder counts [MOTOR 2]
    psi_3_counts: Encoder counts [MOTOR 3]

    Returns:
    --------
    phi_x: Ball rotation along x-axis (rad)
    phi_y: Ball rotation along y-axis (rad)
    phi_z: Ball rotation along z-axis (rad)
    '''

    # Converting counts to rad
    psi = (1/713.0141) * np.array([[psi_1_counts], [psi_2_counts], [psi_3_counts]])

    x = 0.323899 * psi[1] - 0.323899 * psi[2]
    y = -0.374007 * psi[0] + 0.187003 * psi[1] + 0.187003 * psi[2]
    z = 0.187003 * psi[0] + 0.187003 * psi[1] + 0.187003 * psi[2]

    return x, y, z

# ---------------------------------------------------------------------------
#inverse of the compute_phi function:
def compute_encoder_counts(phi_x, phi_y, phi_z):
    '''
    Parameters:
    ----------
    phi_x: Ball rotation along x-axis (rad)
    phi_y: Ball rotation along y-axis (rad)
    phi_z: Ball rotation along z-axis (rad)

    Returns:
    --------
    psi_1_counts: Encoder counts [MOTOR 1]
    psi_2_counts: Encoder counts [MOTOR 2]
    psi_3_counts: Encoder counts [MOTOR 3]
    '''
    
    # Transformation matrix from encoder counts to ball rotation
    A = np.array([
        [0, 0.323899, -0.323899],
        [-0.374007, 0.187003, 0.187003],
        [0.187003, 0.187003, 0.187003]
    ])
    
    # Invert the matrix
    A_inv = np.linalg.inv(A)

    # Desired ball rotation
    phi = np.array([[phi_x], [phi_y], [phi_z]])

    # Calculate psi (in radians)
    psi = A_inv @ phi

    # Convert psi to encoder counts
    counts = 713.0141 * psi

    return counts.flatten()

def rad_2_counts(psi_rad):
    '''
    Parameters:
    ----------
    psi_rad: Wheel rotation in radians

    Returns:
    --------
    psi_counts: Wheel rotation in counts
    '''
    # YOUR
    # CODE 
    # GOES 
    # HERE
    
    psi_counts = psi_rad

    return psi_counts

# ---------------------------------------------------------------------------

if __name__ == "__main__":
    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Test%i' % trial_num
    dl = dataLogger(filename + '.txt')

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    theta_roll = 0.0
    theta_pitch = 0.0

    psi_1 = 0.0
    psi_2 = 0.0
    psi_3 = 0.0

    phi_x = 0.0
    phi_y = 0.0
    phi_z = 0.0

    # Motor torques
    T1 = 0.0
    T2 = 0.0
    T3 = 0.0

    commands['start'] = 1.0

    # Time for comms to sync
    time.sleep(1.0)

    ser_dev.send_topic_data(101, commands)

    print('Beginning program!')
    i = 0

    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            states = ser_dev.get_cur_topic_data(121)[0]
            if i == 0:
                t_start = time.time()
            i = i + 1
        except KeyError as e:
            continue
        t_now = time.time() - t_start

        # Define variables for saving / analysis here - below you can create variables from the available states
        
        # Body lean angles
        theta_x = (states['theta_roll'])  
        theta_y = (states['theta_pitch'])

        # Motor rotations
        psi_1 = states['psi_1']
        psi_2 = states['psi_2']
        psi_3 = states['psi_3']
        psi = [psi_1, psi_2, psi_3]

        # ---------------------------------------------------------
        # Compute motor torques (T1, T2, and T3) with Tx, Ty, and Tz

        # Beginning with torque rolling toward positive y-axis
        # CHANGE THESE TO ADJUST THE ROLLING DIRECTION OF YOUR BALL-BOT
        Tx = 2
        Ty = 0
        Tz = 0

        #Tx = 2*(1/2)
        #Ty = 2*np.sqrt(3)/2
        #Tz = 0

        # YOUR
        # CODE 
        # GOES 
        # HERE
        T1 = (1 / 3) * (Tz - (2 * Ty) / cos_alpha)
        T2 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (-np.sqrt(3) * Tx + Ty))
        T3 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (np.sqrt(3) * Tx + Ty))

        # ---------------------------------------------------------

        print("T1: {}, T2: {}, T3: {}".format(T1, T2, T3))
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3  

        # ---------------------------------------------------------
        # Compute ball rotation (phi) with psi_1, psi_2, and psi_3

        # YOUR
        # CODE 
        # GOES 
        # HERE
        x = 0.323899 * psi[1] - 0.323899 * psi[2]
        y = -0.374007 * psi[0] + 0.187003 * psi[1] + 0.187003 * psi[2]
        z = 0.187003 * psi[0] + 0.187003 * psi[1] + 0.187003 * psi[2]

        # ---------------------------------------------


        # Construct the data matrix for saving - you can add more variables by replicating the format below
        data = [i] + [t_now] + [theta_x] + [theta_y] + [T1] + [T2] + [T3] + [phi_x] + [phi_y] + [phi_z] + [psi_1] + [psi_2] + [psi_3]
        dl.appendData(data)

        print("PHI X: {}, PHI Y: {}, PHI Z: {}".format(phi_x, phi_y, phi_z))
        ser_dev.send_topic_data(101, commands)
    
    print("Saving data...")
    dl.writeOut()

    print("Resetting Motor commands.")
    time.sleep(0.25)
    commands['start'] = 0.0
    time.sleep(0.25)
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    time.sleep(0.25)
    ser_dev.send_topic_data(101, commands)