"""
Lighting Cues for AprilTag Navigation

This module is written to be run simultaneously with '_DEMO_apriltag_navigation.py'. 
On most devices, this module must be run using with "sudo" as the neopixel library 
requires it. The module is designed to read in velocity values of the robot and 
program the lights accordingly. When the robot is searching for the AprilTag, the 
lights will turn red. Conversely, when the robot has located the AprilTag, the lights 
will change to green

Authors: Anuhea Tao, Reina Mezher, Adam Hung, Joseph Fedoronko
Date: May 2025
"""

import board, neopixel, time
WIDTH, HEIGHT = 33, 8
NUMPIX = WIDTH*HEIGHT
PIN = board.D18
BRIGHTNESS = .2
ORDER = neopixel.GRB
import threading
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
from DataLogger import dataLogger  # Import data logger for logging data
import numpy as np
import matplotlib.pyplot as plt
from ballbot_kinematics import compute_phi

dt = 0.1
# small momentum = most of the velocity term comes from the current dq
momentum = 0.1
prev_q = np.array([0,0,0])
running_v = np.array([0,0,0])

# Constants for the control loop
FREQ = 50  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds


pixels = neopixel.NeoPixel(
    PIN, NUMPIX,
    brightness=BRIGHTNESS,
    auto_write=False,
    pixel_order=ORDER
)

def set_pixel_line(pixel1, pixel2, how_bright=None):
    pixel2 -= 1
    if how_bright is not None:
        pixels.brightness = how_bright
    start = min(pixel1, pixel2)
    end = max(pixel1, pixel2)
    for i in range(start, end + 1):
        pixels[i] = (0, 255, 0) 
        # time.sleep(.1)

    pixels.show()
    time.sleep(0.1)


def wipe(color, delay=0.05):
    """Light LEDs one by one in 'color', with a short delay between each."""
    for i in range(NUMPIX):
        pixels[i] = color
        pixels.show()
        time.sleep(delay)

def wipe_fwd(color, delay=0.05):
    """Light LEDs one by one in 'color', with a short delay between each."""
    for i in range(64):
        pixels[i] = color
        pixels.show()
        time.sleep(delay)
def wipe_right(color, delay=0.05):
    for i in range(65, 128):
        pixels[i] = color
        pixels.show()
        time.sleep(delay)
def wipe_back(color, delay=0.05):
    for i in range(128, 192):
        pixels[i] = color
        pixels.show()
        time.sleep(delay)
def wipe_left(color, delay=0.05):
    for i in range(200, NUMPIX):
        pixels[i] = color
        pixels.show()
        time.sleep(delay)



# pixels.fill((0,0,0))
# pixels.show()

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

    # var for i control
    error_x_sum = 0
    error_y_sum = 0

    # var for d control
    error_x_prev = 0
    error_y_prev = 0

    t_start = time.time()
    timer = t_start
    # pixels.fill((0,0,0))
    # pixels.show()
    EPS = .0058#.005
    EPS2 = .001

    detected = False 
    count = 0

    T1p, T2p, T3p = 1, 1, 1

    
    for t in SoftRealtimeLoop(dt=DT, report=True):
            # pixels.fill((0,0,0))
            # pixels.show()
            try:
                # Read sensor data from the robot
                states = ser_dev.get_cur_topic_data(121)[0]

            except:
                #print("waiting")
                print(time.time() - t_start)
                continue
            
            
            # desired_theta_x = 0
            # desired_theta_y = 0
            theta_x = -(states['theta_roll'])  
            theta_y = (states['theta_pitch'])
            # print(f'roll: {theta_x:.3f},           pitch: {theta_y:.3f}')

            # angular position of ball
            dpsi_1 = states['dpsi_1']
            dpsi_2 = states['dpsi_2']
            dpsi_3 = states['dpsi_3']
            
            x,y,z = compute_phi(dpsi_1, dpsi_2, dpsi_3)
            
            Tx = x
            Ty = y
            Tz = 0

            T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)
            print("t1, t2, t3: ", T1, " ", T2, " ", T3)
            

            # if(Tz >= EPS):
            #     detected = False 
            #     count = 0
            #     pixels.fill((255,0,0))
                
            # else:
            #     detected = True
            #     pixels.fill((0,255,0))
          

            # if(T1p!=T1 or T2p!=T2 or T3p!=T3):
            #     pixels.fill((155,0,100))
            #     pixels.show()

            pixels.fill((0,0,0))

            if(T1 - .00024727 < EPS and T2 + .0049454 < EPS and T3 - .00024727 < EPS ):
                detected = False
                count = 0
                pixels.fill((255,0,0))
                pixels.show()
            else:
                detected = False
                count = 0
                pixels.fill((0,255,0))
                pixels.show()

            T1p, T2p, T3p = T1, T2, T3

            # pixels.show()

            time.sleep(.05)

            # if T1 > T2 and T1 > T3: 
            #     pixels.fill((155,0,100))
            #     pixels.show()


            # pixels.fill((155,0,100))
            # pixels.show()

            # time.sleep(2)

            # pixels.fill((0,0,0))
            # pixels.show()

            # while True:
            #     wipe_right((200,0,55))
            #     pixels.show()
            #     wipe_right((0,0,0))


            # print("tx: ", Tx)
            # print("ty: ", Ty)

            # rad = np.arctan2(Ty, Tx)
            # norm = (rad + (np.pi/2))/(np.pi) # need to check bounds

            # norm = norm * 33
            # norm = np.floor(norm) #rounds to nearest int

            # #choose lights
            # high_num_pix = int(norm * 8)
            # low_num_pix = high_num_pix - 8

            # print("low:", low_num_pix)
            # print("high:", high_num_pix)
            # rgb = (0, 255, 0)
            # set_pixel_line(low_num_pix, high_num_pix, how_bright=None)
