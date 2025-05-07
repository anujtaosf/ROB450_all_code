"""
April Tag Navigation for ShortBot

This module implements the AprilTag navigation on ShortBot using the Intel 
RealSense camera module and the apriltag module. The robot will navigate the 
to AprilTag by rotating about the z-axis until it has located an AprilTag 
within its field of view. Once the AprilTag is located, ShortBot will drive 
towards it while keeping the tag centered in its camera frame.

**This module can be run in simultaneity with '_DEMO_apriltag_lights.py' for
LED lights to act as visual cues

Authors: Anuhea Tao, Reina Mezher, Adam Hung, Joseph Fedoronko
Date: May 2025
"""

import threading
import time
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
from DataLogger import dataLogger  # Import data logger for logging data
import numpy as np
import matplotlib.pyplot as plt
from ballbot_kinematics import compute_phi
import pyrealsense2 as rs2
import cv2
import numpy as np
import apriltag
import RPi.GPIO as GPIO

# SERVO STUFF ####################################################################################

camera_angle = 90.0
FREQ = 50  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds
MAX_PLANAR_DUTY = 1.0


GPIO.setmode(GPIO.BCM)
SERVO_PIN = 23
GPIO.setup(SERVO_PIN, GPIO.OUT)

# 50 Hz software PWM
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(7.5)                 # start STOPped

# Tunables -------------------------------------------------------------
MAX_OFFSET = 2.0               # % duty‑cycle at |full error|  →  about 80 % of max speed
DEAD_ZONE  = 5              # normalised error below this => hold still (≈5 % of half‑frame)
Kp = .005
# ----------------------------------------------------------------------

def rotate_camera(error):
    global camera_angle
    if error is None:
        camera_angle += Kp * 200
    elif abs(error) < DEAD_ZONE:
        pass
    elif error > 0:
        camera_angle += Kp * error
    elif error < 0:
        camera_angle -= Kp * error
    
    camera_angle = camera_angle % 180

    # print(f"Camera angle: {camera_angle:.2f} degrees")
    duty = 2 + (camera_angle / 18)
    pwm.ChangeDutyCycle(duty)
    threading.Timer(0.2, lambda: pwm.ChangeDutyCycle(0)).start()

def get_camera_heading():
    x = np.sin(camera_angle * np.pi / 180)
    y = np.cos(camera_angle * np.pi / 180)
    return x,y
    

if __name__ == "__main__":

    print("starting main")
    
    # === Serial Communication Initialization ===
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands['start'] = 1.0  # Activate motors
    states = np.zeros(1, dtype=mo_states_dtype)[0]
    # Allow communication to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)
    # === Main Control Loop ===

    i = 0  # Iteration counter
    t_start = time.time()
    timer = t_start
        
    # CAMERA STUFF ####################################################################################
    rel_x = None
    depth = None

    pipe = rs2.pipeline()
    cfg = rs2.config()
    cfg.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
    cfg.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    profile = pipe.start(cfg)
    align_to = rs2.stream.color
    align = rs2.align(align_to)
    colorizer = rs2.colorizer()
    options = apriltag.DetectorOptions(families='tag36h11')
    detector = apriltag.Detector(options)
    for _ in range(5):
        pipe.wait_for_frames() 

    ###################################################################################################

    for t in SoftRealtimeLoop(dt=DT, report=True):
        # CAMERA STUFF ################################################################################

        frames = pipe.wait_for_frames()
        frames = align.process(frames)  # align depth to color
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not depth_frame or not color_frame:
            continue
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        depth_colored = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        height, width = gray.shape[:2]
        frame_center_x = width // 2
        frame_center_y = height // 2
        # Detect AprilTags
        tags = detector.detect(gray)
        
        if not tags:
            rel_x = None
            depth = None
            print("no tag detected")
        elif len(tags) > 1:
            rel_x = None
            depth = None
            print("multiple tags detected")
        else:
            tag = tags[0]
            corners = tag.corners
            center_x, center_y = int(tag.center[0]), int(tag.center[1])
            depth = depth_frame.get_distance(center_x, center_y)
            rel_x = center_x - frame_center_x
            rel_y = center_y - frame_center_y
            print("tag detected at x: ", rel_x, " , depth: ", depth)

        april_error = rel_x

        rot_speed = 0.2
        forward_speed = 0.2
        april_threshold = 100
        goal = 0.7
        
        if depth and depth < goal:
            print("reached target")
            Tx = 0.0
            Ty = 0.0
            Tz = 0.0
        elif rel_x:
            
            sign = -1 * april_error / abs(april_error) if april_error != 0 else 0
            if abs(april_error) <= april_threshold:
                Tx = 1* forward_speed
                Ty = 0.0
                Tz = 0.0
            else:
                Tx = 0.0
                Ty = 0.0
                Tz =  sign * rot_speed #max(min(0.2, april_error * Kp), 0.1)
                print(Tz)
        else:
            Tx = 0.0
            Ty = 0.0
            Tz = rot_speed

        # Saturate torques to prevent excessive output
        Tx = np.clip(Tx, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
        Ty = np.clip(Ty, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
        Tz = np.clip(Tz, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
        
        # ------------------------------------------------------------------------

        # === Torque Computation ===
        # Compute motor torques using imported function
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz) # From previous labs!

        if max(T1, T2, T3) > MAX_PLANAR_DUTY:
            print("over the max torque")

        #skip torques that are too high 
        if abs(T1) > MAX_PLANAR_DUTY or abs(T2) > MAX_PLANAR_DUTY or abs(T3) > MAX_PLANAR_DUTY:
            continue

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
    


    
