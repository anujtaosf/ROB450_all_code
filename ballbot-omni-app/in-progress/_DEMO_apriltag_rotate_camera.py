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
    # IMU STUFF

    # ser_dev = SerialProtocol()
    # ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    # ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    
    # # Start a separate thread for reading serial data
    # serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    # serial_read_thread.start()

    # # === Command and State Structures ===
    # # Define command structure for controlling motors
    # commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    # commands['start'] = 1.0  # Activate motors

    # # Initialize state structure for reading sensors
    # states = np.zeros(1, dtype=mo_states_dtype)[0]

    # # Allow communication to sync
    # time.sleep(1.0)
    # ser_dev.send_topic_data(101, commands)

    # print("calibrating...")
    ###################################################################################################

    # initial_yaw = None
    for t in SoftRealtimeLoop(dt=DT, report=True):

        ###################################################################################################
        # CALIBRATE IMU

        # try:
        #     states = ser_dev.get_cur_topic_data(121)[0]
        # except:
        #     print(time.time() - t_start)
        #     continue
        # if time.time() - t_start <= 12.0:
        #     print("wait")
        #     initial_yaw = states['theta_yaw']
        #     continue

        # yaw = (states['theta_yaw']) - initial_yaw
        # print("yaw: ", yaw)
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
            print("No tag detected")
        elif len(tags) > 1:
            rel_x = None
            depth = None
            print("Multiple tags detected")
        else:
            tag = tags[0]
            corners = tag.corners
            center_x, center_y = int(tag.center[0]), int(tag.center[1])
            depth = depth_frame.get_distance(center_x, center_y)
            rel_x = center_x - frame_center_x
            print("Tag detected at x: ", rel_x, " , at distance: ", depth)

        april_error = rel_x
        rotate_camera(error = april_error)
        print(camera_angle)

        ###############################################################################################

        # ____________________________________________________________________________
        # Get XYZ Torques
        # ____________________________________________________________________________
        rot_speed = 0.2
        forward_speed = 0.15 #0.2
        april_threshold = 100
        goal = 0.3
        
        if not depth or depth < goal:
            if depth:
                print("reached target")
            Tx = 0.0
            Ty = 0.0
            Tz = 0.0

            x,y = get_camera_heading()
            Tx = x * forward_speed
            Ty = y * forward_speed

        elif abs(april_error) < april_threshold:
            
            x,y = get_camera_heading()
            Tx = x * forward_speed
            Ty = y * forward_speed
            Tz = 0.0

        
        # print(f'Tx: {Tx:.3f},           Ty: {Ty:.3f},           Tz: {Tz:.3f}')

        # print(f'P_x: {KP_THETA_X * error_x:.3f},           P_y: {KP_THETA_Y * error_y:.3f}')
        # print(f'D_x: {KD_THETA_X * (error_x - error_x_prev):.3f},           D_y: {KD_THETA_Y * (error_y - error_y_prev):.3f}')

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
    


    
