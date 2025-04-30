#script for motor control to april tag based on the video feed
import numpy as np
import threading
import time
import cv2
import pyrealsense2 as r
from ballbot_kinematics import compute_phi
from OUR_PID_balancer import compute_motor_torques
from MBot.SerialProtocol.protocol import SerialProtocol
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from pupil_apriltags import Detector


# --- Constants ---
K_LINEAR = 0.5    # Gain for linear velocity
K_ANGULAR = 1.0   # Gain for angular velocity
DISTANCE_THRESHOLD = 0.2  # Stop threshold [meters]

# Assume known intrinsics (you can get these from RealSense intrinsics object)
# This is given from the realsense script
CAMERA_PARAMS = (600, 600, 320, 240)  # fx, fy, cx, cy

# --- AprilTag Detector Setup ---
# might need to change the family?
tag_detector = Detector(families='tag36h11')

# Global storage for error tracking
x_error_log = []
y_error_log = []
position_log = []
prev_psi = np.array([0.0, 0.0, 0.0])  # Initialize previous psi values

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

dt = 0.1
momentum = 0.1
prev_q = np.array([0,0,0])
running_v = np.array([0,0,0])


# function to process the tag and output velocities
def get_tag_pose(pipeline, at_detector, camera_matrix, dist_coeffs):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return None

    color_image = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    tags = at_detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2]),
        tag_size=0.06  # Adjust based on your actual AprilTag size in meters
    )

    if len(tags) == 0:
        return None

    tag = tags[0]
    tvec = tag.pose_t  # Translation vector (x, y, z) in meters
    rvec = tag.pose_R  # Rotation matrix (3x3)

    distance = np.linalg.norm(tvec)
    angle_to_tag = np.arctan2(tvec[0][0], tvec[2][0])  # Left/right deviation

    return {
        "translation": tvec.flatten(),
        "distance": distance,
        "angle": angle_to_tag
    }

def navigate_ballbot(pipeline, at_detector, camera_matrix, dist_coeffs):
    trans, dist, ang = get_tag_pose(pipeline, at_detector, camera_matrix, dist_coeffs)
    # drive in the x direction
    Tx = 0.3  # Move forward in x-direction
    Ty = 0.0
    Tz = 0.0

    T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)

    # Send computed torques to the robot
    commands['motor_1_duty'] = T1
    commands['motor_2_duty'] = T2
    commands['motor_3_duty'] = T3

    # send motor commands
    ser_dev.send_topic_data(101, commands)

    #compute error: 
    psi_1 = states['psi_1']
    psi_2 = states['psi_2']
    psi_3 = states['psi_3']

    # Assume desired trajectory is along +x, so y_desired = 0
    # Get current position error
    x_err, y_err = compute_off_course_error()

    # Proportional gain for y-axis correction
    Kp_y = -1.5  # Negative because you want to correct back toward y=0

    # Desired torques
    Tx = 0.3  # Keep driving forward
    Ty = Kp_y * y_err  # Correction in y to bring it back on course
    Tz = 0.0  # No rotation for now

    T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)


def compute_off_course_error():
    global prev_q, running_v

    # Extract current encoder readings
    dpsi_1 = states['dpsi_1']
    dpsi_2 = states['dpsi_2']
    dpsi_3 = states['dpsi_3']

    psi = np.array([dpsi_1, dpsi_2, dpsi_3])
    
    # angular position of ball
    x,y,z = compute_phi(dpsi_1, dpsi_2, dpsi_3)
    cur_q = np.array([x,y,z])
    dv = (cur_q - prev_q) / dt
    running_v = momentum*running_v + (1-momentum) * dv
    prev_q = cur_q

    print(running_v)

    # get this from camera 
    target_v = np.array([1,0,0])
    error_v =  target_v - running_v
    
    return error_v

    
