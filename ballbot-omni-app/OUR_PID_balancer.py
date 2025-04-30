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
dt = 0.1
# small momentum = most of the velocity term comes from the current dq
momentum = 0.1
prev_q = np.array([0,0,0])
running_v = np.array([0,0,0])

# Constants for the control loop
FREQ = 50  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds

KP_THETA_X = 2 #10.0       #18#18 #10.0  #36 #48 # Proportional gain for roll stability
KP_THETA_Y =  KP_THETA_X#12.05#18#18 #10 #10.0  #36 #48 # Proportional gain for pitch stability

KI_THETA_X = .3#.8         #1.07 #1.0 #6.8#1.8 #7.0 #3.50  #12.0  # Integral gain for roll stability
KI_THETA_Y = KI_THETA_X#1.0#6.8#1.8 #7.0 #3.50  #12.0  # Integral gain for pitch stability

KD_THETA_X = 0#.7          #.64#.6 #10 # Derivative gain for roll stability
KD_THETA_Y = KD_THETA_X  # Derivative gain for pitch stability

MAX_PLANAR_DUTY = 1.0

#Plot T1 torque
t1_log = []
time_log = []

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

    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Read sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]

        except:
            #print("waiting")
            print(time.time() - t_start)
            continue
        

        if time.time() - t_start <= 15.0:
            print("put ball bot on ball")
            continue
        
        desired_theta_x = 0
        desired_theta_y = 0
        theta_x = -(states['theta_roll'])  
        theta_y = (states['theta_pitch'])
        # print(f'roll: {theta_x:.3f},           pitch: {theta_y:.3f}')

        # angular position of ball
        dpsi_1 = states['dpsi_1']
        dpsi_2 = states['dpsi_2']
        dpsi_3 = states['dpsi_3']
        
        x,y,z = compute_phi(dpsi_1, dpsi_2, dpsi_3)
        dq = np.array([x,y,z]).reshape(-1)
        # dq = (cur_q - prev_q)
        running_v = momentum*running_v + (1-momentum) * dq
        # prev_q = cur_q

        np.set_printoptions(precision=2, suppress=True)
        if time.time() - timer > 1.0:
            timer = time.time()
            # print("cur_q: ", cur_q)
            print("dq: ", dq)
            print("running_v: ", running_v)
        
        #Tx, Ty = compute_motor_torques(theta_x,theta_y)
        Tz = 0
        

        # Proportional control
        error_x = desired_theta_x - theta_x
        error_y = desired_theta_y - theta_y

        error_x_sum += error_x
        error_y_sum += error_y

        Tx = KP_THETA_X * error_x
        Ty = KP_THETA_Y * error_y

        # Integral control
        Tx += KI_THETA_X * error_x_sum
        Ty += KI_THETA_Y * error_y_sum

        # reset the error sum if it is near zero
        if abs(error_x) < 0.001:
            error_x_sum = 0
        if abs(error_y) < 0.001:
            error_y_sum = 0

        # ciel it
        error_x_sum = min(error_x_sum, MAX_PLANAR_DUTY/KI_THETA_X)
        error_y_sum = min(error_y_sum, MAX_PLANAR_DUTY/KI_THETA_Y)

        # Derivative control
        Tx += KD_THETA_X * (error_x - error_x_prev)
        Ty += KD_THETA_Y * (error_y - error_y_prev)

        # print(f'P_x: {KP_THETA_X * error_x:.3f},           P_y: {KP_THETA_Y * error_y:.3f}')

        # print(f'I_x: {KI_THETA_X * error_x_sum:.3f},           I_y: {KI_THETA_Y * error_y_sum:.3f}')

        # print(f'D_x: {KD_THETA_X * (error_x - error_x_prev):.3f},           D_y: {KD_THETA_Y * (error_y - error_y_prev):.3f}')



        # Saturate torques to prevent excessive output
        Tx = np.clip(Tx, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)
        Ty = np.clip(Ty, -MAX_PLANAR_DUTY, MAX_PLANAR_DUTY)

        Tz = 0

        # ---------------------------------------------------------------


        # === Torque Computation ===
        # Compute motor torques using imported function
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz) # From previous labs!

        if max(T1, T2, T3) > MAX_PLANAR_DUTY:
            print("over the max torque")

        #skip torques that are too high 
        if abs(T1) > MAX_PLANAR_DUTY or abs(T2) > MAX_PLANAR_DUTY or abs(T3) > MAX_PLANAR_DUTY:
            continue

        #Plot T1 torque
        t1_log.append(T1*2.65)
        time_log.append(time.time() - t_start)

        # Send computed torques to the robot
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands)

        # set for d control
        error_x_prev = error_x
        error_y_prev = error_y

        


    #PLot T1 torque 
    plt.figure()
    plt.plot(time_log, t1_log)
    plt.title("Motor 1 Torque (T1) Over Time")
    plt.xlabel("Time [s]")
    plt.ylabel("T1 Torque (Nm)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("T1_plot.png")  # Saves the plot in the current directory
    plt.close()

    print("Shutting down motors...")
    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)


    
