import RPi.GPIO as GPIO
import time
import threading

GPIO.setmode(GPIO.BCM)

SERVO_PIN = 23
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_servo_angle_nonblocking(angle):
    """Set servo angle and stop after short delay (non-blocking)"""
    angle = max(0, min(180, angle))
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    # Schedule a duty cycle reset in 0.5s without blocking
    threading.Timer(0.5, lambda: pwm.ChangeDutyCycle(0)).start()

# Example loop that updates angle every second
try:
    angle = 0
    while True:
        set_servo_angle_nonblocking(angle)
        angle += 15
        if angle > 180:
            angle = 0
        # Do other things here while servo is moving
        print(f"Angle set to {angle}")
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()


# def rotate_camera(error) -> None:
#     """
#     error_px       : (tag_x − centre_x).  +ve ⇒ tag is RIGHT of centre
#     half_frame_px  : width // 2   (for normalising)
#     """
#     if error is None:
#         pwm.ChangeDutyCycle(7.0 )#7.10)        # no tag → stop
#         return

#     # normalise to −1 … +1
#     err_norm = np.clip(error / 800, -0.2, 0.2)

#     # small error → dead‑zone, keeps camera quiet
#     if abs(err_norm) < DEAD_ZONE:
#         pwm.ChangeDutyCycle(7.0)
#         return

#     duty = 7.0 + err_norm * MAX_OFFSET   # signed speed control
#     pwm.ChangeDutyCycle(duty)