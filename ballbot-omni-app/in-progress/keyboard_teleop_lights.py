import threading
import time
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
import numpy as np
import keyboard

# TODO: 
# add + - for changing the speed

# ---- keyboard helper -------------------------------------------------------
# Requires: pip install keyboard   (run with sudo on Linux so it can read /dev/input)
import keyboard    # cross‑platform, non‑blocking
import sys, termios, tty, select, threading, time

import neopixel
import board
from LED_controls import wipe_fwd, wipe_back, wipe_left, wipe_right, NUMPIX

# LED STUFF ####################################################################################
pixel_pin = board.D18
num_pixels = 256
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.05, auto_write=False, pixel_order=ORDER
)

def set_pixel_line(pixel1, pixel2, how_bright=None):
    if how_bright is not None:
        pixels.brightness = how_bright
    start = pixel1
    end = pixel2 - 1
    for i in range(start, end):
        pixels[i] = (0, 255, 0) 

    pixels.show()
    time.sleep(0.1)

HOLD_TIMEOUT = 0.15         # seconds to keep key "held" after last char

_key_state     = {"w": 0, "a": 0, "s": 0, "d": 0}
_last_char_time = {k: 0.0 for k in _key_state}

_key_state.update({"=": 0, "-": 0})
_last_char_time.update({"=": 0.0, "-": 0.0})

def _stdin_listener():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)                          # raw = immediate single‑byte reads
    try:
        while True:
            r, _, _ = select.select([fd], [], [], 0.01)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()  # make upper/lower identical
            if ch in _key_state:
                _key_state[ch]      = 1
                _last_char_time[ch] = time.time()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

threading.Thread(target=_stdin_listener, daemon=True).start()
# ---------------------------------------------------------------------------

def get_keyboard():
    """
    Return (right, left, up, down) as 0/1 derived from d, a, w, s.
    Value stays 1 while the key is held (bytes keep arriving) and
    flips to 0 after HOLD_TIMEOUT seconds of silence.
    """
    now = time.time()
    for k in _key_state:
        if _key_state[k] and (now - _last_char_time[k] > HOLD_TIMEOUT):
            _key_state[k] = 0
    right = _key_state["d"]
    left  = _key_state["a"]
    up    = _key_state["w"]
    down  = _key_state["s"]
    return right, left, up, down

# Constants for the control loop
FREQ = 200  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds

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

    print("Starting steering control loop...")

    # === Main Control Loop ===
    i = 0  # Iteration counter
    for t in SoftRealtimeLoop(dt=DT, report=True): 


        # === Controller Input ===
        right, left, up, down = get_keyboard()
        # any_key_active = False
        # print(f"right: {right}, left: {left}, up: {up}, down: {down}")
        if right:
            # wipe
            pixels.fill((0,0,0))
            wipe_right((200, 0, 55))
            #time.sleep(.05)
            # any_key_active = True
            # set_pixel_line(65, 128, how_bright=None)
        if left:
            pixels.fill((0,0,0))
            wipe_left((200, 0, 55))
            #time.sleep(.05)
            # any_key_active = True
            # set_pixel_line(200, NUMPIX, how_bright=None)
        
        if up: 
            pixels.fill((0,0,0))
            wipe_fwd((200, 0, 55))
            #time.sleep(.05)
            # any_key_active = True
            # set_pixel_line(0, 64, how_bright=None)

        if down:
            pixels.fill((0,0,0))
            wipe_back((200, 0, 55))
            #time.sleep(.05)
            # any_key_active = True
            # set_pixel_line(128, 192, how_bright=None

        time.sleep(.05)
        pixels.show()