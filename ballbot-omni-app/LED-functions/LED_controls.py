import board, neopixel, time
import threading
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
from DataLogger import dataLogger  # Import data logger for logging data
import numpy as np
import matplotlib.pyplot as plt
from ballbot_kinematics import compute_phi

WIDTH, HEIGHT = 33, 8
NUMPIX = WIDTH*HEIGHT
PIN = board.D18
BRIGHTNESS = .2
ORDER = neopixel.GRB


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
    for i in range(NUMPIX):
        pixels[i] = color
        # pixels.show()
        # time.sleep(delay)


def wipe_fwd(color, delay=0.05):
    for i in range(64):
        pixels[i] = color
        # pixels.show()
        # time.sleep(delay)
    # pixels.show()

def wipe_right(color, delay=0.05):
    for i in range(65, 128):
        pixels[i] = color
        # pixels.show()
        # time.sleep(delay)
    # pixels.show()

def wipe_back(color, delay=0.05):
    for i in range(128, 192):
        pixels[i] = color
        # pixels.show()
        # time.sleep(delay)
    # pixels.show()
    
def wipe_left(color, delay=0.05):
    for i in range(200, NUMPIX):
        pixels[i] = color
        # pixels.show()
        # time.sleep(delay)
    # pixels.show()
        