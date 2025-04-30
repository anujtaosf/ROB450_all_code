import board, neopixel, time
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
    if how_bright is not None:
        pixels.brightness = how_bright
    start = min(pixel1, pixel2)
    end = max(pixel1, pixel2)
    for i in range(start, end + 1):
        pixels[i] = (0, 255, 0) 
        # time.sleep(.1)

    pixels.show()
    time.sleep(0.1)

# pixels.fill((155,0,100))
# pixels.show()

# time.sleep(2)

# pixels.fill((0,0,0))
# pixels.show()

# time.sleep(2)

# # for i in range(0,21):

# set_pixel_line(32, 39, .2) #1, 15 is one strip
# pixels.show()

# time.sleep(5)

pixels.fill((0,0,0))
pixels.show()




# import time
# import board
# import pigpio                          # <-- pigpio Python client
# from neopixel import NeoPixel         # from adafruit-circuitpython-neopixel

# # MATRIX CONFIG
# WIDTH, HEIGHT = 21, 8
# NUMPIX        = WIDTH * HEIGHT
# PIN           = board.D18
# BRIGHTNESS    = 0.2

# # connect to pigpio daemon
# pi = pigpio.pi()

# # tell NeoPixel to use pigpio for timing
# pixels = NeoPixel(
#     PIN, NUMPIX,
#     brightness=BRIGHTNESS,
#     auto_write=False,
#     backend=pi
# )

# # quick test
# pixels.fill((155, 0, 100))
# pixels.show()
# time.sleep(2)
# pixels.fill((0, 0, 0))
# pixels.show()

