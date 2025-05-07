# About this directory
This folder contains working demo scripts, helper scripts, and subdirectories for previous modules.

# Running Demos
Each demo in this folder has specific instructions on how to run it.

### April Tag Navigation
This module requires these libraries that are not already installed in the setup script:

1. Realsense (we used an Intel RealSense D415)  
   [Getting Started with Intel RealSense](https://www.intelrealsense.com/get-started-depth-camera/)

2. apriltag
```bash
pip install apriltag
```

Once installed, run the AprilTag navigation demo as a normal Python script:
```bash
python _DEMO_apriltag_navigation.py
```

### Adding LEDs to AprilTag Navigation
To add LEDs, one more library needs to be installed:
```bash
sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel
```

Also ensure that the hardware is connected properly. In the default script, we use GPIO pin 18 on the Raspberry Pi,  
but the LED strip can be connected to any pin as long as it is updated in the code.

Once installed, run the AprilTag navigation demo as shown above. Then, using a **SEPARATE** terminal, run the apriltag lights script with `sudo`:
```bash
sudo python _DEMO_apriltag_lights.py
```

### Teleoperating Shortbot
To teleoperate the robot, the `keyboard` module needs to be installed on both the local device and the Raspberry Pi.  
Run the command below both locally **and** while SSH'd into the Pi:
```bash
pip install keyboard
```

Once installed, run the keyboard demo and use the `a`, `s`, `w`, `d` keys to control ShortBot:
```bash
python _DEMO_keyboard_teleop.py
```

# Subdirectories
This folder also contains subdirectories created during earlier iterations of Shortbot. These include experiments for hardware integration, test scripts, and ongoing development work that is not yet ready for demo.

### `in-progress`
The `in-progress` subdirectory contains scripts that represent partial implementations or work-in-progress ideas:

1. `_DEMO_apriltag_rotate_camera.py`  
   This script was created during an attempt to integrate a servo motor with the RealSense camera. For ShortBot to become fully omnidirectional,
   the camera must not be limited to a single axis of vision. This script introduces a servo to the navigation module, enabling Shortbot to
   achieve a 360-degree field of view. This would help reduce time and energy spent rotating about the z-axis.

2. `keyboard_teleop_lights.py`  
   This script is v2 of the existing keyboard teleoperation demo. It aims to integrate the LED matrix into the current teleop script,
    allowing for visual feedback while controlling Shortbot using the keyboard.

### `component-tests`
`component-tests` contains scripts to test individual components while debugging.

### `balancer-code`
`balancer-code` contains the control logic and functions used in an attempt to balance ShortBot on a basketball. 
The PID parameters were tuned specifically for our setup but should be adjusted for a new platform.

### `LED-functions`
`LED-functions` includes modules and functions that were found on open source platforms and were used to test our LED matrix.

### `ps4-controller`
`ps4-controller` contains scripts and helper scripts for integrating a ps4 controller into the codebase and using the triggers to teleoperate the ballbot. 

These scripts are not included in the main demos because we encountered Bluetooth connection issues when the PS4 controller was used simultaneously with the RealSense camera. 
However, this limitation may be specific to our hardware setup. The scripts were tested and verified to work independently before this issue arose.
