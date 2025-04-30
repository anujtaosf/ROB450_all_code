"""
ROB311 Bluetooth Controller

This script initializes the PS4 controller using the `PS4InputHandler` class and demonstrates 
how to use it in a runtime environment. The script logs input signals from the controller 
in real-time for testing and debugging purposes.

Key Features:
- Initializes the PS4 controller.
- Runs the controller in a separate thread for asynchronous input handling.
- Logs inputs from thumbsticks, triggers, and shoulder buttons in real-time.

Example Usage:
1. Run this script directly to test PS4 controller inputs:
    python rob311_bt_controller.py
2. Use the logged inputs for debugging or integrating with robotics systems.

Author: Yilin Ma 
Date: January 2025
"""

import threading
import time
from ps4_controller_api import PS4InputHandler
from loop import SoftRealtimeLoop  # Replace or implement your timing loop if unavailable

# Set the frequency and time step for the control loop
FREQ = 200  # Hz
DT = 1 / FREQ  # Time step in seconds

if __name__ == "__main__":
    # Initialize the PS4 controller
    controller = PS4InputHandler(interface="/dev/input/js0")

    # Start the controller in a separate thread
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Ensures the thread stops with the main program
    controller_thread.start()

    print("PS4 Controller is running. Move the thumbsticks, press triggers, and shoulder buttons to see their effects.")

    # Real-time loop to read and log controller signals
    try:
        for t in SoftRealtimeLoop(dt=DT, report=True):
            # Get the current state of all controller signals
            signals = controller.get_signals()

            # Log the signals in a formatted string
            print(
                f"Left Thumbstick: X={signals['left_thumbstick_x']:.2f}, Y={signals['left_thumbstick_y']:.2f} | "
                f"Right Thumbstick: X={signals['right_thumbstick_x']:.2f}, Y={signals['right_thumbstick_y']:.2f} | "
                f"Triggers: L2={signals['trigger_L2']:.2f}, R2={signals['trigger_R2']:.2f} | "
                f"Shoulder Buttons: L1={signals['shoulder_L1']}, R1={signals['shoulder_R1']}"
            )

            # Simulate control loop delay
            time.sleep(0.05)

    except KeyboardInterrupt:
        # Gracefully handle a user interrupt (Ctrl+C)
        print("Exiting the program.")


#98:B6:E9:C