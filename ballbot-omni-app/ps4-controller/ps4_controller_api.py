"""
PS4 Input Handler API

This module provides the `PS4InputHandler` class to interface with a PS4 controller. 
It captures inputs from the left and right thumbsticks, triggers, and shoulder buttons, 
and processes them into normalized values suitable for use in robotics or control systems.

Key Features:
- **Left Thumbstick**:
  - Horizontal (`left_thumbstick_x`): For lateral movement or strafe control.
  - Vertical (`left_thumbstick_y`): For forward/backward movement.
- **Right Thumbstick**:
  - Horizontal (`right_thumbstick_x`): For yaw or rotational control.
  - Vertical (`right_thumbstick_y`): For pitch or tilt control.
- **Triggers**:
  - Left Trigger (`trigger_L2`): E.g., braking or reducing thrust.
  - Right Trigger (`trigger_R2`): E.g., acceleration or increasing thrust.
- **Shoulder Buttons**:
  - Left Shoulder (`shoulder_L1`): Discrete decrement (e.g., reduce mode).
  - Right Shoulder (`shoulder_R1`): Discrete increment (e.g., increase mode).

Continuous inputs (thumbsticks and triggers) are normalized to a range of [0.0, 1.0] 
for easy mapping to real-world applications. All signals are stored in 
a dictionary accessible via the `get_signals` method.

Example Usage:
1. Initialize the `PS4InputHandler` with the controller's interface:
    handler = PS4InputHandler(interface="/dev/input/js0")
2. Start the controller's event listener:
    handler.listen(timeout=10)
3. Retrieve real-time control signals:
    signals = handler.get_signals()

Author: Yilin Ma
Date: January 2025
"""

from pyPS4Controller.controller import Controller
import numpy as np
import sys

# Scale factor for normalizing joystick and trigger values
JOYSTICK_SCALE = 32767


class PS4InputHandler(Controller):
    def __init__(self, interface, **kwargs):
        """
        Initialize the PS4 controller and define the control signals.

        :param interface: The interface for the PS4 controller (e.g., "/dev/input/js0").
        """
        super().__init__(interface, **kwargs)
        self.signals = {
            "left_thumbstick_x": 0.0,   # Left thumbstick horizontal (e.g., strafe control)
            "left_thumbstick_y": 0.0,   # Left thumbstick vertical (e.g., forward/backward)
            "right_thumbstick_x": 0.0,  # Right thumbstick horizontal (e.g., yaw control)
            "right_thumbstick_y": 0.0,  # Right thumbstick vertical (e.g., pitch control)
            "trigger_L2": 0.0,          # Left trigger (e.g., brake or reduce thrust)
            "trigger_R2": 0.0,          # Right trigger (e.g., accelerate or increase thrust)
            "shoulder_L1": 0,           # Left shoulder button (e.g., discrete decrement)
            "shoulder_R1": 0,           # Right shoulder button (e.g., discrete increment)
        }

    # === Left Thumbstick (L3) ===
    def on_L3_left(self, value):
        # Update left thumbstick X-axis (horizontal) for leftward movement.
        self.signals["left_thumbstick_x"] = -value / JOYSTICK_SCALE

    def on_L3_right(self, value):
        # Update left thumbstick X-axis (horizontal) for rightward movement.
        self.signals["left_thumbstick_x"] = -value / JOYSTICK_SCALE

    def on_L3_up(self, value):
        # Update left thumbstick Y-axis (vertical) for upward/forward movement.
        self.signals["left_thumbstick_y"] = -value / JOYSTICK_SCALE

    def on_L3_down(self, value):
        # Update left thumbstick Y-axis (vertical) for downward/backward movement.
        self.signals["left_thumbstick_y"] = -value / JOYSTICK_SCALE

    def on_L3_x_at_rest(self):
        # Reset left thumbstick X-axis (horizontal) to neutral position.
        self.signals["left_thumbstick_x"] = 0.0

    def on_L3_y_at_rest(self):
        # Reset left thumbstick Y-axis (vertical) to neutral position.
        self.signals["left_thumbstick_y"] = 0.0

    # === Right Thumbstick (R3) ===
    def on_R3_left(self, value):
        # Update right thumbstick X-axis (horizontal) for leftward yaw control.
        self.signals["right_thumbstick_x"] = -value / JOYSTICK_SCALE

    def on_R3_right(self, value):
        # Update right thumbstick X-axis (horizontal) for rightward yaw control.
        self.signals["right_thumbstick_x"] = -value / JOYSTICK_SCALE

    def on_R3_up(self, value):
        # Update right thumbstick Y-axis (vertical) for upward pitch control.
        self.signals["right_thumbstick_y"] = -value / JOYSTICK_SCALE

    def on_R3_down(self, value):
        # Update right thumbstick Y-axis (vertical) for downward pitch control.
        self.signals["right_thumbstick_y"] = -value / JOYSTICK_SCALE

    def on_R3_x_at_rest(self):
        # Reset right thumbstick X-axis (horizontal) to neutral position.
        self.signals["right_thumbstick_x"] = 0.0

    def on_R3_y_at_rest(self):
        # Reset right thumbstick Y-axis (vertical) to neutral position.
        self.signals["right_thumbstick_y"] = 0.0

    # === Triggers (L2 and R2) ===
    def on_L2_press(self, value):
        # Update right trigger for acceleration or increasing thrust.
        # Normalized to [0.0, 1.0].
        self.signals["trigger_L2"] = (1.0 + np.abs(value / JOYSTICK_SCALE)) / 2.0

    def on_L2_release(self):
        # Reset left trigger value to zero.
        self.signals["trigger_L2"] = 0.0

    def on_R2_press(self, value):
        # Update right trigger for acceleration or increasing thrust.
        # Normalized to [0.0, 1.0].
        self.signals["trigger_R2"] = (1.0 + np.abs(value / JOYSTICK_SCALE)) / 2.0

    def on_R2_release(self):
        # Reset right trigger value to zero.
        self.signals["trigger_R2"] = 0.0

    # === Shoulder Buttons (L1 and R1) ===
    def on_L1_press(self):
        # Update left shoulder button for discrete decrement.
        self.signals["shoulder_L1"] -= 1

    def on_R1_press(self):
        # Update right shoulder button for discrete increment.

        self.signals["shoulder_R1"] += 1

    def on_options_press(self):
        # Exit the PS4 controller thread gracefully.
        print("Exiting PS4 controller thread.")
        sys.exit()

    # === Method to Expose Signals ===
    def get_signals(self):
        # Get the current state of all control signals.
        # return: A dictionary containing the current values of all signals.

        return self.signals
