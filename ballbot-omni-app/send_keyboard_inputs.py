import socket
import keyboard

# Raspberry Pi's IP address (change to your Pi's actual IP)
RPI_IP = "35.3.53.84"  # Change this!
PORT = 5005  # UDP port

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Sending keyboard inputs to Raspberry Pi. Press ESC to exit.")

while True:
    try:
        # Define control signals
        signals = {
            "left_thumbstick_x": 0.0,  # Left/Right movement
            "left_thumbstick_y": 0.0,  # Forward/Backward movement
            "trigger_L2": 0.0,  # Rotate Left
            "trigger_R2": 0.0,  # Rotate Right
        }

        # Capture keyboard inputs
        if keyboard.is_pressed("a"):
            signals["left_thumbstick_x"] = -1.0
        elif keyboard.is_pressed("d"):
            signals["left_thumbstick_x"] = 1.0

        if keyboard.is_pressed("w"):
            signals["left_thumbstick_y"] = 1.0
        elif keyboard.is_pressed("s"):
            signals["left_thumbstick_y"] = -1.0

        if keyboard.is_pressed("q"):
            signals["trigger_L2"] = 1.0
        if keyboard.is_pressed("e"):
            signals["trigger_R2"] = 1.0

        # Send as a string
        message = f"{signals['left_thumbstick_x']},{signals['left_thumbstick_y']},{signals['trigger_L2']},{signals['trigger_R2']}"
        sock.sendto(message.encode(), (RPI_IP, PORT))

    except KeyboardInterrupt:
        print("\nStopping keyboard sender.")
        break