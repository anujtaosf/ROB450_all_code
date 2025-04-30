import keyboard

# Example of sending a key press
print("Press 'ESC' to exit the script.")
while True:
    if keyboard.is_pressed('esc'):
        print("ESC pressed, exiting.")
        break