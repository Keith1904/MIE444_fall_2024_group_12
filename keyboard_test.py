import keyboard
import time

def main():
    print("Press 'w', 'a', 's', or 'd' to test. Press 'q' to quit.")
    
    while True:
        # Check if 'q' is pressed to quit the program
        if keyboard.is_pressed('q'):
            print("Quitting program.")
            break
        
        # Check for each key press
        if keyboard.is_pressed('w'):
            print("Key 'w' is pressed.")
            time.sleep(0.1)  # Short delay to prevent flooding output
        elif keyboard.is_pressed('a'):
            print("Key 'a' is pressed.")
            time.sleep(0.1)
        elif keyboard.is_pressed('s'):
            print("Key 's' is pressed.")
            time.sleep(0.1)
        elif keyboard.is_pressed('d'):
            print("Key 'd' is pressed.")
            time.sleep(0.1)
        else:
            # No key is pressed
            print("No key is pressed.")
            time.sleep(0.5)  # Longer delay when no key is pressed

if __name__ == "__main__":
    main()