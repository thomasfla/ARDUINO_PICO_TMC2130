#!/usr/bin/env python3
import serial
import time
from pynput.mouse import Controller

def main():
    # Update the port name and baud rate for your setup.
    port = "/dev/ttyACM0"  
    baud_rate = 115200
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Opened serial port {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return

    # Create a mouse controller instance from pynput
    mouse = Controller()

    try:
        while True:
            # Get the current mouse position as a tuple (x, y)
            x, y = mouse.position
            
            # Format the message as "x,y\n"
            message = f"X{int(x)} 1000 10œ00\n"
            ser.write(message.encode('utf-8'))
            print("Sent:", message.strip())

            message = f"Y{int(y)} 1000 1000\n"
            ser.write(message.encode('utf-8'))
            print("Sent:", message.strip())
            
            # Wait a short amount of time (e.g., 10 ms)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
