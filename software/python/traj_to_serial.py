#!/usr/bin/env python3
import serial
import time

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


    while True:
        
        message = f"X 100000 4000 20000\n"
        ser.write(message.encode('utf-8'))
        print("Sent:", message.strip())

        time.sleep(10)
        message = f"X 0 4000 20000\n"
        ser.write(message.encode('utf-8'))
        print("Sent:", message.strip())
            
        time.sleep(10)


if __name__ == "__main__":
    main()
