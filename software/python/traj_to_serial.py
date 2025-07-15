#!/usr/bin/env python3
import serial
import time

def gotoX(ser, x, vel=4000, acc=20000):
	message = f"X {x} {vel} {acc}\n"
	ser.write(message.encode('utf-8'))


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


    for i in range (10):
        gotoX(ser,5000)
        
        time.sleep(2)
        gotoX(ser,0)
        time.sleep(2)
        print (i)


if __name__ == "__main__":
    main()
