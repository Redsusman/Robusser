import serial
import time
import struct

ser = serial.Serial(port='COM4', baudrate=115200)
print(f"connected to: {ser.portstr or 'Unknown Port'}")
message = "Simon"

while True:
    ser.write(message.encode()) # I guess this is encoding via utf8?
    #for b in bytearray("simon was here","UTF-8"):
        #ser.write(b)

    print("sent")
    time.sleep (100.0 / 1000.0)
    result = ser.read(25) # tried readline, just hangs
    print(">> " + result.decode('utf-8')) # tried utf8, ascii


