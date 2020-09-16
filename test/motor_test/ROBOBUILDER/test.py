import time
import serial
cmd = b'\xff\x00\x64\x64'
ser = serial.Serial('/dev/ttyUSB0', 1500000, timeout=1) #시리얼포트 연결

try:
    ser.write(ser)
    time.sleep(0.1)
    response = ser.readline()
    print(response)
except KeyboardInterrupt:
    ser.close()

