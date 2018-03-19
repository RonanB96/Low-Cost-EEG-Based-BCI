import serial
import time
import numpy as np

n = 0
N = 2000
serial_buff = np.zeros(N, dtype='uint16')
ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

filename = time.ctime() + '.csv'
filename = filename.replace(':', '-')
logfile = open(filename, 'w')
if not logfile:
    print('Failed')
    ser.close()
print('OK')

print('waiting')
while ser.inWaiting() == 0:
    pass
print('connected')
ser.reset_input_buffer()

while 1:
    if ser.inWaiting() > 2:
        lowbyte = ser.read()
        highbyte = ser.read()
        serial_buff[n] = (ord(highbyte) << 8) + ord(lowbyte)
        # Write this sample to the data log file
        logfile.write(str(serial_buff[n]) + '\n')
        print(serial_buff[n])
        n = (n+1) % N
