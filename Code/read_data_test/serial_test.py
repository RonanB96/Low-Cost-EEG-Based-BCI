import serial
import time
import numpy as np


ser = serial.Serial('/dev/ttyUSB0', 57600)

input_data = b''

while 1:
    print('waiting')
    while ser.inWaiting() == 0:
        pass

    input_data = b''
    while ser.inWaiting() > 0:
                input_data += ser.read()
    input_data = input_data.hex()
    print(input_data)
    print(len(input_data))

    hex_val = np.array([], dtype='uint16')
    for x in range(0, len(input_data),4):
        hex_val1 = int(input_data[x:x+2], 16) << 8
        hex_val2 = int(input_data[x+2:x+4], 16)
        hex_val = np.append([hex_val], [(hex_val1 | hex_val2)])
    print(hex_val)
    print(len(hex_val))
