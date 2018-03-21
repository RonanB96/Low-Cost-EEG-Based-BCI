import numpy as np
import matplotlib.pyplot as plt

file = 'cutecom.txt'
buff = np.zeros(100000)
n = 0


with open(file, "rb") as f:
    byte = f.read(1)
    while byte != b'':
        #lowbyte = byte
        #highbyte = f.read(1)
        #if(highbyte == b''):
        #    break
        highbyte = byte
        lowbyte = f.read(1)
        if(lowbyte == b''):
            break
        buff[n] = (ord(highbyte) << 8) + ord(lowbyte)
        n += 1
        byte = f.read(1)

plt.plot(buff[:n])
plt.show()