import numpy as np
import matplotlib.pyplot as plt
import math

path = '/home/ronan/Documents/EEG_csv/'
file = 'Tue Mar 27 18-22-16 2018.csv'
buff = []
n = 0
Fs = 1000

padding_fact = 5


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        n += 1
        line = f.readline()

plt.subplot(211)
plt.plot(buff[:n])

y_len = len(buff)
ham = np.hamming(y_len)
y_ham = ham*buff

fft = np.fft.fft(y_ham, y_len*padding_fact)
f_len = len(fft)
bins = np.arange(0,math.ceil(f_len/2))
fax = bins*Fs/f_len
plt.subplot(212)
plt.plot(fax,4/y_len * np.absolute(fft[:math.floor(f_len/2)]))

plt.show()
