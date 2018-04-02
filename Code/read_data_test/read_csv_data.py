import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/'
file = 'eeg_fp0_eyes_open.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()

plt.subplot(211)
plt.plot(buff)

y_len = len(buff)
ham = np.hamming(y_len)
y_ham = ham*buff

'''
fft = np.fft.fft(y_ham, y_len*padding_fact)
f_len = len(fft)
bins = np.arange(0,math.ceil(f_len/2))
fax = bins*Fs/f_len
plt.subplot(212)
plt.plot(fax,4/y_len * np.absolute(fft[:math.floor(f_len/2)]))
'''

rfft = np.fft.rfft(y_ham, y_len*padding_fact)
freq_r = np.fft.rfftfreq(y_len*padding_fact, 1/Fs)

offset_min = 100
f_offset = 0.5
for i in range(0,len(freq_r)):
    diff = abs(f_offset - freq_r[i])
    if diff < offset_min:
        offset_min = diff
        offset_i = i

plt.subplot(212)
plt.plot(freq_r[offset_i:],4/y_len * np.absolute(rfft[offset_i:]))
plt.show()

plt.show()
