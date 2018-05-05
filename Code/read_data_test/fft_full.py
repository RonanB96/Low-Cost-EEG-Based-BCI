import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/4_may/'
file ='alpha_stim.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 1


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()
buff = buff-np.mean(buff)

buff_len = len(buff)
print(buff_len)
freq_r = np.fft.rfftfreq(buff_len*padding_fact, 1/Fs)
ham = np.hamming(buff_len)
rfft_mag = abs(np.fft.rfft(ham*buff, buff_len*padding_fact)) * 2/buff_len

plt.plot(freq_r,rfft_mag)
plt.xlim(0,100)
plt.show()