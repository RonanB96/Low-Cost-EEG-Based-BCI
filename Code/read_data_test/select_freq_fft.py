import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/'
file = 'eyes_closed.csv'
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

rfft_mag = 4/y_len * np.absolute(rfft)

freq_val = 50
freq_tol = 3
freq_diff = freq_r[1]-freq_r[0]
print("x diff is ",freq_diff, " x len is ", len(freq_r))

freq_sum = 0
freq_start_i = 0

#Y(n) = Y(n-1)+X(i)*Ts
for i in range(0, len(freq_r)):
    if freq_r[i] >= (freq_val-freq_tol):
        freq_sum += rfft_mag[i]*freq_diff
    if freq_r[i] >= (freq_val + freq_tol):
        break

print(freq_sum)

plt.subplot(212)
plt.plot(freq_r[offset_i:],rfft_mag[offset_i:])

plt.show()

