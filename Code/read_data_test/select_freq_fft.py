import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_cz/'
file = '20hz_ssvep.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5

frequenies = [5, 10, 12, 15, 20]


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()

plt.subplot(211)
plt.plot(buff)

buff = buff-np.mean(buff)
y_len = len(buff)
ham = np.hamming(y_len)
y_ham = buff #*ham

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

rfft_mag = 2/y_len * np.absolute(rfft)

freq_diff = freq_r[1]-freq_r[0]
print("x diff is ",freq_diff, " x len is ", len(freq_r))

freq_sum = 0
freq_start_i = 0

freq_max_left = []
freq_max_middle = []
freq_max_right = []
freq_max_sign_noise = []
freq_check_har = []
freq_tol = 0.25


for index, f in enumerate(frequenies):
    freq_start = f - freq_tol*3
    freq_end = f - freq_tol
    freq_max = 0
    count = 0
    #Y(n) = Y(n-1)+X(i)*Ts
    for i in range(0, len(freq_r)):
        if freq_r[i] >= freq_start:
            #freq_max += rfft_mag[i]*freq_diff
            if rfft_mag[i] > freq_max:
                freq_max = rfft_mag[i]
        if freq_r[i] >= freq_end:
            freq_start += freq_tol*2
            freq_end += freq_tol*2
            if count == 0:
                freq_max_left.append(freq_max)
            elif count == 1:
                freq_max_middle.append(freq_max)
            elif count == 2:
                freq_max_right.append(freq_max)
                freq_max_sign_noise.append(freq_max_middle[index]*2/(freq_max_left[index]+freq_max_left[index]))
                if freq_max_middle[index] > freq_max_left[index] and freq_max_middle[index] > freq_max_right[index]:
                    freq_check_har.append(True)
                else:
                    freq_check_har.append(False)
                break
            count += 1
            freq_max = 0
    print("For {0}Hz, left {1}, middle {2}, right {3}, signal to noise {4}".format(f, freq_max_left[index],
                                                                                   freq_max_middle[index],
                                                                                   freq_max_right[index],
          freq_max_sign_noise[index]))

sign_noise_sum = np.mean(freq_max_sign_noise)
for i, s in enumerate(freq_max_sign_noise):
    print("for {0}Hz, signal to noise compare to other freq: {1}".format(frequenies[i], s/sign_noise_sum))

'''for i, x in enumerate(freq_check_har):
    if x:
        for i in range(0, len(freq_r)):
            if freq_r[i] >= freq_start:
                # freq_max += rfft_mag[i]*freq_diff
                if rfft_mag[i] > freq_max:
                    freq_max = rfft_mag[i]
            if freq_r[i] >= freq_end:
                freq_start += freq_tol * 2
                freq_end += freq_tol * 2
                if count == 0:
                    freq_max_left.append(freq_max)
                elif count == 1:
                    freq_max_middle.append(freq_max)
                elif count == 2:
                    freq_max_right.append(freq_max)
                    freq_max_sign_noise.append(
                        freq_max_middle[index] * 2 / (freq_max_left[index] + freq_max_left[index]))
                    if freq_max_middle[index] > freq_max_left[index] and freq_max_middle[index] > freq_max_right[index]:
                        freq_check_har.append(True)
                    else:
                        freq_check_har.append(False)
                    break
                count += 1
                freq_max = 0
'''

plt.subplot(212)
plt.plot(freq_r, rfft_mag)

plt.show()

