import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/18_april/oz_left_cz/'
file = 'bci_20hz_check2.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5

frequencies = np.array([5, 10, 12, 15, 20])


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()

plt.subplot(311)
plt.plot(buff)

buff = buff-np.mean(buff)
y_len = len(buff)
section_size = 4000
if not (section_size*padding_fact) % 2:
    fft_len = int(section_size*padding_fact/2)+1
else:
    fft_len = int((section_size*padding_fact+1)/2)
print(fft_len)
rfft_mag = np.zeros([int(y_len/section_size), fft_len])
freq_r = np.fft.rfftfreq(section_size*padding_fact, 1/Fs)
ham = np.hamming(section_size)
#spec = np.zeros([int(y_len/section_size), fft_len])
f,t,spec= signal.spectrogram(buff[section_size:(2)*section_size], Fs)
#for i in range(len(rfft_mag)):
    #rfft_mag[i] = abs(np.fft.rfft(ham*buff[i*section_size:(i+1)*section_size], section_size*padding_fact)) * 2/section_size
#    f,t,spec[i] = signal.spectrogram(buff[i*section_size:(i+1)*section_size], Fs, nfft=section_size*padding_fact)

dt = 1/Fs
frequenies_index = frequencies/(Fs/(section_size*padding_fact))
freq_sig_noise = np.zeros([len(frequencies), len(rfft_mag)])
freq_sig_val = np.zeros([len(frequencies), len(rfft_mag)])

freq_max_left = 0
freq_max_middle = 0
freq_max_right = 0
freq_max_sign_noise = 0
freq_check_har = []
freq_tol = 0.5

frequencies2 = np.append(frequencies, frequencies*2)

for index, f in enumerate(frequencies2):
    for rfft_index in range(len(rfft_mag)):
        freq_start = f - freq_tol * 3
        freq_end = f - freq_tol
        freq_max = 0
        count = 0
        #Y(n) = Y(n-1)+X(i)*Ts
        for i in range(0, len(freq_r)):
            if freq_r[i] >= freq_start:
                #freq_max += rfft_mag[i]*freq_diff
                if rfft_mag[rfft_index][i] > freq_max:
                    freq_max = rfft_mag[rfft_index][i]
            if freq_r[i] >= freq_end:
                freq_start += freq_tol*2
                freq_end += freq_tol*2
                if count == 0:
                    freq_max_left = freq_max
                elif count == 1:
                    freq_max_middle= freq_max
                elif count == 2:
                    freq_max_right = freq_max
                    if index <len(frequencies):
                        if freq_max_middle > freq_max_left and freq_max_middle> freq_max_right:
                            freq_sig_noise[index][rfft_index] = freq_max_middle * 2 / (freq_max_left + freq_max_left)
                            freq_sig_val[index][rfft_index] = freq_max_middle
                        else:
                            freq_sig_noise[index][rfft_index] = 0
                            freq_sig_val[index][rfft_index] = 0
                    else:
                        if freq_max_middle <= freq_max_left or freq_max_middle <= freq_max_right:
                            freq_sig_noise[index-len(frequencies)][rfft_index] = 0
                            freq_sig_val[index-len(frequencies)][rfft_index] = 0
                            pass
                        elif freq_max_middle >= freq_sig_val[index-len(frequencies)][rfft_index]:
                            freq_sig_noise[index - len(frequencies)][rfft_index] = 0
                            freq_sig_val[index - len(frequencies)][rfft_index] = 0
                            pass
                    break
                count += 1
                freq_max = 0
        '''print("For {0}Hz, left {1}, middle {2}, right {3}, signal to noise {4}".format(f, freq_max_left[index],
                                                                                       freq_max_middle[index],
                                                                                       freq_max_right[index],
          freq_max_sign_noise[index]))'''
'''for i in range(len(frequenies_index)):
    for index in range(len(rfft_mag)):
        freq_mag[i][index] = rfft_mag[index][int(frequenies_index[i])]'''

plt.subplot(312)
for i in range(len(freq_sig_noise)):
    plt.plot(freq_sig_noise[i]*freq_sig_val[i], label=frequencies[i])
plt.legend()

plt.subplot(313)
for i in range(len(freq_sig_val)):
    max_index = np.argmax(freq_sig_val[i])
    plt.plot(freq_r[max_index])
plt.show()
