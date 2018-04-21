import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_cz/'
file = '15hz_ssvep.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5
t_start = 0
t_mid = 30
t_end = 0

window_len_avg = 2000
window_len = 2000

devs = 2.2

frequencies = np.array([5, 10, 12, 15, 20])

with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()

buff = buff-np.mean(buff)
print(len(buff))

if t_end == 0:
    t_end = len(buff)/Fs

if not (window_len_avg*padding_fact) % 2:
    avg_fft_len = int(window_len_avg*padding_fact/2)+1
else:
    avg_fft_len = int((window_len_avg*padding_fact+1)/2)

i_start = t_start*Fs
i_end = t_mid * Fs
avg_rfft_mag = np.zeros([int((i_end-i_start)/window_len_avg), avg_fft_len])
avg_freq_r = np.fft.rfftfreq(window_len_avg*padding_fact, 1/Fs)
ham = np.hamming(window_len_avg)
for i in range(len(avg_rfft_mag)):
    avg_rfft_mag[i] = abs(np.fft.rfft(ham*buff[i*window_len_avg+i_start:(i+1)*window_len_avg+i_start], window_len_avg*padding_fact)) * 2/window_len_avg

dt = 1/Fs
frequenies_index = frequencies/(Fs/(window_len_avg*padding_fact))
freq_sig_noise = np.zeros([len(frequencies), len(avg_rfft_mag)])
freq_sig_val = np.zeros([len(frequencies), len(avg_rfft_mag)])

freq_max_middle = 0
freq_tol = Fs/window_len_avg
freq_diff = freq_tol*2/avg_freq_r[1]

for index, f in enumerate(frequencies):
    for rfft_index in range(len(avg_rfft_mag)):
        freq_start = f - freq_tol
        freq_end = f + freq_tol
        freq_max = 0
        for i in range(0, len(avg_freq_r)):
            if avg_freq_r[i] >= freq_start:
                if avg_rfft_mag[rfft_index][i] > freq_max:
                    freq_max = avg_rfft_mag[rfft_index][i]
            if avg_freq_r[i] >= freq_end:
                freq_sig_val[index][rfft_index] = freq_max
                break
        '''print("For {0}Hz, left {1}, middle {2}, right {3}, signal to noise {4}".format(f, freq_max_left[index],
                                                                                       freq_max_middle[index],
                                                                                       freq_max_right[index],
          freq_max_sign_noise[index]))'''
'''for i in range(len(frequenies_index)):
    for index in range(len(rfft_mag)):
        freq_mag[i][index] = rfft_mag[index][int(frequenies_index[i])]'''



print(len(avg_rfft_mag))
freq_sig_val_thresh = np.zeros([len(frequencies), 1])
for i in range(len(frequencies)):
    freq_sig_val_thresh[i] = devs*np.std(freq_sig_val[i])+np.mean(freq_sig_val[i])
print(freq_sig_val_thresh)


if not (window_len*padding_fact) % 2:
    fft_len = int(window_len*padding_fact/2)+1
else:
    fft_len = int((window_len*padding_fact+1)/2)

i_start = t_start*Fs
i_end = t_end * Fs
rfft_mag = np.zeros([int((i_end-i_start)/window_len), fft_len])
freq_r = np.fft.rfftfreq(window_len*padding_fact, 1/Fs)
ham = np.hamming(window_len)
for i in range(len(rfft_mag)):
    rfft_mag[i] = abs(np.fft.rfft(ham*buff[i*window_len_avg+i_start:(i+1)*window_len+i_start], window_len*padding_fact)) * 2/window_len

dt = 1/Fs
frequenies_index = frequencies/(Fs/(window_len*padding_fact))
freq_sig_val = np.zeros([len(frequencies), len(rfft_mag)])

freq_max_middle = 0
freq_tol = Fs/window_len
freq_diff = freq_tol*2/freq_r[1]

for index, f in enumerate(frequencies):
    for rfft_index in range(len(rfft_mag)):
        freq_start = f - freq_tol
        freq_end = f + freq_tol
        freq_max = 0
        for i in range(0, len(freq_r)):
            if freq_r[i] >= freq_start:
                if rfft_mag[rfft_index][i] > freq_max:
                    freq_max = rfft_mag[rfft_index][i]
            if freq_r[i] >= freq_end:
                if freq_sig_val_thresh[index] < freq_max:
                    freq_sig_val[index][rfft_index] = freq_max/freq_sig_val_thresh[index]
                else:
                    freq_sig_val[index][rfft_index] = 0
                break

for i in range(len(freq_sig_val)):
    plt.plot(freq_sig_val[i], label=frequencies[i])
plt.legend()
plt.show()
