import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_cz/'
file = '15hz_ssvep.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5
t_start = 0
t_end = 0

frequencies = np.array([5, 10, 12, 15, 20])


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()


fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.plot(buff)

buff = buff-np.mean(buff)
section_size = 2000

if t_end == 0:
    t_end = len(buff)/Fs

if not (section_size*padding_fact) % 2:
    fft_len = int(section_size*padding_fact/2)+1
else:
    fft_len = int((section_size*padding_fact+1)/2)
print(fft_len)
i_start = t_start*Fs
i_end = t_end * Fs
rfft_mag = np.zeros([int((i_end-i_start)/section_size), fft_len])
freq_r = np.fft.rfftfreq(section_size*padding_fact, 1/Fs)
ham = np.hamming(section_size)
for i in range(len(rfft_mag)):
    rfft_mag[i] = abs(np.fft.rfft(buff[i*section_size+i_start:(i+1)*section_size+i_start], section_size*padding_fact)) * 2/section_size

dt = 1/Fs
frequenies_index = frequencies/(Fs/(section_size*padding_fact))
freq_sig_noise = np.zeros([len(frequencies), len(rfft_mag)])
freq_sig_val = np.zeros([len(frequencies), len(rfft_mag)])

freq_max_middle = 0
freq_tol = Fs/section_size
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
                freq_sig_val[index][rfft_index] = freq_max
                break
        '''print("For {0}Hz, left {1}, middle {2}, right {3}, signal to noise {4}".format(f, freq_max_left[index],
                                                                                       freq_max_middle[index],
                                                                                       freq_max_right[index],
          freq_max_sign_noise[index]))'''
'''for i in range(len(frequenies_index)):
    for index in range(len(rfft_mag)):
        freq_mag[i][index] = rfft_mag[index][int(frequenies_index[i])]'''



bins = 5
print(len(rfft_mag))
print("mean of {0}, std {1}, mean {2}".format(frequencies[0], np.std(freq_sig_val[0]), np.mean(freq_sig_val[0])))
ax0 = plt.subplot(321)
ax0.set_title(str(frequencies[0]))
plt.hist(freq_sig_val[0], density=True, facecolor='g')

print("mean of {0}, std {1}, mean {2}".format(frequencies[1], np.std(freq_sig_val[1]), np.mean(freq_sig_val[1])))
ax1 = plt.subplot(322)
ax1.set_title(str(frequencies[1]))
plt.hist(freq_sig_val[1], density=True, facecolor='g')

print("mean of {0}, std {1}, mean {2}".format(frequencies[2], np.std(freq_sig_val[2]), np.mean(freq_sig_val[2])))
ax2 = plt.subplot(323)
ax2.set_title(str(frequencies[2]))
plt.hist(freq_sig_val[2], density=True, facecolor='g')

print("mean of {0}, std {1}, mean {2}".format(frequencies[3], np.std(freq_sig_val[3]), np.mean(freq_sig_val[3])))
ax3 = plt.subplot(324)
ax3.set_title(str(frequencies[3]))
plt.hist(freq_sig_val[3], density=True, facecolor='g')

print("mean of {0}, std {1}, mean {2}".format(frequencies[4], np.std(freq_sig_val[4]), np.mean(freq_sig_val[4])))
ax4 = plt.subplot(325)
ax4.set_title(str(frequencies[4]))
plt.hist(freq_sig_val[4], density=True, facecolor='g')

#fig1 = plt.figure()
#ax1 = fig1.add_subplot(111)
#ax1.plot(buff)

plt.show()