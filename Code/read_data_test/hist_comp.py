import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_right/'
file = 'no_stim.csv'

comp_path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_right/'
comp_file = '5_stim.csv'
comp_freq = 5
buff = []
comp_buff = []
n = 0
Fs = 1000
section_size = 1000
padding_fact = 5
freq_tol = 0
t_start = 0
t_end = 0

comp_t_start = 0
comp_t_end = 0

frequencies = np.array([5, 10, 12, 15, 20])
comp_index = np.argwhere(frequencies == comp_freq)
comp_index = comp_index[0][0]


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()


if comp_file is not '':
    with open(comp_path+comp_file, "r") as f:
        line = f.readline()
        while line:
            comp_buff.append(int(line))
            line = f.readline()

df = pd.read_csv(path+'no_stim_mean_std_no_tl.csv')
norm_mean = df[str(comp_freq)]
norm_mean = norm_mean[0]

if path == comp_path:
    comp_norm_mean = norm_mean
else:
    df = pd.read_csv(comp_path + 'no_stim_mean_std_no_tl.csv')
    comp_norm_mean = df[str(comp_freq)]
    comp_norm_mean = comp_norm_mean[0]
print(norm_mean)
print(comp_norm_mean)

buff = buff-np.mean(buff)
comp_buff = comp_buff - np.mean(comp_buff)

if t_end == 0:
    t_end = len(buff)/Fs

if comp_t_end == 0:
    comp_t_end = len(comp_buff)/Fs

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

comp_i_start = comp_t_start * Fs
comp_i_end = comp_t_end * Fs
comp_rfft_mag = np.zeros([int((comp_i_end - comp_i_start) / section_size), fft_len])
for i in range(len(comp_rfft_mag)):
    comp_rfft_mag[i] = abs(np.fft.rfft(comp_buff[i * section_size + comp_i_start:(i + 1) * section_size + comp_i_start],
                                       section_size * padding_fact)) * 2 / section_size

dt = 1/Fs
frequenies_index = frequencies/(Fs/(section_size*padding_fact))
freq_sig_noise = np.zeros([len(frequencies), len(rfft_mag)])
freq_sig_val = np.zeros([len(frequencies), len(rfft_mag)])

comp_freq_sig_noise = np.zeros([len(frequencies), len(comp_rfft_mag)])
comp_freq_sig_val = np.zeros([len(frequencies), len(comp_rfft_mag)])

freq_max_middle = 0

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

for index, f in enumerate(frequencies):
    for rfft_index in range(len(comp_rfft_mag)):
        freq_start = f - freq_tol
        freq_end = f + freq_tol
        freq_max = 0
        for i in range(0, len(freq_r)):
            if freq_r[i] >= freq_start:
                if comp_rfft_mag[rfft_index][i] > freq_max:
                    freq_max = comp_rfft_mag[rfft_index][i]
            if freq_r[i] >= freq_end:
                comp_freq_sig_val[index][rfft_index] = freq_max
                break


print(len(rfft_mag))
print(len(comp_rfft_mag))

row_col = 111
fig = []
fig.append(plt.figure())
ax = []
count = 0

freq_sig_val[comp_index] = freq_sig_val[comp_index]/norm_mean
std = np.std(freq_sig_val[comp_index])
mean = np.mean(freq_sig_val[comp_index])
print("freq {0}, normalised mean {1}, normalised std {2}".format(frequencies[comp_index], mean,std))

comp_freq_sig_val[comp_index] = comp_freq_sig_val[comp_index]/comp_norm_mean
comp_std = np.std(comp_freq_sig_val[comp_index])
comp_mean = np.mean(comp_freq_sig_val[comp_index])
print("freq {0}, normalised mean {1}, normalised std {2}".format(frequencies[comp_index], comp_mean, comp_std))

x = np.linspace(min(freq_sig_val[comp_index])/1.5, max(freq_sig_val[comp_index])*1.5, 1000)
parameters_l = st.lognorm.fit(freq_sig_val[comp_index])
fitted_pdf_l = st.lognorm.pdf(x, s=parameters_l[0], loc=parameters_l[1], scale=parameters_l[2])

comp_x = np.linspace(min(comp_freq_sig_val[comp_index]/1.5), max(comp_freq_sig_val[comp_index])*1.5, 1000)
comp_parameters_l = st.chi.fit(comp_freq_sig_val[comp_index])
comp_fitted_pdf_l = st.chi.pdf(comp_x, comp_parameters_l[0], comp_parameters_l[1], comp_parameters_l[2])
plt.title("Probability Distribution of Magnitude at {}Hz".format(frequencies[comp_index]))
plt.xlabel("Magnitude")
plt.ylabel("Probability")
plt.plot(x, fitted_pdf_l, "black", label="Fitted lognorm dist", linestyle="dashed", linewidth=2)
plt.plot(comp_x, comp_fitted_pdf_l, "blue", label="Fitted lognorm dist2", linestyle="dashed", linewidth=2)
plt.legend()

#fig1 = plt.figure()
#ax1 = fig1.add_subplot(111)
#ax1.plot(buff)

plt.show()