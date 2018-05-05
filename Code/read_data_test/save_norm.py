import pandas as pd
import numpy as np

read_path = '/home/ronan/Documents/EEG_csv/4_may/'
save_path = '/home/ronan/Documents/EEG_csv/normalised_data/norm_no_tol/'
read_file = "10_stim.csv"
write_file = "temp_no_stim.csv"
comp_freq = 10
Fs = 1000
freq_tol = 0

buff = []
padding_fact = 5

frequencies = np.array([5, 10, 12, 15, 20])
comp_index = np.argwhere(frequencies == comp_freq)
comp_index = comp_index[0][0]

df = pd.read_csv(read_path+'no_stim_mean_std_no_tl.csv')
norm_mean = df[str(comp_freq)]
norm_mean = norm_mean[0]
print(norm_mean)


with open(read_path+read_file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()

buff = buff-np.mean(buff)
section_size = 1000

if not (section_size*padding_fact) % 2:
    fft_len = int(section_size*padding_fact/2)+1
else:
    fft_len = int((section_size*padding_fact+1)/2)
print(fft_len)

rfft_mag = np.zeros([int(len(buff)/section_size), fft_len])
freq_r = np.fft.rfftfreq(section_size*padding_fact, 1/Fs)
ham = np.hamming(section_size)
for i in range(len(rfft_mag)):
    rfft_mag[i] = abs(np.fft.rfft(buff[i*section_size:(i+1)*section_size], section_size*padding_fact)) * 2/section_size

dt = 1/Fs
frequenies_index = frequencies/(Fs/(section_size*padding_fact))
freq_sig_noise = np.zeros([len(frequencies), len(rfft_mag)])
freq_sig_val = np.zeros([len(frequencies), len(rfft_mag)])

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

print(len(rfft_mag))
df = pd.read_csv(save_path+write_file)
#old_val = df[str(comp_freq)].values
series =freq_sig_val[comp_index]/norm_mean
#appended = np.append(old_val,series)
#df.reset_index()
df[str(comp_freq)] = pd.Series(series)
#df = df[str(comp_freq)].append(series, ignore_index=True)
#df=pd.DataFrame(frame,columns=str(comp_freq))

#df2 = pd.DataFrame({str(comp_freq):series})
#df = pd.concat([df,df2])
df.to_csv(save_path+write_file, index=False)
