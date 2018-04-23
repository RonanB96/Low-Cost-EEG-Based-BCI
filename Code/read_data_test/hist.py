import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_right/'
file = '10hz_ssvep.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 10
t_start = 0
t_end = 30

frequencies = np.array([5, 10, 12, 15, 20])


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()


buff = buff-np.mean(buff)
section_size = 1000

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
freq_tol = 1

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

row_col = 111
fig = []
fig.append(plt.figure())
ax = []
count = 0
for i in range(len(frequencies)):
    std = np.std(freq_sig_val[i])
    mean = np.mean(freq_sig_val[i])
    print("mean of {0}, mean {1}, std {2}".format(frequencies[i], mean,std))
    if count == 1:
        fig.append(plt.figure())
        count = 0
    ax.append(fig[-1].add_subplot(row_col + count))
    x = np.linspace(min(freq_sig_val[i]), max(freq_sig_val[i]), 100)
    parameters_r = st.rayleigh.fit(freq_sig_val[i])
    fitted_pdf_r = st.rayleigh.pdf(x, loc=parameters_r[0],scale=parameters_r[1])
    parameters_g = st.gamma.fit(freq_sig_val[i])
    g_a = (mean/std)**2
    g_b = (std**2)/mean
    fitted_pdf_g = st.gamma.pdf(x, a=g_a,loc=1/g_b, scale=g_b)
    parameters_n = st.norm.fit(freq_sig_val[i])
    fitted_pdf_n = st.norm.pdf(x, loc=parameters_n[0],scale=parameters_n[1])
    ax[-1].hist(freq_sig_val[i], density=True, facecolor='g', bins='auto')
    ax[-1].set_title("Probability Distribution of Magnitude at {}Hz".format(frequencies[i]))
    ax[-1].set_xlabel("Magnitude")
    ax[-1].set_ylabel("Probability")
    ax[-1].plot(x,fitted_pdf_n,"red",label="Fitted normal dist",linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_r, "orange", label="Fitted rayleigh dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_g, "black", label="Fitted gamma dist", linestyle="dashed", linewidth=2)
    ax[-1].legend()

    count+=1

    distributions = [st.norm, st.rayleigh, st.gamma]
    mles = []
    for distribution in distributions:
        pars = distribution.fit(freq_sig_val[i])
        mle = distribution.nnlf(pars, freq_sig_val[i])
        mles.append(mle)
    results = [(distribution.name, mle) for distribution, mle in zip(distributions, mles)]
    best_fit = sorted(zip(distributions, mles), key=lambda d: d[1])[0]
    print("Best fit reached using {}, MLE value: {}".format(best_fit[0].name, best_fit[1]))

#fig1 = plt.figure()
#ax1 = fig1.add_subplot(111)
#ax1.plot(buff)

plt.show()