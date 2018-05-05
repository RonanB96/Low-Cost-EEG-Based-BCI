import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

path = '/home/ronan/Documents/EEG_csv/4_may/'
file = '20_stim.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5
freq_tol = 0
t_start = 0
t_end = 0



frequencies = np.array([5, 10, 12, 15, 20])

df = pd.read_csv(path+'no_stim_mean_std_no_tl.csv')
norm_mean =[]
for index, f in enumerate(frequencies):
    norm_mean.append(df[str(f)])
    norm_mean[-1] = norm_mean[-1][0]


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
    freq_sig_val[i] = freq_sig_val[i]/norm_mean[i]
    std = np.std(freq_sig_val[i])
    mean = np.mean(freq_sig_val[i])
    print("mean of {0}, mean {1}, std {2}".format(frequencies[i], mean,std))
    if count == 1:
        fig.append(plt.figure())
        count = 0
    ax.append(fig[-1].add_subplot(row_col + count))
    x = np.linspace(min(freq_sig_val[i]), max(freq_sig_val[i]) * 1.5, 1000)
    parameters_r = st.rayleigh.fit(freq_sig_val[i])
    fitted_pdf_r = st.rayleigh.pdf(x, parameters_r[0], parameters_r[1])
    parameters_g = st.gamma.fit(freq_sig_val[i])
    fitted_pdf_g = st.gamma.pdf(x, parameters_g[0], parameters_g[1], parameters_g[2])
    parameters_n = st.norm.fit(freq_sig_val[i])
    fitted_pdf_n = st.norm.pdf(x, parameters_n[0], parameters_n[1])
    parameters_l = st.lognorm.fit(freq_sig_val[i])
    fitted_pdf_l = st.lognorm.pdf(x, parameters_l[0], parameters_l[1], parameters_l[2])
    parameters_c = st.chi.fit(freq_sig_val[i])
    fitted_pdf_c = st.chi.pdf(x, parameters_c[0], parameters_c[1], parameters_c[2])
    parameters_c2 = st.chi2.fit(freq_sig_val[i])
    fitted_pdf_c2 = st.chi2.pdf(x, parameters_c2[0], parameters_c2[1], parameters_c2[2])
    ax[-1].hist(freq_sig_val[i], density=True, facecolor='g', bins='auto')
    ax[-1].set_title("Probability Distribution of Magnitude at {}Hz".format(frequencies[i]))
    ax[-1].set_xlabel("Magnitude")
    ax[-1].set_ylabel("Probability")
    ax[-1].plot(x, fitted_pdf_n, "red", label="Fitted normal dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_r, "orange", label="Fitted rayleigh dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_g, "black", label="Fitted gamma dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_l, "blue", label="Fitted lognorm dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_c, "violet", label="Fitted chi dist", linestyle="dashed", linewidth=2)
    #ax[-1].plot(x, fitted_pdf_c2, "yellow", label="Fitted chi2 dist", linestyle="dashed", linewidth=2)
    ax[-1].legend()

    count+=1

    distributions = [st.norm, st.rayleigh, st.gamma, st.chi2, st.chi, st.lognorm]
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