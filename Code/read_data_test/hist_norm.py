import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

path = '/home/ronan/Documents/EEG_csv/normalised_data/norm_no_tol/'
file = 'stim.csv'
Fs = 1000

frequencies = np.array([5, 10, 12, 15, 20])

row_col = 111
fig = []
fig.append(plt.figure())
ax = []
count = 0
df = pd.read_csv(path + file)
#no_stim_buff= []
for i,f in enumerate(frequencies):
    buff = df[str(f)].values
    try:
        buff = buff[~np.isnan(buff)]
    except:
        pass
    print(len(buff))
    std = np.std(buff)
    mean = np.mean(buff)
    print("mean of {0}, mean {1}, std {2}".format(frequencies[i], mean, std))
    if count == 1:
        fig.append(plt.figure())
        count = 0
    ax.append(fig[-1].add_subplot(row_col + count))
    x = np.linspace(min(buff)/1.5, max(buff)*1.5, 1000)
    parameters_r = st.rayleigh.fit(buff)
    fitted_pdf_r = st.rayleigh.pdf(x, loc=parameters_r[0],scale=parameters_r[1])
    parameters_g = st.gamma.fit(buff)
    #g_a = (mean/std)**2
    #g_b = (std**2)/mean
    fitted_pdf_g = st.gamma.pdf(x, a=parameters_g[0], loc=parameters_g[1], scale=parameters_g[2])
    parameters_n = st.norm.fit(buff)
    fitted_pdf_n = st.norm.pdf(x, loc=parameters_n[0],scale=parameters_n[1])
    parameters_l = st.lognorm.fit(buff)
    fitted_pdf_l = st.lognorm.pdf(x, s=parameters_l[0],loc=parameters_l[1], scale=parameters_l[2])
    parameters_c = st.chi.fit(buff)
    fitted_pdf_c = st.chi.pdf(x, df =parameters_c[0], loc=parameters_c[1], scale=parameters_c[2])
    parameters_c2 = st.chi2.fit(buff)
    fitted_pdf_c2 = st.chi2.pdf(x, df =parameters_c2[0], loc=parameters_c2[1], scale=parameters_c2[2])
    ax[-1].hist(buff, density=True, facecolor='g', bins='auto')
    ax[-1].set_title("Probability Distribution of Magnitude at {}Hz".format(frequencies[i]))
    ax[-1].set_xlabel("Magnitude")
    ax[-1].set_ylabel("Probability")
    ax[-1].plot(x,fitted_pdf_n,"red",label="Fitted normal dist",linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_r, "orange", label="Fitted rayleigh dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_g, "black", label="Fitted gamma dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_l, "blue", label="Fitted lognorm dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_c, "violet", label="Fitted chi dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_c2, "yellow", label="Fitted chi2 dist", linestyle="dashed", linewidth=2)
    ax[-1].legend()

    count+=1

    distributions = [st.norm, st.rayleigh, st.gamma, st.chi2, st.chi, st.lognorm]
    mles = []
    for distribution in distributions:
        pars = distribution.fit(buff)
        mle = distribution.nnlf(pars, buff)
        mles.append(mle)
    results = [(distribution.name, mle) for distribution, mle in zip(distributions, mles)]
    best_fit = sorted(zip(distributions, mles), key=lambda d: d[1])[0]
    print("Best fit reached using {}, MLE value: {}".format(best_fit[0].name, best_fit[1]))

#fig1 = plt.figure()
#ax1 = fig1.add_subplot(111)
#ax1.plot(buff)

plt.show()