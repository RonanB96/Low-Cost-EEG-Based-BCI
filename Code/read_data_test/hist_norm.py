# hist_norm.py
#
# Reads in data of normalised FFT data from a csv file
# to plot a histagram of it along multiple probabilty distributation functions
# for fitting
# Author: Ronan Byrne
# Last Updated: 09/05/2018

import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

path = '/home/ronan/Documents/EEG_csv/normalised_data/norm_no_tol/'
file = 'stim.csv'
Fs = 1000

frequencies = np.array([5, 10, 12, 15, 20])

# rows,columns and current index for figure
row_col = 111
fig = []
fig.append(plt.figure())
ax = []
count = 0

# Read in dataframe from csv file
# Data is setup in 5 columns of various lengths. The header is the freq e.g '5'
df = pd.read_csv(path + file)

# Loop through all of the columns of data
for i,f in enumerate(frequencies):
    buff = df[str(f)].values
    # Discard NaN
    try:
        buff = buff[~np.isnan(buff)]
    except:
        pass
    print(len(buff))
    std = np.std(buff)
    mean = np.mean(buff)
    print("mean of {0}, mean {1}, std {2}".format(frequencies[i], mean, std))

    # This can be used to plot multiple graphs on one figure, but right now, its just 1 plot per figure
    if count == 1:
        fig.append(plt.figure())
        count = 0
    ax.append(fig[-1].add_subplot(row_col + count))
    # Calculate an X axis for the pdf
    x = np.linspace(min(buff)/1.5, max(buff)*1.5, 1000)

    # Calculate the pdf's for varies distributions
    parameters_r = st.rayleigh.fit(buff)
    fitted_pdf_r = st.rayleigh.pdf(x, loc=parameters_r[0],scale=parameters_r[1])
    parameters_g = st.gamma.fit(buff)
    fitted_pdf_g = st.gamma.pdf(x, a=parameters_g[0], loc=parameters_g[1], scale=parameters_g[2])
    parameters_n = st.norm.fit(buff)
    fitted_pdf_n = st.norm.pdf(x, loc=parameters_n[0],scale=parameters_n[1])
    parameters_l = st.lognorm.fit(buff)
    fitted_pdf_l = st.lognorm.pdf(x, s=parameters_l[0],loc=parameters_l[1], scale=parameters_l[2])
    parameters_c = st.chi.fit(buff)
    fitted_pdf_c = st.chi.pdf(x, df=parameters_c[0], loc=parameters_c[1], scale=parameters_c[2])

    # Plot histagram of data
    ax[-1].hist(buff, density=True, facecolor='g', bins='auto')
    ax[-1].set_title("Probability Distribution of Magnitude at {}Hz".format(frequencies[i]))
    ax[-1].set_xlabel("Magnitude")
    ax[-1].set_ylabel("Probability")
    # Plot the pdf's to fit the histagram
    ax[-1].plot(x,fitted_pdf_n,"red",label="Fitted normal dist",linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_r, "orange", label="Fitted rayleigh dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_g, "black", label="Fitted gamma dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_l, "blue", label="Fitted lognorm dist", linestyle="dashed", linewidth=2)
    ax[-1].plot(x, fitted_pdf_c, "violet", label="Fitted chi dist", linestyle="dashed", linewidth=2)
    ax[-1].legend()

    sum = 0
    for x in fitted_pdf_c:
        sum = sum+math.log(x)
    print(-1*sum)
    #print(-1*np.sum(math.log(fitted_pdf_c)))

    count+=1

    # Calculate the max likeihood estimate for each of the pdf's
	# REFERENCE
    distributions = [st.norm, st.rayleigh, st.gamma, st.chi, st.lognorm]
    mles = []
    for distribution in distributions:
        pars = distribution.fit(buff)
        mle = distribution.nnlf(pars, buff)
        mles.append(mle)
    results = [(distribution.name, mle) for distribution, mle in zip(distributions, mles)]
    best_fit = sorted(zip(distributions, mles), key=lambda d: d[1])[0]
    print("Best fit reached using {}, MLE value: {}".format(best_fit[0].name, best_fit[1]))

plt.show()