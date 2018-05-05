import scipy.stats as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

path = '/home/ronan/Documents/EEG_csv/normalised_data/norm_no_tol/'
file = 'no_stim.csv'

comp_path = '/home/ronan/Documents/EEG_csv/normalised_data/norm_no_tol/'
comp_file = 'stim.csv'
comp_freq = 20
n = 0
Fs = 1000
padding_fact = 5

frequencies = np.array([5, 10, 12, 15, 20])
comp_index = np.argwhere(frequencies == comp_freq)
comp_index = comp_index[0][0]

df = pd.read_csv(path+'no_stim.csv')
no_stim_buff = df[str(comp_freq)].values

try:
    no_stim_buff = no_stim_buff[~np.isnan(no_stim_buff)]
except:
    pass
df2 =pd.read_csv(path+'stim.csv')
stim_buff = df2[str(comp_freq)].values
try:
    stim_buff = stim_buff[~np.isnan(stim_buff)]
except:
    pass



row_col = 111
fig = []
fig.append(plt.figure())
ax = []
count = 0

std = np.std(no_stim_buff)
mean = np.mean(no_stim_buff)
print("freq {0}, normalised mean {1}, normalised std {2}".format(frequencies[comp_index], mean,std))

comp_std = np.std(stim_buff)
comp_mean = np.mean(stim_buff)
print("freq {0}, normalised mean {1}, normalised std {2}".format(frequencies[comp_index], comp_mean, comp_std))

x = np.linspace(min(no_stim_buff), max(no_stim_buff)*1.5, 1000)
parameters_l = st.lognorm.fit(no_stim_buff)
fitted_pdf_l = st.lognorm.pdf(x, s=parameters_l[0],loc=parameters_l[1], scale=parameters_l[2])

comp_x = np.linspace(min(stim_buff), max(stim_buff)*1.5, 1000)
comp_parameters_l = st.chi.fit(stim_buff)
comp_fitted_pdf_l = st.chi.pdf(comp_x, comp_parameters_l[0],comp_parameters_l[1], comp_parameters_l[2])
plt.title("Probability Distribution of Magnitude at {}Hz".format(frequencies[comp_index]))
plt.xlabel("Magnitude")
plt.ylabel("Probability")
plt.plot(x, fitted_pdf_l, "black", label="Fitted lognorm dist, no stimulus", linestyle="dashed", linewidth=2)
plt.plot(comp_x, comp_fitted_pdf_l, "blue", label="Fitted chi dist, stimulus ", linestyle="dashed", linewidth=2)
plt.legend()

#fig1 = plt.figure()
#ax1 = fig1.add_subplot(111)
#ax1.plot(buff)

plt.show()