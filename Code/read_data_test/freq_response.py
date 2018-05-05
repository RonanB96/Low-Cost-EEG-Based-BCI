import pandas as pd
import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/25_april/'
file = '12hz_checker2.csv'
t_start = 0
t_end = 0

input_step = 0.0255+0.024

df = pd.read_csv(path+file, header=None)

dt = 0.001
output = df[0].values
length = len(output)

#t = df[3].values
#dt = t[2]-t[1]
t = np.arange(0, int(length*dt), dt)

if t_end == 0:
    t_end = t[-1]
t_end_i = int(t_end/dt)
t_start_i = int(t_start/dt)
print(t_end)
'''
input = df[4].values
input = sig.medfilt(input[:int(t_end/dt)])
input_step = np.max(input)
print(input_step)
output = df[10].values
output = sig.medfilt(output[:int(t_end/dt)])/input_step
t = t[:int(t_end/dt)]
'''

output = sig.medfilt(output[t_start_i:t_end_i])/input_step
ax1 = plt.subplot(311)
plt.plot(t[t_start_i:t_end_i], output)
plt.show()
#input_change = np.mean(input[len(input)-10:])-np.mean(input[:10])
#num = 1
#den = [1, 1]

#input_diff = np.diff(input)
#step_start = np.argmax(input_diff)-1
output_diff = np.diff(output)

plt.subplot(312, sharex=ax1)
plt.plot(t[t_start_i:t_end_i-1], output_diff)

padding_fact = 5
rfft = np.fft.rfft(output_diff, len(output_diff)*padding_fact)
rfft_mag = abs(rfft)*2/len(output_diff)
freq_axis = np.fft.rfftfreq(len(output_diff)*padding_fact, dt)

plt.subplot(313)
plt.plot(freq_axis,rfft_mag)
plt.show()

