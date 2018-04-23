import pandas as pd
import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt

file = "pos_output_step.csv"
t_start = 0
t_end = 0

df = pd.read_csv(file, header=None, usecols=[3, 4, 10])

t = df[3].values
dt = t[2]-t[1]

if t_end == 0:
    t_end = t[-1]

print(t_end)
input = df[4].values
input = sig.medfilt(input[:int(t_end/dt)])
input_step = np.max(input)
print(input_step)
output = df[10].values
output = sig.medfilt(output[:int(t_end/dt)])/input_step
t = t[:int(t_end/dt)]

ax1 = plt.subplot(311)
plt.plot(t, input, t, output)

input_change = np.mean(input[len(input)-10:])-np.mean(input[:10])
num = 1
den = [1, 1]

input_diff = np.diff(input)
step_start = np.argmax(input_diff)-1
output_diff = np.diff(output[step_start:])

plt.subplot(312, sharex=ax1)
plt.plot(t[step_start:len(t)-1], input_diff[step_start:], t[step_start:len(t)-1], output_diff)

padding_fact = 5
rfft = np.fft.rfft(output_diff, len(output_diff)*padding_fact)
rfft_mag = abs(rfft)*2/len(output_diff)
freq_axis = np.fft.rfftfreq(len(output_diff)*padding_fact, dt)

plt.subplot(313)
plt.plot(freq_axis,rfft_mag)
plt.show()

