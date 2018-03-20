import numpy as np
import matplotlib.pyplot as plot
import math

T = 1
Fs = 8000
f = 20
padding_fact = 5
x = np.arange(Fs*T)
y = np.sin(2 * np.pi * f * x / Fs)
y_len = len(y)
ham = np.hamming(y_len)
y_ham = ham*y

plot.subplot(211)
plot.plot(x/Fs,y_ham)

fft = np.fft.fft(y_ham, y_len*padding_fact)
f_len = len(fft)
bins = np.arange(0,math.ceil(f_len/2))
fax = bins*Fs/f_len
plot.subplot(212)
plot.plot(fax,4/y_len * np.absolute(fft[:math.floor(f_len/2)]))
plot.show()
