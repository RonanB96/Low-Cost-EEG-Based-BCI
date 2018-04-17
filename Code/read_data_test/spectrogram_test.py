import matplotlib.pyplot as plt
import numpy as np


path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_cz/'
file = '20hz_ssvep.csv'
buff = []
n = 0
Fs = 1000
padding_fact = 5


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff.append(int(line))
        line = f.readline()
#buff = buff-np.mean(buff)

dt = 1/Fs
t = np.arange(0,int(len(buff)*dt),dt)
NFFT = 1024      # the length of the windowing segments

# Pxx is the segments x freqs array of instantaneous power, freqs is
# the frequency vector, bins are the centers of the time bins in which
# the power is computed, and im is the matplotlib.image.AxesImage
# instance

ax1 = plt.subplot(211)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("ADC value")
ax1.set_title("ADC values and Frequency content from Alpha Wave Test")
plt.plot(t, buff[0:len(t)])
ax2 = plt.subplot(212, sharex=ax1)
ax2.set_ylabel("Frequency (Hz)")
ax2.set_xlabel("Time (s)")
Pxx, freqs, bins, im = plt.specgram(buff[0:len(t)], NFFT=NFFT, Fs=Fs,
                                    noverlap=900, pad_to=NFFT*padding_fact,
                                    detrend='mean', #window=np.blackman(NFFT)
                                    #mode='magnitude'
                                    )
ax2.set_ylim(0, 100)
plt.show()