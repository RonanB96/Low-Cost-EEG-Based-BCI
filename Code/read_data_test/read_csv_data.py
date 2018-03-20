import numpy as np
import matplotlib.pyplot as plt

path = '/home/ronan/Documents/EEG_csv/'
file = 'ecg2_lead1_ina.csv'
buff = np.zeros(100000)
n = 0


with open(path+file, "r") as f:
    line = f.readline()
    while line:
        buff[n] = line
        n += 1
        line = f.readline()

plt.plot(buff[:n])
plt.show()