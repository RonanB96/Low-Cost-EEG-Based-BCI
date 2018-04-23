import matplotlib.pyplot as plt
import numpy as np


path = '/home/ronan/Documents/EEG_csv/14_april/oz_left_cz/'
writting_file = "15_stim.csv"
reading_file ='15hz_ssvep2.csv'
buff = []
Fs = 1000

t_start = 45
t_end = 75

f_write = open(path+writting_file, 'a')

count = 1
with open(path+reading_file, "r") as f_read:
    line = f_read.readline()
    while line:
        if count >= (t_start*Fs) and count < (t_end*Fs):
            f_write.write(line)
        line = f_read.readline()
        count+=1

f_write.close()
f_read.close()

