import matplotlib.pyplot as plt
import numpy as np


path = '/home/ronan/Documents/EEG_csv/4_may/'
writting_file = "10_stim.csv"
reading_file ='10hz_checker2.csv'
buff = []
Fs = 1000

t_start = 30
t_end = 0


f_write = open(path+writting_file, 'a')

count = 1
with open(path+reading_file, "r") as f_read:
    line = f_read.readline()
    while line:
        if t_end is not 0:
            if count >= (t_start*Fs) and count < (t_end*Fs):
                f_write.write(line)
        else:
            if count>= (t_start*Fs):
                f_write.write(line)
        line = f_read.readline()
        count+=1

f_write.close()
f_read.close()

