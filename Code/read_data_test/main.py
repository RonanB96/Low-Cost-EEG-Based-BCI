import numpy as np
with open("sample_data.txt", 'r') as f:
    read_data = f.read()

data_size = (len(read_data)-1)
hex_val = np.array([], dtype='uint16')
for x in range(0, data_size,4):
    hex_val1 = int(read_data[x:x+2], 16) << 8
    hex_val2 = int(read_data[x+2:x+4], 16)
    hex_val = np.append([hex_val], [(hex_val1 | hex_val2)])
print(hex_val)

