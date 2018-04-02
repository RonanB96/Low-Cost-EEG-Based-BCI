import numpy as np
import timeit

# enhanced from IronManMark20 version
def shift1(arr, num, fill_value=np.nan):
    arr = np.roll(arr,num)
    if num < 0:
        arr[num:] = fill_value
    elif num > 0:
        arr[:num] = fill_value
    return arr

# use np.roll and np.put by IronManMark20
def shift2(arr,num):
    arr=np.roll(arr,num)
    if num<0:
         np.put(arr,range(len(arr)+num,len(arr)),np.nan)
    elif num > 0:
         np.put(arr,range(num),np.nan)
    return arr

# use np.pad and slice by me.
def shift3(arr, num, fill_value=np.nan):
    l = len(arr)
    if num < 0:
        arr = np.pad(arr, (0, abs(num)), mode='constant', constant_values=(fill_value,))[:-num]
    elif num > 0:
        arr = np.pad(arr, (num, 0), mode='constant', constant_values=(fill_value,))[:-num]

    return arr

# use np.concatenate and np.full by chrisaycock
def shift4(arr, num, fill_value=np.nan):
    if num >= 0:
        return np.concatenate((np.full(num, fill_value), arr[:-num]))
    else:
        return np.concatenate((arr[-num:], np.full(-num, fill_value)))

# preallocate empty array and assign slice by chrisaycock
def shift5(arr, num, fill_value=np.nan):
    result = np.empty_like(arr)
    if num > 0:
        result[:num] = fill_value
        result[num:] = arr[:-num]
    elif num < 0:
        result[num:] = fill_value
        result[:num] = arr[-num:]
    else:
        result = arr
    return result

def shift6(arr,num,fill_value=np.nan):
    arr[:-num] = arr[num:]
    arr[num] = fill_value
    return arr

arr = np.arange(2000).astype(float)

def benchmark_shift1():
    shift1(arr, 1)

def benchmark_shift2():
    shift2(arr, 1)

def benchmark_shift3():
    shift3(arr, 1)

def benchmark_shift4():
    shift4(arr, 1)

def benchmark_shift5():
    shift5(arr, 1)

def benchmark_shift6():
    shift6(arr,1)

benchmark_set = ['benchmark_shift1', 'benchmark_shift2', 'benchmark_shift3', 'benchmark_shift4', 'benchmark_shift5',
                 'benchmark_shift6']

for x in benchmark_set:
    number = 10000
    t = timeit.timeit('%s()' % x, 'from __main__ import %s' % x, number=number)
    print('%s time: %f' % (x, t))
