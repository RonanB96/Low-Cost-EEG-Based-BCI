import multiprocessing
import numpy as np
import time
import eegInterface
import eegScope

if __name__ == '__main__':
    window_size = [1200, 700]
    cycles = 4
    size = 160
    frequency = np.array([10, 20, 15, 5, 12])
    tex = np.array([[1, -1], [-1, 1]])

    port = '/dev/ttyUSB0'

    # creating a pipe
    parent_conn, child_conn = multiprocessing.Pipe()

    interface_pro = multiprocessing.Process(target=eegInterface.BCI, args=(window_size, frequency,
                                                        160, 4, tex, child_conn))
    graphing_pro = multiprocessing.Process(target=eegScope.Scope, args=(port, parent_conn))

    interface_pro.start()
    graphing_pro.start()

    interface_pro.join()
    graphing_pro.join()