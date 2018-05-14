import multiprocessing
import numpy as np
import alphaInterface
import eegScope

if __name__ == '__main__':
    # Interface arguments
    window_size = [1200, 700]
    checker_cycles = 4                                  # Number of times texture repeats in box
    checker_size = 250
    checker_tex = np.array([[1, 1], [1, 1]])          # One black and one white box
    checker_frequency = np.array([10, 20, 15, 5, 12])   # Flashing Frequencies

    # Serial port
    port = '/dev/ttyUSB0'

    # Creating a pipe
    parent_conn, child_conn = multiprocessing.Pipe()

    # Create processes for user interface and scope
    interface_pro = multiprocessing.Process(target=alphaInterface.BCI, args=(window_size, checker_frequency,
                                                        checker_size, checker_cycles, checker_tex, child_conn))
    graphing_pro = multiprocessing.Process(target=eegScope.Scope, args=(port, parent_conn))

    interface_pro.start()
    graphing_pro.start()

    # Close the processes before finishing
    interface_pro.join()
    graphing_pro.join()