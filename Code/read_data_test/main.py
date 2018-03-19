import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import threading
from collections import deque
import numpy as np

#data = deque(0 for _ in range(612))
data = np.array(0)
data.astype(int)
sample_rate = 256
frame_rate = sample_rate/4
tail = 0
head = frame_rate


def stm32():
    print("Connecting...")
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
    print("Connected!")

    while True:
        try:
            input_data = ser.read(2).hex()
            hex_val1 = int(input_data[0:2], 16) << 8
            hex_val2 = int(input_data[2:4], 16)
            val = hex_val1 | hex_val2
            np.append(data,val)
            #data.append(val)
            #data.popleft()
            #print(val)
        except ValueError:
            pass


if __name__ == "__main__":
    threading.Thread(target=stm32).start()

    fig, p1 = plt.subplots(1,1)
    plot_data, = p1.plot(data, animated=True)
    p1.set_ylim(0, 4200)


    def animate(i):
        global head, tail
        if tail == sample_rate:
            plt.show()
            tail = 0
            head = frame_rate
        plot_data.set_ydata(data[tail:head])
        print(data)
        plot_data.set_xdata(range(frame_rate))
        tail = tail+frame_rate
        head = head+frame_rate
        return [plot_data]


    ani = animation.FuncAnimation(fig, animate, frames=range(1000),
                                  interval=frame_rate, blit=True)
    plt.show()
