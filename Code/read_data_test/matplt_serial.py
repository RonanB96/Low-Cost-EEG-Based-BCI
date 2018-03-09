import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial


class Scope(object):
    def __init__(self, ax, maxt=0.5, dt=0.001):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-.1, 4200)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt:  # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


def serial_read():
    while True:
        if ser.inWaiting() > 0:
            input_data = ser.read(2).hex()
            hex_val1 = int(input_data[0:2], 16) << 8
            hex_val2 = int(input_data[2:4], 16)
            yield (hex_val1 | hex_val2)
        else:
            print("no data")
            yield 0


ser = serial.Serial('/dev/ttyUSB0', 57600)

fig, ax = plt.subplots()
scope = Scope(ax)

ani = animation.FuncAnimation(fig, scope.update, serial_read, save_count=1000, interval=0,
                              blit=True)

plt.show()

