from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import numpy as np
import serial
import threading
import time
import sys


class EEGScope:
    def __init__(self):
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('EEG Scope')
        self.p = self.win.addPlot()

        self.y_min = 0
        self.y_max = 4096
        self.t_start = 0
        self.t_size = 2000
        self.t_end = self.t_size
        #self.p.setXRange(self.t_start, self.t_end)
        self.p.setYRange(self.y_min, self.y_max)
        self.p.setLabel('left', 'Bits (Du)')
        self.p.setLabel('bottom', 'Time (ms)')
        self.graph = self.p.plot()

        self.N = 2000
        self.buff = int(self.y_max/2) * np.ones(self.N, dtype='uint16')
        self.head = 0
        self.tail = 0
        self.pos = 0
        self.data_read = 0

        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)

        if not self.ser.isOpen():
            sys.stdout.write('Failed to open port')
            sys.stdout.flush()
            return

        self.filename = '/home/ronan/Documents/EEG_csv/' + time.ctime() + '.csv'
        self.filename = self.filename.replace(':', '-')  # colon not allowed in windows codename
        sys.stdout.write('Opening data log file ' + self.filename + ' ...')
        sys.stdout.flush()
        self.logfile = open(self.filename, 'w')

        if not self.logfile:
            sys.stdout.write('Failed to open logfile')
            sys.stdout.flush()
            return

        self.ser_thread = threading.Thread(target=self.serial_read, daemon=True)
        self.ser_thread.start()

        self.update_plot()

    def serial_read(self):
        self.ser.reset_input_buffer()
        while True:
            while self.ser.inWaiting() >= 2:
                self.data_read = 1
                lowbyte = self.ser.read()
                highbyte = self.ser.read()

                #self.buff = np.roll(self.buff, -1)
                self.buff[:-1] = self.buff[1:]
                self.buff[-1] = (ord(highbyte) << 8) + ord(lowbyte)
                self.logfile.write(str(self.buff[-1]) + '\n')
                self.head = (self.head + 1) % self.N

    def update_plot(self):
        while True:
            if self.data_read is 1:
                if self.head > self.tail:
                    self.pos += self.head - self.tail
                else:
                    self.pos += self.head+self.N-self.tail
                self.graph.setPos(self.pos, 0)
                self.graph.setData(self.buff)
                # curve.setPos(head,0)
                # if tail > head:
                # p.setXRange(t_start+t_size, t_end+t_size)
                self.tail = self.head
                self.app.processEvents()
                self.data_read = 0
            time.sleep(0.025)


EEGScope()
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()