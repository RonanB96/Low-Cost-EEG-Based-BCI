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
        self.main_plot = self.win.addPlot()
        self.win.nextRow()
        self.fft_plot = self.win.addPlot()

        self.y_min = 0
        self.y_max = 4096
        self.Fs = 1000

        self.t_start = 0
        self.t_size = 2000
        self.t_end = self.t_size
        #self.p.setXRange(self.t_start, self.t_end)
        self.main_plot.setYRange(self.y_min, self.y_max)
        self.main_plot.setLabel('left', 'Bits','Du')
        self.main_plot.setLabel('bottom', 'Time', 'ms')
        self.graph = self.main_plot.plot()

        self.N = 2000
        self.buff = int(self.y_max/2) * np.ones(self.N, dtype='uint16')
        self.head = 0
        self.tail = 0
        self.pos = 0
        self.data_read = 0

        self.fft_sample_size = 1000
        self.fft_sample_num = 0
        self.fft_padding = 5
        self.fft_plot.setYRange(self.y_min, self.y_max)
        self.fft_plot.setLabel('left', 'Bits', 'Du')
        self.fft_plot.setLabel('bottom', 'Frequency', 'Hz')
        self.fft_graph = self.fft_plot.plot()

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
        read_diff = 0
        while True:
            if self.data_read is 1:
                if self.head > self.tail:
                    read_diff = self.head - self.tail
                    self.pos += read_diff
                    self.fft_sample_num += read_diff
                else:
                    read_diff = self.head + self.N - self.tail
                    self.pos += read_diff
                    self.fft_sample_num += read_diff
                self.tail = self.head
                self.graph.setPos(self.pos, 0)
                self.graph.setData(self.buff)
                # curve.setPos(head,0)
                # if tail > head:
                # p.setXRange(t_start+t_size, t_end+t_size)
                if self.fft_sample_num > self.fft_sample_size:
                    self.fft_sample_num = 0
                    temp_data = self.fft_calc(self.buff[:self.fft_sample_size])
                    self.fft_graph.setData(temp_data[0], temp_data[1])
                self.app.processEvents()
                self.data_read = 0
            time.sleep(0.025)

    def fft_calc(self, buff):
        y_len = len(buff)
        ham = np.hamming(y_len)
        y_ham = ham * buff
        rfft = np.fft.rfft(y_ham, y_len * self.fft_padding)
        freq_r = np.fft.rfftfreq(y_len * self.fft_padding, 1 / self.Fs)

        '''offset_min = 100
        f_offset = 0.5
        for i in range(0, len(freq_r)):
            diff = abs(f_offset - freq_r[i])
            if diff < offset_min:
                offset_min = diff
                offset_i = i
        '''
        return freq_r, 4 / y_len * np.abs(rfft)


EEGScope()
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()