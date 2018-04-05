from pyqtgraph.Qt import QtCore, QtGui
#from PyQt5 import QtCore, QtGui
import pyqtgraph as pg
import numpy as np
import serial
import threading
import time
import sys


class EEGScope:
    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)
        self.app.aboutToQuit.connect(self.exit)
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('EEG Scope')

        self.main_plot = self.win.addPlot(title="Raw Data")
        self.avg_main_plot = self.win.addPlot(title="Averaged Data")

        self.win.nextRow()
        self.fft_plot = self.win.addPlot(title="FFT of Raw Data")
        self.avg_fft_plot = self.win.addPlot(title="FFT of Averaged Data")

        self.y_min = 0
        self.y_max = 4095
        self.Fs = 1000

        # Main plot setup
        self.main_plot_t_start = 0
        self.main_plot_t_size = 2047
        self.main_plot_t_end = self.main_plot_t_size
        #self.main_plot.setXRange(self.t_start, self.t_end)
        self.main_plot.setYRange(self.y_min, self.y_max)
        self.main_plot.setLabel('left', 'Bits','Du')
        self.main_plot.setLabel('bottom', 'Time', 'ms')
        self.graph = self.main_plot.plot()
        self.graph_pos = 0

        # Averaged main plot setup
        self.avg_main_plot_t_start = self.main_plot_t_start
        self.avg_main_plot_t_size = self.main_plot_t_size
        self.avg_main_plot_t_end = self.main_plot_t_end
        #self.avg_main_plot.setXRange(self.t_start, self.t_end)
        self.avg_main_plot.setYRange(self.y_min, self.y_max)
        self.avg_main_plot.setLabel('left', 'Bits','Du')
        self.avg_main_plot.setLabel('bottom', 'Time', 'ms')
        self.avg_graph = self.avg_main_plot.plot()
        self.avg_graph_pos = 0
        self.avg_graph_time = np.arange(self.avg_main_plot_t_start, self.avg_main_plot_t_end, 1)
        self.avg_graph_values = int(self.y_max / 2) * np.ones(self.avg_main_plot_t_size, dtype='uint16')

        # FFT variables
        self.fft_sample_size = 1000
        self.fft_sample_num = 0
        self.fft_padding = 5
        self.fft_freq = np.fft.rfftfreq(self.fft_sample_size * self.fft_padding, 1 / self.Fs)

        # FFT graph setup
        #self.fft_plot.setYRange(0, 500)
        self.fft_plot.setXRange(0, 100)
        self.fft_plot.setLabel('left', 'Bits', 'Du')
        self.fft_plot.setLabel('bottom', 'Frequency', 'Hz')
        self.fft_graph = self.fft_plot.plot()
        self.fft_graph_fft_mag = np.zeros(int((self.fft_sample_size*self.fft_padding)/2)+1)

        # Averaged FFT graph setup
        self.avg_fft_plot.setXRange(0, 100)
        self.avg_fft_plot.setLabel('left', 'Bits', 'Du')
        self.avg_fft_plot.setLabel('bottom', 'Frequency', 'Hz')
        self.avg_fft_graph = self.avg_fft_plot.plot()
        self.avg_fft_graph_fft_mag = self.fft_graph_fft_mag

        # Calculation Variables
        self.avg_calc = False
        self.fft_calc = False
        self.calc_thread = threading.Thread(target=self.calc, daemon=True)

        # Serial variables
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)
        self.ser_N = 2047
        self.ser_head = 0
        self.ser_tail = self.ser_data_read = self.ser_head
        self.ser_buff = int(self.y_max / 2) * np.ones(self.ser_N, dtype='uint16')
        self.ser_thread = threading.Thread(target=self.serial_read, daemon=True)

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

        self.ser_thread.start()
        self.calc_thread.start()
        self.update_plot()

    def serial_read(self):
        self.ser.reset_input_buffer()
        while True:
            while self.ser.inWaiting() >= 2:
                self.ser_data_read = 1
                lowbyte = self.ser.read()
                highbyte = self.ser.read()

                self.ser_buff[:-1] = self.ser_buff[1:]
                self.ser_buff[-1] = (ord(highbyte) << 8) + ord(lowbyte)
                self.logfile.write(str(self.ser_buff[-1]) + '\n')
                self.ser_head = (self.ser_head + 1) % self.ser_N
                if self.ser_head == 0:
                    self.avg_calc = True

    def update_plot(self):
        read_diff = 0
        while True:
            if self.ser_data_read is 1:
                self.ser_data_read = 0
                if self.ser_head >= self.ser_tail:
                    read_diff = self.ser_head - self.ser_tail
                    self.graph_pos += read_diff
                    self.fft_sample_num += read_diff
                else:
                    read_diff = self.ser_head + self.ser_N - self.ser_tail
                    self.graph_pos += read_diff
                    self.fft_sample_num += read_diff

                self.ser_tail = self.ser_head
                self.graph.setPos(self.graph_pos, 0)
                self.graph.setData(self.ser_buff)

                if self.fft_sample_num > self.fft_sample_size:
                    self.fft_sample_num = 0
                    self.fft_calc = True

                self.app.processEvents()

            time.sleep(0.025)

    def calc(self):
        while True:
            if self.avg_calc is True:
                self.avg_calc = False
                buff = self.ser_buff
                self.avg_graph_values = (np.array(self.avg_graph_values) + np.array(buff)) / 2.0
                self.avg_graph.setData(self.avg_graph_time, self.avg_graph_values)

            if self.fft_calc is True:
                self.fft_calc = False

                buff = self.ser_buff[:self.fft_sample_size]
                buff = buff - np.mean(buff)
                ham = np.hamming(self.fft_sample_size)
                y_ham = ham * buff
                self.fft_graph_fft_mag = 4/self.fft_sample_size * \
                                         np.abs(np.fft.rfft(y_ham, self.fft_sample_size * self.fft_padding))

                buff = self.avg_graph_values[:self.fft_sample_size]
                buff = buff - np.mean(buff)
                y_ham = ham * buff
                self.avg_fft_graph_fft_mag = 4/self.fft_sample_size * \
                                         np.abs(np.fft.rfft(y_ham, self.fft_sample_size * self.fft_padding))

                self.fft_graph.setData(self.fft_freq, self.fft_graph_fft_mag)
                self.avg_fft_graph.setData(self.fft_freq, self.avg_fft_graph_fft_mag)
            time.sleep(0.025)

    def exit(self):
        print("exit called")
        sys.exit()

EEGScope()
