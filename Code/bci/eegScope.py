from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
import serial
import threading
import time
import sys


class Scope:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        #self.app.aboutToQuit.connect(self.exit())
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
        self.sample_interval = 1/self.Fs

        # Main plot setup
        self.main_plot_t_start = 0
        self.main_plot_t_size = 2047
        self.main_plot_t_end = self.main_plot_t_size
        #self.main_plot.setXRange(self.t_start, self.t_end)
        self.main_plot.setYRange(self.y_min, self.y_max)
        self.main_plot.setLabel('left', 'ADC Value', '')
        self.main_plot.setLabel('bottom', 'Time', 's')
        self.graph = self.main_plot.plot()
        self.graph_time = np.arange(self.main_plot_t_start, self.main_plot_t_end/self.Fs, self.sample_interval)
        self.graph_pos = self.graph_time[-1]

        # Averaged main plot setup
        self.avg_main_plot_t_start = self.main_plot_t_start
        self.avg_main_plot_t_size = self.main_plot_t_size
        self.avg_main_plot_t_end = self.main_plot_t_end/self.Fs
        #self.avg_main_plot.setXRange(self.t_start, self.t_end)
        self.avg_main_plot.setYRange(self.y_min, self.y_max)
        self.avg_main_plot.setLabel('left', 'ADC Value','')
        self.avg_main_plot.setLabel('bottom', 'Time', 's')
        self.avg_graph = self.avg_main_plot.plot()
        self.avg_graph_time = np.arange(self.avg_main_plot_t_start, self.avg_main_plot_t_end, self.sample_interval)
        self.avg_graph_values = int(self.y_max / 2) * np.ones(self.avg_main_plot_t_size, dtype='uint16')

        # FFT variables
        self.fft_sample_size = 1000
        self.fft_sample_num = 0
        self.fft_padding = 5
        self.fft_freq = np.fft.rfftfreq(self.fft_sample_size * self.fft_padding, 1 / self.Fs)

        # FFT graph setup
        #self.fft_plot.setYRange(0, 250)
        self.fft_plot.setXRange(0, 100)
        self.fft_plot.setLabel('left', 'Bits', 'Du')
        self.fft_plot.setLabel('bottom', 'Frequency', 'Hz')
        self.fft_graph = self.fft_plot.plot()
        self.fft_graph_fft_mag = np.zeros(int((self.fft_sample_size*self.fft_padding)/2)+1)

        # Averaged FFT graph setup
        #self.avg_fft_plot.setYRange(0, 250)
        self.avg_fft_plot.setXRange(0, 100)
        self.avg_fft_plot.setLabel('left', 'Bits', 'Du')
        self.avg_fft_plot.setLabel('bottom', 'Frequency', 'Hz')
        self.avg_fft_graph = self.avg_fft_plot.plot()
        self.avg_fft_graph_fft_mag = self.fft_graph_fft_mag

        # Calculation Variables
        self.avg_calc = False
        self.fft_calc = False
        self.avg_fft_update = False
        self.first_run = True
        self.saved_values = np.zeros(self.main_plot_t_size, dtype='uint16')
        self.calc_thread = threading.Thread(target=self.calc, daemon=True)

        self.win.removeItem(self.avg_main_plot)
        self.win.removeItem(self.avg_fft_plot)

    def start(self):
        # Serial variables
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)
        self.graph_N = 2047
        self.graph_head = 0
        self.graph_tail = self.graph_data_read = self.graph_head
        self.graph_buff = int(self.y_max / 2) * np.ones(self.graph_N, dtype='uint16')
        self.serial_thread = threading.Thread(target=self.serial_read, daemon=True)

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

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(25)
        self.serial_thread.start()

        # Main Loop
        self.app.exec_()

    def serial_read(self):
        self.ser.reset_input_buffer()
        while True:
            while self.ser.inWaiting() >= 2:
                self.graph_data_read = 1
                lowbyte = self.ser.read()
                highbyte = self.ser.read()

                self.graph_buff[:-1] = self.graph_buff[1:]
                self.graph_buff[-1] = (ord(highbyte) << 8) + ord(lowbyte)

                self.graph_time[:-1] = self.graph_time[1:]
                self.graph_pos += self.sample_interval
                self.graph_time[-1] = self.graph_pos

                self.logfile.write(str(self.graph_buff[-1]) + '\n')

                self.graph_head = (self.graph_head + 1) % self.graph_N
                '''if self.graph_head == 0:
                    if self.first_run:
                        self.avg_graph_values = self.saved_values = np.copy(self.graph_buff)
                        self.avg_graph.setData(self.avg_graph_time, self.saved_values)
                        self.first_run = 0
                    else:
                        self.avg_graph_values = (np.array(self.saved_values) + np.array(self.graph_buff)) / 2.0
                        self.avg_graph.setData(self.avg_graph_time, self.avg_graph_values)
                        self.saved_values = np.copy(self.graph_buff)

                    self.avg_fft_update = True'''

    def update_plot(self):
        if self.graph_data_read is 1:
            self.graph_data_read = 0
            if self.graph_head >= self.graph_tail:
                self.fft_sample_num += self.graph_head - self.graph_tail
            else:
                self.fft_sample_num += self.graph_head + self.graph_N - self.graph_tail

            self.graph_tail = self.graph_head
            self.graph.setData(self.graph_time, self.graph_buff)

            if self.fft_sample_num > self.fft_sample_size:
                self.fft_sample_num = 0
                self.fft_calc = True
                self.calc()
            #self.app.processEvents()

    def calc(self):
        if self.fft_calc:
            self.fft_calc = False

            temp_buff = self.graph_buff[self.main_plot_t_size-self.fft_sample_size:]
            temp_buff = temp_buff - np.mean(temp_buff)
            ham = np.hamming(self.fft_sample_size)
            y_ham = temp_buff * ham
            self.fft_graph_fft_mag = 4/self.fft_sample_size * \
                                     np.abs(np.fft.rfft(y_ham, self.fft_sample_size * self.fft_padding))

            fft_mag_max_i = np.argmax(self.fft_graph_fft_mag)
            print("fft max is:     ", self.fft_freq[fft_mag_max_i])
            self.fft_graph.setData(self.fft_freq, self.fft_graph_fft_mag)

        if self.avg_fft_update:
            self.avg_fft_update = False
            buff = self.avg_graph_values[self.main_plot_t_size-self.fft_sample_size:]
            buff = buff - np.mean(buff)
            ham = np.hamming(self.fft_sample_size)
            y_ham = buff * ham
            self.avg_fft_graph_fft_mag = 4/self.fft_sample_size * \
                                         np.abs(np.fft.rfft(y_ham, self.fft_sample_size * self.fft_padding))

            avg_fft_mag_max_i = np.argmax(self.avg_fft_graph_fft_mag)
            print("avg fft max is: ", self.fft_freq[avg_fft_mag_max_i])
            self.avg_fft_graph.setData(self.fft_freq, self.avg_fft_graph_fft_mag)

        time.sleep(0.025)

    def exit(self):
        print("exit called")
        sys.exit()


if __name__ == '__main__':
    scope = Scope()
    scope.start()
