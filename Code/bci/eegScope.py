# eegScope.py
# 
# Receives analog data over serial port and plots the data
# in real time along with a real time FFT
# Author: Ronan Byrne
# Last Updated: 09/05/2018
#

from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
import serial
import threading
import time
import sys


class Scope(object):
    def __init__(self, port, pipe):
        self.pipe = pipe
        self.app = QtWidgets.QApplication(sys.argv)
        #self.app.aboutToQuit.connect(self.exit())  # Close start away when uncommented

        # Create Window
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('EEG Scope')

        # Add Real time plot
        self.main_plot = self.win.addPlot(title="Raw Data")

        # Add FFT plot
        self.win.nextRow()
        self.fft_plot = self.win.addPlot(title="FFT of Raw Data")

        self.y_min = 0
        self.y_max = 4095
        self.Fs = 1000
        self.sample_interval = 1/self.Fs

        # Main plot setup
        self.main_plot_t_start = 0
        self.main_plot_t_size = 2047
        self.main_plot_t_end = self.main_plot_t_size
        self.main_plot.setYRange(self.y_min, self.y_max)
        self.main_plot.setLabel('left', 'ADC Value', '')
        self.main_plot.setLabel('bottom', 'Time', 's')

        # Actually create the plot
        self.graph = self.main_plot.plot()
        self.graph_time = np.arange(self.main_plot_t_start, self.main_plot_t_end/self.Fs, self.sample_interval)
        self.graph_pos = self.graph_time[-1]  # Place cursor at the far right of the screen

        # FFT variables
        self.fft_sample_size = 1000
        self.fft_sample_num = 0
        self.fft_padding = 5
        # Frequency axis
        self.fft_freq = np.fft.rfftfreq(self.fft_sample_size * self.fft_padding, 1 / self.Fs)

        # FFT graph setup
        #self.fft_plot.setYRange(0, 250)
        # Frequencies above 100Hz aren't of interest
        self.fft_plot.setXRange(0, 100)
        self.fft_plot.setLabel('left', 'ADC Value', '')
        self.fft_plot.setLabel('bottom', 'Frequency', 'Hz')

        self.fft_graph = self.fft_plot.plot()
        self.fft_graph_fft_mag = np.zeros(int((self.fft_sample_size*self.fft_padding)/2)+1)

        # Calculation Variables
        self.first_run = True
        self.saved_values = np.zeros(self.main_plot_t_size, dtype='uint16')

        self.port = port

        # Graphing and serial variables
        # Roughly 2 seconds of data
        self.graph_N = 2047
        self.graph_data_read = False
        self.graph_head = 0
        self.graph_tail = self.graph_head
        self.graph_buff = int(self.y_max / 2) * np.ones(self.graph_N, dtype='uint16')
        self.serial_thread = threading.Thread(target=self.serial_read, daemon=True)
        self.ser = None
        self.plot_timer = None

        # Logging variables
        self.filename = ''
        self.logfile = None

        # Start automatically if pipe was given
        if self.pipe is not None:
            self.start()

    def start(self):
        # Serial variables
        self.ser = serial.Serial(self.port, baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)

        if not self.ser.isOpen():
            print('Failed to open port')
            return

        # Create log file
        self.filename = '/home/ronan/Documents/EEG_csv/' + time.ctime() + '.csv'
        self.filename = self.filename.replace(':', '-')  # colon not allowed in windows codename
        print('Opening data log file ' + self.filename + ' ...')
        self.logfile = open(self.filename, 'w')

        if not self.logfile:
            print('Failed to open logfile')
            return

        # Create a timer to update graph every timeout
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        # 25ms timer
        self.plot_timer.start(25)

        # Start thread to read from serial port
        self.serial_thread.start()

        # Main Loop
        self.app.exec_()

    def serial_read(self):
        self.ser.reset_input_buffer()
        start_up = True
        print("waiting for uC")
        while True:
            # Wait for the uC to handshake to ensure byte alignment is correct
            while start_up:
                line = self.ser.readline()
                # decode will not work if the uC is sending analog data
                try:
                    line = line.decode('utf-8')
                except UnicodeDecodeError:
                    print("Unable to decode, try restart uC")
                print(line)
                # uC is starting, move on
                if line == "sstarting\n":
                    start_up = False
                # uC will continuously send ss until start is sent
                elif line == "ss\n":
                    self.ser.write('start\n'.encode())
            # uC sends data in 2 bytes
            while self.ser.inWaiting() >= 2:
                # data has been read, so if plot timer is called, it can update
                self.graph_data_read = True
                lowbyte = self.ser.read()
                highbyte = self.ser.read()

                # Shift buffer to the left, this is faster than np.roll
                self.graph_buff[:-1] = self.graph_buff[1:]
                self.graph_buff[-1] = (ord(highbyte) << 8) + ord(lowbyte)

                self.graph_time[:-1] = self.graph_time[1:]
                # Shift cursor
                self.graph_pos += self.sample_interval
                self.graph_time[-1] = self.graph_pos

                # Write value to log file
                self.logfile.write(str(self.graph_buff[-1]) + '\n')

                # Shift head forward
                self.graph_head = (self.graph_head + 1) % self.graph_N

                self.fft_sample_num += 1
                # If we have enough samples for FFT
                if self.fft_sample_num == self.fft_sample_size:
                    self.fft_sample_num = 0
                    self.calc()

    def update_plot(self):
        # Only update plot is data has been read
        if self.graph_data_read:
            self.graph_data_read = False
            # Set set new data to plot
            self.graph.setData(self.graph_time, self.graph_buff)

    def calc(self):
        # store current buff so it doesn't changed as we're doing calculations
        temp_buff = self.graph_buff[self.main_plot_t_size-self.fft_sample_size:]
        # Remove DC offset
        temp_buff = temp_buff - np.mean(temp_buff)

        # If there is a pipe, send temp before
        if self.pipe is not None:
            self.pipe.send(temp_buff[:])

        # FFT calculations
        ham = np.hamming(self.fft_sample_size)
        y_ham = temp_buff * ham
        self.fft_graph_fft_mag = 4/self.fft_sample_size * \
            np.abs(np.fft.rfft(y_ham, self.fft_sample_size * self.fft_padding))

        # Set FFT data
        self.fft_graph.setData(self.fft_freq, self.fft_graph_fft_mag)

    def exit(self):
        print("exit called")
        sys.exit()


if __name__ == '__main__':
    scope = Scope('/dev/ttyUSB0', None)
    scope.start()
