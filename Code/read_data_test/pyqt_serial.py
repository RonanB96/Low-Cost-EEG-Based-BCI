import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import serial
import threading
import time
import sys

win = pg.GraphicsWindow()
win.setWindowTitle('pyqtgraph example: Scrolling Plots')

size_of_data = 2000
app = QtGui.QApplication([])
p = win.addPlot()
serial_buff = 2048*np.ones(size_of_data, dtype='uint16')
p.setXRange(0, 2000)
p.setYRange(0, 4094)
curve = p.plot()
head = 0
data_read = 0


def serial_read():
    global head, data_read, serial_buff
    ser.reset_input_buffer()
    filename = '/home/ronan/Documents/EEG_csv/' + time.ctime() + '.csv'
    filename = filename.replace(':', '-')  # colon not allowed in windows codename
    sys.stdout.write('Opening data log file ' + filename + ' ...')
    sys.stdout.flush()
    logfile = open(filename, 'w')
    while True:
        while ser.inWaiting() >= 2:
            data_read = 1
            lowbyte = ser.read()
            highbyte = ser.read()
            serial_buff[head] = (ord(highbyte) << 8) + ord(lowbyte)
            logfile.write(str(serial_buff[head]) + '\n')
            head = (head+1) % size_of_data


def update_plot():
    global data_read, curve, serial_buff, head
    tail = 0
    while True:
        if data_read is 1:
            curve.setData(serial_buff[:head])
            #curve.setPos(head,0)
            tail = head
            QtGui.QApplication.processEvents()
            data_read = 0
        time.sleep(0.025)


ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
thread1 = threading.Thread(target=serial_read, daemon=True)
thread1.start()
update_plot()
# thread1.start()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()