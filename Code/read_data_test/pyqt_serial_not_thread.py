import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import serial

win = pg.GraphicsWindow()
win.setWindowTitle('pyqtgraph example: Scrolling Plots')

size_of_data = 2000
p = win.addPlot()
serial_buff = 2048*np.ones(size_of_data, dtype='uint16')
p.setXRange(0, 2000)
p.setYRange(0, 4094)
curve = p.plot(serial_buff)
head = 0


def serial_read(plot ,buff):
    global head
    data_read = 0
    while True:
        while ser.inWaiting() >= 2:
            data_read = 1
            lowbyte = ser.read()
            highbyte = ser.read()
            buff[head] = (ord(highbyte) << 8) + ord(lowbyte)
            head = (head+1) % size_of_data
        if data_read:
            plot.setData(buff[:head])
            QtGui.QApplication.processEvents()


ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
ser.reset_input_buffer()
serial_read(curve, serial_buff)
# thread1.start()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
