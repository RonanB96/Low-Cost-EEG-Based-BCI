import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import threading
import serial

win = pg.GraphicsWindow()
xAxis = pg.AxisItem('bottom')
win.setWindowTitle('pyqtgraph example: Scrolling Plots')

size_of_data = 16384
size_of_dis_data = 512
xAxis.setLabel("Time(ms)")
p2 = win.addPlot(axisitems={'bottom': xAxis})
data = np.zeros(size_of_data, dtype='uint16')
serial_buff = data
table_values = serial_buff
p2.setYRange(0, 4094)
# p2.setClipToView(True)
# p2.setDownsampling(mode="mean")
curve = p2.plot(data)
ptr1 = 0
head = 0
tail = 0
tail_overflow = 0
head_overflow = 0
first_pass = True


def update():
    global data, curve, ptr1, serial_buff, tail, head, head_overflow, tail_overflow
    while 1:
        if (tail < head) or (tail_overflow < head_overflow):
            data[:-1] = data[1:]  # shift data in the array one sample left
            # (see also: np.roll)
            data[-1] = serial_buff[tail]
            tail += 1
            if tail == size_of_data:
                tail = 0
                tail_overflow += 1

            ptr1 += 1
            curve.setData(data[0:size_of_dis_data])
            curve.setPos(ptr1, 0)


def serial_read():
    global head, tail, head_overflow, serial_buff, data, first_pass, table_values
    while True:
        if ser.inWaiting() >= 2:
            lowbyte = ser.read()
            highbyte = ser.read()
            serial_buff[head] = (ord(highbyte) << 8) + ord(lowbyte)
            #print(serial_buff[head])
            if serial_buff[head] == 0:
                print("serial buff equals 0 at head: {0}and tail :{1}, value : {2}".format(str(head), str(tail),
                                                                                        str(serial_buff[head])))
            head = (head + 1) % size_of_data
            if head == tail:
                print("head equal tail: {0}".format(str(head)))
            if first_pass and head > size_of_dis_data:
                data = serial_buff
                thread1.start()
                first_pass = False
            #if head == size_of_data:
            #    head = 0
            #    head_overflow += 1
            #    table_values = serial_buff
            #    thread3.start()
        else:
            # print("no data")
            pass


def save_values():
    np.savetxt("serial_data.csv", table_values, delimiter=',')


ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

thread1 = threading.Thread(target=update)
thread2 = threading.Thread(target=serial_read)
thread3 = threading.Thread(target=save_values, daemon=True)
ser.reset_input_buffer()
thread2.start()
# thread1.start()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
