#
# SEEEScope.py - written by Ted Burke - last updated 9-3-2016
#

from Tkinter import *
import serial
import sys
import numpy
import time

class SEEEScope:
	
	def __init__(self, master):
		
		# Create Tkinter interface
		frame = Frame(master, width=850, height=600)
		frame.pack()
		self.canvas = Canvas(frame, width=800, height=512, bg="#ffffff")
		self.canvas.pack()
		buttonframe = Frame(frame)
		buttonframe.pack(side=LEFT, fill=BOTH, expand=1)
		self.recordbutton = Button(buttonframe, text='Record', command=self.startrecording)
		self.recordbutton.pack(side=LEFT)
		self.stopbutton = Button(buttonframe, text='Stop', command=self.stoprecording)
		self.stopbutton.pack(side=LEFT)
		self.status = Label(buttonframe, text="Ready", bd=2, relief=SUNKEN)
		self.status.pack(side=LEFT, fill=BOTH, expand=1)
		
		# Bind space key to start/stop recording
		master.bind("<space>", self.spacekey)
		
		# data buffer
		self.n = 0
		self.N = 2000
		self.buf = 512 * numpy.ones(self.N, dtype=numpy.uint16)
		
		# recording flag
		self.recording = 0;
	
	# The space key starts and stops recording
	def spacekey(self, event):
		if self.recording:
			self.stoprecording()
		else:
			self.startrecording()

	def startrecording(self):
		# Open serial port
		sys.stdout.write('Opening COM3...')
		sys.stdout.flush()
		self.ser = serial.Serial(
			port='COM3',
			baudrate=115200,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS)
		if not self.ser.isOpen():
			print('Failed')
			return
		print('OK')
		
		# Open data log file
		self.filename = time.ctime() + '.csv'
		self.filename = self.filename.replace(':','-') #colon not allowed in windows codename
		sys.stdout.write('Opening data log file ' + self.filename + ' ...')
		sys.stdout.flush()
		self.logfile = file(self.filename, 'w')
		if not self.logfile:
			print('Failed')
			self.ser.close()
			return
		print('OK')
		
		# Set recording flag and call readdata method for first time
		self.status.config(text='Recording to file: "' + self.filename + '"')
		self.recording = 1
		self.readdata()
	
	def readdata(self):
		# Read any available samples from serial port buffer
		while self.ser.inWaiting() >= 2:
			
			# Read next sample data
			lowbyte = self.ser.read()
			highbyte = self.ser.read()
			
			# Combine bytes into sample value
			self.buf[self.n] = (ord(highbyte) << 8) + ord(lowbyte)
			
			# Write this sample to the data log file
			self.logfile.write(str(self.buf[self.n])+'\n')
			
			# Increment buffer index
			self.n = (self.n + 1) % self.N
			
		# Redraw waveform
		w = self.canvas.winfo_width()
		h = self.canvas.winfo_height()
		self.canvas.delete(ALL)
		for m in range(w):
			m1 = (self.n-1+self.N-w+m) % self.N
			m2 = (self.n  +self.N-w+m) % self.N
			h1 = int(h*(1023.0-self.buf[m1])/1023.0)
			h2 = int(h*(1023.0-self.buf[m2])/1023.0)
			self.canvas.create_line(m,h1,m+1,h2)
		
		# Print some text information on the canvas
		self.canvas.create_text(100,40,text='value='+str(self.buf[(self.n-1)%self.N]))
		
		# Either keep on recording or close serial port
		if self.recording:
			# Ask for this function to be called again in 1ms
			root.after(1, self.readdata)
		else:
			# Close serial port
			sys.stdout.write('Closing serial port...')
			sys.stdout.flush()
			self.ser.close()
			if self.ser.isOpen():
				print('Failed')
			else:
				print('OK')
			
			# Close log file
			print('Closing data log file')
			self.logfile.close()
			
			# Update status bar
			self.status.config(text='Data were recorded to file "' + self.filename + '"')
	
	def stoprecording(self):
		self.status.config(text='Stopping...')
		print('Setting recording flag to false')
		self.recording = 0

# Create Tk root object and application window
root = Tk()
root.title("SEEEScope")
seeescope = SEEEScope(root)
root.mainloop()
