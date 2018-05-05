from psychopy import visual, core, event
import numpy as np
import time

tex = np.array([[1, -1], [-1, 1]])
cycles = 7
size = 512
frequency = 5

win = visual.Window([1024, 700], units='pix')
stim = visual.GratingStim(win, tex=tex, size=size, units='pix',
                        sf=cycles / size, interpolate=False)

frame_rate = win.getActualFrameRate()
frame_interval = 1/frame_rate
interval = 1/frequency
stim_frames = np.round(interval/frame_interval)
print("Frame rate is {0}. Actual Flashing Frequency will be {1}".format(frame_rate, str(1/(stim_frames*frame_interval))))

win.flip()
n = 0
while not event.getKeys('escape'):
    if n % stim_frames == 0:
        stim.tex = -stim.tex
    stim.draw()
    win.flip()
    n += 1

