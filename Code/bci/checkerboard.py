from psychopy import visual, core, event
import numpy as np
import time

tex = np.array([[1, -1], [-1, 1]])
cycles = 7
size = 512
frequency = 10

win = visual.Window([1024, 700], units='pix')
stim = visual.GratingStim(win, tex=tex, size=size, units='pix',
                        sf=cycles / size, interpolate=False)


def busy_wait(seconds):
    c = time.clock()
    while time.clock()-c < seconds:
        pass


interval = 1/frequency
while not event.getKeys('escape'):
    stim.tex = -stim.tex
    stim.draw()
    win.flip()
    busy_wait(interval)

