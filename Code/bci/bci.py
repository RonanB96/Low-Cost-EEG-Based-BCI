from psychopy import visual, core, event
import numpy as np
import time
import random
import string

tex = np.array([[1, -1], [-1, 1]])
window_size = [1024, 700]
cycles = 3
size =128
frequency = np.array([7.5, 10, 12, 15])

win = visual.Window(window_size, units='pix')
stim = []
xy = np.empty([6, 2])
for i in range(0, 4):
    xy[i] = [i*256-window_size[0]/2+size, window_size[1]/2-10-size/2]
    stim.append(visual.GratingStim(win, tex=tex, size=size, units='pix',
                              sf=cycles / size, interpolate=False, pos=xy[i]))
    stim[i].draw()


def draw_text():
    instructions_text.draw()
    word_text.draw()
    typed_word_text.draw()
    for i in letters_text:
        i.draw()

def draw_boards():
    for i in stim:
        i.draw()


instructions = "Please Spell the word below by looking at the correct letters"
instructions_text = visual.TextStim(win, text=instructions, pos=[0, 0])
word = "'Cat'"
word_text = visual.TextStim(win, text=word, pos=[0, instructions_text.pos[1]-50], bold=True)
typed_word = "''"
letters = [word[1]]
letters_height = 50
letters_color = [1,-1,-1]
shuffled_stim_index= np.arange(0, len(stim))
random.shuffle(shuffled_stim_index)
letters_text = [visual.TextStim(win, text=letters[0], pos=stim[shuffled_stim_index[0]].pos, color=letters_color,
                height=letters_height)]# visual.TextStim(win, text=instructions, pos=[0, 0])
for i in range(1, len(stim)):
    letters.append(random.choice(string.ascii_letters))
    letters_text.append(visual.TextStim(win, text=letters[i], pos=stim[shuffled_stim_index[i]].pos,
                        color=letters_color, height=letters_height))
typed_word_text = visual.TextStim(win, text=typed_word, pos=[0, word_text.pos[1]-50], bold=True)
draw_text()


def busy_wait(seconds):
    c = time.clock()
    while time.clock()-c < seconds:
        pass


frame_rate = win.getActualFrameRate()
frame_interval = 1/frame_rate
interval = 1/frequency
stim_frames = np.round(interval/frame_interval)
max_frame_stim_index = np.argmax(stim_frames)

win.flip()
n = 0
while not event.getKeys('escape'):
    for i, x in enumerate(stim_frames):
        if (n % x)== 0:
            stim[i].tex = -stim[i].tex

    draw_boards()
    draw_text()
    win.flip()
    n+=1
