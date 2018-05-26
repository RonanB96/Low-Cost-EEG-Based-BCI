# eegInterface.py
# 
# Creates a keyboard made of flashing checkerboxes which can
# selected by the user looking and concentrating on an individual box
# A baseline is recorded for first 30s. The EEG data is compared against
# the baseline data to determine if user if looking at a certain box
# Author: Ronan Byrne
# Last Updated: 09/05/2018
#

from psychopy import visual, event
import numpy as np
import scipy.stats as st
import threading

# Interface arguments
window_size = [1200, 700]
checker_cycles = 4                                  # Number of times texture repeats in box
checker_size = 160
checker_tex = np.array([[1, -1], [-1, 1]])          # One black and one white box
checker_frequency = np.array([10, 20, 15, 5, 12])   # Flashing Frequencies

special_text = 'SPECIAL'
num_text = 'NUM'
text_boxes = [
    ['A', 'B', 'C', 'D', 'E'],
    ['F', 'G', 'H', 'I', 'J'],
    ['K', 'L', 'M', 'N', 'O'],
    ['P', 'Q', 'R', 'S', 'T'],
    ['U', 'V', 'W', 'X', 'Y', 'Z', num_text, special_text]
]
num_boxes = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']

enter_text = 'ENTER'
del_text = 'DEL'
space_text = 'SPACE'
special_boxes = [space_text, '.', '?', enter_text, del_text]


class BCI(object):
    def __init__(self, win_size, freq_array, checker_size, checker_cycles, checker_tex, pipe):
        self.pipe = pipe
        self.win_size = win_size
        self.win = visual.Window(self.win_size,
                                 units='pix',
                                 monitor='testMonitor',
                                 )
        self.mouse = event.Mouse(win=self.win)

        self.tex = checker_tex
        self.checker_size = checker_size
        self.checker_cycles = checker_cycles

        self.freq_array = freq_array

        self.stim = []
        self.selection_boxes = []
        self.xy = []
        self.top_level_boxes = text_boxes
        # create checker boxes and text above boxes
        # Some magic numbers here for the positions, not fully customisable
        for i in range(len(self.top_level_boxes)):
            self.xy.append([i * 220 - self.win_size[0] / 2 + self.checker_size,
                            -self.win_size[1] / 2 + 0.75 * self.checker_size +
                            (not ((i + 1) % 2)) * (1.5 * self.checker_size)])
            self.stim.append(visual.GratingStim(self.win,
                                                tex=self.tex,
                                                size=self.checker_size,
                                                units='pix',
                                                sf=self.checker_cycles / self.checker_size,
                                                pos=self.xy[i]))
            self.selection_boxes.append(visual.TextStim(self.win,
                                                        text=','.join(self.top_level_boxes[i]),
                                                        pos=[self.stim[i].pos[0], self.stim[i].pos[1]
                                                             + 0.60 * self.checker_size]))
        self.num_of_stim = len(self.stim)

        self.instructions_text1 = 'Recording Baseline, please stare into the center box until it starts flashing'
        self.instructions_text2 = 'Stare into the center of the box which corresponds to the character ' \
                                  'you want to choose'
        self.instructions_box_update = False
        # TextStim is slower than TextBox but TextBox was unreliable
        self.instructions_box = visual.TextStim(self.win,
                                                text=self.instructions_text1,
                                                pos=[0, self.win_size[1] / 2 - 50],
                                                alignHoriz='center',
                                                alignVert='center')
        # White rectangle for textbox
        self.entered_background = visual.Rect(self.win,
                                              units='pix',
                                              width=self.win_size[0]/2,
                                              height=40,
                                              fillColor=[1, 1, 1],
                                              pos=[0, self.instructions_box.pos[1] - 75])
        # Textbox text
        self.entered_textbox = visual.TextStim(self.win,
                                               text='|',
                                               color=[-1, -1, -1],
                                               pos=[self.entered_background.pos[0], self.entered_background.pos[1]],
                                               alignHoriz='center',
                                               alignVert='center')
        # Green box to outline which box was selected
        self.selected_box = visual.Rect(self.win,
                                        units='pix',
                                        width=self.checker_size,
                                        height=self.checker_size,
                                        lineWidth=40,
                                        lineColor=[-1, 1, -1],
                                        fillColor=None,
                                        pos=[0, 0]  # will be changed when used
                                        )

        # Number of frames the green box is shown for
        self.selected_box_on_frames = 30
        self.selected_box_frames = 0
        self.selected_index = -1

        # Average Frame rate
        self.frame_rate = self.win.getActualFrameRate()
        self.frame_interval = 1 / self.frame_rate
        # The time interval for each box
        self.interval = 1 / self.freq_array
        # Number of frames each checker box is shown
        self.stim_frames = np.round(self.interval / self.frame_interval)
        print("Frame rate is {0}. Actual Flashing Frequency will be {1}".format(self.frame_rate, str(
            1 / (self.stim_frames * self.frame_interval))))

        self.box_selected = False
        # Flag for if we are in the bottom level of the selection tree
        self.bottom_level = False

        self.baseline_count = 0
        self.pipe_thread = threading.Thread(target=self.pipeReceive, daemon=True)

        if self.pipe is not None:
            self.setting_baseline = True
            self.pipe_thread.start()
            self.start()
        else:
            self.setting_baseline = False

    def start(self):
        self.mouse.clickReset()
        self.win.flip()
        frame_count = 0
        while not event.getKeys('escape'):

            # If 'b' pressed, recorded baseline again
            if event.getKeys('b') and self.pipe is not None:
                self.setting_baseline = True
                self.baseline_count = 0
                self.instructions_box.text = self.instructions_text1
                self.instructions_box_update = True
                self.entered_textbox.text = '|'

            # If not recording baseline, check if any boxes were selected or need to be updated
            if not self.setting_baseline:
                # Check if the left button was clicked and a box is not already selected
                if self.mouse.getPressed().count(1) and (self.selected_index == -1):
                    self.mouse.clickReset()
                    pos = self.mouse.getPos()
                    # Check if the mouse was clicked inside one of the boxes
                    for i in range(self.num_of_stim):
                        if self.stim[i].contains(pos):
                            self.selected_index = i
                            self.selected_box.pos = self.stim[self.selected_index].pos
                            self.selected_box_frames = self.selected_box_on_frames
                            self.selected_box.draw()
                            break
                # A Box was selected, redraw
                elif self.selected_index is not -1:
                    self.selected_box.draw()
                    self.selected_box_frames -= 1
                    # Last redraw of selection box, update boxes with new selection
                    if self.selected_box_frames <= 0:
                        self.update_selection()

                # Check if any of the checker boards need to be updated
                for i, x in enumerate(self.stim_frames):
                    if (frame_count % x) == 0:
                        # Swap checkerboard pattern
                        self.stim[i].tex = -self.stim[i].tex

            # Update all things on screen
            self.draw_screen()
            # win.flip() blocks until the screen fresh so is used to count number of frames passed
            self.win.flip()
            frame_count += 1

    # Group choices together
    def group_choices(self, boxes):
        # Selection boxes can hold 1-4 values each
        boxes_len = len(boxes)
        if boxes_len / 2 <= self.num_of_stim:
            j_max = 2
        elif boxes_len / 3 <= self.num_of_stim:
            j_max = 3
        elif boxes_len / 4 <= self.num_of_stim:
            j_max = 4
        else:
            print("unsupported length, resetting to top level")
            self.reset_to_top_level()
            return
        offset = 0
        # Update boxes with new selections
        for i in range(0, self.num_of_stim):
            self.selection_boxes[i].text = ''
            for j in range(j_max):
                try:
                    self.selection_boxes[i].text += (boxes[i + j + offset] + ',')
                except:
                    # reach end of selections, remove ',' from last box
                    self.selection_boxes[i].text = self.selection_boxes[i].text[:-1]
                    break
            offset += 1
            # remove ',' from end of each box selection
            self.selection_boxes[i].text = self.selection_boxes[i].text[:-1]

    # Reset the selection to the top level
    def reset_to_top_level(self):
        self.bottom_level = False
        for i, x in enumerate(self.stim):
            self.selection_boxes[i].text = ','.join(self.top_level_boxes[i])
            self.selection_boxes[i].draw()

    # draw everything on screen
    def draw_screen(self):
        for i, x in enumerate(self.stim):
            self.stim[i].draw()
            self.selection_boxes[i].draw()
        if self.instructions_box_update:
            self.instructions_box.text = self.instructions_text2
            self.instructions_box_update = False
        self.instructions_box.draw()
        self.entered_background.draw()
        self.entered_textbox.draw()

    # Update the screen with new selections
    def update_selection(self):
        # Not to the lowest level selection
        if not self.bottom_level:
            # cant display all element with each box
            if len(self.selection_boxes[self.selected_index].text.split(',')) > self.num_of_stim:
                self.group_choices(self.selection_boxes[self.selected_index].text.split(','))
            # Non empty selection which can be split into max boxes or less
            elif len(self.selection_boxes[self.selected_index].text) > 1:
                temp_text = self.selection_boxes[self.selected_index].text.split(',')
                self.bottom_level = True
                # Place one choice in each box
                for i in range(self.num_of_stim):
                    try:
                        self.selection_boxes[i].text = temp_text[i]
                    except:
                        # No more text to display, rest of boxes will have no text
                        self.selection_boxes[i].text = ''
                    self.selection_boxes[i].draw()
            # Empty box chosen, reset to top level
            elif len(self.selection_boxes[self.selected_index].text) == 0:
                print("Empty box chosen, resetting to top level")
                self.reset_to_top_level()
            else:
                print("Unknown state, resetting")
                self.reset_to_top_level()
        # Bottom level selection
        else:
            temp_boxes = []
            # Non single character selection
            if len(self.selection_boxes[self.selected_index].text) > 1:
                # Special selection choices
                # NUM box was chosen
                if self.selection_boxes[self.selected_index].text == num_text:
                    temp_boxes = num_boxes
                    if len(temp_boxes) > self.num_of_stim:
                        self.group_choices(temp_boxes)
                        self.bottom_level = False
                # SPECIAL box was chosen
                elif self.selection_boxes[self.selected_index].text == special_text:
                    temp_boxes = special_boxes
                    if len(temp_boxes) > self.num_of_stim:
                        self.group_choices(temp_boxes)
                        self.bottom_level = False
                # ENTER box was chosen
                elif self.selection_boxes[self.selected_index].text == enter_text:
                    self.bottom_level = False
                    self.entered_textbox.text = '|'
                    # TODO something on enter
                # DEL box was chosen
                elif self.selection_boxes[self.selected_index].text == del_text:
                    self.bottom_level = False
                    self.entered_textbox.text = self.entered_textbox.text[:-1]
                # SPACE was chosen
                elif self.selection_boxes[self.selected_index].text == space_text:
                    self.bottom_level = False
                    self.entered_textbox.text += ' '
                else:
                    print('Unknown case, reseting to top level')
                    self.bottom_level = False

                # Display bottom level selection
                if self.bottom_level:
                    for i in range(self.num_of_stim):
                        try:
                            self.selection_boxes[i].text = temp_boxes[i]
                        except:
                            self.selection_boxes[i].text = ''
                        self.selection_boxes[i].draw()
                else:
                    # something was selected and there was no sub-level
                    # reset selection to top level
                    if len(temp_boxes) == 0:
                        self.reset_to_top_level()
            # Empty box chosen, reset to top level
            elif len(self.selection_boxes[self.selected_index].text) == 0:
                print("Empty box chosen, resetting to top level")
                self.reset_to_top_level()
            # Single character selection
            else:
                # Remove cursor
                if self.entered_textbox.text is '|':
                    self.entered_textbox.text = self.selection_boxes[self.selected_index].text
                # Append selection
                else:
                    self.entered_textbox.text += self.selection_boxes[self.selected_index].text
                self.reset_to_top_level()
        self.selected_index = -1

    def pipeReceive(self):
        # TODO receive initial parameters through pipe
        fft_padding = 5          # pad fft with 5 times the length
        window_len = 1000        # fft window size
        recv_window_len = 1000   # size of data sent through pipe
        fs = 1000
        cdf_per = 10.0           # 10% probability from cumulative density function
        max_baseline_time = 30   # time to get baseline
        max_baseline_count = int(max_baseline_time*fs/window_len)

        ham = np.hamming(window_len)
        # Frequency points from fft
        freq_axis = np.fft.rfftfreq(window_len * fft_padding, 1 / fs)
        signal_buff = np.zeros(window_len)

        freq_array_len = len(self.freq_array)

        # Signal magnitude for each fft
        freq_sig_snr = np.zeros([freq_array_len, 2])
        # SNR of last two fft
        freq_sig_mean_snr = np.zeros([freq_array_len, 1])
        # Baseline magnitude of each frequencies
        freq_sig_base_val = np.zeros([freq_array_len, max_baseline_count])
        # Signal threshold which is 10% or less probablitiy
        freq_sig_val_thresh = np.zeros([freq_array_len, 1])
        # Freq tolerance to check magnitude
        freq_tol = 0
        while True:
            # Read in data from pipe, pipe.recv blocks until data is received
            for i in range(int(window_len/recv_window_len)):
                signal_buff[i*recv_window_len:i*recv_window_len+recv_window_len] = self.pipe.recv()

            y_ham = signal_buff * ham
            # Calculate fft assuming signal is real, returns first half of spectrum
            rfft = np.fft.rfft(y_ham, window_len * fft_padding)
            # Calculate magnitude
            rfft_mag = 4 / window_len * np.absolute(rfft)

            # loop through each frequency calculating maximum magnitude within the freq tol
            for index, f in enumerate(self.freq_array):
                freq_start = f - freq_tol
                freq_end = f + freq_tol
                freq_max = 0
                for i in range(0, len(freq_axis)):
                    if freq_axis[i] >= freq_start:
                        if rfft_mag[i] > freq_max:
                            freq_max = rfft_mag[i]
                    if freq_axis[i] >= freq_end:
                        if self.setting_baseline:
                            # Add max value to baseline array for later
                            freq_sig_base_val[index][self.baseline_count] = freq_max
                        else:
                            # Only save value is greater than threshold
                            if freq_sig_val_thresh[index] < freq_max:
                                freq_sig_snr[index][1] = freq_max / freq_sig_val_thresh[index]
                            else:
                                freq_sig_snr[index][1] = 0
                        break

            if self.setting_baseline:
                self.baseline_count += 1
                # Enough baseline values
                if self.baseline_count == max_baseline_count:
                    # Calculate gamma cumulative distribution function for each frequency
                    for i in range(freq_array_len):
                        std = np.std(freq_sig_base_val[i])
                        mean = np.mean(freq_sig_base_val[i])
                        # create a line from the min magnitude to 1.5 * max magnitude
                        x = np.linspace(min(freq_sig_base_val[i]), max(freq_sig_base_val[i]) * 1.5, 1000)
                        # Calculate shape, scale and location of gamma distribution
                        parameters_l = st.gamma.fit(freq_sig_base_val[i])
                        # Calculate gamma cdf
                        fitted_cdf = st.gamma.cdf(x, parameters_l[0], parameters_l[1], parameters_l[2])
                        # Find the point on the cdf where the magnitude is less tha the cdf percent threshold
                        for j in range(len(x)):
                            if (1 - fitted_cdf[j]) < (cdf_per / 100.0):
                                freq_sig_val_thresh[i] = x[j]
                                break
                        # if the cdf percent threshold is outside of range, just use 2 standard deviations
                        if freq_sig_val_thresh[i] == 0:
                            freq_sig_val_thresh[i] = mean+2*std
                        print("freq {} mean {}, std {}, thresh {}".format(self.freq_array[i], mean, std,
                                                                          freq_sig_val_thresh[i]))
                        freq_sig_snr[i][0] = freq_sig_base_val[i][-1]/freq_sig_val_thresh[i]
                    self.baseline_count = 0
                    self.setting_baseline = False
                    # Have to set a flag here instead of updating because setting the text in
                    # another thread causes problems
                    self.instructions_box_update = True
            else:
                print("freq sig val{}".format(freq_sig_snr.tolist()))
                # Loop through frequencies to calculate mean snr
                for i in range(freq_array_len):
                    # If the last two fft snr are not zero, it may be from the stimulus
                    if freq_sig_snr[i][0] > 0 and freq_sig_snr[i][1] > 0:
                        freq_sig_mean_snr[i] = np.mean(freq_sig_snr[i])
                    else:
                        freq_sig_mean_snr[i] = 0
                    # shift snr value back
                    freq_sig_snr[i][0] = freq_sig_snr[i][1]
                # Find index of max snr
                max_sig_val_index = np.argmax(freq_sig_mean_snr)
                # if all snr's are zero, it will return the first in the array
                if freq_sig_mean_snr[max_sig_val_index] > 0:
                    print("max freq snr {}".format(self.freq_array[max_sig_val_index]))
                    # draw a green box around selected box
                    self.selected_index = max_sig_val_index
                    self.selected_box.pos = self.stim[self.selected_index].pos
                    self.selected_box_frames = self.selected_box_on_frames
                    self.selected_box.draw()


if __name__ == '__main__':
    bci = BCI(win_size=window_size,
              freq_array=checker_frequency, checker_cycles=checker_cycles, checker_size=checker_size,
              checker_tex=checker_tex, pipe=None)
    bci.start()
