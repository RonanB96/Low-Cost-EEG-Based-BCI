from psychopy import visual, event
import numpy as np
import time

window_size = [1200, 700]
cycles = 4
size = 160
frequency = np.array([10, 20, 15, 5, 12])

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


win = visual.Window(window_size,
                    units='pix',
                    monitor='testMonitor',
                    )
mouse = event.Mouse(win=win)
tex = np.array([[1, -1], [-1, 1]])
stim = []
selection_boxes = []
xy = []
for i in range(5):
    xy.append([i*220-window_size[0]/2+size, -window_size[1]/2+0.75*size+(not((i+1)%2))*(1.5*size)])
    stim.append(visual.GratingStim(win,
                                   tex=tex,
                                   size=size,
                                   units='pix',
                                   sf=cycles/size,
                                   pos=xy[i]))

    selection_boxes.append(visual.TextStim(win,
                                           text=','.join(text_boxes[i]),
                                            pos=[stim[i].pos[0], stim[i].pos[1] + 0.60 * size]))

instructions_text = 'Stare into the center of the box which corresponds to the character you want to choose'
instructions_box =visual.TextStim(win,
                                  text=instructions_text,
                                  pos=[0, window_size[1]/2-50],
                                  alignHoriz='center',
                                  alignVert='center')
'''visual.TextBox(window=win, text=instructions_text,
                             # possible letter area, # 50% transparent
                                  #font_size=18,
                                  font_color=[1, 1, 1],
                                  # possible letter area, # 50% transparent
                                  size=[1000,1000],
                                  # by the number of cols x # number of rows of text to support # instead of by a screen # units width x height.
                                  units='pix',
                                  pos=[0.0, 0.0],)'''


entered_background = visual.Rect(win,
                                 units='pix',
                                 width=window_size[0]/2,
                                 height=40,
                                 fillColor=[1,1,1],
                                 pos=[0, instructions_box.pos[1]-75])
entered_textbox = visual.TextStim(win,
                                  text='|',
                                  color=[-1,-1,-1],
                                  pos=[entered_background.pos[0], entered_background.pos[1]],
                                  alignHoriz='center',
                                  alignVert='center')
'''visual.TextBox(window=win, text=entered_text,
                             font_size=18,
                             font_color=[-1,-1,-1],
                             border_color=[-1,-1,1], # draw a blue border around stim
                             border_stroke_width=4, # border width of 4 pix.
                             background_color=[1,1,1], # fill the stim background
                             # possible letter area, # 50% transparent
                             textgrid_shape=[15,2], # specify area of text box
                            grid_color=[1,-1,-1,0.5],
                            grid_stroke_width=1,
                             # by the number of cols x # number of rows of text to support # instead of by a screen # units width x height.
                             units='pix',
                             pos=(0.0, instructions_box.pos[1]-100), # If the text string length < num rows * num cols in # textgrid_shape, how should text be justified?
                             grid_horz_justification='left', grid_vert_justification='top', )'''

selected_box = visual.Rect(win,
                           units='pix',
                           width=size,
                           height=size,
                           lineWidth=40,
                           lineColor=[-1, 1, -1],
                           fillColor=None,
                           pos=stim[2].pos)

frame_rate = win.getActualFrameRate()
frame_interval = 1/frame_rate
interval = 1/frequency
stim_frames = np.round(interval/frame_interval)
print("Frame rate is {0}. Actual Flashing Frequency will be {1}".format(frame_rate, str(1/(stim_frames*frame_interval))))


def group_choices(boxes):
    # Selection boxes can hold 1-4 values each
    boxes_len = len(boxes)
    if boxes_len / 2 <= len(stim):
        j_max = 2
    elif boxes_len / 3 <= len(stim):
        j_max = 3
    elif boxes_len / 4 <= len(stim):
        j_max = 4
    else:
        print("unsupported length, resetting to top level")
        reset_to_top_level()
        return
    offset = 0
    for i in range(0, len(stim)):
        selection_boxes[i].text = ''
        for j in range(j_max):
            try:
                selection_boxes[i].text += (boxes[i + j + offset] + ',')
            except:
                # reach end of selections, remove ',' from last box
                selection_boxes[i].text = selection_boxes[i].text[:-1]
                break
        offset += 1
        # remove ',' from end of each box selection
        selection_boxes[i].text = selection_boxes[i].text[:-1]


def reset_to_top_level():
    global bottom_level
    bottom_level = False
    for i, x in enumerate(stim):
        selection_boxes[i].text = ','.join(text_boxes[i])
        selection_boxes[i].draw()


def draw_screen():
    global bottom_level, n, entered_text, selected_index
    for i, x in enumerate(stim):
        stim[i].draw()
        selection_boxes[i].draw()
    instructions_box.draw()
    entered_background.draw()
    entered_textbox.draw()


def update_selection():
    global bottom_level
    # Not to the lowest level selection
    if not bottom_level:
        # cant display all element with each box
        if len(selection_boxes[selected_index].text.split(',')) > len(stim):
            group_choices(selection_boxes[selected_index].text.split(','))
        # Non empty selection which can be split into max boxes or less
        elif len(selection_boxes[selected_index].text) > 1:
            temp_text = selection_boxes[selected_index].text.split(',')
            bottom_level = True
            for i in range(len(stim)):
                try:
                    selection_boxes[i].text = temp_text[i]
                except:
                    # No more text to display, rest of boxes will have no text
                    selection_boxes[i].text = ''
                selection_boxes[i].draw()
        # Empty box chosen
        elif len(selection_boxes[selected_index].text) == 0:
            print("Empty box chosen, resetting to top level")
            reset_to_top_level()
        else:
            print("Unknown state, resetting")
            reset_to_top_level()
    # Bottom level selection
    else:
        temp_boxes = []
        # Non single character selection
        if len(selection_boxes[selected_index].text) > 1:
            # Special selection choices
            if selection_boxes[selected_index].text == num_text:
                temp_boxes = num_boxes
                if len(temp_boxes) > len(stim):
                    group_choices(temp_boxes)
                    bottom_level = False
            elif selection_boxes[selected_index].text == special_text:
                temp_boxes = special_boxes
                if len(temp_boxes) > len(stim):
                    group_choices(temp_boxes)
                    bottom_level = False
            elif selection_boxes[selected_index].text == enter_text:
                bottom_level = False
                entered_textbox.text = '|'
                #TODO something happens on enter
            elif selection_boxes[selected_index].text == del_text:
                bottom_level = False
                entered_textbox.text = entered_textbox.text[:-1]
            elif selection_boxes[selected_index].text == space_text:
                bottom_level = False
                entered_textbox.text += ' '
            else:
                print('Unknown case')
                pass

            # Display bottom level selection
            if bottom_level:
                for i in range(len(stim)):
                    try:
                        selection_boxes[i].text = temp_boxes[i]
                    except:
                        selection_boxes[i].text = ''
                    selection_boxes[i].draw()
            else:
                # something was selected and there was no sub-level
                # reset selection to top level
                if len(temp_boxes) == 0:
                    reset_to_top_level()
        # Empty box chosen
        elif len(selection_boxes[selected_index].text) == 0:
            print("Empty box chosen, resetting to top level")
            reset_to_top_level()
        # Single character selection
        else:
            bottom_level = False
            # Remove vertical bar
            if entered_textbox.text is '|':
                entered_textbox.text = selection_boxes[selected_index].text
            # Append selection
            else:
                entered_textbox.text += selection_boxes[selected_index].text
            reset_to_top_level()


selected_index = -1
selected_box_frames = 0
box_selected = False
bottom_level = False

def bci():
    global selected_index, selected_box_frames
    mouse.clickReset()
    win.flip()
    n = 0
    while not event.getKeys('escape'):
        if mouse.getPressed().count(1) and (selected_index == -1):
            mouse.clickReset()
            pos = mouse.getPos()
            for i in range(len(stim)):
                if stim[i].contains(pos):
                    selected_index = i
                    selected_box.pos = stim[selected_index].pos
                    selected_box_frames = 30
                    selected_box.draw()
                    break
        elif selected_index is not -1:
            selected_box.draw()
            selected_box_frames -= 1
            if selected_box_frames <= 0:
                update_selection()
                selected_index = -1

        for i, x in enumerate(stim_frames):
            # Update checkerboard
            if (n % x) == 0:
                stim[i].tex = -stim[i].tex

        draw_screen()
        win.flip()
        n += 1


bci()
