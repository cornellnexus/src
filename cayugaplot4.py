'''
cayugaplot4 is the current working version of cayuga plot. As of now, it
plots the robot's location and heading using an arbitrarily defined animate 
function. It uses matplotlib wedges and circles and the funcAnimation 
feature to plot. 
The goal is to 
1) rewrite the animate function so that the robot's state
   is plotted according to the Kalman Filter output, and 
2) embed this graph into the GUI. 
'''

import numpy as np
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from matplotlib.widgets import Button
from UserUtils import *

# resources needed for GUI
from matplotlib.ticker import NullFormatter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
matplotlib.use('TkAgg')

# ------------------------------------- GUI 1 --------------------------------------------------
layout = [[sg.Text('Enter Map Bounds:')],   
        [sg.Text('Minimum Longitude:')],
        [sg.InputText(key='-MINLONG-')],   
        [sg.Text('Maximum Longitude:')],
        [sg.InputText(key='-MAXLONG-')], 
        [sg.Text('Minimum Lattitude:')],
        [sg.InputText(key='-MINLAT-')], 
        [sg.Text('Maximum Lattitude:')],
        [sg.InputText(key='-MAXLAT-')],    
        [sg.Submit(), sg.Cancel()]]      

window = sg.Window('Cornell Nexus', layout)    

while True: # The Event Loop
    event, values = window.read() 
    #TODO fix close out window to not reference popup map bounds
    #check for (event == sg.WIN_CLOSED) 
    if event == 'Cancel':
        window.close()
        break

    try:
        long_min = int(values['-MINLONG-'])
        long_max = int(values['-MAXLONG-'])
        lat_min = int(values['-MINLAT-'])
        lat_max = int(values['-MAXLAT-'])

        if long_max <= long_min or lat_max <= lat_min:
            sg.popup('Invalid map bounds')
        else:
            window.close()
            break
    except:
        sg.popup('Invalid map bounds')


def get_coord_inputs():
    return (long_min, long_max, lat_min, lat_max)

# ------------------------------------- matplotlib --------------------------------------------------

'''
When running this file, input 0, 10, 0, 10 for the corresponding values
below. get_coord_inputs() is documented in UserUtils.py
'''
longMin, longMax, latMin, latMax = get_coord_inputs()

'''
Set up window
'''
BoundaryBox = [longMin, longMax, latMin, latMax]
ruh_m = plt.imread('map.png')
fig, ax = plt.subplots(figsize=(8, 7))
ax.set_title('Cayuga Lake Shore')
ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')
fig.set_dpi(100)
# fig.patch.set_facecolor('blue')
# fig.patch.set_alpha(0.5)
# fig.set_size_inches(7, 6.5) original size
fig.set_size_inches(6, 5)

'''
Create circle patch object to represent moving robot, and wedge patch for
robot's heading
'''
circle_patch = plt.Circle((5, 5), 0.1, fc='black')
wedge_patch = patch.Wedge(
    (5, 1), 1, 30, 50, animated=True, fill=False, width=.9, ec='r', hatch='xx')

def init():
    circle_patch.center = (5, 5)
    ax.add_patch(circle_patch)
    # ax.add_patch(arc_patch)
    ax.add_patch(wedge_patch)
    return circle_patch, wedge_patch

'''
Update circle and wedge patch poses arbitrarily with each time step, to 
simulate movement
'''
def animate(i):
    x, y = circle_patch.center
    x = 5 + 3 * np.sin(np.radians(i))
    y = 5 + 3 * np.cos(np.radians(i))
    circle_patch.center = (x, y)
    wedge_patch.update({'center': [x, y]})
    wedge_patch.theta1 += (.5 * x)
    wedge_patch.theta2 += (.5 * x)
    wedge_patch.theta1 = wedge_patch.theta1 % 360
    wedge_patch.theta2 = wedge_patch.theta2 % 360

    # print(wedge_patch.theta1, wedge_patch.theta2)
    # print(wedge_patch.center)
    return circle_patch, wedge_patch

'''
Begins the animation
'''
anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=360,
                               interval=20,
                               blit=True)

#plt.show()
fig = plt.gcf()

# ------------------------------------- GUI 2 --------------------------------------------------

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

# ------ handle images
import os.path
import PIL.Image
import io
import base64
def convert_to_bytes(file_or_bytes, resize=None):
    '''
    Will convert into bytes and optionally resize an image that is a file or a base64 bytes object.
    Turns into  PNG format in the process so that can be displayed by tkinter
    :param file_or_bytes: either a string filename or a bytes base64 image object
    :type file_or_bytes:  (Union[str, bytes])
    :param resize:  optional new size
    :type resize: (Tuple[int, int] or None)
    :return: (bytes) a byte-string object
    :rtype: (bytes)
    '''
    if isinstance(file_or_bytes, str):
        img = PIL.Image.open(file_or_bytes)
    else:
        try:
            img = PIL.Image.open(io.BytesIO(base64.b64decode(file_or_bytes)))
        except Exception as e:
            dataBytesIO = io.BytesIO(file_or_bytes)
            img = PIL.Image.open(dataBytesIO)

    cur_width, cur_height = img.size
    if resize:
        new_width, new_height = resize
        scale = min(new_height/cur_height, new_width/cur_width)
        img = img.resize((int(cur_width*scale), int(cur_height*scale)), PIL.Image.ANTIALIAS)
    bio = io.BytesIO()
    img.save(bio, format="PNG")
    del img
    return bio.getvalue()

cwd = os.getcwd()
images = ['Images/Progress Bar.png', 'Images/zoomview.png', 'Images/Camera.png', 'Images/final_logo 1.png']
image_data = []
for image in images:
    image_path = os.path.join(cwd,image)
    image_data.append(convert_to_bytes(image_path))

# -------- end of handling images

current_output = "Welcome! If you enter commands in the text field above, \
the results will appear here. Try typing <print_coords>."

current_data = "Pounds of Collected Plastic: ____\nAcceleration: ____\nCurrent Distance to Next Node: ____\nTotal Area Traversed: ____\nRotation: ____\nLast Node Visited: ____\nEstimated Time of Arrival:____\nMotor Velocity: ____\nNext Node to Visit: ____"

left_col = [[sg.Canvas(key="-CANVAS-")], [sg.Image(key='-PROGRESS-', data=image_data[0])], [sg.Image(key='-MINIMAP-', data=image_data[1]), sg.Image(key='-CAMERA-', data=image_data[2])]]
right_col = [
            [sg.Image(key='-LOGO-', data=image_data[3])], 
            [sg.InputText(size=(30,1), key="-COMMANDLINE-")], 
            [sg.Button('Submit', visible=False, bind_return_key=True)],
            [sg.Multiline(current_output, key = "-OUTPUT-", size=(40,8))],
            [sg.Text("Current Coordinates: ______")],
            [sg.Button('Autonomous'), sg.Button('Store Data'), sg.Button('Track Location')],
            [sg.Multiline(current_data, key = "-DATA-", size=(40,8))]
        ]
 
layout = [[sg.Column(left_col, element_justification='c'), sg.VSeperator(), \
sg.Column(right_col, element_justification='c')]]

# create the form and show it without the plot
window = sg.Window('Cornell Nexus', layout, finalize=True, \
element_justification='center', font='Helvetica 18', location=(0,0), \
size=(1200,700), resizable=True)

# add the plot to the window
fig_canvas_agg = draw_figure(window['-CANVAS-'].TKCanvas, fig)

'''
Update outputlog to reflect the result of entering the command [str] in the
commandline textfield
'''
def update_input(str, current_output):
    if str == "print_coords":
        #update the output
        new_output = "Current Coordinates: (0,1)" + "\n" + current_output
    else:
        new_output = "<" + str + "> is not a valid command. Please try again. " \
        + "\n" + current_output
    window['-OUTPUT-'].update(new_output) 
    return new_output
    

while True:  # Event Loop
    event, values = window.read()
    # print(event, values)
    if event == sg.WIN_CLOSED or event == 'Cancel':
        break
    if event == 'Show':
        # Update the "output" text element to be the value of "commandline" 
        #element
        window['-OUTPUT-'].update(values['-COMMANDLINE-'])
    if event == 'Submit':
        print('Command entered: %s'% window['-COMMANDLINE-'].get())
        current_output = update_input(window['-COMMANDLINE-'].get(), \
        current_output)
        # Empty Command Line for next input
        window['-COMMANDLINE-'].update("")

        

window.close()


