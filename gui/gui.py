import numpy as np
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from matplotlib.widgets import Button
import sys
#from csv.datastore import *

matplotlib.use('TkAgg')

'''
Set up window
'''
def setup(bounds): 
    longMin, longMax, latMin, latMax = bounds
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
    return fig, ax

'''
Create circle patch object to represent moving robot, and wedge patch for
robot's heading
'''
def make_robot_symbol():
    circle_patch = plt.Circle((5, 5), 0.1, fc='black')
    wedge_patch = patch.Wedge(
        (5, 1), 1, 30, 50, animated=True, fill=False, width=.9, ec='r', hatch='xx')
    return circle_patch, wedge_patch

def init():
    circle_patch.center = (5, 5)
    ax.add_patch(circle_patch)
    # ax.add_patch(arc_patch)
    ax.add_patch(wedge_patch)
    print('init')
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
    # only getting called once 
    # should be getting called repeatedly
    return circle_patch, wedge_patch


# import matplotlibGui
import gui_popup

import matplotlib
import os.path
import PIL.Image
import io
import base64
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
matplotlib.use('TkAgg')

# ---------------------------- handle images -------------------------------------------------------------

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
sys.path.append(cwd[0:cwd.index('gui')-1]+"/images")
images = ['/Progress Bar.png', '/zoomview.png', '/Camera.png', '/final_logo 1.png']
image_data = []
for image in images:
    image_path = os.path.join(cwd,image)
    image_path = sys.path[-1] + image
    image_data.append(convert_to_bytes(image_path))

# ---------------------------- end of handling images -------------------------------------------------------------

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def setup_gui(bounds):
    data = {
        "current_output" : "Welcome! If you enter commands in the text field above, \nthe results will appear here. Try typing <print_coords>.",
        "current_data" : "Pounds of Collected Plastic: ____\nAcceleration: ____\nCurrent Distance to Next Node: ____\nTotal Area Traversed: ____\nRotation: ____\nLast Node Visited: ____\nEstimated Time of Arrival:____\nMotor Velocity: ____\nNext Node to Visit: ____"
    }

    left_col = [[sg.Canvas(key="-CANVAS-")], [sg.Image(key='-PROGRESS-', data=image_data[0])], [sg.Image(key='-MINIMAP-', data=image_data[1]), sg.Image(key='-CAMERA-', data=image_data[2])]]
    right_col = [
                [sg.Image(key='-LOGO-', data=image_data[3])], 
                [sg.InputText(size=(30,1), key="-COMMANDLINE-")], 
                [sg.Button('Submit', visible=False, bind_return_key=True)],
                [sg.Multiline(data["current_output"], key = "-OUTPUT-", size=(40,8))],
                [sg.Text("Current Coordinates: ______")],
                [sg.Button('Autonomous'), sg.Button('Track Location')],
                [sg.Multiline(data["current_data"], key = "-DATA-", size=(40,8))]
            ]
    
    layout = [[sg.Column(left_col, element_justification='c'), sg.VSeperator(), \
    sg.Column(right_col, element_justification='c')]]

    # create the form and show it without the plot
    window = sg.Window('Cornell Nexus', layout, finalize=True, \
    element_justification='center', font='Helvetica 18', location=(0,0), \
    size=(1200,700), resizable=True)
    
    fig = plt.gcf()
    # add the plot to the window
    fig_canvas_agg = draw_figure(window['-CANVAS-'].TKCanvas, fig)
    return (window, data)

'''
Update outputlog to reflect the result of entering the command [str] in the
commandline textfield
'''
def update_input(str, window, current_output):
    if str == "print_coords":
        #update the output
        new_output = "Current Coordinates: (0,1)" + "\n" + current_output
    else:
        new_output = "<" + str + "> is not a valid command. Please try again. " \
        + "\n" + current_output
    window['-OUTPUT-'].update(new_output) 
    return new_output


'''
Run the GUI
'''
def run_gui(bounds):
    window, data = setup_gui(bounds)

    current_row = 0
    while True:  # Event Loop
        event, values = window.read()

        # f1 = open('datastore.csv', "r")
        # last_line = f1.readlines()[-1]

        # print(last_line.strip()) 
        # if store data toggled, store data function
        # update circle and wedge
        # wedge orientation corresponds to theta
        # print(row)

        if event == sg.WIN_CLOSED or event == 'Cancel':
            break
        if event == 'Show':
            # Update the "output" text element to be the value of "commandline" 
            #element
            window['-OUTPUT-'].update(values['-COMMANDLINE-'])
            break
        if event == 'Submit':
            print('Command entered: %s'% window['-COMMANDLINE-'].get())
            data["current_output"] = update_input(window['-COMMANDLINE-'].get(), \
            window, data["current_output"])
            # Empty Command Line for next input
            window['-COMMANDLINE-'].update("")
    window.close()

#runs the gui popup asking for latitude and longitude
bounds = gui_popup.run_popup()

#set up matplotlib animation
fig, ax = setup(bounds)
circle_patch, wedge_patch = make_robot_symbol()
#Begins the animation
anim = animation.FuncAnimation(fig, animate,
                            init_func=init,
                            frames=360,
                            interval=20,
                            blit=True)

#run the gui if the user doesn't close out of the window
if bounds != None:
    run_gui(bounds)