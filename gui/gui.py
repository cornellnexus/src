'''
[gui] is the main file that will execute the GUI window.
Due to the nature of Matplotlib's animation module as well as other libraries we are importing,
we cannot break these sections into different files.
Thus, we will will break up the file into different sections:

1. MATPLOTLIB ROBOT MAPPING
2. MANAGING GUI WINDOW
3. GUI PROGRAM FLOW/SCRIPT

'''
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
#from csv.datastore import *
import gui_popup #pop-up window
from images import get_images

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg

#################### BEGINNING OF SECTION 1. MATPLOTLIB ROBOT MAPPING ####################
matplotlib.use('TkAgg')

def setup(bounds):
    """
    Return Matplotlib Figure [fig] and single Matplotlib Axes object [ax].

    Sets up and stores the structure of the bounding map for robot traversal in [fig] and [ax].

    Parameters: [bounds] is the minimum longitude, maximum longitude, minimum latitude, and maximum latitude
    (receieved from gui_popup).

    Precondition: [bounds] is an integer 4-tuple.
    """

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


def make_robot_symbol():
    """
    Return Matplotlib Circle [circle_patch] and Matplotlib patches.Wedge [wedge_patch].

    Create circle patch object to represent moving robot, and wedge patch for
    robot's heading.
    """

    circle_patch = plt.Circle((5, 5), 0.1, fc='black')
    wedge_patch = patch.Wedge(
        (5, 1), 1, 30, 50, animated=True, fill=False, width=.9, ec='r', hatch='xx')
    return circle_patch, wedge_patch

def init():
    """
    Return Matplotlib Circle [circle_patch] and Matplotlib patches.Wedge [wedge_patch].

    Initializes the global [circle_patch] and [wedge_patch] objects onto the bounding map, e.g. global [ax].
    """

    circle_patch.center = (5, 5)
    ax.add_patch(circle_patch)
    # ax.add_patch(arc_patch)
    ax.add_patch(wedge_patch)
    print('init')
    return circle_patch, wedge_patch

def animate(i):
    """
    Return Matplotlib Circle [circle_patch] and Matplotlib patches.Wedge [wedge_patch].

    Update circle and wedge patch poses arbitrarily with each time step, to
    simulate movement. Follows the requirement for Matplotlib animation functions:
    'The function to call at each frame. The first argument will be the next value in frames.
    Any additional positional arguments can be supplied via the fargs parameter.'

    Parameters: [i] is the frame.

    Precondition: [i] is an integer.
    """

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


#################### END OF SECTION 1. MATPLOTLIB ROBOT MAPPING ####################
#################### BEGINNING OF SECTION 2. MANAGING GUI WINDOW ####################

def draw_figure(canvas, figure):
    """
    Return Matplotlib Circle [figure_canvas_agg] and Matplotlib patches.Wedge [wedge_patch].

    Update circle and wedge patch poses arbitrarily with each time step, to
    simulate movement. Follows the requirement for Matplotlib animation functions:
    'The function to call at each frame. The first argument will be the next value in frames.
    Any additional positional arguments can be supplied via the fargs parameter.'

    Parameters: [i] is the frame.

    Precondition: [i] is an integer.
    """

    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def setup_gui():
    """
    Return [(window, data)], a 2-tuple of a PySimpleGUI window object, containing the GUI window information,
    and a dictionary containing static data information.

    Initialize and setup the GUI window.
    """

    data = {
        "current_output" : "Welcome! If you enter commands in the text field above, \nthe results will appear here. Try typing <print_coords>.",
        "current_data" : "Pounds of Collected Plastic: ____\nAcceleration: ____\nCurrent Distance to Next Node: ____\nTotal Area Traversed: ____\nRotation: ____\nLast Node Visited: ____\nEstimated Time of Arrival:____\nMotor Velocity: ____\nNext Node to Visit: ____"
    }
    image_data = get_images();
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


def update_input(str, window, current_output):
    """
    Return the String [new_output]

    Update the outputlog to reflect the result of entering the command [str] in the
    commandline textfield.

    Parameters: [str] is the user's input in the commandline textfield
                [window] is the object corresponding to the GUI window
                [current_output] is the most recent output in the outputlog

    Preconditions:  [str] is a String
                    [window] is a PySimpleGUI window object
                    [current_output] is a String
    """

    if str == "print_coords":
        #update the output
        new_output = "Current Coordinates: (0,1)" + "\n" + current_output
    else:
        new_output = "<" + str + "> is not a valid command. Please try again. " \
        + "\n" + current_output
    window['-OUTPUT-'].update(new_output) 
    return new_output


def run_gui():
    """
    Run the main GUI window.

    Contains main control loop, which constantly checks for user interaction with the window and adjusts accordingly.
    """

    window, data = setup_gui()

    current_row = 0
    while True:  # Event Loop
        event, values = window.read()

        #TBD test code
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

#################### END OF SECTION 2. MANAGING GUI WINDOW ####################
#################### BEGINNING OF SECTION 3. GUI PROGRAM FLOW/SCRIPT ####################

'''
General Flow of GUI program:

1. Open popup window to determine user inputs and store necessary information 
(If the user inputted valid information/didn't close out of the window, continue)
2. Use user input information from the popup to setup Matplotlib robot mapping animation
3. Open main GUI window
4. Run GUI
'''
close_gui = gui_popup.run_popup()
if not close_gui:
    input_data = gui_popup.get_input_data() #Runs the gui popup asking for latitude and longitude bounds
    bounds = input_data["long_min"], input_data["long_max"], input_data["lat_min"], input_data["lat_max"]

    #Run the gui if the user doesn't close out of the window
    if bounds != None:
        fig, ax = setup(bounds)  # Set up matplotlib map figure
        circle_patch, wedge_patch = make_robot_symbol()  # Create a circle and wedge objet for robot location and heading, respectively
        # Begins the constant animation/updates of robot location and heading
        anim = animation.FuncAnimation(fig, animate,
                                       init_func=init,
                                       frames=360,
                                       interval=20,
                                       blit=True)
        run_gui() #Start up the main GUI window

#################### END OF SECTION 3. GUI PROGRAM FLOW/SCRIPT ####################