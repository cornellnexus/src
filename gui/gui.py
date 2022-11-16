'''
[gui] is the main file that will execute the GUI window.
Due to the nature of Matplotlib's animation module as well as other libraries we are importing,
we cannot break these sections into different files.
Thus, we will will break up the file into different sections:

1. MATPLOTLIB ROBOT MAPPING
2. MANAGING GUI WINDOW
3. GUI PROGRAM FLOW/SCRIPT

'''

from engine import robot
from gui.gui_popup import *
from gui.images import get_images
from gui.robot_data import RobotData
import gui.retrieve_inputs as retrieve_inputs
from engine.mission import ControlMode
from electrical.radio_module import RadioModule

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
from engine.is_raspberrypi import is_raspberrypi

import os
import math
import sys

import logging
import threading
import time
from constants.definitions import ROOT_DIR, CSV_PATH
import serial

#################### BEGINNING OF SECTION 1. MATPLOTLIB ROBOT MAPPING ####################
matplotlib.use('TkAgg')
# ser = serial.Serial("/dev/tty.usbserial-017543DC", 57600)
is_sim = is_raspberrypi() # Change to False when testing RPI to GUI connection

def get_control_mode(window):
    """
    Returns the last control mode given in the control_mode.csv file
    """
    #TODO: replace using telemetry data

    if is_sim: 
        file = open(CSV_PATH+"/control_mode_test.csv", "r")
        try:
            last_line = file.readlines()[-1]
            control_mode = last_line[last_line.index(".")+1:len(last_line)]
            window['-CONTROL_MODE_BUTTON-'].update(control_mode)
        except:
            pass
        file.close()
    else: 
        print("testing telemetry control mode")
        control_mode = ControlMode(robot_data.ctrl).name 
        print("control mode is: ", control_mode)
        window['-CONTROL_MODE_BUTTON-'].update(control_mode)

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
    ruh_m = plt.imread(ROOT_DIR + '/gui/map.png')
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_title('Cayuga Lake Shore')
    ax.set_xlim(BoundaryBox[0], BoundaryBox[1])
    ax.set_ylim(BoundaryBox[2], BoundaryBox[3])
    ax.imshow(ruh_m, zorder=0, extent=BoundaryBox, aspect='equal')
    fig.set_dpi(100)
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

    if is_sim:
        try:
            # TODO: Update to use gui database once it is updated in move_to_target_node
            # sim_packet = rpi_to_gui.readlines()[-1]  # get last line of csv file
            # robot_data.update_data(sim_packet)
            last_line = robot_loc_file.readlines()[-1]
            x, y, alpha = last_line.strip().split(',')
            x = float(x)
            y = float(y)
            alpha = float(alpha)
        except IndexError:
            return circle_patch, wedge_patch
    else:
        x, y, alpha = robot_data.coord[0], robot_data.coord[1], robot_data.coord[2]
        
    degrees = math.degrees(alpha)
    circle_patch.center = (x, y)
    wedge_patch.update({'center': [x, y]})
    wedge_patch.theta1 = degrees - 10
    wedge_patch.theta2 = degrees + 10 #10 is a temporary constant we will use

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


    image_data = get_images()
    left_col = [[sg.Canvas(key="-CANVAS-")], [sg.Image(key='-PROGRESS-', data=image_data[0])], [sg.Image(key='-MINIMAP-', data=image_data[1]), sg.Image(key='-CAMERA-', data=image_data[2])]]
    right_col = [
                [sg.Image(key='-LOGO-', data=image_data[3])], 
                [sg.InputText(size=(30,1), key="-COMMANDLINE-", font=('Courier New', 20))],
                [sg.Button('Submit', visible=False, bind_return_key=True)],
                [sg.Multiline(current_output, key = "-OUTPUT-", size=(40,8), disabled=True, font=('Courier New', 20))],
                [sg.Text("Current Coordinates: ______", size=(30,))],
                [sg.Text("Current Phase: ______", key = "-PHASE-", size=(30,))],
                [sg.Button('Autonomous', key = "-CONTROL_MODE_BUTTON-"), sg.Button('Track Location'), sg.Button('Traversal Phase'), sg.Button('Simulation'), sg.Button("Startup Base Station"), sg.Button('Read RPI Comms')],
                [sg.Multiline(str(robot_data), key = "-DATA-", size=(40,8), disabled=True, font=('Courier New', 20))]
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
    return window


def update_input(str, window):
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

def update_robot_data(window, packet):
    """

    Args:
        window: PySimpleGUI main window

    Updates the data multiline textbook to display current robot telemetry data

    """
    try:
        robot_data.update_data(packet)
        new_text = str(robot_data)
        window['-DATA-'].update(new_text)
    except:
        pass


def run_gui():
    """
    Run the main GUI window.

    Contains main control loop, which constantly checks for user interaction with the window and adjusts accordingly.
    """
    def run_simulation(name):
        logging.info("Thread %s: starting", name)
        os.system("python -m gui.retrieve_inputs &")
        os.system("python -m engine.sim_trajectory")
        logging.info("Thread %s: finishing", name)

    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    window = setup_gui()
    current_row = 0
    reading_inputs = False
    packets = []
    rs = RadioModule(False)

    while True:  # Event Loop
        event, values = window.read(timeout=10)

        try:
            last_phase_line = robot_phase_file.readlines()[-1].strip()
            phase_loc = last_phase_line.find(".")
            phase = last_phase_line[phase_loc+1:]
            window["-PHASE-"].update("Current Phase: " + phase)
        except:
            pass

        if event == sg.WIN_CLOSED or event == 'Cancel':
            # once gui.gui.py is closed, close any other simulation scripts
            os.system("pkill -f engine.sim_trajectory &")
            os.system("pkill -f gui.retrieve_inputs")
            break
        if event == 'Show':
            # Update the "output" text element to be the value of "commandline" 
            #element
            window['-OUTPUT-'].update(values['-COMMANDLINE-'])
            break
        if event == 'Submit':
            print('Command entered: %s'% window['-COMMANDLINE-'].get())
            global current_output
            current_output = update_input(window['-COMMANDLINE-'].get(), window)
            # Empty Command Line for next input
            window['-COMMANDLINE-'].update("")
        if event == 'Simulation':
            simulation_thread = threading.Thread(target=run_simulation, args=(1,), daemon=True)
            simulation_thread.start()
        if event == 'Read RPI Comms':
            #Replacement for csv
            print("reading comms")
            reading_inputs = True

        if (reading_inputs):
            if len(packets) < 5:
                try:
                    packet = ser.readline().decode('utf-8')
                    if 80 < len(packet) < 150:  # check if packet length is appropriate
                        packets.append(packet)
                except:
                    pass
            if len(packets) == 5:
                try: 
                    valid_packet = retrieve_inputs.validate_packet(packets)
                    update_robot_data(window, valid_packet)
                except: 
                    pass 
                packets = []

        if event == 'Startup Base Station':
            rs.setup_basestation()

        get_control_mode(window)
        
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
print("starting gui")
close_gui = run_popup()
if not close_gui:
    input_data = get_input_data() #Runs the gui popup asking for latitude and longitude bounds
    bounds = input_data["long_min"], input_data["long_max"], input_data["lat_min"], input_data["lat_max"]

    #Run the gui if the user doesn't close out of the window
    if bounds != None:

        fig, ax = setup(bounds)  # Set up matplotlib map figure
        circle_patch, wedge_patch = make_robot_symbol()  # Create a circle and wedge objet for robot location and heading, respectively
        # Begins the constant animation/updates of robot location and heading
        
        robot_phase_file = open(CSV_PATH + '/phases.csv', "r")
        if is_sim:
            # open csv file for simulated rpi to gui data
            robot_loc_file = open(CSV_PATH + '/datastore.csv', "r")  # open csv file of robot location
            # rpi_to_gui = open(CSV_PATH+"/rpi_to_gui_simulation.csv", "r")

        current_output = "Welcome! If you enter commands in the text field above, \nthe results will appear here. Try typing <print_coords>."
        robot_data = RobotData("phse:1;p_weight:00.0;acc:0.00,0.00,0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00,000.00;bat:000;ctrl:1")

        anim = animation.FuncAnimation(fig, animate,
                                       init_func=init,
                                       frames=360,
                                       interval=20,
                                       blit=True)
        run_gui() #Start up the main GUI window
        robot_loc_file.close()
        robot_phase_file.close()
        # if is_sim:
        #     rpi_to_gui.close()


#################### END OF SECTION 3. GUI PROGRAM FLOW/SCRIPT ####################
