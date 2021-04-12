'''
gui_popup
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

popup_open = True
while True: # The Event Loop
    event, values = window.read() 
    #TODO fix close out window to not reference popup map bounds
    #check for (event == sg.WIN_CLOSED) 
    if event == 'Cancel':
        window.close()
        popup_open = False
        break
    
    if event == sg.WIN_CLOSED:
        popup_open = False
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
            popup_open = False
            break
    except:
        sg.popup('Invalid map bounds')


def get_coord_inputs():
    return (long_min, long_max, lat_min, lat_max)


