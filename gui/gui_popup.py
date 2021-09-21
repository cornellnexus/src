'''
[gui_popup] contains functions for the initial pop-up window.
The main functions of this file is [run_popup()].
This will setup the pop-up window and manage the window activity.


Purpose: The pop-up window will prompt the user to input information for the GUI settings (e.g. map bounds).

Future Plans: If other featurue are added to the GUI, add necessary setup prompts and components to [set_up].
'''
 
import PySimpleGUI as sg
 

def set_up():
    """
    Returns: PySimpleGUI Window object [window].

    [set_up] creates the layout for the popup window.

    """
    layout = [[sg.Text('Enter Map Bounds:')],
           [sg.Text('Minimum Longitude:')],
           [sg.InputText(key='-MINLONG-')],
           [sg.Text('Maximum Longitude:')],
           [sg.InputText(key='-MAXLONG-')],
           [sg.Text('Minimum Lattitude:')],
           [sg.InputText(key='-MINLAT-')],
           [sg.Text('Maximum Lattitude:')],
           [sg.InputText(key='-MAXLAT-')],
           [sg.Button('Store Data')],
           [sg.Submit(), sg.Cancel()]]
    window = sg.Window('Cornell Nexus', layout)
    return window
 

def run_popup():
    """
    Return (long_min, long_max, lat_min, lat_max) where (long_min, long_max, lat_min, lat_max) are
    the minimum longitude, maximum longitude, minimum latitude, and maximum latitude respectively.

    [run_popup] runs the popup window. The <Submit> button will proceed to opening the GUI if inputs are valid.
    The <Cancel> button and window close button will close out the pop-window and terminate the gui.py script.

    Example usage of pop-up window: Input 0, 10, 0, 10 for the minimum longitude,
    maximum longitude, minimum latitude, and maximum latitude respectively.
    This will set the map bounds accordingly on the GUI window after submit is pressed.
    """
    window = set_up() #Setup window
    popup_open = True # Keeps track of pop-window status
    while popup_open: # The Event Loop
       event, values = window.read()
       #TODO fix close out window to not reference popup map bounds
       if event == 'Cancel':
           window.close()
           popup_open = False
           break

       if event == sg.WIN_CLOSED:
           popup_open = False
           break

       if event == 'Store Data':
           print("TODO")
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
               return (long_min, long_max, lat_min, lat_max)
               popup_open = False
               break
       except:
           sg.popup('Invalid map bounds')
 
# def get_coord_inputs():
#     return (long_min, long_max, lat_min, lat_max)
 
 
 

