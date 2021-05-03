'''
gui_popup
'''

import PySimpleGUI as sg

'''
Creates layout for the popup window
'''
def set_up():
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
    return window    

'''
Runs the popup window. Input 0, 10, 0, 10 for the 
corresponding values below.
'''
def run_popup():
    window = set_up()
    popup_open = True
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


