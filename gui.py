import matplotlibGui
import gui_popup

import matplotlib
import os.path
import PIL.Image
import io
import base64
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
matplotlib.use('TkAgg')

# ------ handle images

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
                [sg.Button('Autonomous'), sg.Button('Store Data'), sg.Button('Track Location')],
                [sg.Multiline(data["current_data"], key = "-DATA-", size=(40,8))]
            ]
    
    layout = [[sg.Column(left_col, element_justification='c'), sg.VSeperator(), \
    sg.Column(right_col, element_justification='c')]]

    # create the form and show it without the plot
    window = sg.Window('Cornell Nexus', layout, finalize=True, \
    element_justification='center', font='Helvetica 18', location=(0,0), \
    size=(1200,700), resizable=True)

    plt, anim = matplotlibGui.get_graph_info(bounds)
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
    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)
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
#run the gui if the user doesn't close out of the window
if bounds != None:
    run_gui(bounds)