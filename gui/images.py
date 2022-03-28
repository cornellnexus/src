'''

[images] handles loading images for the GUI

'''
import sys
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

def get_images():
    """
    Return [image_data], an array of image information.

    Stores desired images to display on the main GUI window into [image_data].
    """
    cwd = os.getcwd()
    # sys.path.append(cwd[0:cwd.index('gui')-1]+"/images")
    sys.path.append(cwd + "/gui_images")
    images = ['/Progress Bar.png', '/zoomview.png', '/Camera.png', '/final_logo 1.png']
    image_data = []
    for image in images:
        # image_path = os.path.join(cwd,image)
        image_path = sys.path[-1] + image
        image_data.append(convert_to_bytes(image_path))
    return image_data