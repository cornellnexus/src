from pynput import keyboard
from movement import *

# int rpm = 0;
# int
c = Commands(0.0, 0.0, 0.0)


def on_press(key):
    try:
        if key.char == 'w':
            c.move_with_key(1)
        elif key.char == 's':
            c.move_with_key(-1)
        elif key.char == 'a':
            c.turn_with_key(-1)
        elif key.char == 'd':
            c.turn_with_key(1)
        else:
            print('alphanumeric key {0} pressed'.format(
                key.char))

    except AttributeError:
        print('special key {0} pressed'.format(
            key))


def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

# on_press(keyboard.KeyCode)
