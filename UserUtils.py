"""
Asks the user to provide long min, long max, lat min, lat max. 
Precondition: Minimum float values must be less than maximum float values
Returns the values in the format: (long_min, long_max, lat_min, lat_max)
"""
def get_user_inputs():
        try:
            long_min = float(input("Enter minimum longitude: "))
            long_max = float(input("Enter maximum longitude: "))
            if long_max <= long_min:
                raise Exception ("The maximum longitude must be larger"
                                    " than the minimum longitude")
            lat_min = float(input ("Enter minimum latitude: "))
            lat_max = float(input("Enter maximum latitude: "))
            if lat_max <= lat_min:
                raise Exception ("The maximum latitude must be larger"  
                                    " than the minimum latitude")
            
            return (long_min, long_max, lat_min, lat_max)
        except ValueError:
            print("Please enter a number")


"""Asks the user in the terminal whether they want to display the traversal in 
a GUI. If 'y' is pressed, a gui is displayed.
Precondition: User must input 'y' or 'n'
"""
def get_display_gui():
    try:
        display_gui = input("Do you want to display a GUI of the" 
                    "traversal in real-time? Type y for yes and n for no. \n")
        if display_gui == "y":
            return True
        elif display_gui == "n":
            return False
        else:
            raise Exception("Invalid Input")
    except:
        print("Invalid Input. Try again. \n")
        get_display_gui()