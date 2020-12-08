"""
Asks the user to provide long min, long max, lat min, lat max. Raises exception if the inputs are
malformed. Returns the values in the format: (long_min, long_max, lat_min, lat_max)
"""
def getLongLatMinMaxFromUser():
        try:
            long_min = float(input("Enter minimum longitude: "))
            long_max = float(input("Enter maximum longitude: "))
            if long_max <= long_min:
                raise Exception ("The maximum longitude must be larger than the minimum longitude")
            lat_min = float(input ("Enter minimum latitude: "))
            lat_max = float(input("Enter maximum latitude: "))
            if lat_max <= lat_min:
                raise Exception ("The maximum latitude must be larger than the minimum latitude")
            
            return (long_min, long_max, lat_min, lat_max)
        except ValueError:
            print("Please enter a number")


def getDisplayGuiFromUserInput():
    try:
        shouldDisplayGui = input("Do you want to display a GUI of the DFS traversal in real-time? Type y for yes and n for no. \n")
        if shouldDisplayGui == "y":
            return True
        elif shouldDisplayGui == "n":
            return False
        else:
            raise Exception("Invalid Input")
    except:
        print("Invalid Input. Try again. \n")
        getDisplayGuiFromUserInput()