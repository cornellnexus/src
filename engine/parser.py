import argparse
import math

def parse_main():
   """
   Parse user defined parameters in command-line.

   Searching for
      xpos: a float for the robot's initial x position in coordinate space
      ypos: a float for the robot's initial y position in coordinate space
      heading: a float for the robot's initial heading
      base_lat: a float for the base stations's latitude coordinate"
      base_long: a float for the base stations's longitude coordinate"
      lawnmower: a boolean defaulted to True unless the user specifies otherwise

   By default, the robot is centered at (0, 0) in coordinate space facing northeast.
   The base station is centered at (latitude, longitude)
   corresponding to (0, 0) with respect to the engineering quad grid.

   Return a dictionary of the user inputted arguments above.
   """
   parser = argparse.ArgumentParser(description='Process Mission information.')
   parser.add_argument("xpos", type=float, nargs='?', const=1, default=0,
                    help="a float for the robot's initial x position in coordinate space")
   parser.add_argument("ypos", type=float, nargs='?', const=1, default=0,
                    help="a float for the robot's initial y position in coordinate space")
   parser.add_argument("heading", type=float, nargs='?', const=1, default=math.pi/4,
                    help="a float for the robot's initial heading")
   parser.add_argument("base_lat", type=float, nargs='?', const=1, default=42.444250,
                    help="a float for the base stations's latitude coordinate")
   parser.add_argument("base_long", type=float, nargs='?', const=1, default=-76.483682,
                    help="a float for the base stations's longitude coordinate")
   parser.add_argument("-l", "--lawnmower", action="store_false",
                    help="Set intial control mode to lawnmower") # TODO: add for additional control modes, after centralizing the different lawnmowers
   args = parser.parse_args()
   return(vars(args))

parse_main()