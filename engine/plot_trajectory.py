import numpy as np
import math
import matplotlib.pyplot as plt
from engine.control_mode import ControlMode

from matplotlib import animation as animation
from matplotlib import patches as patch


def waypoints_to_array(waypoints):
    """
    Arguments:
        waypoints: a list of Node objects

    Returns:
        waypoints_arr: a 1D np array of coordinates, each of which corresponds to a waypoint in waypoints
    """

    n = len(waypoints)
    waypoints_arr = np.empty([n, 2])
    for i in range(n):
        waypoints_arr[i, :] = np.asarray(waypoints[i].get_m_coords())
    return waypoints_arr


def get_plot_boundaries(nodes, delta):
    """
    Given some grid to be plotted, and a delta value, returns the desired 
    x limits and y limits for the plot.

    Arguments:
        nodes: a Grid object
        delta: the width of the border between
    Returns:
        xlim: a list containing the left and right boundaries of the grid, taking delta into account
        ylim: a list containing the top and bottom boundaries of the grid, taking delta into account
    """

    size = np.shape(nodes)
    min_coords = nodes[0, 0].get_m_coords()
    max_coords = nodes[size[0] - 1, size[1] - 1].get_m_coords()
    xlim = [min_coords[0] - delta, max_coords[0] + delta]
    ylim = [min_coords[1] - delta, max_coords[1] + delta]
    return xlim, ylim

def init():
    circle_patch.center = (0, 0)
    circle_patch_base.center = mission.mission_state.base_station.position
    ax.add_patch(circle_patch)
    ax.add_patch(wedge_patch)
    ax.add_patch(circle_patch_base)
    ax.add_patch(wedge_patch_base)
    return circle_patch, wedge_patch

def animate(i, m):
    x_coord = m.mission_state.robot.robot_state.truthpose[i, 0]
    y_coord = m.mission_state.robot.robot_state.truthpose[i, 1]
    circle_patch.center = (x_coord, y_coord)
    wedge_patch.update({"center": [x_coord, y_coord]})
    wedge_patch.theta1 = np.degrees(m.mission_state.robot.robot_state.truthpose[i, 2]) - 10
    wedge_patch.theta2 = np.degrees(m.mission_state.robot.robot_state.truthpose[i, 2]) + 10
    return circle_patch, wedge_patch

def plot_sim_traj(m):
  global mission, ax, circle_patch, wedge_patch, circle_patch_base, wedge_patch_base, anim
  plt.style.use('seaborn-whitegrid')
  x_coords = m.mission_state.robot.robot_state.truthpose[:, 0]
  y_coords = m.mission_state.robot.robot_state.truthpose[:, 1]
  fig, ax = plt.subplots()
  ax.plot(x_coords, y_coords, '-b')
  ax.plot(x_coords[0], y_coords[0], 'gx')
  margin = 5
 
  mission = m
  circle_patch = plt.Circle((5, 5), 1, fc="green")

  wedge_patch = patch.Wedge(
      (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
  )

  if m.mission_state.control_mode == ControlMode.ROOMBA:
      range = m.mission_state.roomba_radius + margin
      init_x = m.mission_state.base_station.position[0]
      init_y = m.mission_state.base_station.position[1]
      plt.xlim([init_x-range, init_x+range])
      plt.ylim([init_y-range, init_y+range])
      circle = plt.Circle((init_x, init_y), m.mission_state.roomba_radius)
      ax.add_patch(circle)

  elif m.mission_state.control_mode != ControlMode.MANUAL:
      goals = waypoints_to_array(m.mission_state.all_waypoints)
      active_nodes = waypoints_to_array(m.mission_state.active_waypoints)
      inactive_nodes = waypoints_to_array(m.mission_state.inactive_waypoints)
      ax.plot(active_nodes[:, 0], active_nodes[:, 1], 'bx')
      ax.plot(inactive_nodes[:, 0], inactive_nodes[:, 1], 'rx')
      xbounds, ybounds = get_plot_boundaries(m.mission_state.grid.nodes, margin)
      plt.xlim(xbounds)
      plt.ylim(ybounds)

  # Plot base station:
  circle_patch_base = plt.Circle((5, 5), 1, fc="red")
  # The heading of base station in degrees
  base_angle_degrees = math.degrees(m.mission_state.base_station.heading)
  wedge_patch_base = patch.Wedge(
      m.mission_state.base_station.position, 3, base_angle_degrees-10, base_angle_degrees+10, fill=False, width=2, ec="r", hatch="xx"
  )

  anim = animation.FuncAnimation(
      fig, animate, init_func=init, fargs = (m,), frames=np.shape(m.mission_state.robot.robot_state.truthpose)[0], interval=20, blit=True
  )

  plt.show()
  return
