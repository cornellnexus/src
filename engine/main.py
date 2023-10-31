import math
import threading
import json

from constants.definitions import ENGINE_PATH
from engine.robot import Robot
from engine.robot_state import Robot_State
from engine.mission import Mission
from engine.mission_state import Mission_State
from engine.control_mode import ControlMode
from engine.database import DataBase
from engine.transmission import send_packet_to_gui
from engine.is_raspberrypi import is_raspberrypi
from engine.plot_trajectory import plot_sim_traj
from engine.parser import parse_main

if __name__ == "__main__":
    # Load config from JSON file
    with open(ENGINE_PATH + "/config.json", "r") as fp:
        config_args = json.load(fp)

    # Load user-defined arguments
    user_args = parse_main()

    r2d2_state = Robot_State(
        xpos=user_args.get("xpos", 0),
        ypos=user_args.get("ypos", 0),
        heading=user_args.get("heading", math.pi / 4),
        store_data=config_args.get("store_data"),
        is_sim=config_args.get("is_sim") if is_raspberrypi() else not is_raspberrypi(),
    )  # Let user define if on the pi, otherwise set to True
    r2d2 = Robot(robot_state=r2d2_state)
    database = DataBase(r2d2)  # TODO: Replace w new packet transmission impl
    mission_state = Mission_State(
        robot=r2d2,
        base_station_coord=(
            user_args.get("base_lat", 42.444250),
            user_args.get("base_long", -76.483682),
        ),
        init_control_mode=ControlMode.LAWNMOWER
        if user_args.get("lawnmower", True)
        else ControlMode.ROOMBA,
    )
    m = Mission(mission_state=mission_state)

    """------------------- MISSION EXECUTION -------------------"""
    if config_args.get(
        "is_transmit"
    ):  # Set to true when the rpi/robot is communicating w/ the GUI
        packet_sender = threading.Thread(
            target=send_packet_to_gui, args=(1, r2d2_state, database), daemon=True
        )  # Thread to read and send robot properties
        packet_sender.start()
    m.execute_mission(database)  # Run main mission
    """ ---------- MISSION COMPLETE, PLOT TRUTH POSE --------------"""
    if config_args.get("simulate_trajectory"):
        plot_sim_traj(m=m)  #  Plot the trajectory of the completed mission
