import os
import signal
import subprocess
from langchain_core.tools import tool, Tool
from .helper_funcs import publish_to, get_direction_coordinates, capture_image
from std_msgs.msg import Float64MultiArray, Bool

COORDINATE_TYPE = Float64MultiArray
COORDINATE_TOPIC = "published_coordinates"
BOOL_TYPE = Bool    
STOP_TOPIC = "stop_robot"

@tool   
def move_to_home_pose ():
    """
    Moves the Robot arm coordinates for home pose
    """
    # home_pose_coordinates = [1.0, 0.0, 0.5, 0.5, 0.3, -0.5, 0.5, 0.5]
    home_pose_coordinates = [0.0, 0.0, -0.8, -3.15, -2.0, 0.0, -1.2, 1.55]
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=home_pose_coordinates)
       
@tool
def move_to_retract_pose ():
    """
    Moves the Robot to the Retract position
    """
    retract_pose_coordinates = [0.0, 0.0, 0.0, -3.15, -1.5, 0.0, -1.6, 1.55]
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=retract_pose_coordinates)

    
@tool
def move_to_zero_position ():
    """
    Moves the Robot Arm to Zero Position
    """
    zero_coordinates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=zero_coordinates)
    
    
@tool
def move_forward():
    """
    Moves the Arm forward in the X-direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("forward"))
    
        
@tool
def move_backward():
    """
    Moves the Arm forward in the X-direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("backward"))


@tool
def move_left():
    """
    Moves the Arm left in the Y direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("left"))


@tool
def move_right():
    """
    Moves the Arm right in the Y direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("right"))


@tool
def move_upwards():
    """
    Moves the Arm upwards in Z direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("upward"))


@tool
def move_downwards():
    """
    Moves the arm downwards in Z direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("downward"))


@tool
def open_gripper():
    """
    Opens the Gripper
    """
    open_coordinates = [2.0, -0.0, 0.0]
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=open_coordinates)


@tool
def close_gripper():
    """
    Closes the Gripper
    """
    close_coordinates = [2.0, 0.8, -0.8]
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=close_coordinates)


@tool
def describe_what_you_see():
    """
    Describes what the robot sees in the Camera's FOV
    """
    capture_image()
    
@tool
def rotate_the_gripper_clockwise():
    """
    Rotate the gripper by 90 Degrees in clockwise direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("clockwise"))


@tool 
def rotate_the_gripper_anticlockwise():
    """
    Rotate the gripper by 90 Degrees in anticlockwise direction
    """
    publish_to(type_name=COORDINATE_TYPE, topic_name=COORDINATE_TOPIC, coordinates=get_direction_coordinates("anticlockwise"))

    
@tool 
def stop():
    """
    Stops the Robot movement by killing all the active terminals on the system
    """
    
    # List of all terminal processes
    terminal_processes = ["gnome-terminal", "xterm", "konsole", "terminator", "mate-terminal", "tilix"]
    for terminal in terminal_processes:
        try:
            # Get the process ID of the terminal
            process_ids = subprocess.check_output(["pgrep", terminal]).decode().splitlines()
            
            for pid in process_ids:
                # Kill the terminals
                os.kill(int(pid), signal.SIGTERM)
        except subprocess.CalledProcessError:
            pass


def get_tools () -> list[Tool]:
    return [rotate_the_gripper_anticlockwise,
            rotate_the_gripper_clockwise,
            describe_what_you_see,
            move_to_zero_position,
            move_to_retract_pose,
            move_to_home_pose,
            move_downwards,
            move_backward,
            close_gripper,
            open_gripper, 
            move_forward, 
            move_upwards,
            move_right,
            move_left,
            stop,]
