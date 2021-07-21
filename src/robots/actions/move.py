'''
Contains the necesary code to move a robot
in a 2D space
'''
import roslibpy
import time

from src.utils.training_tools import EXECUTION_TIME

STOP_ROBOT = {
    'linear': {
        'y': 0.0, 
        'x': 0.0, 
        'z': 0.0
    }, 
    'angular': {
        'y': 0.0, 
        'x': 0.0, 
        'z': 0.0
    }
}

'''
Publish the move to the ros topic to execute it
'''
def execute_move(move_msg, move_topic):
    # move_msg = movements[move] # Get the ROS message for the move selected

    # Execute the move
    move_topic.publish(roslibpy.Message(move_msg))
    time.sleep(EXECUTION_TIME)
    move_topic.publish(roslibpy.Message(STOP_ROBOT))
    # time.sleep(EXECUTION_TIME)