'''
These script contains several functions to move the robot
around
NOTE: Needs to be adapted after to the physical robot way to move (In process)
'''
import roslibpy
import time

from src.robots.Robot_adap import Robot

class Teresa(Robot):
    """
    Class that represents the robot Teresa. This is the version 1.
    """
    LINEAR_SPEED = 0.5 # Speed to move forward or backward
    ROTATION_SPEED = 0.2 # Speed to rotate left or right

    """
    Array that contains all the posible movements of the robot.
    These are the posible movements: (The numbers representes the index in the array)
        0 rotate right
        1 rotate left
        2 backwards
        3 forward

    Note: the message of type geometry_msgs/Twist has the following structure
    {
        'linear': {
            'x': Float,
            'y': Float,
            'z': Float
        },
        'angular': {
            'x': Float,
            'y': Float,
            'z': Float
        }
    }
    """
    POSSIBLE_MOVES = [
        # Rotate Right
        {
            'angular': {
                'y': ROTATION_SPEED, 
                'x': ROTATION_SPEED, 
                'z': ROTATION_SPEED
            }
        },
        # Rotate Left
        {
            'angular': {
                'y': -ROTATION_SPEED, 
                'x': -ROTATION_SPEED, 
                'z': -ROTATION_SPEED
            }
        },
        # Move Backward
        {
            'linear': {
                'x': -LINEAR_SPEED, 
            }, 
        },
        # Move Forward
        {
            'linear': {
                'x': LINEAR_SPEED, 
            } 
        }
    ]
    NUMBER_MOVEMENTS = len(POSSIBLE_MOVES)

    MOVE_TOPIC = {
        'topic_name': '/cmd_vel',
        'msg_type': 'geometry_msgs/Twist'
    }

    def __init__(self, client):
        """Constructor of Giraff
        """    
        super(Teresa, self).__init__(client, '/teresa_robot/head_camera', self.MOVE_TOPIC, self.POSSIBLE_MOVES)
        # self.move_topic = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
