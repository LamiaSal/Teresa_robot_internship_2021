"""
Contains the necessary code to control bebop 2
"""
import roslibpy
import time
from src.robots.Robot import Robot
from src.robots.actions.camera import take_picture, EXECUTION_TIME

class Bebop(Robot):
    """
    Class that represents the Bebop 2 Power Drone
    """
    LINEAR_SPEED = 10.0 # Speed to move forward or backward
    ROTATION_SPEED = 10.0

    POSSIBLE_MOVES = [
        #Stop
        {
            'angular': {
                'y': 0, 
                'x': 0, 
                'z': 0
            }
        },
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
        },
        # Move left
        {
            'linear': {
                'y': LINEAR_SPEED,
            }
        },
        # Move right
        {
            'linear': {
                'y': -LINEAR_SPEED
            }
        },
        # Move up
        {
            'linear': {
                'z': LINEAR_SPEED
            }
        },
        # Move down
        {
            'linear': {
                'z': -LINEAR_SPEED
            }
        }
    ]

    MOVE_TOPIC = {
        'topic_name': '/bebop2/cmd_vel',
        'msg_type': '/geometry_msgs/Twist'
    }
    NUMBER_MOVEMENTS = len(POSSIBLE_MOVES)
    def __init__(self, client):
        """
        Constructor of the class
        """
        super(Bebop, self).__init__(client, '/bebop2', self.MOVE_TOPIC, self.POSSIBLE_MOVES)
        self.takeoff_topic = roslibpy.Topic(client, '/bebop2/takeoff', 'std_msgs/Empty')
        self.land_topic = roslibpy.Topic(client, '/bebop2/land', 'std_msgs/Empty')

    def takeoff(self):
        """
        With this we can takeoff the drone
        """
        self.takeoff_topic.publish(roslibpy.Message())
        time.sleep(EXECUTION_TIME)

    def land(self):
        """
        Land the drone in the ground
        """
        self.land_topic.publish(roslibpy.Message())
        time.sleep(EXECUTION_TIME)
