import roslibpy

class GazeboController:
    """This module will contains the connection with
    the gazebo nodes in ROS. It will manage all related
    operation with the environment.
        > Reset simulation
        > Move Models
        > etc....
    """
    PERSON_MODEL = "person_standing" # Name of the model to connect to
     
    def __init__(self, client=None):
        """Constructor of the class

        Args:
            client (WebSocket): Contains the connection with the ROS server
        """
        self.set_position_service = roslibpy.Service(client, '/gazebo/set_model_state', 'gazebo_msgs/SetModelState')
        self.get_position_service = roslibpy.Service(client, '/gazebo/get_model_state', 'gazebo_msgs/GetModelState')

    def get_position(self):
        """Get the actual position of the model (This case is hard wired to the person).

        Returns:
            Dictionary: with coordinates of the object in the simulation
        """
        body = {
            'model_name': self.PERSON_MODEL
        }
        request = roslibpy.ServiceRequest(body)
        return self.get_position_service.call(request)['pose']

    def set_position(self, y_amount_move):
        """Set the new position for a model in the simulation (This case is hard wired to the person).

        Args:
            y_amount_move (Float): The amount to move from the original position in the y-axis
        
        NOTE: We can expand this to the other axis
        """
        body = {
            'model_state': {
                'model_name': self.PERSON_MODEL
            }
        }
        actual_position = self.get_position()
        actual_position['position']['y'] += y_amount_move
        # actual_position['position']['x'] += y_amount_move
        body['model_state']['pose'] = {
            'position': actual_position['position'],
            'orientation': actual_position['orientation']
        }
        request = roslibpy.ServiceRequest(body)
        self.set_position_service.call(request)