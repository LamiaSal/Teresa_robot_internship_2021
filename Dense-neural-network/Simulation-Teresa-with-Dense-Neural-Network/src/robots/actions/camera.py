'''
Control the camera of the robot. It is in charge
to send the command to take a picture of the
environment
'''
import base64
import time
import logging

from src.utils.training_tools import EXECUTION_TIME

# Print important information (Debug purpose only)
fmt = "%(asctime)s %(levelname)8s: %(message)s"
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

'''
Callback function, it process the image received and save it into
the computer
'''
def receive_image(msg):
    # log.info('Received image seq=%d', msg['header']['seq'])
    base64_bytes = msg['data'].encode('ascii')
    image_bytes = base64.b64decode(base64_bytes)
    # 'received-image-{}-{}'.format(msg['header']['seq'], msg['format'])
    with open('env_observation', 'wb') as image_file:
        image_file.write(image_bytes)

'''
Send the command to take the picture
Input
- camera_topic ---> it is the topic where the camera send the image
'''
def take_picture(camera_topic):
    camera_topic.subscribe(receive_image)
    time.sleep(EXECUTION_TIME)
    camera_topic.unsubscribe() # This avoid to take many photos after one movement