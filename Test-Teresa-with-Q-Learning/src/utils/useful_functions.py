'''
In here we implement some useful functions that
we can use across all the project
'''

def is_modified(old_time, new_time):
    '''
    This function returns a boolean to recognize if
    a file has been modified
    Parameters:
        - old_time (Float) --> The time (In seconds) of the old file
        - new_time (Float) --> The time (In seconds) of the new file
    Returns:
        - True if the file has been modified
        - False otherwise
    '''
    return new_time > old_time

def calculate_center(image_size, width, height):
    """Function calculate the center for a rectangle given a image size

    Args:
        image_size (Tuple): Contain the width and high of the image 
        width (Integer): Width of the rectangle
        height (Integer): Height of the rectangle

    Returns:
        [type]: [description]
    """
    point_x = 0
    point_y = 0
    for i in range(0, image_size[0]):
        l1 = i
        l2 = image_size[0] - (i + width)
        if (l2 - l1 < 10):
            point_x = i
            break
    
    for i in range(0, image_size[1]):
        l1 = i
        l2 = image_size[1] - (i + height)
        if (l2 - l1 < 10):
            point_y = i
            break

    return point_x, point_y