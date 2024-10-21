import numpy as np
import math

def distance(pt1, pt2):
    return np.linalg.norm(np.array(pt1) - np.array(pt2))


def angle_difference(angle1, angle2):
	if (angle1 > 0 and angle2 > 0):
		difference = angle1 - angle2
	if (angle1 < 0 and angle2 < 0):
		difference = angle1 - angle2

	if (angle1 > 0 and angle2 < 0):
		difference = -360*(angle1 - angle2)
		
	if (angle1 < 0 and angle2 > 0):
		difference = 360*(angle1 - angle2)
		
	return difference

def heading(current_point, next_point, current_angle):
    # Calculate the difference in coordinates
    delta_x = angle_difference(next_point[0], current_point[0])
    delta_y = angle_difference(next_point[1], current_point[1])
    
    # Calculate the angle to the next point in radians
    angle_to_next = math.atan2(delta_y, delta_x)  # Angle in radians

    # Convert angle to degrees
    angle_to_next_degrees = math.degrees(angle_to_next)

    # Calculate the heading by adjusting for the current angle
    heading_angle = -(angle_to_next_degrees - current_angle)

    # Normalize the heading angle to be between -180 and 180 degrees
    heading_angle = (heading_angle + 180) % 360 - 180
    
    return heading_angle

