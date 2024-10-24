import numpy as np
import math
import cv2 as cv

def distance(pt1, pt2):
    return np.linalg.norm(np.array(pt1) - np.array(pt2))


def angle_difference(angle1, angle2):
	if (angle1 >= 0 and angle2 >= 0):
		difference = angle1 - angle2
	if (angle1 < 0 and angle2 < 0):
		difference = angle1 - angle2

	if (angle1 >= 0 and angle2 < 0):
		difference = -360+(angle1 - angle2)
		
	if (angle1 < 0 and angle2 >= 0):
		difference = 360+(angle1 - angle2)
		
	return difference

def heading(current_point, next_point):
    # Calculate the difference in coordinates
    delta_x = next_point[0] - current_point[0]
    delta_y = (800 - next_point[1]) - (800 - current_point[1])
	
    print("delta y ", delta_y)
    print("delta x", delta_x)
	
    heading_angle = math.degrees(math.atan2(delta_y, delta_x))  # Angle in radians
 
    print("heading", heading_angle)
	
    return heading_angle

