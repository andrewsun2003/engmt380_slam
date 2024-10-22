import cv2 as cv
import numpy as np
import bot_math as bm
import math


def landmarks_from_global(input_image):

	landmarks = []

	try:
		image = cv.cvtColor(input_image, cv.COLOR_BGR2GRAY)
	except:
		image = input_image.copy()

	harris_corners = cv.cornerHarris(image, 20, 11, 0.24)
	ret, harris_corners = cv.threshold(harris_corners, 127, 255, cv.THRESH_BINARY)
	harris_corners = harris_corners.astype(np.uint8)
	
	contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

	for contour in contours:
		moment = cv.moments(contour)
		try:
			cx = int(moment["m10"] / moment["m00"])
			cy = int(moment["m01"] / moment["m00"])
		
			landmarks.append([cx, cy])	
			
			image[int(cy)][int(cx)] = 200
		except:
			continue

	return landmarks


def estimate_transformation(landmarks_current, landmarks_previous):
    # Ensure the landmarks are 2D arrays of type np.float32
    landmarks_current = np.array(landmarks_current, dtype=np.float32)
    landmarks_previous = np.array(landmarks_previous, dtype=np.float32)
    
    if len(landmarks_current) >= 3 and len(landmarks_previous) >= 3:
        # Estimate affine transformation matrix (2D) between previous and current landmarks
        transformation_matrix, inliers = cv.estimateAffinePartial2D(landmarks_previous, landmarks_current)
        
        # Extract translation offset (tx, ty) from the matrix
        tx = transformation_matrix[0, 2]
        ty = transformation_matrix[1, 2]
        return (tx, ty), transformation_matrix
    else:
        print("Not enough landmark points for transformation")
        return (0, 0), None


def combine_maps(current_map, previous_map, transformation_matrix):
    # Get the dimensions of the maps
    rows, cols, channels = current_map.shape
    
    # Apply the transformation matrix to the previous map to align it with the current map
    transformed_previous_map = cv.warpAffine(previous_map, transformation_matrix, (cols, rows))
    
    # Combine the maps by overlaying the transformed previous map onto the current map
    combined_map = cv.addWeighted(current_map, 0.5, transformed_previous_map, 0.5, 0)
    
    return combined_map


# Step 1: Load an image
current_map = cv.imread('current_map.png')  # Replace with your image path
mask = (current_map != [0, 0, 0]).any(axis=-1)
current_map[mask] = [255, 255, 255]  # Set non-black pixels to white

previous_map = cv.imread('previous_map.png')  # Replace with your image path
mask = (previous_map != [0, 0, 0]).any(axis=-1)
previous_map[mask] = [255, 255, 255]  # Set non-black pixels to white

landmarks_current = landmarks_from_global(current_map)
landmarks_previous = landmarks_from_global(previous_map)

previous_array = []
for point1 in landmarks_current:
    min_distance = float('inf')
    for i, point2 in enumerate(landmarks_previous):
        distance = bm.distance(point1, point2)
        if distance < min_distance:
            min_distance = distance
            min_index = i
    previous_array.append(landmarks_previous[min_index])
	
landmarks_previous = previous_array

# Estimate transformation (translation, rotation) between the two sets of landmarks
offset, transformation_matrix = estimate_transformation(landmarks_current, landmarks_previous)
combined_map = combine_maps(current_map, previous_map, transformation_matrix)

for point in landmarks_current:
	cv.circle(current_map, point, 5, 255, -1)
for point in landmarks_previous:
	cv.circle(previous_map, point, 5, 255, -1)

#combined_map = process_map(combined_map)
combined_map = cv.cvtColor(combined_map, cv.COLOR_BGR2GRAY)
#_, combined_map = cv.threshold(combined_map, 150, 255, cv.THRESH_BINARY)
#edges = cv.Canny(combined_map, 75, 150, apertureSize=3)
#dilated_map = cv.dilate(edges, (3, 3), iterations=5)
#combined_map = cv.dilate(dilated_map, (3,3), iterations=5)

# Step 7: Display the result
cv.imshow('Current Map', current_map)
cv.imshow('Previous Map', previous_map)
cv.imshow('Combined Map', combined_map)
cv.waitKey(0)
cv.destroyAllWindows()