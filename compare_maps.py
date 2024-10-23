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

    harris_corners = cv.cornerHarris(image, 30, 11, 0.15)
    ret, harris_corners = cv.threshold(harris_corners, 127, 255, cv.THRESH_BINARY)
    harris_corners = harris_corners.astype(np.uint8)

    contours, hierarchy = cv.findContours(harris_corners, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area
    contours = [contour for contour in contours if cv.contourArea(contour) > 500]

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


def match_landmarks(landmarks_current, landmarks_previous):
    if len(landmarks_current) <= len(landmarks_previous):	
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

    elif len(landmarks_current) > len(landmarks_previous):	
        current_array = []
        for point1 in landmarks_previous:
            min_distance = float('inf')
            for i, point2 in enumerate(landmarks_current):
                distance = bm.distance(point1, point2)
                if distance < min_distance:
                    min_distance = distance
                    min_index = i
            current_array.append(landmarks_current[min_index])
            
        landmarks_current = current_array
        
    return landmarks_current, landmarks_previous


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
current_map = cv.imread('example1.png')  # Replace with your image path
current_map = cv.cvtColor(current_map, cv.COLOR_BGR2GRAY)
_, binary_map = cv.threshold(current_map, 10, 255, cv.THRESH_BINARY)
current_map = cv.cvtColor(binary_map, cv.COLOR_GRAY2BGR)

previous_map = cv.imread('example1_2ndscan.png')
previous_map = cv.cvtColor(previous_map, cv.COLOR_BGR2GRAY)
_, binary_map = cv.threshold(previous_map, 10, 255, cv.THRESH_BINARY)
previous_map = cv.cvtColor(binary_map, cv.COLOR_GRAY2BGR)

landmarks_current = landmarks_from_global(current_map)
landmarks_previous = landmarks_from_global(previous_map)

landmarks_current, landmarks_previous = match_landmarks(landmarks_current, landmarks_previous)

offset, transformation_matrix = estimate_transformation(landmarks_current, landmarks_previous)
combined_map = combine_maps(current_map, previous_map, transformation_matrix)

for point in landmarks_current:
	cv.circle(current_map, point, 5, 255, -1)
for point in landmarks_previous:
	cv.circle(previous_map, point, 5, 255, -1)
 

combined_map = cv.cvtColor(combined_map, cv.COLOR_BGR2GRAY)
_, binary_map = cv.threshold(combined_map, 254, 255, cv.THRESH_BINARY)
combined_map = cv.cvtColor(binary_map, cv.COLOR_GRAY2BGR)


# Step 7: Display the result
cv.imshow('Combined Map', combined_map)
cv.imshow('current map', current_map)
cv.imshow('previous map', previous_map)
cv.waitKey(0)
cv.destroyAllWindows()
