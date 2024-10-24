import cv2 as cv
import numpy as np 
import astar
import bot_math as bm
import math


def process_map(src):
# Apply edge detection (Canny)
    src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    _, binary_map = cv.threshold(src, 10, 255, cv.THRESH_BINARY)
    edges = cv.Canny(binary_map, 75, 150, apertureSize=3) 
    #src = cv.dilate(src, (3, 3), iterations=1) 
    cv.imshow('edges', edges)

# Detect lines using Hough Line Transform
    lines = cv.HoughLinesP(edges, 0.5, np.pi/180, 10, minLineLength=30, maxLineGap=50)

# Draw the lines on the images
    src = np.zeros(src.shape, dtype='uint8')
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(src, (x1,y1), (x2,y2), 255, 1)

    return src, lines


def find_nearest_endpoints(endpoints, threshold):
    merged_endpoints = []
    while len(endpoints) > 0:
        point = endpoints.pop(0)  # Take one endpoint
        
        close_points = [point]  # List to store all close points
        far_points = []
        
        for other in endpoints:
            if bm.distance(point, other) < threshold:
                close_points.append(other)  # Cluster points that are close
            else:
                far_points.append(other)
        
        # Average the points that are close together
        avg_x = int(np.mean([p[0] for p in close_points]))
        avg_y = int(np.mean([p[1] for p in close_points]))
        
        merged_endpoints.append((avg_x, avg_y))  # Add merged point
        endpoints = far_points  # Update endpoints list to remove merged points
    
    return merged_endpoints, endpoints


map  = cv.imread('middle.png')
approx_map, lines = process_map(map)
#approx_map = np.subtract(255, approx_map)
#approx_map = cv.cvtColor(approx_map, cv.COLOR_GRAY2BGR)


endpoints = []
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]  # Get the start and end points of the line
        endpoints.append((x1, y1))  # Append the start point
        endpoints.append((x2, y2))  # Append the end point

# Merge endpoints that are close together (e.g., at junctions)
merged_endpoints, endpoints = find_nearest_endpoints(endpoints, threshold=10)

# Draw the merged endpoints (the true wall endpoints) on the image
approx_map = cv.cvtColor(approx_map, cv.COLOR_GRAY2BGR)
for x, y in endpoints:
    cv.circle(approx_map, (x, y), 5, (0, 255, 255), -1)  # Draw in red 
for x, y in merged_endpoints:
    cv.circle(approx_map, (x, y), 5, (0, 0, 255), -1)  # Draw in red 
    

while(True):
    cv.imshow("Original Map", map)
    cv.imshow("Processed Map", approx_map)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
