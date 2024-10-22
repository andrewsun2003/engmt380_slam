import cv2 as cv
import numpy as np 
import astar
import bot_math as bm

# Global lists for corners and contour ends
corners = [] 
contour_ends = []
corner_groups = []
contour_end_groups = []
open_space_points = []
pairs = []

def is_circle_valid(x, y, dilated_map):
    for angle in range(0, 360, 10):  # Check 30 degree points around the circle
        rad = np.radians(angle)
        check_x = int(x + 15 * np.cos(rad))
        check_y = int(y + 15 * np.sin(rad))

        if dilated_map[check_y, check_x] == 1:  # White
            return False
    return True

def process_image(map):
    # Convert to grayscale if it's not already
    if len(map.shape) == 3:  # If the map has color channels
        map_gray = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
    else:
        map_gray = map  # Already grayscale
    
    # Apply binary thresholding
    _, binary_map = cv.threshold(map_gray, 127, 255, cv.THRESH_BINARY)
    
    # Perform edge detection
    edges = cv.Canny(binary_map, 75, 150, apertureSize=3)

    # Dilate edges
    kernel = np.ones((5, 5), np.uint8)
    dilated_map = cv.dilate(edges, kernel, iterations=4)
    #erroded_map = cv.erode(dilated_map, kernel, iterations=10)
    kernel = np.ones((5, 5), np.uint8)
    dilated_map = cv.morphologyEx(dilated_map, cv.MORPH_CLOSE, kernel)

    cv.imshow("Dilaated Img", dilated_map)
    cv.waitKey(0)
    
    approx_map = np.zeros(map.shape, dtype='uint8')

    # Find contours on the binary image
    contours, hierarchy = cv.findContours(dilated_map, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    approx_points = []
    filtered_contours = []

    for contour in contours:
        area = cv.contourArea(contour)
        if area > 500:
            perimeter = cv.arcLength(contour, True)
            epsilon = 0.01 * perimeter  
            approx = cv.approxPolyDP(contour, epsilon, True)
            approx_points.append(approx)
            cv.drawContours(approx_map, [contour], -1, 255, 1)
            filtered_contours.append(contour)

    return approx_map, approx_points, contours, filtered_contours, dilated_map

def calculate_angle(pt1, pt2, pt3):
    pt1 = pt1.ravel()
    pt2 = pt2.ravel()
    pt3 = pt3.ravel()

    vec1 = pt1 - pt2
    vec2 = pt3 - pt2
    dot_product = np.dot(vec1, vec2)
    magnitude1 = np.linalg.norm(vec1)
    magnitude2 = np.linalg.norm(vec2)

    # Calculate the angle in radians and convert it to degrees
    cos_angle = dot_product / (magnitude1 * magnitude2)
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip the value to avoid floating-point errors
    return np.degrees(angle)

def average_points(group):
    avg_x = int(np.mean([p[0] for p in group]))
    avg_y = int(np.mean([p[1] for p in group]))
    return (avg_x, avg_y)

def find_corners(approx_points):
    global corners, contour_ends, corner_groups, contour_end_groups
    for point in approx_points:
        for i in range(len(point)):
            pt1 = point[i-1][0]
            pt2 = point[i][0]
            pt3 = point[(i+1) % len(point)][0]
            
            angle = calculate_angle(pt1, pt2, pt3)
            
            # Check if the angle is approximately 90 degrees (within a tolerance)
            if 80 <= angle <= 100:
                corners.append(pt2)  # Store the point
            else:
                contour_ends.append(pt2)   

    threshold_distance = 50  # Adjust the threshold for grouping close points
    for corner in corners:
        added = False
        for group in corner_groups:
            # If the corner is close to any point in the group, add it to the group
            if any(bm.distance(corner, g) < threshold_distance for g in group):
                group.append(corner)
                added = True
                break
        if not added:
            corner_groups.append([corner])  # Create a new group if no close group was found

    threshold_distance = 50
    for contour_end in contour_ends:
        added = False
        for group in contour_end_groups:
            # If the corner is close to any point in the group, add it to the group
            if any(bm.distance(contour_end, g) < threshold_distance for g in group):
                group.append(contour_end)
                added = True
                break
        if not added:
            contour_end_groups.append([contour_end])  # Create a new group if no close group was found

def find_closest_pairs(points):
    pairs = []
    for i, point1 in enumerate(points):
        min_distance = float('inf')
        closest_point = None
        for j, point2 in enumerate(points):
            if i != j:
                dist = bm.distance(point1, point2)
                if dist < min_distance:
                    min_distance = dist
                    closest_point = point2
        pairs.append((point1, closest_point))
    return pairs

def find_open_space_points(pairs, dilated_map):
    global open_space_points
    print("in function pairs ", pairs)
    for i in range(len(pairs)):
        x = int((pairs[i][0][0] + pairs[i][1][0]) / 2)
        y = int((pairs[i][0][1] + pairs[i][1][1]) / 2)
        
        if is_circle_valid(x, y, dilated_map):
            open_space_points.append((x, y))

    print("In Function", open_space_points)

def find_next_position(current_position):
    global open_space_points
    max_index = 0
    max_distance = 0

    for i, point in enumerate(open_space_points):
        d = bm.distance(current_position, point)
        if d > max_distance:
            max_distance = d
            max_index = i
            print(i)
    next_position = open_space_points[max_index]

    return next_position

def show_path(path, approx_map):
    for point in path:
        cv.circle(approx_map, point, 1, [255, 255, 0], 2)

    cv.imshow("Path", approx_map)

def main(map, current_position):
    if map is None:
        print("Error: Could not read the image.")
        return

    print("Image loaded successfully.")
    cv.imshow("A_star", map)

    approx_map, approx_points, contours, filtered_contours, dilated_map = process_image(map)
    contours = filtered_contours
    
    find_corners(approx_points)

    averaged_corners = [average_points(corner_group) for corner_group in corner_groups]
    averaged_ends = [average_points(contour_end_group) for contour_end_group in contour_end_groups]

    pairs = find_closest_pairs(averaged_ends)
    find_open_space_points(pairs, dilated_map)

    next_position = find_next_position(current_position)

    for averaged_end in averaged_ends:
        cv.circle(approx_map, tuple(averaged_end), 5, 150, -1)
    for averaged_corner in averaged_corners:
        cv.circle(approx_map, tuple(averaged_corner), 5, 255, -1)

    for point in open_space_points:
        cv.circle(approx_map, tuple(point), 5, 100, -1)

    cv.circle(approx_map, tuple(current_position), 15, [0, 255, 0], -1)
    cv.circle(approx_map, tuple(next_position), 15, [0, 0, 255], -1)

    cv.imshow("A_star", approx_map)
    cv.waitKey(0)

    print(current_position)

    path = astar.a_star_pathfinding(current_position, next_position, contours, approx_map)
    path = path[::25]
    path.append(next_position)

    cv.imshow("A_star", approx_map)
    print("Path", path)

    cv.waitKey(50)

    show_path(path, approx_map)

    return path

