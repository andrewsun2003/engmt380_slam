import cv2 as cv
import numpy as np 
import astar
import bot_math as bm

# Global lists for corners and contour ends
contour_ends = []
contour_end_groups = []
open_space_points = []
pairs = []


def process_image(map):
    map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
    _, binary_map = cv.threshold(map, 10, 255, cv.THRESH_BINARY)
    
    dilated_map = np.ones(map.shape, dtype='uint8')
    for y in range(800):
        for x in range(800):
        # Check if the pixel is black (intensity 0)
            if binary_map[y, x] == 0:
                # Draw a black circle at that point (radius=3, color=(0, 0, 0), thickness=-1 to fill)
                cv.circle(dilated_map, (x, y), radius=5, color=255, thickness=-1)

    _, binary_map = cv.threshold(dilated_map, 127, 255, cv.THRESH_BINARY)
    edges = cv.Canny(binary_map, 127, 255, apertureSize=3)

    approx_map = np.zeros(map.shape, dtype='uint8')

    contours, hierarchy = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    approx_points = []
    filtered_contours = []

    for contour in contours:
        perimeter = cv.arcLength(contour, True)
        if perimeter > 200:
            epsilon = 0.023 * perimeter  
            approx = cv.approxPolyDP(contour, epsilon, True)
            approx_points.append(approx)
            cv.drawContours(approx_map, [contour], -1, 255, 1)
            filtered_contours.append(contour)
    return approx_map, approx_points, contours, filtered_contours


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

def find_ends(approx_points):
    global contour_ends, contour_end_groups
    for point in approx_points:
        for i in range(len(point)):
            pt1 = point[i-1][0]
            pt2 = point[i][0]
            pt3 = point[(i+1) % len(point)][0]
            
            angle = calculate_angle(pt1, pt2, pt3)
            
            # Check if the angle is approximately 90 degrees (within a tolerance)
            if angle < 60 or angle > 120:
                contour_ends.append(pt2)   

    threshold_distance = 80
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

def find_open_space_points(pairs, contours):
    global open_space_points
    for i in range(len(pairs)):
        x = int((pairs[i][0][0] + pairs[i][1][0]) / 2)
        y = int((pairs[i][0][1] + pairs[i][1][1]) / 2)
        for contour in contours:
            isInside = cv.pointPolygonTest(contour, (x,y), True)
            if isInside >= -25:
                isInside = True
                break
            else:
                isInside = False
        if isInside == False:
            open_space_points.append((x, y))

def find_next_position(current_position):
    global open_space_points
    max_index = 0
    max_distance = 0

    for i, point in enumerate(open_space_points):
        d = bm.distance((400,400), point)
        if d > max_distance:
            max_distance = d
            max_index = i

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

    approx_map, approx_points, contours, filtered_contours= process_image(map)
    contours = filtered_contours

    cv.imshow("approx", approx_map)
    cv.waitKey(0)
    
    find_ends(approx_points)

    averaged_ends = [average_points(contour_end_group) for contour_end_group in contour_end_groups]

    pairs = find_closest_pairs(averaged_ends)
    if pairs is None:
        print("No pairs")
        return
    
    find_open_space_points(pairs, contours)

    if open_space_points is None:
        print("No opensapce points")
        return

    next_position = find_next_position(current_position)
        
    approx_map = cv.cvtColor(approx_map, cv.COLOR_GRAY2BGR)

    for averaged_end in averaged_ends:
        cv.circle(approx_map, tuple(averaged_end), 5, [0, 255, 255], -1)

    for point in open_space_points:
        cv.circle(approx_map, tuple(point), 5, [255, 0, 0], -1)
    
    cv.circle(approx_map, tuple(current_position), 15, [0, 255, 0], -1)
    cv.circle(approx_map, tuple(next_position), 15, [0, 0, 255], -1)

    path = astar.a_star_pathfinding(current_position, next_position, contours, approx_map)

    if path is not None:
        path = path[::25]
        path.append(next_position)
    else:
        print("No Path")
        return

    cv.imshow("A_star", approx_map)
    print("Path", path)

    show_path(path, approx_map)

    cv.waitKey(50)

    return path

