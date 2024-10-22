import cv2 as cv
import numpy as np 
import astar
import bot_math as bm


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
    cv.imshow("binary map", binary_map)
    cv.waitKey(0)
    edges = cv.Canny(binary_map, 127, 255, apertureSize=3)
    cv.imshow("edges", edges)
    cv.waitKey(0)
    approx_map = np.zeros(map.shape, dtype='uint8')

    contours, hierarchy = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

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
    contour_map = np.zeros(map.shape, dtype='uint8')
    i = 50
    for contour in contours:
        cv.drawContours(contour_map, [contour], -1, i, 1)
        i+=100
    cv.imshow("contours", contour_map)
    cv.waitKey(0)
    cv.imshow("approx", approx_map)
    cv.waitKey(0)
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
    for point in approx_points:
        for i in range(len(point)):
            pt1 = point[i-1][0]
            pt2 = point[i][0]
            pt3 = point[(i+1)%len(point)][0]
            
            angle = calculate_angle(pt1, pt2, pt3)
            
            # Check if the angle is approximately 90 degrees (within a tolerance)
            if (angle < 80 or angle > 100):
                contour_ends.append(pt2)   

    threshold_distance = 100
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


def find_open_space_points():
    for i in range(len(pairs)):
        x = int((pairs[i][0][0] + pairs[i][1][0])/2)
        y = int((pairs[i][0][1] + pairs[i][1][1])/2)
        open_space_points.append((x,y))


def find_next_position():
    max_index = 0
    max_distance = 0
    print(open_space_points)

    for i, point in enumerate(open_space_points):
        d = bm.distance(current_position, point)
        if (d > max_distance):
            max_distance = d
            max_index = i
    next_position = open_space_points[max_index]

    return next_position


map  = cv.imread('example1.png')
approx_map, approx_points, contours, filtered_contours = process_image(map)
contours = filtered_contours

current_position = (400, 400)

contour_ends = []
contour_end_groups = []
open_space_points = []
pairs = []
find_ends(approx_points)

averaged_ends = [average_points(contour_end_group) for contour_end_group in contour_end_groups]

pairs = find_closest_pairs(averaged_ends)
find_open_space_points()

next_position = find_next_position()

for averaged_end in averaged_ends:
    cv.circle(approx_map, tuple(averaged_end), 5, 150, -1)

approx_map = cv.cvtColor(approx_map, cv.COLOR_GRAY2BGR)
cv.circle(approx_map, (current_position), 15, [0, 255, 0], -1)
cv.circle(approx_map, (next_position), 15, [0, 0, 255], -1)

path = astar.a_star_pathfinding(current_position, next_position, (contours), approx_map)

if path == None:
    print("No path found")
else:
    path = path[::25]
    path.append(next_position)
    print(f'Path: {path}')
    for i in range(len(path)):
        cv.circle(approx_map, path[i], 1, [255, 255, 0], 2)


while(True):
    cv.imshow("Processed Map", approx_map)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
