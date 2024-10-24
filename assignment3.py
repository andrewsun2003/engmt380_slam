#Name:Andrew Sun    ID:1607608
#Task A

import cv2 as cv
import numpy as np
import heapq

#Import the image
map = cv.imread('/Users/andrewsun/Documents/Engineering 2024/ENGMT380/Assignment 3/my_map_bw.bmp')

(original_height, original_width) = map.shape[0:2]
new_height = 800
new_width = 800
scale = new_height/original_height

map = cv.resize(map, (new_width, new_height), interpolation=cv.INTER_LINEAR)

#Noise removal
map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
threshold, map = cv.threshold(map, 150, 255, cv.THRESH_BINARY)
blur_map = cv.GaussianBlur(map, (15,15), 0)
edges = cv.Canny(blur_map, 75, 150, apertureSize=3)
dilate_edges = cv.dilate(edges, (1,1), iterations=2)

#Find contours and map cleanup
contours, hierarchies = cv.findContours(dilate_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
internal_contours = np.zeros(map.shape, dtype='uint8')
external_contour = np.zeros(map.shape, dtype='uint8')
max_area = 0

for contour in contours:
    area = cv.contourArea(contour)
    if area > max_area:
        max_area = area
        boundary_contour = contour

for contour in contours:
    if contour is not boundary_contour:
        approx_boundary = cv.convexHull(contour)
        cv.drawContours(internal_contours, [approx_boundary], -1, (255, 255, 255), thickness=2)

cv.drawContours(external_contour, [boundary_contour], -1, (255, 255, 255), thickness=2)

contours, hierarchies = cv.findContours(external_contour, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
external_contour = np.zeros(map.shape, dtype='uint8')
for contour in contours:
    cv.drawContours(external_contour, [contour], -1, (255, 255, 255), thickness=2)

# Combine the boundary and internal objects into one image
recreated_map = cv.bitwise_or(external_contour, internal_contours)
recreated_map = cv.erode(recreated_map, (1, 1), iterations=2)
cv.imshow("Original Map", map)


#Task B
contours, hierarchies = cv.findContours(recreated_map, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
recreated_map = cv.cvtColor(recreated_map, cv.COLOR_GRAY2BGR)

square_size = int(16*scale)
height, width = recreated_map.shape[:2]
clearance = 42

start_point = None
end_point = None

def check_point_validity(point, contours):
    for contour in contours:
        isInside = cv.pointPolygonTest(contour, point, True)
        if isInside >= -50*scale:
            print("Invalid point: Selected point intersects with contour")
            isInside = True
            break
        else:
            isInside = False
    return isInside

def mouse_event_handler(event, x, y, flags, param):

    global start_point, end_point
    global start_point_replace, end_point_replace

    if event == cv.EVENT_LBUTTONDOWN:
        if start_point is None:  
            start_point_replace = (x, y)     
            if check_point_validity((x, y), contours) is False:
                cv.circle(recreated_map, (x,y), int(35*scale), (0, 255, 0), -1)  # Mark start point in green
                print("Start Point:", (x,y))
                start_point = (x//square_size, y//square_size)

        elif end_point is None:
            end_point_replace = (x, y)
            if check_point_validity((x, y), contours) is False:
                cv.circle(recreated_map, (x,y), int(35*scale), (0, 0, 255), -1)  # Mark end point in red
                print("End Point:", (x,y))
                print("Estimating optimal path...")
                end_point = (x//square_size, y//square_size)

                # Run A* pathfinding after selecting the end point
                path = a_star_pathfinding(start_point, end_point, recreated_map)
                if path:
                    cv.line(recreated_map, path[0], path[1], (255, 0, 0), 2)
                    cv.line(recreated_map, (path[-2][0], path[-2][1]), path[-1], (255, 0, 0), 2)

                    for current_point, next_point in zip(path[1:], path[2:-1]):
                        cv.line(recreated_map, (current_point[0], current_point[1]), (next_point[0], next_point[1]), (255, 0, 0), 2) 
                    print("Path found and displayed.")
                else:
                    print("No path found between start and end points.")
                    print("Press r to reset, or press q to quit")

# A* Pathfinding Implementation
def get_neighbors(node, recreated_map, contours):
    neighbors = []
    x, y = node

    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
        isInside = False
        nx, ny = x + dx, y + dy

        for contour in contours:
            isInside = cv.pointPolygonTest(contour, (nx*square_size+int(square_size/2), ny*square_size+int(square_size/2)), True)
            if isInside >= -clearance*scale:
                isInside = True
                break
            else:
                isInside = False
        
        if isInside == False:
                midx, midy = (x + nx) / 2, (y + ny) / 2
                mid_pixel = (int(midx * square_size + int(square_size/2)), int(midy * square_size + int(square_size/2)))

                for contour in contours:
                    isInside = cv.pointPolygonTest(contour, mid_pixel, True)
                    if isInside >= -clearance*scale:
                        isInside = True
                        break
                    else:
                        isInside = False

        if (0 <= nx < (recreated_map.shape[1]//square_size) and 0 <= ny < (recreated_map.shape[0]//square_size) and isInside == False):  #(np.any(grid_cell == 255)) == False and
            neighbors.append((nx, ny))

    return neighbors

def a_star_pathfinding(start, end, recreated_map):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_cost = {start: 0}
    h_cost = np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)*2
    f_cost = {start: h_cost}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            if len(path) < 3:
                path = [(p[0] * square_size + int(square_size/2), p[1] * square_size + int(square_size/2)) for p in path]
                path.insert(0, start_point_replace)
                path.append(end_point_replace)
            else:
                path = [(p[0] * square_size + int(square_size/2), p[1] * square_size + int(square_size/2)) for p in path]
                path[0] = start_point_replace
                path[-1] = end_point_replace
            return path
        
        for neighbor in get_neighbors(current, recreated_map, contours):
            h_cost = np.sqrt((neighbor[0] - end[0]) ** 2 + (neighbor[1] - end[1]) ** 2)*2
                             
            if (abs(neighbor[0] - current[0]) == 1 and abs(neighbor[1] - current[1]) == 1):
                tentative_g_cost = g_cost[current] + np.sqrt(2)
            else:
                tentative_g_cost = g_cost[current] + 1  # Assumes uniform cost

            if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                came_from[neighbor] = current
                g_cost[neighbor] = tentative_g_cost
                f_cost[neighbor] = tentative_g_cost + h_cost

                heapq.heappush(open_set, (f_cost[neighbor], neighbor))

    return None  # No path found

# Create a window and set the mouse callback
cv.namedWindow('Recreated Map')
cv.setMouseCallback('Recreated Map', mouse_event_handler)


# Display the image and wait for user input
reset_map = recreated_map.copy()
print("Select start and end points, press r to reset, or press q to quit")
while True:
    cv.imshow('Recreated Map', recreated_map)
    key = cv.waitKey(1) & 0xFF
    if key == ord('r'):
        print("Resetting")
        no_path = 0
        start_point = None
        end_point = None
        recreated_map = reset_map.copy()
        cv.imshow('Recreated Map', recreated_map)
        print("Select start and end points, press r to reset, or press q to quit")
    if key == ord('q'):
        print("Quitting program...")
        break

cv.destroyAllWindows()