#Name:Andrew Sun    ID:1607608
#Task A

import cv2 as cv
import numpy as np
import heapq


def get_neighbors(node, map, contours, square_size=1, clearance=28):
    neighbors = []
    x, y = node

    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
        isInside = False
        nx, ny = x + dx, y + dy

        for contour in contours:
            isInside = cv.pointPolygonTest(contour, (nx*square_size+int(square_size/2), ny*square_size+int(square_size/2)), True)
            if isInside >= -clearance:
                isInside = True
                break
            else:
                isInside = False
        
        if isInside == False:
                midx, midy = (x + nx) / 2, (y + ny) / 2
                mid_pixel = (int(midx * square_size + int(square_size/2)), int(midy * square_size + int(square_size/2)))

                for contour in contours:
                    isInside = cv.pointPolygonTest(contour, mid_pixel, True)
                    if isInside >= -clearance:
                        isInside = True
                        break
                    else:
                        isInside = False

        if (0 <= nx < (map.shape[1]//square_size) and 0 <= ny < (map.shape[0]//square_size) and isInside == False):  #(np.any(grid_cell == 255)) == False and
            neighbors.append((nx, ny))

    return neighbors

def a_star_pathfinding(start, end, contours, map, square_size=16):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_cost = {start: 0}
    h_cost = np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)*1000
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
            #path = [(p[0] * square_size + int(square_size/2), p[1] * square_size + int(square_size/2)) for p in path]
            #path.insert(0, start)
            #path.append(end)
            return path
      
        for neighbor in get_neighbors(current, map, contours):
            h_cost = np.sqrt((neighbor[0] - end[0]) ** 2 + (neighbor[1] - end[1]) ** 2)*1000
                             
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