import cv2 as cv
import numpy as np


map = cv.imread('cone_map.png')  # Replace with your image path
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

contours, hierarchy = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_KCOS)
filtered_cone = []

contour_map = np.ones(map.shape, dtype='uint8')*255
for contour in contours:
    perimeter = cv.arcLength(contour, True)
    if perimeter > 70 and perimeter < 90:
        print(perimeter)
        perimeter = cv.arcLength(contour, True)
        cv.drawContours(contour_map, [contour], -1, 0, thickness=-1)
        filtered_cone.append(contour)

cv.imshow("contours", contour_map)
cv.waitKey(0)

cv.imshow('Processed Map', map)
cv.waitKey(0)
cv.destroyAllWindows()
