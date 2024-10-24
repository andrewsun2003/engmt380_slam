import numpy as np      
import cv2 as cv 
import slamBotHD as sb
import integrated_particle_filter as pf


sb.startUp()  
sb.readCoreData()

print(f'Battery: {sb.botBatt}V')

key = 0

map = cv.imread('fullmap.png')

map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)

dilated_map = np.ones(map.shape, dtype='uint8')
for y in range(800):
    for x in range(800):
    # Check if the pixel is black (intensity 0)
        if map[y, x] == 0:
            # Draw a black circle at that point (radius=3, color=(0, 0, 0), thickness=-1 to fill)
            cv.circle(dilated_map, (x, y), radius=2, color=255, thickness=-1)

_, binary_map = cv.threshold(dilated_map, 127, 255, cv.THRESH_BINARY)

contours, hierarchy = cv.findContours(binary_map, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

mask = np.zeros(map.shape, dtype='uint8')

# Fill in the conours in the mask to create white square
for contour in contours:
    perimeter = cv.arcLength(contour, True)
    if perimeter > 100:
        cv.drawContours(mask, [contour], -1, 255, -1)

# Draw dilated map over mask
for y in range(800):
    for x in range(800):
    # Check if the pixel is black (intensity 0)
        if dilated_map[y, x] == 255:
            # Draw a black circle at that point (radius=3, color=(0, 0, 0), thickness=-1 to fill)
            cv.circle(mask, (x, y), radius=1, color=0, thickness=-1)

contours, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) 
for contour in contours:
    perimeter = cv.arcLength(contour, True)
    if perimeter > 200:
        cv.drawContours(mask, [contour], -1, 255, -1)

# Invert the colors
inverted_image = cv.bitwise_not(mask)

# Define a structuring element
kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))  # Adjust size as needed
# Perform dilation
dilated_image = cv.dilate(inverted_image, kernel, iterations=2)
eroded_image = cv.erode(dilated_image, kernel, iterations=3)

inverted_image = cv.bitwise_not(eroded_image)

final_map = cv.cvtColor(inverted_image, cv.COLOR_GRAY2BGR)

while key != 27:
    cv.imshow("Final Map", final_map)
    cv.waitKey(1000)
    
    initial_angle = np.degrees(sb.imuYaw)
    initial_pos = (400, 400)

    pf.filter(final_map, initial_pos, initial_angle)

    key = cv.waitKey(0)
    

sb.shutDown()