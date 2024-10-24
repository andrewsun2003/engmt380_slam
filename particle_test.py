import numpy as np      
import cv2 as cv 
import slamBotHD as sb
import integrated_particle_filter as pf
import astar
import next_locations as nl

# Initialize the robot and read core data
sb.startUp()  
sb.readCoreData()

# Global variables
end_point = None

def mouse_event_handler(event, x, y, flags, param):
    global end_point
    if event == cv.EVENT_LBUTTONDOWN:
        # Mark end point in red
        cv.circle(final_map, (x, y), 15, (0, 0, 255), -1)  
        end_point = (x, y)  # Store the endpoint

def draw_contours(contours, map):
    for contour in contours:
        perimeter = cv.arcLength(contour, True)
        if perimeter > 100:
            cv.drawContours(map, [contour], -1, 255, -1)
#--------------------------------------------------------------------------------------------------------


# Print battery information
print(f'Battery: {sb.botBatt}V')

key = 0
map = cv.imread('fullmap.png')
map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
mask = np.zeros(map.shape, dtype='uint8')
test = np.ones(map.shape, dtype='uint8')

# Create a dilated map
dilated_map = np.ones(map.shape, dtype='uint8')
for y in range(map.shape[0]):
    for x in range(map.shape[1]):
        if map[y, x] == 0:
            cv.circle(dilated_map, (x, y), radius=2, color=255, thickness=-1)

_, binary_map = cv.threshold(dilated_map, 127, 255, cv.THRESH_BINARY)

edges = cv.Canny(binary_map, 127, 255, apertureSize=3)
contours_edges, hierarchy = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)


# Fill in the contours in the mask to create a white square
contours_mask, hierarchy = cv.findContours(binary_map, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
draw_contours(contours_mask, mask)

# Draw dilated map over mask
for y in range(map.shape[0]):
    for x in range(map.shape[1]):
        if dilated_map[y, x] == 255:
            cv.circle(mask, (x, y), radius=1, color=0, thickness=-1)

contours_final, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) 

draw_contours(contours_final, mask)

# Invert the colors
inverted_image = cv.bitwise_not(mask)

# Define a structuring element and perform dilation/erosion
kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
dilated_image = cv.dilate(inverted_image, kernel, iterations=2)
eroded_image = cv.erode(dilated_image, kernel, iterations=3)

inverted_image = cv.bitwise_not(eroded_image)
final_map = cv.cvtColor(inverted_image, cv.COLOR_GRAY2BGR) 


#----------------------------------------------------------------------------------------------------------


# Set up the window and mouse callback
cv.namedWindow("Final Map")
cv.setMouseCallback("Final Map", mouse_event_handler)
cv.imshow("Final Map", final_map)
cv.waitKey(1000)

# Run particle filtering to get initial position
initial_angle = np.degrees(sb.imuYaw)
initial_position = (400, 400)
bot_pose = pf.filter(final_map, initial_position, initial_angle)

#bot_pose = (400, 400)
bot_location = (bot_pose[0], bot_pose[1])
# Draw the initial bot position
cv.circle(final_map, bot_location, 15, [0, 255, 0], -1)

while key != 27:  # Escape key to exit 
    cv.imshow("Final Map", final_map)

    if end_point is not None:
        # Use the endpoint in your logic (if needed)
        print("Start Point:", bot_location)
        print("Selected Endpoint:", end_point)

        path = astar.a_star_pathfinding(bot_location, end_point, contours_edges, final_map)

        if path is not None:
            path = path[::25]
            path.append(end_point)
        else:
            print("No Path")
        
        nl.show_path(path, final_map)
    
        cv.waitKey(0)
        break

    key = cv.waitKey(100)  # Shorter wait time for responsiveness

sb.shutDown()
