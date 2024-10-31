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
final_map = None

def mouse_event_handler(event, x, y, flags, param):
    global end_point
    if event == cv.EVENT_LBUTTONDOWN:
        # Mark end point in red
        cv.circle(final_map, (x, y), 15, (0, 0, 255), -1)  
        end_point = (x, y)  # Store the endpoint

def draw_contours(contours, map):
    for contour in contours:
        perimeter = cv.arcLength(contour, True)
        if perimeter > 1:
            cv.drawContours(map, [contour], -1, 255, -1)

def cone_extraction(map):
    _, binary_map = cv.threshold(map, 10, 255, cv.THRESH_BINARY)

    dilated_map = np.ones(map.shape, dtype='uint8')
    for y in range(800):
        for x in range(800):
        # Check if the pixel is black (intensity 0)
            if binary_map[y, x] == 0:
                # Draw a black circle at that point (radius=3, color=(0, 0, 0), thickness=-1 to fill)
                cv.circle(dilated_map, (x, y), radius=2, color=255, thickness=-1)

    _, binary_map = cv.threshold(dilated_map, 127, 255, cv.THRESH_BINARY)

    edges = cv.Canny(binary_map, 127, 255, apertureSize=3)
  

    contours, hierarchy = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_KCOS)
    filtered_cone = []

    contour_map = np.ones(map.shape, dtype='uint8')*255
    for contour in contours:
        perimeter = cv.arcLength(contour, True)
        if perimeter > 60 and perimeter < 120:
            print(perimeter)
            perimeter = cv.arcLength(contour, True)
            cv.drawContours(contour_map, [contour], -1, 0, thickness=-1)
            filtered_cone.append(contour)
    
    return filtered_cone

#--------------------------------------------------------------------------------------------------------

def main(map):
    global final_map

    key = 0

    approx_map, _, _, filtered_contours = nl.process_image(map)

    map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
    line_map = np.ones(map.shape, dtype='uint8')*255

    cone = cone_extraction(map)

    mask = np.zeros(map.shape, dtype='uint8')

    _, binary_map = cv.threshold(map, 127, 255, cv.THRESH_BINARY)

    edges = cv.Canny(binary_map, 127, 255, apertureSize=3)

    lines = cv.HoughLinesP(edges, 1, np.pi/180, threshold=60, minLineLength=90, maxLineGap=50)

    # Draw the lines on the original image
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(line_map, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv.imshow("test", line_map)
    cv.waitKey(0)

    dilated_map = np.ones(map.shape, dtype='uint8')
    for y in range(map.shape[0]):
        for x in range(map.shape[1]):
            if line_map[y, x] == 0:
                cv.circle(dilated_map, (x, y), radius=2, color=255, thickness=-1)

    _, binary_map = cv.threshold(dilated_map, 127, 255, cv.THRESH_BINARY)


    # Fill in the contours in the mask to create a white square
    contours_mask, _ = cv.findContours(binary_map, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    draw_contours(contours_mask, mask)

    # Draw dilated map over mask
    for y in range(map.shape[0]):
        for x in range(map.shape[1]):
            if dilated_map[y, x] == 255:
                cv.circle(mask, (x, y), radius=1, color=0, thickness=-1)

    contours_final, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) 

    draw_contours(contours_final, mask)

    cv.drawContours(mask, cone, -1, 0, -1)

    # Invert the colors
    inverted_image = cv.bitwise_not(mask)

    # Define a structuring element and perform dilation/erosion
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    eroded_image = cv.erode(inverted_image, kernel, iterations=1)

    inverted_image = cv.bitwise_not(eroded_image)
    final_map = cv.cvtColor(mask, cv.COLOR_GRAY2BGR) 

    #----------------------------------------------------------------------------------------------------------

    # Set up the window and mouse callback
    cv.namedWindow("Final Map")
    cv.setMouseCallback("Final Map", mouse_event_handler)
    cv.imshow("Final Map", final_map)
    cv.waitKey(1000)

    # Run particle filtering to get initial position
    initial_angle = np.degrees(sb.imuYaw)
    initial_position = (0, 0)

    x, y, theta = pf.filter(final_map, initial_position, initial_angle) # (x, y, theta)

    #bot_pose = (400, 400)
    bot_location = (x, y)
    # Draw the initial bot position
    cv.circle(final_map, bot_location, 15, [0, 255, 0], -1)
    cv.line(final_map, bot_location, (int(x + 20 * np.cos(-theta)), int(y + 20 * np.sin(-theta))),  [0, 0, 0], 2)

    while key != 27:  # Escape key to exit 
        cv.imshow("Final Map", final_map)
        

        if end_point is not None:
            # Use the endpoint in your logic (if needed)
            print("Start Point:", bot_location)
            print("Selected Endpoint:", end_point)

            path = astar.a_star_pathfinding(bot_location, end_point, filtered_contours, approx_map)

            if path is not None:
                path = path[::25]
                path.append(end_point)
            else:
                print("No Path")
            
            nl.show_path(path, final_map)
        
            return path

        key = cv.waitKey(100)  # Shorter wait time for responsiveness
