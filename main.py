import slamBotHD as sb  
import numpy as np      
import cv2 as cv 
import math        
import csv
import next_locations as nl
import movement as mv
import bot_math as bm
import compare_maps as cm
import distance as d

sb.startUp()  
sb.readCoreData()

def save_map_to_csv(map, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'value'])
        
        for y in range(map.shape[0]):
            for x in range(map.shape[1]):
                pixel_value = map[y, x]
                writer.writerow([x, y, pixel_value])


def draw_boundary(current_angle, bot_x, bot_y, imgRow):
    stAngle = -0.5
    incAngle = 1 / 639
    current_angle = np.radians(current_angle)

    for dist in imgRow:
        if dist != 0 and dist < 2.2:
            dist2 = (dist / math.cos(stAngle)) * 100
            x = bot_x + dist2 * math.cos(stAngle - current_angle)
            y = bot_y + dist2 * math.sin(stAngle - current_angle)
            cv.circle(map, (int(x), int(y)), 0, [0, 0, 0], -1) 
        stAngle += incAngle
    
    cv.imshow('Map', map)


def update_bot(bot_x, bot_y, current_angle):
    current_angle = np.radians(current_angle)
    cv.circle(map, (int(bot_x), int(bot_y)), 18, [0, 255, 0], -1)
    cv.line(map, (bot_x, bot_y), (int(bot_x + 20 * math.cos(-current_angle)), int(bot_y + 20 * math.sin(-current_angle))), [0, 0, 255], 2)
    
    cv.imshow('Map', map)
    
    cv.circle(map, (int(bot_x), int(bot_y)), 18, [255, 255, 255], -1)
    cv.line(map, (bot_x, bot_y), (int(bot_x + 20 * math.cos(-current_angle)), int(bot_y + 20 * math.sin(-current_angle))), [255, 255, 255], 2)


def full_rotation(bot_x, bot_y):

    mv.stop_moving()
    cv.waitKey(1000)

    step_angle = 30
    current_angle = sb.imuYaw
    if (current_angle < step_angle) and current_angle >= 0:
        while np.floor(current_angle) != (np.floor(step_angle)+10):
            current_angle = sb.imuYaw
            mv.rotate_left()

    mv.stop_moving()
    current_angle = sb.imuYaw
    initial_angle = sb.imuYaw



    while np.floor(current_angle) == np.floor(initial_angle):
        mv.rotate_left()
        current_angle = sb.imuYaw

    while np.floor(current_angle) != np.floor(initial_angle):
        mv.rotate_left()
        current_angle = sb.imuYaw
        print(current_angle)
        print("Initial Angle", np.floor(initial_angle))
        update_bot(bot_x, bot_y, current_angle)
        cv.waitKey(1)

        if np.floor(current_angle) % np.floor(step_angle) == 0:
            if np.floor(current_angle) == np.floor(initial_angle):
                return
            mv.stop_moving()
            cv.waitKey(100)
            current_angle = sb.imuYaw
            imgRow = d.read_img()
            draw_boundary(current_angle, bot_x, bot_y, imgRow)

            while np.floor(current_angle) % np.floor(step_angle) == 0:
                mv.rotate_left()
                current_angle = sb.imuYaw

def move_path(path):
    for i in range(len(path) - 1):
        current_point = path[i]
        next_point = path[i + 1]
        
        # Calculate new heading
        new_heading = bm.heading(current_point, next_point)
        print("New Heading", int(new_heading))
        # Adjust bot's heading
        if abs(new_heading) == 180:
            new_heading = 179

        current_angle = sb.imuYaw

        if bm.angle_difference(int(new_heading), int(current_angle)) > 0:
            while int(current_angle) != int(new_heading):
                mv.rotate_left()
                print("Current Angle", current_angle)
                print("New Heading", int(new_heading))
                current_angle = sb.imuYaw

        else:
            while int(current_angle) != int(new_heading):
                mv.rotate_right()
                print("Current Angle", current_angle)
                print("New Heading", int(new_heading))
                current_angle = sb.imuYaw
                
        print("moving forward")
        
        mv.stop_moving()
        cv.waitKey(1000)

        # Move towards the next point if heading is correct
        mv.move_to_next_position(current_point, next_point, current_angle)

        bot_x, bot_y = next_point
        update_bot(bot_x, bot_y, current_angle)

    return next_point, current_angle, bot_x, bot_y
            
        
# Initialize maps
map = np.ones([800, 800], np.uint8) * 255
map = cv.cvtColor(map, cv.COLOR_GRAY2BGR)
bot_x, bot_y = 400, 400
key = 0
turns = 0


print(f'Battery: {sb.botBatt}V')

#-----------------------------Main-------------------------------#

while True:
    
    cv.imshow('Map', map)

    full_rotation(bot_x, bot_y)

    turns += 1

    if turns >= 2:
        cv.imshow("Map", map)
        cv.imshow("Previous Map", previous_map)
        cv.waitKey(20)
        combined_map, current_corners, previous_corners = cm.main(map, previous_map)

        cv.imshow("combined map", combined_map)
        cv.imshow("current corners", current_corners)
        cv.imshow("previous corners", previous_corners)

        map = combined_map

    cv.waitKey(10)

    path = nl.main(map, (bot_x, bot_y))
    print("Path", path)
    cv.waitKey(0)

    _, _, bot_x, bot_y = move_path(path)

    # Reset map and set previous map to the combined map
    previous_map = map
    
    cv.imshow("Previous Map", previous_map)
    map = np.ones([800, 800], np.uint8) * 255
    map = cv.cvtColor(map, cv.COLOR_GRAY2BGR)

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break

sb.shutDown()
