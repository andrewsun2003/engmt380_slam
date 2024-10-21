import slamBotHD as sb  
import numpy as np      
import cv2 as cv 
import math        
import time
import csv
import next_locations as nl
import movement as mv
import bot_math as bm

sb.startUp()  
sb.readCoreData()


def read_img():
    depth_img = sb.imgDepth

    while depth_img is None:
        depth_img = sb.imgDepth

    imgRow = depth_img[200:240, :]

    if len(imgRow.shape) != 2:
        raise ValueError("img_rows does not have two dimensions")

    rows, cols = imgRow.shape
    averaged_slice = np.zeros(cols, dtype=float)

    for y in range(cols):
        total = 0
        count = 0

        for x in range(min(8, rows)):
            if imgRow[x, y] > 0:
                total += imgRow[x, y]
                count += 1

        averaged_slice[y] = total / count if count != 0 else 0
    
    imgRow = np.nan_to_num(averaged_slice)  # Handle NaN values 

    return imgRow


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
        if dist != 0 and dist < 2:
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


def full_rotation(initial_angle, bot_x, bot_y):
    step_angle = 30
    current_angle = sb.imuYaw
    
    while int(current_angle) == int(initial_angle):
        current_angle = sb.imuYaw
        mv.rotate_left()

    while int(current_angle) != int(initial_angle):
        mv.rotate_left()
        current_angle = sb.imuYaw
        print(current_angle)
        update_bot(bot_x, bot_y, current_angle)
        cv.waitKey(1)

        if int(current_angle) % int(step_angle) == 0:
            mv.stop_moving()
            cv.waitKey(30)
            current_angle = sb.imuYaw
            imgRow = read_img()
            draw_boundary(current_angle, bot_x, bot_y, imgRow)

            while int(current_angle) % int(step_angle) == 0:
                mv.rotate_left()
                current_angle = sb.imuYaw
            
        
# Initialize maps
map = np.ones([800, 800], np.uint8) * 255
map = cv.cvtColor(map, cv.COLOR_GRAY2BGR)
bot_x, bot_y = 400, 400
key = 0

print(f'Battery: {sb.botBatt}V')


while True:
    
    cv.imshow('Map', map)

    initial_angle = sb.imuYaw

    full_rotation(initial_angle, bot_x, bot_y)

    path = nl.main(map, (bot_x, bot_y))
    print("Path", path)

    for i in range(len(path) - 1):
        current_point = path[i]
        next_point = path[i + 1]
        
        # Calculate new heading
        new_heading = bm.heading(current_point, next_point, current_angle)
        print("New Heading", int(new_heading))
        # Adjust bot's heading
        
        if bm.angle_difference(int(new_heading), int(np.degrees(current_angle))) < 0:
            while int(new_heading) != int(np.degrees(current_angle)):
                mv.rotate_left()
                current_angle = np.radians(sb.imuYaw)
                print("Current Angle", np.degrees(current_angle))
                print("New Heading", int(new_heading))

        else:
            while int(new_heading) != int(np.degrees(current_angle)):
                mv.rotate_right()
                print("Current Angle", np.degrees(current_angle))
                print("New Heading", int(new_heading))
                current_angle = np.radians(sb.imuYaw)
        print("moving forward")
        
        mv.stop_moving()

        # Move towards the next point if heading is correct
        mv.move_to_next_position(current_point, next_point, current_angle)

        bot_x, bot_y = next_point
        update_bot(bot_x, bot_y, current_angle)
    
        start_angle = current_angle
    
    key = cv.waitKey(0) & 0xFF
    if key == ord('q'):
        break

sb.shutDown()