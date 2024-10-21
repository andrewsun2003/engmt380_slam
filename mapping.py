import slamBotHD as sb  
import numpy as np      
import cv2 as cv 
import math        
import time
import csv
import next_locations as nl
import movement as mv
import bot_math as bm

key = 0  # Initialise key variable to handle exit condition
sb.startUp()  # Start the robot's systems
sb.readCoreData()  # Read the core data from the robot's sensors

# Initialize maps
map = np.ones([800, 800], np.uint8) * 255
map = cv.cvtColor(map, cv.COLOR_GRAY2BGR)
previous_map = np.ones([800, 800], np.uint8) * 255
previous_map = cv.cvtColor(previous_map, cv.COLOR_GRAY2BGR)

# Initialize variables
currentRotation = 0
linVel = 0
angVel = 0
start_time = time.time()
start_angle = sb.imuYaw 
bot_x, bot_y = 400, 400
iterations = 40
rotating = True
moving = True  # Variable to track whether the bot is moving
update_flag = 0
moving_start_time = 0
start_distance = (sb.encLeft / 113.32) / 100
timer = 0
rotations = 1
shift_header = 0

# PID Control Variables
iErrors = 0
errors = 0
P = 2
I = 1
previous_angle = 0

scan_count = 0

# Constants for calibration
ANTICLOCKWISE = 1
CLOCKWISE = -1
LEFT = True
RIGHT = False
UP = True
DOWN = False

print(f'Battery: {sb.botBatt}V')

def check_battery():
    elapsed_time = time.time() - start_time
    if np.floor(elapsed_time) % 60 == 0:
        print(f'Battery: {sb.botBatt}V')

def average(img_rows):
    if len(img_rows.shape) != 2:
        raise ValueError("img_rows does not have two dimensions")

    rows, cols = img_rows.shape
    averaged_slice = np.zeros(cols, dtype=float)

    for y in range(cols):
        total = 0
        count = 0

        for x in range(min(8, rows)):
            if img_rows[x, y] > 0:
                total += img_rows[x, y]
                count += 1

        averaged_slice[y] = total / count if count != 0 else 0

    return averaged_slice

def check_to_scan():
    global previous_angle, current_angle, start_angle, iterations, angVel, timer, rotations, angle
    if int(np.degrees(current_angle) % iterations) == 0 and abs(angle_difference(start_angle, np.degrees(current_angle))) > 1 and abs(angle_difference(np.degrees(previous_angle), np.degrees(current_angle))) > 1:
        if timer == 0:
            angle = angVel

        angVel = 0
        timer += 1
        if timer > 3:
            angVel = angle
            previous_angle = current_angle
            draw_boundary()
            rotations = 0
    else:
        timer = 0

def angle_difference(start_angle, current_angle):
    diff = (current_angle - start_angle + 180) % 360 - 180
    return diff

def draw_boundary():
    global bot_x, bot_y, current_angle  # Declare global variables
    stAngle = -0.5
    incAngle = 1 / 639

    for dist in imgRow:
        if dist != 0 and dist < 2.5:
            dist2 = (dist / math.cos(stAngle)) * 100
            x = bot_x + dist2 * math.cos(stAngle - current_angle)
            y = bot_y + dist2 * math.sin(stAngle - current_angle)
            cv.circle(map, (int(x), int(y)), 0, [0, 0, 0], -1) 
        stAngle += incAngle
        
    cv.imshow('Map', map)

def update_bot():
    global bot_x, bot_y
    cv.circle(map, (int(bot_x), int(bot_y)), 18, [0, 255, 0], -1)
    cv.line(map, (bot_x, bot_y), (int(bot_x + 20 * math.cos(-current_angle)), int(bot_y + 20 * math.sin(-current_angle))), [0, 0, 255], 2)

    cv.imshow('Map', map)

    cv.circle(map, (int(bot_x), int(bot_y)), 18, [255, 255, 255], -1)
    cv.line(map, (bot_x, bot_y), (int(bot_x + 20 * math.cos(-current_angle)), int(bot_y + 20 * math.sin(-current_angle))), [255, 255, 255], 2)

def save_map_to_csv(map, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'value'])
        
        for y in range(map.shape[0]):
            for x in range(map.shape[1]):
                pixel_value = map[y, x]
                writer.writerow([x, y, pixel_value])

def turnLeft():
    global rotating, straight, linVel, angVel  # Declare global variables
    rotating = True
    straight = False
    linVel = 0
    angVel = 0.2
    print("Turning Left")

def turnRight():
    global rotating, straight, linVel, angVel  # Declare global variables
    rotating = True
    straight = False
    linVel = 0
   
    angVel = -0.2  # Increase right turn rate
    print("Turning Right")
    
def moveForward():
    global update_flag, rotating, straight, reference_angle, angVel, start_distance, linVel  # Declare global variables
    update_flag = 0
    rotating = False
    straight = True
    reference_angle = sb.imuYaw
    angVel = 0
    start_distance = abs((sb.encLeft / 250) / 100)
    linVel = 0.4
   
    print("Increased Speed")


turnRight()

while key != 27:
    key = cv.waitKey(1) & 0xFF
    
    if key == ord('q'):
        moving = False
        linVel = 0
        angVel = 0
        print("Bot Stopped")

    if moving:
        depth_img = sb.imgDepth
        if depth_img is not None:
            cv.imshow("Depth Image", depth_img)
            imgRow = depth_img[200:240, :]
    
            imgRow = average(imgRow)  # Get the depth data from the middle row of the image
            imgRow = np.nan_to_num(imgRow)  # Handle NaN values 

            if rotating:
                linVel = 0
                sb.moveBot(linVel, angVel)
                current_angle = np.radians(sb.imuYaw)
                update_bot()
                check_to_scan()
                   
                if abs(angle_difference(int(start_angle), int(np.degrees(current_angle)))) == 0 and rotations == 0:
                    path = nl.main(map, (bot_x, bot_y))
                    print("Path", path)
                    shift_header = 1
                    cv.waitKey(0)
                

                    '''
                    if currentRotation == 1:
                        draw_boundary()
                        previous_map = map
                        save_map_to_csv(previous_map, 'previous_map.csv')
                        map = np.ones([800, 800], np.uint8) * 255
                        moveForward()
                    
                    if currentRotation == 2:
                        map = cv.cvtColor(map, cv.COLOR_GRAY2BGR)
                        save_map_to_csv(map, 'map.csv')
                    if currentRotation > 2:
                        moving = False
                        '''
                    currentRotation += 1
                    rotations = 1

                    
                if shift_header and len(path) > 1:  # Ensure there are at least two points in the path
                    for i in range(len(path) - 1):
                        current_point = path[i]
                        next_point = path[i + 1]
                        
                        # Calculate new heading
                        new_heading = bm.heading(current_point, next_point, current_angle)
                        print("New Heading", int(new_heading))
                        # Adjust bot's heading
                       
                        if angle_difference(int(new_heading), int(np.degrees(current_angle))) < 0:
                            while int(new_heading) != int(np.degrees(current_angle)):
                                turnLeft()
                                sb.moveBot(linVel, angVel)
                                current_angle = np.radians(sb.imuYaw)
                                print("Current Angle", np.degrees(current_angle))
                                print("New Heading", int(new_heading))

                        else:
                            while int(new_heading) != int(np.degrees(current_angle)):
                                turnRight()
                                sb.moveBot(linVel, angVel)
                                print("Current Angle", np.degrees(current_angle))
                                print("New Heading", int(new_heading))
                                current_angle = np.radians(sb.imuYaw)
                        print("moving forward")
                        angVel = 0

                        # Move towards the next point if heading is correct
                        mv.move_to_next_position(current_point, next_point, current_angle, angVel)

                        bot_x, bot_y = next_point
                        update_bot()
                        
                        shift_header = 0  # Reset if we have processed all points in the path
                        start_angle = current_angle
            
             
            if straight:
                error = reference_angle - sb.imuYaw
                angleChange = 0.1 / (0.1 - (P * error + I * iErrors) / 100)

                if update_flag == 0:
                    sb.moveBot(linVel, angVel)
                    moving_start_time = time.time()

                moving_elapsed_time = time.time() - moving_start_time
                moving_distance = ((abs(sb.encLeft) / 250) / 100 - start_distance)

                if moving_distance > 0.25:   
                    update_flag = 1                                             
                    linVel = 0
                    straight = False
                    sb.moveBot(linVel, angVel)
                    bot_x += round(moving_distance * 100 * math.cos(-current_angle))
                    bot_y += round(moving_distance * 100 * math.sin(-current_angle))
                    draw_boundary()
                    update_bot()
                    turnLeft()
                    rotations = 0

                iErrors += error

sb.shutDown()
