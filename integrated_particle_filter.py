import cv2 as cv
import sys
import numpy as np
import pandas as pd
from numba import njit
import distance as d
import next_locations as nl
import slamBotHD as sb
import movement as mv
from sklearn.cluster import DBSCAN
import bot_math as bm

# Global variables
mm_per_pix = 10
bot_diameter_mm = 350
num_particles = 10000  # Number of particles
radius = int(bot_diameter_mm / mm_per_pix / 2)  # Circle radius
width, height = 0, 0  # To be defined later
threshold_img = None  # To be defined later
display_img = None  # To be defined later
set_initial = 0
initial_angle = 0
rotate_flag = 1
prev_scan_av = 0
next_angle = 0
scan = 0
threshold = 0.1

# ------------------------------------------ Functions ----------------------------------------------- #

# Function to check if a circle is valid
@njit
def is_circle_valid(x, y):
    for angle in range(0, 360, 10):  # Check 30 degree points around the circle
        rad = np.radians(angle)
        check_x = int(x + radius * np.cos(rad))
        check_y = int(y + radius * np.sin(rad))
        if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
            return False  # Out of bounds
        if threshold_img[check_y, check_x] == 0:  # Black area
            return False
    return True

# Generate new particles around the existing ones with the best score
@njit
def resample_particles(best_particles, weights):
    new_particles = []
    for i, particle in enumerate(best_particles):
        x, y, theta = particle
        num_of_new = int(15 * weights[i])  # Number of new particles generated around each existing particle
        
        for j in range(num_of_new):
            new_x = x + np.random.normal(0, int(1 / weights[i]))  # Perturb X position 
            new_y = y + np.random.normal(0, int(1 / weights[i]))  # Perturb Y position 
            new_theta = theta + np.random.normal(0, 0.1)  # Perturb angle (sigma = 0.1 radians)

            while not is_circle_valid(new_x, new_y):
                new_x = x + np.random.normal(0, int(1 / weights[i]))
                new_y = y + np.random.normal(0, int(1 / weights[i]))
                new_theta = theta + np.random.normal(0, 0.1)
            
            new_particles.append((new_x, new_y, new_theta))
    
    return np.array(new_particles)

# Draw the circles on the map
def draw_circles(particles):
    global display_img  # Access the global display_img variable
    display_img = cv.cvtColor(threshold_img, cv.COLOR_GRAY2BGR)
    
    for particle in particles:
        x, y = int(particle[0]), int(particle[1])
        theta = particle[2]  # Get the orientation
        cv.circle(display_img, (x, y), radius, (0, 0, 255), -1)  # Draw particles as red dots
        
        line_length = 20  # Length of the direction line
        end_x = int(x + line_length * np.cos(theta))
        end_y = int(y + line_length * np.sin(theta))
        cv.line(display_img, (x, y), (end_x, end_y), (255, 0, 0), 2)  # Draw line in blue

    cv.imshow("Particle Filter", display_img)

# Get what the particles see
@njit
def particle_scan(particle):
    x, y, theta = particle
    particle_scan = []
    max_range = 5000
    lower_limit = 500
    
    laser_range = 1  # Convert 57 degrees to radians
    num_readings = 640  # Total number of readings
    start_angle = theta - laser_range / 2
    step_size = laser_range / (num_readings - 1)
    
    angle = start_angle
    for _ in range(num_readings):
        for distance in range(10, max_range, 1):  # Incrementally check distances
            distance /= mm_per_pix

            end_x = (x + distance * np.cos(angle))
            end_y = (y + distance * np.sin(angle))
            if 0 <= end_x < width and 0 <= end_y < height:
                if threshold_img[int(end_y), int(end_x)] == 0:  # Obstacle found   
                    if distance * mm_per_pix < lower_limit:
                        particle_scan.append(lower_limit)
                    else:
                        particle_scan.append(int(distance * mm_per_pix))  # Store distance as mm
                    break
        else:
            particle_scan.append(max_range)  # Store distance as mm
        angle += step_size
    
    #print(particle_scan)

    return np.array(particle_scan)

# Find the best of the current particles
def find_best(particles, scan_data, threshold):
    best_particles = []
    weights = []
    
    for particle in particles:
        particle_scan_data = particle_scan(particle)  # Finds the distance data for the particles in mm

        if np.std(particle_scan_data) < 10:
            if np.std(scan_data) < 100:
                best_particles.append(particle)
                weights.append(1)
        else:
            coef = np.corrcoef(scan_data, particle_scan_data)[0, 1]
            if coef > threshold:
                best_particles.append(particle)
                weights.append(coef)
                
    return np.array(best_particles), np.array(weights)

@njit
def move_particles(particles, distance, delta_theta):
    valid_particles = []
    for i in range(len(particles)):
        new_theta = particles[i, 2] + delta_theta
        new_x = particles[i, 0] + distance * np.cos(new_theta)
        new_y = particles[i, 1] + distance * np.sin(new_theta)
        
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi  # Keep θ between -π and π

        if is_circle_valid(new_x, new_y):
            valid_particles.append((new_x, new_y, new_theta))

    return np.array(valid_particles)

def count_clusters(particles, eps=10):  # Adjust eps based on your scale
    if len(particles) == 0:
        return 0
    
    # Create an array with angles normalized
    angles = particles[:, 2] % (2 * np.pi)  # Normalize angles to [0, 2π]
    x_y_angles = np.column_stack((particles[:, :2], angles))  # Combine x, y, and angles

    # Use DBSCAN to find clusters
    clustering = DBSCAN(eps=eps, min_samples=1).fit(x_y_angles)  # Consider x, y and angle for clustering
    return len(set(clustering.labels_))  # Return the number of unique clusters


def get_pose(bot_pose):
    global set_initial, initial_angle, rotate_flag, prev_scan_av, next_angle, scan, threshold
    
    if rotate_flag == 1:

        if set_initial == 0:
            initial_angle = sb.imuYaw - 1

        step_angle = 10
        current_angle = sb.imuYaw
        if (current_angle < step_angle) and current_angle >= 0:
            while np.floor(current_angle) != (np.floor(step_angle)+10):
                if np.floor(current_angle) == np.floor(initial_angle) - 1:   # Break
                    rotate_flag = 0 
                current_angle = sb.imuYaw
                mv.rotate_left()
        
        if set_initial == 0:
            initial_angle = sb.imuYaw - 1
            set_initial = 1
        
        if np.floor(current_angle) == np.floor(initial_angle):   # Break
            rotate_flag = 0   

        while np.floor(current_angle) % np.floor(step_angle) != 0:
            if np.floor(current_angle) == np.floor(initial_angle) - 1:   # Break
                rotate_flag = 0 
            mv.rotate_left()
            current_angle = sb.imuYaw
            print(current_angle)
            print("Initial Angle", np.floor(initial_angle))
        
        while np.floor(current_angle) % np.floor(step_angle) == 0:  
            if np.floor(current_angle) == np.floor(initial_angle) - 1:   # Break
                rotate_flag = 0 
            mv.rotate_left()
            current_angle = sb.imuYaw     

        mv.stop_moving()
        cv.waitKey(100)
        current_angle = sb.imuYaw   # Return angle in rad

        scan_data = d.read_img()
        scan_data *= 1000
        
        middle_data = scan_data[240:400]
        

        standard_deviation = np.std(middle_data)
        print(standard_deviation)

        if standard_deviation < 15:

            scan_av = np.mean(middle_data)
            print("Scan Average: ", scan_av)

            if scan_av > prev_scan_av:
                next_angle = current_angle
                scan = scan_av
                print("Next Angle: ", next_angle)
            prev_scan_av = scan_av
        
        bot_pose = (bot_pose[0], bot_pose[1], np.radians(current_angle))  # Return same x, y, because only rotating (no change in position), uptate angle (rad)

        return bot_pose, 0
    
    elif rotate_flag == 0:

        current_angle = sb.imuYaw

        if bm.angle_difference(np.floor(next_angle), np.floor(current_angle)) > 0:
            while np.floor(current_angle) != np.floor(next_angle):
                mv.rotate_left()
                print("Current Angle", current_angle)
                print("New Heading", int(next_angle))
                current_angle = sb.imuYaw

        else:
            while np.floor(current_angle) != np.floor(next_angle):
                mv.rotate_right()
                print("Current Angle", current_angle)
                print("New Heading", int(next_angle))
                current_angle = sb.imuYaw
        
        distance = ((scan/10)-60)   # Give function cm

        mv.move_to_next_position(None, None, next_angle, distance)

        new_x = int(bot_pose[0] + np.cos(np.radians(current_angle)) * distance)
        new_y = int(bot_pose[1] - np.sin(np.radians(current_angle)) * distance)

        print("X: ", new_x)
        print("Y: ", new_y)
        print("Distance: ", distance)

        rotate_flag = 1
        scan_av = 0
        scan = 0
        prev_scan_av = 0
        next_angle = 0
        set_initial = 0

        threshold = 0

        bot_pose = (new_x, new_y, np.radians(current_angle))

        return bot_pose, distance   # (x, y, theta), scan data (mm)
    
    else:
        return bot_pose, distance
    

# -------------------------------------------MAIN------------------------------------------------------- #

def filter(img, initial_pos, initial_angle):
    global width, height, threshold_img, display_img, threshold  # Declare globals

    if img is None:
        sys.exit("Could not read the image.")

    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, threshold_img = cv.threshold(gray_img, 254, 255, cv.THRESH_BINARY)
    display_img = cv.cvtColor(threshold_img, cv.COLOR_GRAY2BGR)

    height, width = threshold_img.shape
    particles = np.zeros((num_particles, 3))  # Each particle: (X, Y, θ)

    white_pixels = np.argwhere(threshold_img == 255)
    for i in range(num_particles):
        idx = np.random.choice(len(white_pixels))
        y, x = white_pixels[idx]
        while not is_circle_valid(x, y):
            idx = np.random.choice(len(white_pixels))
            y, x = white_pixels[idx]
        particles[i, 0] = x
        particles[i, 1] = y
        particles[i, 2] = np.random.uniform(-np.pi, np.pi)

    draw_circles(particles)
    cv.waitKey(1)

    timestep = 0
    key = 0

    # Get initial bot pose 
    bot_pose = (initial_pos[0], initial_pos[1], initial_angle)
    prev_pose = bot_pose
    

    while key != 27:  # Escape key to exit 

        
        bot_pose, distance = get_pose(prev_pose)     # Moves the bot and returns the pose

        scan_data = d.read_img()
        scan_data *= 1000
        scan_data[scan_data < 500] = 500
        scan_data = np.round(scan_data)
        
        #cv.waitKey(0)
        
        if timestep > 0:
            """delta_x = (bot_pose[0] - prev_pose[0])  # Convert from mm to pixels
            delta_y = (bot_pose[1] - prev_pose[1])
            
            
            distance = np.sqrt(delta_x**2 + delta_y**2)

            if distance > 0:
                print(delta_x)
                print(delta_y)
                print(delta_theta)
                print(distance)
                cv.waitKey(0)"""

            delta_theta = bot_pose[2] - prev_pose[2]
            particles = move_particles(particles, distance, -delta_theta)
        
        draw_circles(particles)

        particles, weights = find_best(particles, scan_data, threshold)

        #print(scan_data)
        #print(weights)
        print(threshold)

        if particles.shape[0] == 0:
            print("No best particles.")
            cv.waitKey(0)
            return

        if len(particles) < 100:
            particles = resample_particles(particles, weights)
            threshold += 0.05  # Increase threshold everytime we resample
            if threshold > 0.3:
                threshold = 0.3   # Limit the threshold
            print(threshold)

            # Check for clusters
            num_clusters = count_clusters(particles)
            if num_clusters <= 1:
                print("Only one cluster left")
                #Average the particles positions and angles
                av_x = int(np.mean(particles[:, 0]))  # Average x positions
                av_y = int(np.mean(particles[:, 1]) ) # Average y positions
                av_angle = np.mean(particles[:, 2])  # Average angles

                return av_x, av_y, av_angle
            
        draw_circles(particles)

        prev_pose = bot_pose

        timestep += 1
        key = cv.waitKey(1)

    cv.destroyAllWindows()

