import slamBotHD as sb
import time 
import bot_math as bm
import math
import mapping_copy as mp


sb.readCoreData()


def move_to_next_position(current_position, next_position, set_point):
    global l_prev, r_prev, ticks_per_meter


    ticks_per_meter = 11239
    linear_velocity = 0.2
    l_init = sb.encLeft
    r_init = sb.encRight
    initial_ticks = (l_init + r_init)/2
    l_prev = l_init
    r_prev = r_init
    distance = (bm.distance(current_position, next_position)/100)*ticks_per_meter
    difference_ticks = 0
    error = 0
    integral = 0
    P = 0.02
    I = 0.000003

    while(difference_ticks < distance):

        left, right = read_encoders()
        current_ticks = (left + right)/2
        difference_ticks = current_ticks - initial_ticks

        current_angle = sb.imuYaw

        #print(F"CurrentAngle {current_angle}")
        error = bm.angle_difference(set_point, current_angle)

        #print(f"Error: {error}")

        integral += error

        angular_velocity = P*error + I*integral

        if (angular_velocity > 1.3):
            angular_velocity = 1.3
        elif (angular_velocity < -1.3):
            angular_velocity = -1.3

        print(difference_ticks)
        
        sb.moveBot(linear_velocity, 0)  


def rotate_left():
    sb.moveBot(0, 0.2)
    print("Left")

def rotate_right():
    sb.moveBot(0, -0.2)
    print("Left")


def stop_moving():
     sb.moveBot(0, 0)
     print("Stop")


def read_encoders():
	global r_prev, l_prev
	
	l = sb.encLeft
	r = sb.encRight
      
	if (l_prev - l > 50000):
		l += 65535
	
	if (r_prev - r > 50000):
		r += 65535
		
	l_prev = l
	r_prev = r
			
	return [l, r]