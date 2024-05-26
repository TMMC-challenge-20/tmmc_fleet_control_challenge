#Start with imports, ie: import the wrapper
#import other libraries as needed
import time
import TMMC_Wrapper
import rclpy
import numpy as np
import math
from yv5_func import detect_stop_sign

#start ros
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = True
if not TMMC_Wrapper.is_SIM:
    #specify hardware api
    TMMC_Wrapper.use_hardware()
    
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#debug messaging 
print("running main")

#start processes
robot.start_keyboard_control()   #this one is just pure keyboard control

#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.5)

stop_counter = 0
stop_sign_detected = False
stop_time = 3
stop_end_time = None

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:

        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.5)

        #Add looping functionality here

        # # Keyboard control (level 1)
        robot.start_keyboard_control()

        scan = robot.checkScan()
        obs_dist, obs_angle = robot.detect_obstacle(scan.ranges)
        for result in detect_stop_sign('stop.mp4', True):
            if result == 1:
                stop_counter += 1
            else:
                stop_counter = 0

            if stop_counter == 4:
                stop_sign_detected = True
                stop_end_time = time.time() + stop_time
                break

        if stop_sign_detected and time.time() < stop_end_time:
            # If a stop sign is detected and the stop time hasn't elapsed, stop the robot
            robot.stop_keyboard_control()
        elif (obs_dist < 1) and (obs_dist > 0):
            # If too close to wall, move backward
            robot.move_backward()
        else: 
            # If not too close to wall, move forward
            robot.move_forward()

        # If the stop time has elapsed, resume motion
        if stop_sign_detected and time.time() >= stop_end_time:
            stop_sign_detected = False

except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.stop_keyboard_control()
    robot.destroy_node()
    rclpy.shutdown()
