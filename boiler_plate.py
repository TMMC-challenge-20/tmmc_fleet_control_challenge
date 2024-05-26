#Start with imports, ie: import the wrapper
#import other libraries as needed
import TMMC_Wrapper
import rclpy
import numpy as np
import math

#start ros
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = False
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

stopSignFrameCount = 0

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
        isStopSign = robot.checkStopSignInImage()
        if isStopSign == 1:
            stopSignFrameCount = stopSignFrameCount + 1
        elif isStopSign == 0:
            stopSignFrameCount = 0
        obs_dist, obs_angle = robot.detect_obstacle(scan.ranges)
        if stopSignFrameCount == 5:
            print('stop')
            robot.stop()
            continue
        if (obs_dist < 1) and (obs_dist > 0):
            robot.move_backward()
        else: # # If not too close to wall
            robot.move_forward()

except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.stop_keyboard_control()
    robot.destroy_node()
    rclpy.shutdown()
