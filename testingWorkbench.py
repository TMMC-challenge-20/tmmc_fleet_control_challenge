#start with imports, ie: import the wrapper
import TMMC_Wrapper
import rclpy
import cv2

#start ros
if not rclpy.ok():
    rclpy.init()

DEBUGGING = False

#specify hardware api
TMMC_Wrapper.use_hardware()
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#debug messaging 
print("running main")

#start processes
robot.start_keyboard_control()
rclpy.spin_once(robot, timeout_sec=0.1)

#testing sensor returns
if DEBUGGING == True:
    batteryValue = robot.checkBattery()
    print(batteryValue)
    imuValue = robot.checkImu()
    print(imuValue)
    scanValue = robot.checkScan()
    print(scanValue)

#testing oak-d returns 
imageValue = robot.checkImage()
print(imageValue)

cameraValue = robot.checkCamera()
print(cameraValue)

# test Camera features
# process image data
image = robot.checkImage()
height = image.height
width = image.width
print(f"image with height {height} and width {width}")
img_data = image.data
print(f"data length: {len(img_data)}")
img_3D = np.reshape(img_data, (height, width, 3))
print("close the image window to continue.")
cv2.imshow("image", img_3D)
cv2.waitKey(0)

print("result from apriltag detection:\n", robot.detect_april_tag_from_img(img_3D))


try:
    print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
    while True: 
        rclpy.spin_once(robot, timeout_sec=0.1)
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    robot.stop_keyboard_control()
    robot.destroy_node()
    rclpy.shutdown()


