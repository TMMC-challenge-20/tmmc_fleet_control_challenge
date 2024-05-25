#-----------------
#---Features we want---
#---    - subscribe and publish to cmd vel  (done)
#---    - setting max velocity (done)
#---    - stopping at stop sign (camera integration from allyn)
#---    - WASD control (done)
#---    - dock and undock  (done)
#---    - simulator (in progress)
#---    - sensor values (done)
#-----------------

#-----imports-----
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.action import Dock,Undock
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.srv import ResetPose
from geometry_msgs.msg import Quaternion
import time
import rclpy
import tf2_ros
import tf2_ros.buffer
import tf2_ros.transform_listener
import eigenpy
import numpy as np
import os
import subprocess
from pynput.keyboard import Listener
import math

#---imports for vision---
import cv2
import apriltag
import numpy as numpy
import matplotlib.pyplot as plt

#---constants---
CONST_speed_control = 1 #set this to 1 for full speed, 0.5 for half speed
DEBUG = False #set to false to disable terminal printing of some functions

is_SIM = False #to disable some functions that can not be used on the sim

#not sure if we need , modify later, seems like an init thing
def use_hardware():
    global is_SIM
    if not is_SIM:
        # import ROS settings for working locally or with the robot (equivalent of ros_local/ros_robot in the shell)
        env_file = ".env_ros_robot"
        os.environ.update(dict([l.strip().split("=") for l in filter(lambda x: len(x.strip())>0,open(env_file).readlines())]))
        try:
            output = subprocess.check_output("ip addr show",shell=True)
            import re
            robot_id = int(re.search(r"tap[0-9]\.([0-9]+)@tap",output.decode()).groups()[0])
        except Exception as ex:
            raise Exception("VPN does not seem to be running, did you start it?:",ex)
        print("You are connected to uwbot-{:02d}".format(robot_id))
        try:
            subprocess.check_call("ping -c 1 -w 10 192.168.186.3",shell=True,stdout=subprocess.DEVNULL)
        except Exception as ex:
            raise Exception("Could not ping robot (192.168.186.3)")
        print("Robot is reachable")
        try:
            subprocess.check_call("ros2 topic echo --once /ip",shell=True,stdout=subprocess.DEVNULL)
        except Exception as ex:
            print("ros2 topic echo --once /ip failed. Proceed with caution.")
        print("ros2 topic subscription working. Everything is working as expected.")
    

class Robot(Node):
    def __init__(self):
        super().__init__('notebook_wrapper')
        # Create custom qos profile to make subscribers time out faster once notebook
        import rclpy.qos
        import rclpy.time
        from copy import copy
        qos_profile_sensor_data = copy(rclpy.qos.qos_profile_sensor_data)
        qos_policy = copy(rclpy.qos.qos_profile_sensor_data)
        #qos_policy.liveliness = rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC
        #qos_policy.liveliness_lease_duration = rclpy.time.Duration(seconds=10)

        self.last_scan_msg = None
        self.last_imu_msg = None

        self.scan_future = rclpy.Future()
        self.scan_subscription = self.create_subscription(LaserScan,'/scan',self.scan_listener_callback,qos_policy)
        self.scan_subscription  # prevent unused variable warning
        
        self.imu_future = rclpy.Future()
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_listener_callback,qos_profile_sensor_data)
        self.imu_subscription  # prevent unused variable warning
        
        self.image_future = rclpy.Future()
        self.image_subscription = self.create_subscription(Image,'/oakd/rgb/preview/image_raw',self.image_listener_callback,qos_profile_sensor_data)
        self.image_subscription  # prevent unused variable warning
        
        self.camera_info_future = rclpy.Future()
        self.camera_info_subscription = self.create_subscription(CameraInfo,'/oakd/rgb/preview/camera_info',self.camera_info_listener_callback,qos_profile_sensor_data)
        self.camera_info_subscription  # prevent unused variable warning
        
        self.battery_state_future = rclpy.Future()
        self.battery_state_subscription = self.create_subscription(BatteryState,'/battery_state',self.battery_state_listener_callback,qos_profile_sensor_data)
        self.battery_state_subscription  # prevent unused variable warning

        global is_SIM
        if (not(is_SIM)): 
            self.dock_client = ActionClient(self, Dock, '/dock')
            self.undock_client = ActionClient(self, Undock, '/undock')
            self.dock_client.wait_for_server()
            self.undock_client.wait_for_server()
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.logging_topics = ["/tf","/tf_static","/scan","/odom"]
        
        self._reset_pose_client = self.create_client(ResetPose, '/reset_pose')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keyboard_listener = None #temp placeholder for the keyboard listener
    
    def get_tf_transform(self,parent_frame,child_frame,wait=True,time_in=rclpy.time.Time()):
        if wait:
            myfuture = self.tf_buffer.wait_for_transform_async(parent_frame,child_frame,time_in)
            self.spin_until_future_completed(myfuture)
        t = self.tf_buffer.lookup_transform(parent_frame,child_frame,time_in).transform
        q = eigenpy.Quaternion(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z)
        R = q.toRotationMatrix()
        v = numpy.array([[t.translation.x,t.translation.y,t.translation.z,1]]).T
        T = numpy.hstack([numpy.vstack([R,numpy.zeros((1,3))]),v])
        return T
    
    def reduce_transform_to_2D(self,transform_3D):
        return transform_3D[(0,1,3),:][:,(0,1,3)]
    
    def rotation_from_transform(self, transform_2D):
        import numpy
        fake_R3D = numpy.eye(3)
        fake_R3D[0:2,0:2] = transform_2D[0:2,0:2]
        import eigenpy
        aa = eigenpy.AngleAxis(fake_R3D)
        return aa.angle
   
    def configure_logging(self,topics):
        self.logging_topics = topics
            
    def start_logging(self):
        if hasattr(self,'logging_instance'):
            raise Exception("logging already active")
        self.logging_dir = bag_dir = '/tmp/notebook_bag_'+str(int(time.time()))
        self.logging_instance = subprocess.Popen("ros2 bag record -s mcap --output "+self.logging_dir+" "+' '.join(self.logging_topics)+" > /tmp/ros2_bag.log 2>&1",shell=True,stdout=subprocess.PIPE,preexec_fn=os.setsid)
        # Wait until topics are subscribed
        # TODO: check log for this
        time.sleep(5)
        
    def stop_logging(self):
        import signal
        os.killpg(os.getpgid(self.logging_instance.pid), signal.SIGINT)
        self.logging_instance.wait()
        del self.logging_instance
        return self.logging_dir
            
    def get_logging_data(self, logging_dir):
        # get log data
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=logging_dir,storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('','')
        reader.open(storage_options,converter_options)
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        log_content = dict()
        while reader.has_next():
            (topic,data,t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            if topic not in log_content.keys():
                log_content[topic] = []
            log_content[topic].append((t,msg))
        return log_content
    
    def delete_logging_data(self, logging_dir):
        import shutil
        shutil.rmtree(logging_dir)

#-----scan listeners and grabbers-----    
    def scan_listener_callback(self, msg):
        self.last_scan_msg = msg
        if DEBUG == True:
            print(f"Laserscan data recieved: Range - {msg.ranges[:5]}")
        self.scan_future.set_result(msg)
        self.scan_future.done()

    def checkScan(self):
        self.scan_future = rclpy.Future()
        self.spin_until_future_completed(self.scan_future)
        return self.last_scan_msg
        
#-----imu listeners and grabbers-----
    def imu_listener_callback(self, msg):
        self.last_imu_msg = msg
        if DEBUG == True:
            print(f"IMU Data recieved: orientation - {msg.orientation}")
        self.imu_future.set_result(msg)
        self.imu_future.done()
        
    def checkImu(self):
        self.imu_future = rclpy.Future()
        self.spin_until_future_completed(self.imu_future)
        return self.last_imu_msg
    
    def rotation_angle(self,q):
        # Extract the angle of rotation from the quaternion
        # w = cos(theta/2)
        w_clamped = max(-1.0, min(1.0, q.w))
        return 2 * math.acos(w_clamped)
    
    def conjugate_q(self, q):
         
         return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)
    
    def quaternion_multiply(self,q1, q2):
        # Quaternion multiplication q1 * q2
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
        return Quaternion(
           w = w1*w2 - x1*x2 - y1*y2 - z1*z2,
           x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
           y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
           z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    )
    
    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if yaw<0:
            yaw += math.pi*2

        return roll, pitch, yaw

    def has_rotation_occurred(self,orientation1, orientation2, desired_rotation_angle):
        # Conjugate of orientation 1
        q1_inv = self.conjugate_q(orientation1)
        
        # Relative quaternion 
        q_rel = self.quaternion_multiply(orientation2, q1_inv)
        
        #Angle calculation
        rotation_angle = self.rotation_angle(q_rel)
        #print(f"Original q: {round(orientation1.w,2), round(orientation1.x,2), round(orientation1.y,2), round(orientation1.z,2)} and current q: {round(orientation2.w,2), round(orientation2.x,2), round(orientation2.y,2), round(orientation2.z,2)}")
        print("current rotation: ", math.degrees(rotation_angle))
    
        #is desired angle met
        return math.isclose(rotation_angle, desired_rotation_angle, abs_tol=0.01)  # Adjust tolerance as needed
    
#-----image listeners and grabbers-----     
    def image_listener_callback(self, msg):
        self.last_image_msg = msg
        self.image_future.set_result(msg)
        self.image_future.done()
    
    def checkImage(self):
        self.image_future = rclpy.Future()
        self.spin_until_future_completed(self.image_future)
        return self.last_image_msg

    def checkImageRelease(self): #this one returns an actual image instead of all the data
        image = self.checkImage()
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        cv2.imshow("image", img_3D)
        cv2.waitKey(10)
        
#-----camera listeners and grabbers-----  
    def camera_info_listener_callback(self, msg):
        self.last_camera_info_msg = msg
        self.camera_info_future.set_result(msg)
        self.camera_info_future.done()

    def checkCamera(self):
        self.camera_info_future = rclpy.Future()
        self.spin_until_future_completed(self.camera_info_future)
        return self.last_camera_info_msg 
        # ^ this might have more data, test this

#-----battery listeners and grabbers-----   
    def battery_state_listener_callback(self, msg):
        self.last_battery_state_msg = msg
        self.battery_state_future.set_result(msg)
        self.battery_state_future.done()
    
    def checkBattery(self):
        self.battery_state_future = rclpy.Future()
        self.spin_until_future_completed(self.battery_state_future)
        return self.last_battery_state_msg.percentage
        
    def cmd_vel_timer_callback(self):
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate and self.cmd_vel_stop:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.cmd_vel_publisher.publish(msg)
            
    def set_cmd_vel(self, velocity_x, velocity_phi, duration, stop=True):
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        self.cmd_vel_stop = stop
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self,self.cmd_vel_future)  
        
    def spin_until_future_completed(self,future):
        rclpy.spin_until_future_complete(self,future)
        return future.result()
    
    def undock(self):
        # does not wait until finished
        global is_SIM
        if not is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self,action_completed_future)
            return action_completed_future.result()
        
    def dock(self):
        global is_SIM
        if not is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self,action_completed_future)
            return action_completed_future.result()

#----some functions for telop control----
    def rotate(self, angle, direction):
        '''Rotate by a certain angle and direction
            Params : angle in deg, direction 1 or -1
            Return : none
        '''
        #get the starting quaternion
        q1 = self.checkImu().orientation
        #get the yaw angle in rad from the quaternion
        _,_,yaw1 = self.euler_from_quaternion(q1)
        #Convert to deg
        yaw1 = math.degrees(yaw1)
        #start the second yaw at the start yaw
        yaw2 = yaw1
        print(f"angle: {angle}")
        print(f"yaw 1: {yaw1} yaw2: {yaw2}")
        #while the angle between the new yaw while rotating is not the desired angle rotate
        while abs(yaw2 - yaw1) <= abs(angle):
            print(f"yaw 1: {yaw1} yaw2: {yaw2}")
            rclpy.spin_once(self, timeout_sec=0.1)
            q2 = self.checkImu().orientation
            _,_,yaw2 = self.euler_from_quaternion(q2)
            yaw2 = math.degrees(yaw2)
            self.send_cmd_vel(0.0,direction * 0.5)
        #set final vel = 0
        self.send_cmd_vel(0.0,0.0)
        print("turn complete")
        
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
    
    def start_keyboard_control(self):
        if self.keyboard_listener is None:
            def on_press(key):
                try:
                    print(f"Key {key.char} pressed")
                    key_char = key.char
                except AttributeError:
                    print(f"Special key {key} pressed")
                    key_char = str(key) #---the below cluster of if statements can be removed to make level one more challenging---
                if key_char == 'w':
                    self.move_forward()
                if key_char == 's':
                    self.move_backward()
                if key_char == 'a':
                    self.turn_left()
                if key_char == 'd':
                    self.turn_right()
            def on_release(key):
                self.send_cmd_vel(0.0, 0.0)
                print("Key released and robot stopping.")
            self.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.keyboard_listener.start()
        else:
            print("Keyboard listener already running")

    def stop_keyboard_control(self):
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()
            self.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

    def on_press(self, key):
        try:
            if hasstr(key, 'char') and key.char in self.action_mape:
                self.action_map[key.char]()
        except:
            pass

    def move_forward(self):
        self.send_cmd_vel(1.0*CONST_speed_control, 0.0)
    
    def move_backward(self):
        self.send_cmd_vel(-1.0*CONST_speed_control, 0.0)
    
    def turn_left(self):
        self.send_cmd_vel(0.0, 1.0*CONST_speed_control)

    def turn_right(self):
        self.send_cmd_vel(0.0, -1.0*CONST_speed_control)

    def lidar_data_too_close(self, scan, th1, th2, min_dist):
        #returns points between angles th1, th2 that are closer tha min_dist
        if th2 < th1:
            temp = th1
            th1 = th2 
            th2 = temp 
        th1 = max(th1, scan.angle_min)
        th2 = min(th2, scan.angle_max)
        ind_start = int((th1-scan.angle_min)/scan.angle_increment)
        ind_end = int((th2-scan.angle_min)/scan.angle_increment)
        meas = scan.ranges[ind_start:ind_end]
        total = len(meas)
        meas = [m for m in meas if np.isfinite(m)]
        print(meas)
        print(len(meas))
        if len(meas) == 0:
            return 0.0
        num_too_close = 0.0
        for m in meas: 
            if m < min_dist: 
                print(f"m < min dist addition is {m}")
                num_too_close = num_too_close + 1
        print(float(num_too_close))
    
        return float(num_too_close) / total

    #read lidar data from 45 degrees to 135 degrees, if the minimum distances are less than X, return the angle and distance, otherwise return -1, -1
    def detect_obstacle(self, scan):
        # calculate indeces (this reflects reading data from 45 to 135 degrees)
        front_index = 180
        #90 for right and 270 for left
        front_right_index = front_index - 90
        front_left_index = front_index + 90

        # define maximum distance threshold for obstacles
        obstacle_dist = 0.5

        # read lidar scan and extract data in angle range of interest
        data = scan[front_right_index:front_left_index + 1]

        min_dist = min(data)
        min_dist_index = data.index(min_dist)
        min_dist_angle = (min_dist_index-90)/2

        if min_dist <= obstacle_dist:
            #print(f"dist= {min_dist}, angle= {min_dist_angle}")
            return min_dist, min_dist_angle
        return -1, -1

    def test_lidar_orientation(self):
       #---this was used to find the front heading of the robot, should not be used in solutions
        ranges = self.last_scan_msg.ranges
        num_ranges = len(ranges)
        quarter_segment = num_ranges // 4
        degrees_per_range = 360 / num_ranges

        def index_range_for_segment(start_angle, end_angle):
            start_index = int(start_angle / degrees_per_range)
            end_index = int(end_angle / degrees_per_range)
            return ranges[start_index:end_index]
        
                    #calculate the min distance in each quadrant 
        def find_smallest_unique(segment):
            sorted_segment = sorted(segment)
            unique_distances = []
            last_added = None
            for distance in sorted_segment: 
                if (last_added is None and abs(distance - last_added) > 0.15) and distance > 0.23:
                    unique_distances.append(distance)
                    last_added = distance 
                    if len(unique_distances) >=5:
                        break
            return unique_distances
        
        def analyze_segment(segment):
            sorted_segment = sorted(segment)
            unique_distances = []
            last_added = None
            total = 0

            for distance in sorted_segment: 
                if (last_added is None or abs(distance - last_added) > 0.15) and distance > 0.23:
                    unique_distances.append(distance)
                    last_added = distance 
                    if len(unique_distances) >=5:
                        break
                total += distance
            average_distance = total / len(segment)
            return unique_distances, average_distance
        front_segment = index_range_for_segment(45, 90+45)
        right_segment = index_range_for_segment(90, 180)
        back_segment = index_range_for_segment(180, 270)
        left_segment = index_range_for_segment(270, 359)
        front = analyze_segment(front_segment)
        right = analyze_segment(right_segment)
        back = analyze_segment(back_segment)
        left = analyze_segment(left_segment)


        print(f"Front (Lidar left): {front} meters") #front is 45-135
       # print(f"Right (Lidar Front): {right} meters")
        # print(f"Back (Lidar Right): {back} meters")
        #print(f"Left (Lidar Back): {left} meters")

#--functions for camera detections

    def detect_april_tag_from_img(self, img):
        """
            returns the april tag id, translation vector and rotation matrix from
            :param img: image from camera stream, np array
            :return: dict: {int tag_id: tuple (float distance, float angle)}
            """
        # convert image to grayscale
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray",img_gray)
        options = apriltag.DetectorOptions(families="tag16h5, tag25h9")
        # Apriltag detection
        detector = apriltag.Detector(options)
        detections = detector.detect(img_gray)
        # dictionary for return
        dict = {}
        # process each apriltag found in image to get the id and spacial information
        for detection in detections:
            tag_id = detection.tag_id
            translation_vector, rotation_matrix = self.homography_to_pose(detection.homography)
            dict[int(tag_id)] = (self.translation_vector_to_distance(translation_vector), self.rotation_matrix_to_angles(rotation_matrix))
        return dict

#--Helper Functions for Computer Vision - not supposed to be called outside of the class
# AprilTag
    @staticmethod
    def homography_to_pose(H):
        """
        Convert a homography matrix to rotation matrix and translation vector.
        :param H: list homography matrix
        :return: tuple (list translation_vector, list rotational_matrix)
        """
        # Perform decomposition of the homography matrix
        R, Q, P = np.linalg.svd(H)

        # Ensure rotation matrix has determinant +1
        if np.linalg.det(R) < 0:
            R = -R

        # Extract translation vector
        t = H[:, 2] / np.linalg.norm(H[:, :2], axis=1)

        return t, R

    @staticmethod
    def rotation_matrix_to_angles(R):
        """
        Convert a 3x3 rotation matrix to Euler angles (in degrees).
        Assumes the rotation matrix represents a rotation in the XYZ convention.
        :param R, rotation_matrix: list
        :return: list [float angle_x, float angle_y, float angle_z]
        """
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])

    @staticmethod
    def translation_vector_to_distance(translation_vector):
        """
        convert 3D translation vector to distance
        :param translation_vector: list
        :return: float
        """
        # Calculate the distance from the translation vector
        distance = np.linalg.norm(translation_vector)
        return distance

# stop sign
    @staticmethod
    def red_filter(img):
        """
        mask image for red only area, note that the red HSV bound values are tunable and should be adjusted base on evironment
        :param img: list RGB image array
        :return: list RGB image array of binary filtered image
        """
        # Colour Segmentation
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        # Define lower and upper bounds for red and brown hue
        lower_red_1 = np.array([-3, 100, 0])     # Lower bound for red hue (reddish)
        lower_red_2 = np.array([170, 70, 50])   # Lower bound for red hue (reddish)
        upper_red_1 = np.array([3, 255, 255])  # Upper bound for red hue (reddish)
        upper_red_2 = np.array([180, 255, 255]) # Upper bound for red hue (reddish)
        lower_brown = np.array([10, 60, 30])    # Lower bound for brown hue
        upper_brown = np.array([30, 255, 255])  # Upper bound for brown hue
        
        # Create masks for red and brown
        red_mask_1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv_img, lower_red_2, upper_red_2)
        brown_mask = cv2.inRange(hsv_img, lower_brown, upper_brown)
    
        # Combine red masks
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        # Exclude brown by subtracting its mask from the red mask
        red_mask = cv2.subtract(red_mask, brown_mask)
    
        # Apply the red mask to the original image then convert to grayscale
        red_img = cv2.bitwise_and(img, img, mask=red_mask)
        gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
        #get binary image with OTSU thresholding
        (T, threshInv) = cv2.threshold(blurred, 0, 255,
        cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        cv2.imshow("Threshold", threshInv)
        # print("[INFO] otsu's thresholding value: {}".format(T))
    
        #Morphological closing
        kernel_dim = (21,21)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_dim)
        filtered_img = cv2.morphologyEx(threshInv, cv2.MORPH_CLOSE, kernel)
        
        return filtered_img
    
    @staticmethod
    def add_contour(img):
        """
        apply contour detection to the red only masked image
        :param img: list image array
        :return: contoured img, max area and centroid(cy,cx)
        """
        max_area = 0    # stores the largest red area detected
        #edges = cv2.Canny(img, 100, 200)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    
        #loop through and get the area of all contours
        areas_of_contours = [cv2.contourArea(contour) for contour in contours]
        contoured = np.zeros((img.shape[0], img.shape[1], 3), dtype = np.uint8)

        try:
            max_poly_indx = np.argmax(areas_of_contours)
            stop_sign = contours[max_poly_indx]
            #approximate contour into a simpler shape
            epsilon = 0.01 * cv2.arcLength(stop_sign, True)
            approx_polygon = cv2.approxPolyDP(stop_sign, epsilon, True)
            area = cv2.contourArea(approx_polygon)
            max_area = max(max_area, area)
            cv2.drawContours(contoured, [approx_polygon], -1, (0, 255, 0), 3)
    
            # compute the center of the contour
            M = cv2.moments(stop_sign)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.circle(contoured, (cX, cY), 2, (255, 255, 255), -1)
        except:
            x=1
    
        return contoured, max_area ,(cX,cY)

    def rosImg_to_cv2(self):
        #returns np array for image to be used for open cv
        image = self.checkImage()
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        return img_3D
    
    def ML_predict_stop_sign(model, img):
        # height, width = image.shape[:2]
        # imgsz = (width, height)

        stop_sign_detected = False

        x1 = -1
        y1 = -1 
        x2 = -1 
        y2 = -1

        # Predict stop signs in image using model
        results = model.predict(img, classes=[11], conf=0.25, imgsz=640, max_det=1)
        
        # Results is a list containing the results object with all data
        results_obj = results[0]
        
        # Extract bounding boxes
        boxes = results_obj.boxes.xyxy

        try:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                stop_sign_detected = True
        except:
            stop_sign_detected = False

        cv2.imshow("Bounding Box", img)

        return stop_sign_detected, x1, y1, x2, y2   