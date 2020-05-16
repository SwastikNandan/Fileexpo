import rospy
import cv2
import numpy as np
from numpy import asarray
from mavros_msgs.msg import State, PositionTarget
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from image_geometry import StereoCameraModel
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import math
import time
from std_msgs.msg import String, Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class OffbPosCtl:
    curr_pose = PoseStamped()
    waypointIndex = 0
    detections = []
    tag_pt_x = 0
    tag_pt_y = 0
    distThreshold= 2
    x_sum_error= 0
    y_sum_error= 0
    x_prev_error = 0
    y_prev_error = 0
    x_change = 1
    y_change = 1
    curr_xerror=0
    curr_yerror=0
    detection_count=0
    KP=0.005
    KD=0.0004
    KI=0.00005
    prev_tag_pt_x=0
    prev_tag_pt_y=0
    updated_x=0
    updated_y=0
    left_image = Image()
    left_height = 0
    left_width = 0
    right_image = Image()
    right_height = 0
    right_width = 0
    camera=StereoCameraModel()
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    left_matrix = []
    right_matrix = []
    cv_image_left = []
    cv_image_right = []
    hover_loc = [-3,1.91,10,0,0,0,0]
    mode="HOVER"
    flag_x = "allow_x"
    flag_y = "allow_y"
    
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
	self.bridge = CvBridge()
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
	rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback=self.tag_detections)
        #rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback=self.yolo)
        self.camera.fromCameraInfo(self.camera_info_left(), self.camera_info_right())
	#self.lawnmover(10,5,5,0,2.5)
	#self.depth_estm(self.left_matrix,self.right_matrix)
	self.hover()
	self.descent()

    def camera_info_left(self):
        msg_header = Header()
        msg_header.frame_id = "camera_link"
        msg_roi = RegionOfInterest()
        msg_roi.x_offset = 0
        msg_roi.y_offset = 0
        msg_roi.height = 0
        msg_roi.width = 0
        msg_roi.do_rectify = 0
        msg = CameraInfo()
        msg.header = msg_header
        msg.height = 480
        msg.width = 640
        msg.distortion_model = 'plumb_bob'
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.K = [1.0, 0.0, 320.5, 0.0, 1.0, 240.5, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [1.0, 0.0, 320.5, -0.0, 0.0, 1.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi = msg_roi
        return msg

    def camera_info_right(self):
        msg_header = Header()
        msg_header.frame_id = "camera_link"
        msg_roi = RegionOfInterest()
        msg_roi.x_offset = 0
        msg_roi.y_offset = 0
        msg_roi.height = 0
        msg_roi.width = 0
        msg_roi.do_rectify = 0
        msg = CameraInfo()
        msg.header = msg_header
        msg.height = 480
        msg.width = 640
        msg.distortion_model = 'plumb_bob'
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.K = [1.0, 0.0, 320.5, 0.0, 1.0, 240.5, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [1.0, 0.0, 320.5, -0.0, 0.0, 1.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi = msg_roi
        return msg

    def copy_pose(self , pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x , quat.y , quat.z , quat.w)
        return copied_pose

    def mocap_cb(self,msg1):
        self.curr_pose = msg1
        #print msg1
    
    def state_cb(self,msg):
        if msg.mode == 'OFFBOARD':
            #print msg
            #print("The mode is OFFBOARD")
            self.isReadyToFly = True
        else:
            #print("I am in state_cb")
            #print msg
            print msg.mode
    def tag_detections(self,msgs):
        rate = rospy.Rate(10)
	if len(msgs.detections)>0:
	    self.detection_count += 1
	    self.detections = msgs.detections
	    self.tag_pt_x = self.detections[0].pose.pose.position.x 
	    self.tag_pt_y = self.detections[0].pose.pose.position.y 
	

    def update_state_cb(self,data):
        self.mode= data.data
        #print self.mode

    def hover(self):
        location = self.hover_loc
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]
        #print loc       
        rate = rospy.Rate(10)
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size= 10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        #print("I am here")
	
        while self.mode=="HOVER" and self.detection_count < 5 and not rospy.is_shutdown():
	    print(len(self.detections))
            if waypoint_index==shape:
                waypoint_index = 0            # changing the way point index to 0
                sim_ctr = sim_ctr + 1
                print "HOVER STOP COUNTER:" + str(sim_ctr)
            if self.isReadyToFly:
                des_x = loc[waypoint_index][0]
                des_y = loc[waypoint_index][1]
                des_z = loc[waypoint_index][2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = loc[waypoint_index][3]
                des_pose.pose.orientation.y = loc[waypoint_index][4]
                des_pose.pose.orientation.z = loc[waypoint_index][5]
                des_pose.pose.orientation.w = loc[waypoint_index][6]
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                #print('I am here')
                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
                if dist<self.distThreshold :
                    waypoint_index += 1
                  
            if sim_ctr == 50:
                pass
            pose_pub.publish(des_pose)
            #print(des_pose)    
	    rate.sleep()
	while(self.detection_count >= 5):
            self.mode="SWOOP" 
            break

    def get_descent(self,x,y,z):
	des_vel = PositionTarget()
	des_vel.header.frame_id = "world"
	des_vel.header.stamp=rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame= 8
	des_vel.type_mask = 3527
	des_vel.velocity.x = x
	des_vel.velocity.y = y
	des_vel.velocity.z = z
	return des_vel

    def descent(self):
        
        des_pose = self.copy_pose(self.curr_pose)
	pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size= 10)
        #print self.mode
        count = 0
        while self.mode == "SWOOP" and self.curr_pose.pose.position.z > 0.1 and not rospy.is_shutdown():
            rate = rospy.Rate(20)
	    count_step = self.detection_count + 30
	    count = count+1
	    self.updated_x = self.tag_pt_x
	    self.updated_y = self.tag_pt_y
	    while self.detection_count < count_step:
		self.gamma = 1/self.detection_count
	        self.updated_x = (self.updated_x*self.gamma) + (self.tag_pt_x*(1-self.gamma))
 	        self.updated_y = (self.updated_y*self.gamma) + (self.tag_pt_y*(1-self.gamma))
            print(count)
            print(self.updated_x)
	    print(self.updated_y)
	    next = True
	    if next:
                des_pose.pose.position.x = -(self.updated_x/12) + self.curr_pose.pose.position.x
                des_pose.pose.position.y = -(self.updated_y/12) + self.curr_pose.pose.position.y 
                des_pose.pose.position.z = self.curr_pose.pose.position.z * 0.95
	        des_x = des_pose.pose.position.x 
	        des_y = des_pose.pose.position.y
	        des_z = des_pose.pose.position.z
	        des_pose.pose.orientation.x = 0
	        des_pose.pose.orientation.y = 0
	        des_pose.pose.orientation.z = 0
	        des_pose.pose.orientation.w = 0
	        curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
	        if dist<self.distThreshold :
                    next = False

                pose_pub.publish(des_pose)
            self.pub.publish("PICKUP COMPLETE")   
            rate.sleep()


if __name__=='__main__':
    OffbPosCtl()
