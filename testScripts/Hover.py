import rospy
import cv2
import numpy as np
from numpy import asarray
from mavros_msgs.msg import State, PositionTarget
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import math
import time
from std_msgs.msg import String, Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class OffbPosCtl:
    target =[]
    curr_pose = PoseStamped()
    waypointIndex = 0
    detections = []
    tag_pt_x = 0
    tag_pt_y = 0
    distThreshold= .05
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
    camera=PinholeCameraModel()
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    left_matrix = []
    right_matrix = []
    cv_image_left = []
    cv_image_right = []
    hover_loc = [-5,-5,.5,0,0,0,0]
    hover_loc1 = [-5,-5,5,0,0,0,0]
    mode="HOVER"
    flag_x = "allow_x"
    flag_y = "allow_y"
    #attach_pub = rospy.Publisher('/link_attacher_node/attach_models', Attach, queue_size=1)
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
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback=self.yolo)
        self.camera.fromCameraInfo(self.camera_info_right())
	#self.lawnmover(10,5,5,0,2.5)
	#self.depth_estm(self.left_matrix,self.right_matrix)
	#rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
	#attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)

	#attach_srv.wait_for_service()
	#rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
	self.hover()

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

    def yolo(self,data):
	for a in data.bounding_boxes:
	    if a.Class == "truck" or a.Class == "bus":
		self.target = self.camera.projectPixelTo3dRay((a.xmin +(a.xmax - a.xmin) / 2, a.ymin + (a.ymax - a.ymin) / 2))
		print(self.target)

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

    def attach(self):
	print('Running attach_srv')
	attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	attach_srv.wait_for_service()
	print('Trying to attach')

	req = AttachRequest()
	req.model_name_1 = "iris"
	req.link_name_1 = "base_link"
	req.model_name_2 = "dummy_probe"
	req.link_name_2 = "link"

	attach_srv.call(req)

    def detach(self):
	attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
	attach_srv.wait_for_service()
	
	req = AttachRequest()
	req.model_name_1 = "iris"
	req.link_name_1 = "base_link"
	req.model_name_2 = "dummy_probe"
	req.link_name_2 = "link"

	attach_srv.call(req)

    def hover(self):
        location = self.hover_loc
	location1 = self.hover_loc1
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]
	loc1 = [location1,
		location1,
	        location1,
	        location1,
	        location1,
	        location1,
		location1,
                location1,
		location1]
        #print loc       
        rate = rospy.Rate(10)
        shape = len(loc)
	shape1 =len(loc1)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size= 10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        #print("I am here")
	
        while self.mode=="HOVER" and not rospy.is_shutdown():
	    #print(len(self.detections))
            if waypoint_index==shape:
                waypoint_index = 0            # changing the way point index to 0
                sim_ctr = sim_ctr + 1
                #print "HOVER STOP COUNTER:" + str(sim_ctr)
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
		    #print('Hit the if statement')
                    waypoint_index += 1
		    self.attach()
		    loc = loc1
            if sim_ctr == 50:
                pass
            pose_pub.publish(des_pose)
            #print(des_pose)
	    rate.sleep()



if __name__=='__main__':
    OffbPosCtl()
