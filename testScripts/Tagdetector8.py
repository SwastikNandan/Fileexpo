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

class OffbPosCtl:
    curr_pose = PoseStamped()
    waypointIndex = 0
    detections = []
    tag_pt_x = 0
    tag_pt_y = 0
    distThreshold= 2
    detection_count=0
    KP=0.005
    KD=0.0004
    KI=0.00005
    prev_tag_pt_x=0
    prev_tag_pt_y=0
    upd_tag_pt_x=0
    upd_tag_pt_y=0
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
    hover_loc = [-3,1.91,10,0,0,0,0]
    mode="HOVER"
    flag_x = "allow_x"
    flag_y = "allow_y"
    target = []
    count = 1
    
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
        rospy.Subscriber('/stereo/left/image_raw',Image, callback= self.leftimg)
        rospy.Subscriber('/stereo/right/image_raw', Image, callback=self.rightimg)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback=self.yolo)
        self.camera.fromCameraInfo(self.camera_info_left())
	#self.lawnmover(10,5,5,0,2.5)
	#self.depth_estm(self.left_matrix,self.right_matrix)
	self.hover()
	while (self.count<100):
	   if self.count % 10 == 0:
	       self.depth_estm(self.cv_image_left,self.cv_image_right)

    def leftimg(self,left_pic):

	self.count = self.count+1
        try:
	   
    	    self.cv_image_left = CvBridge().imgmsg_to_cv2(left_pic,"mono8")
    	    
	except CvBridgeError as e:
	    print(e)
	#cv2.imshow("Image window",self.cv_image_left)
	#cv2.waitKey(3)
    def rightimg(self,right_pic):
        try:
    	    self.cv_image_right = CvBridge().imgmsg_to_cv2(right_pic,"mono8")
	    
	    #self.cv_image_right = cv2.cvtColor(np.array(cv_image_right_i), cv2.COLOR_BGR2GRAY)
	    
	except CvBridgeError as e:
	    print(e)
	#cv2.imshow("Image window",self.cv_image_right)
	#cv2.waitKey(3)
	#print(self.right_image.data)

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

    def depth_estm(self,li,ri,):
	rate = rospy.Rate(1)
	#li1 = np.array(li)
	#ri1 = np.array(ri)
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)
        disparity = stereo.compute(np.array(li),np.array(ri))
        min_val = disparity.min()
        max_val = disparity.max()
        disparity = np.uint8(6400*(disparity - min_val)/ (max_val - min_val))
	#print(disparity)
        #cv2.imshow('disparittet',np.hstack((li, ri, disparity)))
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
	if self.detection_count > 10:
	    f = 1 #focal length of the camera
	    b = 0.07    # baseline is 0.07
	    D = f*b*inv(disparity)
	    K = np.array([[1.0, 0.0, 320.5],[0.0, 1.0, 240.5],[0.0, 0.0, 1.0]])
	    M = D * inv(K)*(self.target)
	    print(M)
	rate.sleep()

    def yolo(self,data):
	for a in data.bounding_boxes:
	    if a.Class == "truck" or a.Class == "bus":
		self.detection_count = self.detection_count + 1
		self.target = [a.xmin +(a.xmax - a.xmin) / 2; a.ymin + (a.ymax - a.ymin) / 2;1]
		#print(self.target)
        
          
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

		    

if __name__=='__main__':
    OffbPosCtl()
