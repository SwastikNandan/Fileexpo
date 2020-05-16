import rospy
from mavros_msgs.msg import State, PositionTarget
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from image_geometry import StereoCameraModel
import matplotlib.pyplot as plt
import math
import time
from std_msgs.msg import String, Header

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
    upd_tag_pt_x=0
    upd_tag_pt_y=0
    camera=StereoCameraModel()
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    hover_loc = [2,.5,30,0,0,0,0]
    mode="HOVER"
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
	rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, callback = self.yolo1)
	rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, callback = self.yolo2)
	rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback=self.tag_detections)
	#self.lawnmover(10,5,5,0,2.5)
	self.camera.fromCameraInfo(self.camera_info_front(),self.camera_info_back())
	self.hover()
	self.descent()

    def camera_info_back(self):
	msg_header = Header()
	msg_header.frame_id= "uav/robot_camera_down"
	msg_roi = RegionOfInterest()
	msg_roi.x_offset = 0
	msg_roi.y_offset = 0
	msg_roi.height = 0
	msg_roi.width = 0
	msg_roi.do_rectify=0
	msg=CameraInfo()
	msg.header= msg_header
	msg.height = 480
	msg.width = 640
	msg.distortion_model = 'plump_bob'
	msg.D = [0.0,0.0,0.0,0.0,0.0,0.0]
	msg.K = [1.0,0.0,320.5, 0.0,1.0,240.5,0.0,0.0,1]
	msg.R = [1.0,0.0,0.0,0.0,0.866,0.5,0.0,-0.5,.866]
	msg.P = [1.0,0.0,320.5,0.0,0.0,1.0,240.5,0.0,0.0,0.0,1.0,0.0]
	msg.binning_x = 0
	msg.binning_y = 0
	msg.roi = msg_roi
	return msg

    def camera_info_front(self):
	msg_header = Header()
	msg_header.frame_id= "f450/robot_camera"
	msg_roi = RegionOfInterest()
	msg_roi.x_offset = 0
	msg_roi.y_offset = 0
	msg_roi.height = 0
	msg_roi.width = 0
	msg_roi.do_rectify=0
	msg=CameraInfo()
	msg.header= msg_header
	msg.height = 480
	msg.width = 640
	msg.distortion_model = 'plump_bob'
	msg.D = [0.0,0.0,0.0,0.0,0.0,0.0]
	msg.K = [1.0,0.0,320.5, 0.0,1.0,240.5,0.0,0.0,1]
	msg.R = [1.0,0.0,0.0,0.0,0.866,0.5,0.0,-0.5,.866]
	msg.P = [1.0,0.0,320.5,0.0,0.0,1.0,240.5,0.0,0.0,0.0,1.0,0.0]
	msg.binning_x = 0
	msg.binning_y = 0
	msg.roi = msg_roi
	return msg
	
    def yolo1(self, data):
        for i in data.bounding_boxes:
    	    if i.Class == "Box":
                self.cam1target = self.camera_front.projectPixelTo3dRay((i.xmin + (i.xmax - i.xmin) /2, i.ymin + (i.ymax - i.ymin)/2))#centre of bounding box
    
    def yolo2(self, data):
        for i in data.bounding_boxes:
    	    if i.Class == "Box":
                self.cam2target = self.camera_front.projectPixelTo3dRay((i.xmin + (i.xmax - i.xmin) /2, i.ymin + (i.ymax - i.ymin)/2))#centre of bounding box

    def depthest():
    	x_cam1 =
        y_cam1 =
        cam_cod1 = np.array([x_cam1, y_cam1,1]).T
	x_cam2 =
	y_cam2 = 
	cam_cod2 = np.array([x_cam2, y_cam2,1]).T
        K_front = np.array([1.0,0.0,320.5],[0.0,1.0,240.5],[0.0,0.0,1])
	K_back = np.array([1.0,0.0,320.5],[0.0,1.0,240.5],[0.0,0.0,1])
	K_front_inv = np.linalg.inv(K_front)
	K_back_inv = np.linalg.inv(K_back)
	rotation = np.array([0.9238 0 0.3826],[0 1 0],[-0.3826 0 0.9238])
	rel_pose = np.array([0.24,0,0.05]).T
        Hom_trans = np.array([rotation, rel_pose],[0,0,0,1])
	real_cod1 = K_front_inv*cam_cod1
	real_cod2 = K_back_inv*cam_cod2
	real_cod2_hom = np.array([real_cod2],[1])
	real_cod2_trans= Hom_trans*real_cod2_hom

        
          
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
	    gamma = 1/(self.detection_count)
	    self.tag_pt_x= self.detections[0].pose.pose.position.x
	    self.tag_pt_y= self.detections[0].pose.pose.position.y
	    if self.detection_count==1:
	        self.prev_tag_pt_x = self.tag_pt_x
                self.prev_tag_pt_y = self.tag_pt_y
	    if self.detection_count > 1:
	    	self.upd_tag_pt_x = (self.prev_tag_pt_x*(1-gamma)) + (self.tag_pt_x*gamma)          # recursively calling previous detection and relying more on previous detections
	    	self.prev_tag_pt_x = self.upd_tag_pt_x
	    	self.upd_tag_pt_y = (self.prev_tag_pt_y*(1-gamma)) + (self.tag_pt_y*gamma)
	    	self.prev_tag_pt_y = self.upd_tag_pt_y

    def update_state_cb(self,data):
        self.mode= data.data
        #print self.mode

    def lawnmover(self, rect_x, rect_y, height, offset, offset_x):
        print("I am in lawnmover")
        if len(self.loc)== 0 :
            takeoff = [self.curr_pose.pose.position.x, self.curr_pose.pose.position.y, height ,  0 , 0 , 0 , 0]
            move_to_offset = [self.curr_pose.pose.position.x + offset, self.curr_pose.pose.position.y - rect_y/2, height, 0 , 0 , 0 ,0 ]
        self.loc.append(takeoff)
        self.loc.append(move_to_offset)
	sim_ctr=1
        left = True
        
        while True:
            if left:
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1] + rect_y/3
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1] + rect_y/3
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1] + rect_y/3
                z = self.loc[len(self.loc)-1][2]
                left = False
                x = self.loc[len(self.loc)-1][0] + offset_x
                y = self.loc[len(self.loc)-1][0]
                z = self.loc[len(self.loc)-1][0]
                self.loc.append([x,y,z,0,0,0,0])
                if x > rect_x :
                    break
            else:
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1]-rect_y/3
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1]-rect_y/3
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                x = self.loc[len(self.loc)-1][0]
                y = self.loc[len(self.loc)-1][1]-rect_y/3
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                left = True
                x = self.loc[len(self.loc)-1][0]+ offset_x
                y = self.loc[len(self.loc)-1][1]
                z = self.loc[len(self.loc)-1][2]
                self.loc.append([x,y,z,0,0,0,0])
                if x > rect_x:
                    break
                    
            rate = rospy.Rate(10)
            #print(self.loc)
            self.des_pose = self.copy_pose(self.curr_pose)                        
            shape = len(self.loc)
            pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
            print self.mode
            
            while self.mode == "SURVEY" and sim_ctr< 5 and not rospy.is_shutdown():
		sim_ctr=sim_ctr+1
                if self.waypointIndex == shape :
                    self.waypointIndex = 1                  # resetting the waypoint index
                
                if self.isReadyToFly:
                    #print(self.isReadyToFly)
                    #print("I am in controller")
                    des_x = self.loc[self.waypointIndex][0]
                    des_y = self.loc[self.waypointIndex][1]
                    des_z = self.loc[self.waypointIndex][2]
                    self.des_pose.pose.position.x = des_x
                    self.des_pose.pose.position.y = des_y
                    self.des_pose.pose.position.z = des_z
                    self.des_pose.pose.orientation.x = self.loc[self.waypointIndex][3]
                    self.des_pose.pose.orientation.y = self.loc[self.waypointIndex][4]
                    self.des_pose.pose.orientation.z = self.loc[self.waypointIndex][5]
                    self.des_pose.pose.orientation.w = self.loc[self.waypointIndex][6]
                    curr_x = self.curr_pose.pose.position.x
                    curr_y = self.curr_pose.pose.position.y
                    curr_z = self.curr_pose.pose.position.z
 		    f= (curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y ) + (curr_z - des_z)*(curr_z - des_z)
                    dist = math.sqrt(f)
                    #print(self.curr_pose)
                    if dist < self.distThreshold:
                        self.waypointIndex += 1
                
                pose_pub.publish(self.des_pose)
                rate.sleep()
	self.mode="HOVER"

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
	while(self.detection_count != 20):
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
        rate = rospy.Rate(20) # 20 Hz
        time_step = 1/20
        #print self.mode
        self.x_change = 1
        self.y_change = 1
        self.x_prev_error = 0
        self.y_prev_error = 0
        self.x_sum_error=0
        self.y_sum_error=0
        self.curr_xerror=0
        self.curr_yerror=0
	tag_fixed_x =self.tag_pt_x
        tag_fixed_y = self.tag_pt_y
	count = 0
	count_arr =[]
        err_arr_x = []
	err_arr_y = []
        
        while self.mode == "SWOOP" and self.curr_pose.pose.position.z > 0.1 and not rospy.is_shutdown():
	    count=count+1
            count_arr = count_arr + [count]
            print("I am in Descent's while loop")
	    #print(self.upd_tag_pt_x)
	    #print(tag_fixed_y)
	    #print(self.curr_pose.pose.position.x)
	    #print(self.curr_pose.pose.position.y)
	    #print(self.curr_pose.pose.position.y)
            err_x = self.upd_tag_pt_x - self.curr_pose.pose.position.x
            err_y = self.upd_tag_pt_y - self.curr_pose.pose.position.y 
            err_arr_x = err_arr_x + [err_x]
            err_arr_y = err_arr_y + [err_y]
	    #print(err_x)
	    #print(err_y)
            x_change = (((err_x*self.KP*48))+(((err_x - self.x_prev_error)*20)* self.KD*18)) #+ (self.x_sum_error * self.KI*1.2))
            y_change = (((err_y*self.KP*48))+(((err_y - self.y_prev_error)*20)* self.KD*18)) #+ (self.y_sum_error * self.KI*1.2))
	    #print(x_change)
            #print(y_change)
            #x_change =  max(min(0.4,self.x_change), -0.2)
            #y_change =  max(min(0.4,self.y_change), -0.4)
            des = self.get_descent(x_change,y_change, -0.2)
            #if err_x > 0 and err_y < 0:
            #    des = self.get_descent(-1*abs(x_change), 1*abs(y_change), -0.8)
            #elif err_x > 0 and err_y > 0:
            #    des = self.get_descent(-1*abs(self.x_change), -1*abs(self.y_change), -0.8)
            #elif err_x < 0 and err_y > 0:
            #    des = self.get_descent(1*abs(self.x_change), -1*abs(self.y_change), -0.8)
            #elif err_x < 0 and err_y < 0:
            #    des = self.get_descent(1*abs(self.x_change), 1*abs(self.y_change), -0.8)
            #else:
            #    des = self.get_descent(self.x_change, self.y_change, -0.08)
            self.vel_pub.publish(des)
            self.x_prev_error= err_x
            self.y_prev_error= err_y
            #self.x_sum_error += err_x
            #self.y_sum_error += err_y
	    
                
            #if self.curr_pose.pose.position.z < 0.2:
            #Perform the necessary action to complete pickup instruct actuators
            self.pub.publish("PICKUP COMPLETE")   
            rate.sleep()
	plt.plot(count_arr,err_arr_x)
	plt.plot(count_arr,err_arr_y)
	plt.show()

if __name__=='__main__':
    OffbPosCtl()