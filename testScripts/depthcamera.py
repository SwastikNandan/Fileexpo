from __future__ import division
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

class test:
    hover_loc = [-1,.5,10,0,0,0,0]
    curr_pose = PoseStamped()
    depth = Image()
    def __init__(self):
	print("I am in init")
	rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
	rospy.Subscriber('/camera/depth/image_raw',Image,callback=self.depth_image)
	self.hover()
        
    def depth_image(self,Img):
	self.depth = Img
	print('Extracting depth image')
	print(self.depth)
	#print(np.shape(Img))

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
	
        while self.mode=="HOVER" and not rospy.is_shutdown():
	    
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

if __name__=='__main__':
    test()


