import rospy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
import math
import time
from std_msgs.msg import String, Header

class OffbPosCtl:
    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold= 2
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    hover_loc = [4,0,10,0,0,0,0]
    mode="SURVEY"
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
	
	self.lawnmover(10,10,10,2.5,2.5)
          
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
        print msg1
    
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
        print self.mode

    def lawnmover(self, rect_x, rect_y, height, offset, offset_x):
        print("I am in lawnmover")
        if len(self.loc)== 0 :
            takeoff = [self.curr_pose.pose.position.x, self.curr_pose.pose.position.y, height ,  0 , 0 , 0 , 0]
            move_to_offset = [self.curr_pose.pose.position.x + offset, self.curr_pose.pose.position.y - rect_y/2, height, 0 , 0 , 0 ,0 ]
        self.loc.append(takeoff)
        self.loc.append(move_to_offset)
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
        
            while self.mode == "SURVEY" and not rospy.is_shutdown():
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
if __name__=='__main__':
    OffbPosCtl()
