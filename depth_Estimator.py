import rospy
import cv2
from mavros_msgs.msg import State, PositionTarget
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest, image_encodings
import math
from image_geometry import PinholeCameraModel
import time
from std_msgs.msg import String, Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class OffbPosCtl:
    curr_pose = PoseStamped()
    camera = StereoCameraModel()
    left_image = Image()
    left_height = 0
    left_width = 0
    right_image = Image()
    right_height = 0
    right_width = 0
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)
    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
        rospy.Subscriber('/mavros/state', State, callback=self.state_cb)
        rospy.Subscriber('/dreams/state', String, callback=self.update_state_cb)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback=self.tag_detections)
        rospy.Subscriber('/stereo/left/image_raw',Image, callback= self.leftimg)
        rospy.Subscriber('/stereo/right/image_raw', Image, callback=self.rightimg)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback=self.yolo)
        rospy.Subscriber('/data', String, callback=self.planner)
        self.camera.fromCameraInfo(self.camera_info_left(),self.camera_info_right)
        self.controller()
        
     def leftimg(left_pic):
        self.left_image = left_pic
        self.left_height = self.left_image.height
        self.left_width = self.left_image.width
        self.left_matrix= self.left_image.data
    def rightimg(right_pic):
        self.right_image = right_pic
        self.right_height = self.right_image.height
        self.right_width = self.right_image.width
        self.right_matrix=self.right_image.data
     def camera_info_left(self):
        msg_header = Header()
        msg_header.frame_id = "uav_camera_down"
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
        msg_header.frame_id = "uav_camera_down"
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
        def depth_estm(li,ri)
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)
            disparity = stereo.compute(li,ri)
            min_val = disparity.min()
            max_val = disparity.max()
            disparity = np.unit8(6400*(disparity - min_val)/ (max_val - min_val))
            cv2.imshow('disparittet',np.hstack((imgLeft, imgRight, disparity)))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
