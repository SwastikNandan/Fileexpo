import roslib
#roslib.load_manifest('turtlebot_teleop')
import rospy
import time
from mavros_msgs.msg import PositionTarget
#from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   i moves drone in +ve x direction
   j moves drone in +ve y direction
   l moves drone in -ve x direction
   m moves drone in -ve x direction 
   u moves drone in the +ve z direction
   d moves drone in the -ve z direction
"""

moveBindings = {
        'i':(0.2,0,0),
        'j':(0,0.2,0),
        'm':(-0.2,0,0),
        'l':(0,-0.2,0),
        'u':(0,0,0.2),
        'd':(0,0,-0.2)
           }


def displacement(x_disp,y_disp,z_disp):
    print("I am in displacement")
    des_disp = PositionTarget()
    des_disp.header.frame_id = "world"
    des_disp.header.stamp = rospy.Time.from_sec(time.time())
    des_disp.coordinate_frame = 8
    des_disp.type_mask = 3527
    des_disp.velocity.x = x_disp
    des_disp.velocity.y = y_disp
    des_disp.velocity.z = z_disp
    return des_disp
    
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 10)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop_node')
    pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    #des = displacement(x_change, y_change, z_change)
    #pub.publish(des)
    x_change = 0
    y_change = 0
    z_change = 0
    try:
        print msg

        while(1):
	    #print('I am here')
            key = getKey()
            if key in moveBindings.keys():
		print('I am here fetching key inputs')
                x_change = moveBindings[key][0]
                y_change = moveBindings[key][1]
                x_change = moveBindings[key][2]
		y_change = moveBindings[key][3]
		z_change = moveBindings[key][4]
		z_change = moveBindings[key][5]
                count = 0

            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    z = 0
                if (key == '\x03'):
                    break


            des = displacement(x_change, y_change, z_change)
            pub.publish(des)

    except:
        print('error')

    finally:
        des = displacement(0,0,0)
        pub.publish(des)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
