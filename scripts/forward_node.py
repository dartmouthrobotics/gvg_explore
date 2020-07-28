#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped

class Forward(object):
    def __init__(self):
        rospy.init_node('forward_node')
        robot_name=rospy.get_param('~robot_name')
        rospy.Subscriber('/pose',PoseStamped, self.pose_foward_callback)
        self.pose_pub=rospy.Publisher('pose'.format(robot_name),PoseStamped,queue_size=1000)

    def pose_foward_callback(self, data):
        self.pose_pub.publish(data)
    def spin(self):
        rospy.spin()

if __name__=='__main__':
     Forward().spin()
