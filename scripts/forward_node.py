#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, BatteryState


class Forward(object):
    def __init__(self):
        rospy.init_node('forward_node')
        rospy.Subscriber('/pose', PoseStamped, self.pose_forward_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_forward_callback)
        rospy.Subscriber('/reset_odom', Bool, self.reset_odom_forward_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_forward_callback)
        rospy.Subscriber('/battery', BatteryState, self.battery_forward_callback)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1000)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.reset_odom_pub = rospy.Publisher('reset_odom', Twist, queue_size=1000)
        self.jointstate_pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
        self.battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1000)

    def pose_forward_callback(self, data):
        self.pose_pub.publish(data)

    def cmd_vel_forward_callback(self, data):
        self.cmd_vel_pub.publish(data)

    def reset_odom_forward_callback(self, data):
        self.reset_odom_pub.publish(data)

    def joint_state_forward_callback(self, data):
        self.jointstate_pub.publish(data)

    def battery_forward_callback(self, data):
        self.battery_pub.publish(data)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    Forward().spin()
