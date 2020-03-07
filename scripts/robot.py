#!/usr/bin/python

import time

import rospy
from std_msgs.msg import *
from std_srvs.srv import *
from nav2d_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import math
from gvgexploration.msg import *
from gvgexploration.srv import *
import numpy as np
from time import sleep
import tf
import project_utils as pu
from actionlib import SimpleActionClient

MAX_COVERAGE_RATIO = 0.8

INF = 1000000000000
NEG_INF = -1000000000000
MAX_ATTEMPTS = 2
RR_TYPE = 2
TURNING_ANGLE = np.deg2rad(45)


class Robot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.is_exploring = False

        self.fetch_frontier_points = rospy.ServiceProxy('/robot_{}/frontier_points'.format(self.robot_id),
                                                        FrontierPoint)
        self.check_intersections = rospy.ServiceProxy('/robot_{}/check_intersections'.format(self.robot_id),
                                                      Intersections)
        rospy.Subscriber('/robot_{}/gvg_explore/feedback'.format(self.robot_id), GvgExploreActionFeedback,
                         self.explore_feedback_callback)
        rospy.Subscriber('/robot_{}/map'.format(self.robot_id), OccupancyGrid, self.map_update_callback)

        # ======= pose transformations====================
        self.listener = tf.TransformListener()
        self.exploration = SimpleActionClient("/robot_{}/gvg_explore".format(self.robot_id), GvgExploreAction)
        self.exploration.wait_for_server()
        rospy.loginfo("Robot {} Initialized successfully!!".format(self.robot_id))

    def spin(self):
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            if self.is_exploring:
                self.check_data_sharing_status()
            r.sleep()

    def map_update_callback(self, data):
        self.request_and_share_frontiers()

    def check_data_sharing_status(self):
        robot_pose = self.get_robot_pose()
        p = Pose()
        p.position.x = robot_pose[pu.INDEX_FOR_X]
        p.position.y = robot_pose[pu.INDEX_FOR_Y]
        response = self.check_intersections(IntersectionsRequest(pose=p))
        if response.result:
            rospy.logerr("Robot {}: Sending data to available devices...".format(self.robot_id))

    def explore_feedback_callback(self, data):
        self.is_exploring = True

    def request_and_share_frontiers(self):
        frontier_point_response = self.fetch_frontier_points(FrontierPointRequest(count=4 + 1))
        frontier_points = self.parse_frontier_response(frontier_point_response)
        robot_pose = self.get_robot_pose()
        self.frontier_point = None
        dist_dict = {}
        for point in frontier_points:
            dist_dict[pu.D(robot_pose, point)] = point
        frontier_point = dist_dict[min(list(dist_dict))]
        pose = Pose()
        pose.position.x = frontier_point[pu.INDEX_FOR_X]
        pose.position.y = frontier_point[pu.INDEX_FOR_Y]
        self.start_exploration_action(pose)

    def start_exploration_action(self, pose):
        goal = GvgExploreGoal(pose=pose)
        self.exploration.wait_for_server()
        self.goal_handle = self.exploration.send_goal(goal)
        self.exploration.wait_for_result()
        self.exploration.get_result()

    def parse_frontier_response(self, data):
        frontier_points = []
        received_poses = data.poses
        if received_poses:
            for p in received_poses:
                yaw = self.get_elevation((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                frontier_points.append((p.position.x, p.position.y, yaw))
        return frontier_points

    def get_robot_pose(self):
        robot_pose = None
        while not robot_pose:
            try:
                self.listener.waitForTransform("robot_{}/map".format(self.robot_id),
                                               "robot_{}/base_link".format(self.robot_id), rospy.Time(),
                                               rospy.Duration(4.0))
                (robot_loc_val, rot) = self.listener.lookupTransform("robot_{}/map".format(self.robot_id),
                                                                     "robot_{}/base_link".format(self.robot_id),
                                                                     rospy.Time(0))
                robot_pose = (math.floor(robot_loc_val[0]), math.floor(robot_loc_val[1]), robot_loc_val[2])
                sleep(1)
            except:
                pass
        return robot_pose

    def cancel_exploration(self):
        self.is_exploring = False
        if self.goal_handle:
            self.goal_handle.cancel()
            rospy.logerr("Robot {} Canceling exploration ...".format(self.robot_id))

    def get_elevation(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw


if __name__ == "__main__":
    # initializing a node
    rospy.init_node("robot_node", anonymous=True)
    robot_id = 0
    robot = Robot(robot_id)
    robot.spin()
