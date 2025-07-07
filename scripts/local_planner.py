#!/usr/bin/env python3
import os, sys
# make sure this script’s directory is on PYTHONPATH
sys.path.insert(0, os.path.dirname(__file__))

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# now this will work:
from kinematic_controller import KinematicController


class LocalPlannerPF:
    def __init__(self):
        rospy.init_node("local_planner_pf")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/global_path", Path, self.path_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        self.path = []
        self.robot_pose = None
        self.lookahead = 5        # number of points ahead
        self.goal_tol = 0.1

        # instantiate kinematic controller
        self.controller = KinematicController()

        rospy.loginfo("Local planner started.")

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot_pose = (pos.x, pos.y, yaw)
        self.control_loop()

    def control_loop(self):
        if not self.path or not self.robot_pose:
            return

        # find index of closest path point
        rx, ry, _ = self.robot_pose
        dists = [np.hypot(px - rx, py - ry) for px, py in self.path]
        idx = int(np.argmin(dists))
        target_idx = min(idx + self.lookahead, len(self.path) - 1)
        tx, ty = self.path[target_idx]

        # switch to final goal if close enough
        if np.hypot(self.path[-1][0] - rx, self.path[-1][1] - ry) < self.goal_tol:
            tx, ty = self.path[-1]

        # compute and publish command
        cmd = self.controller.compute(self.robot_pose, (tx, ty))
        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    LocalPlannerPF()
    rospy.spin()
