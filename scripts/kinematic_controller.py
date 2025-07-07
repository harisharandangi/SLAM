# kinematic_controller.py
#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist

class KinematicController:
    def __init__(self, k_rho=0.5, k_alpha=2.0, max_lin=0.3, max_ang=1.0, ang_thresh=0.3):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.ang_thresh = ang_thresh

    def normalize(self, angle):
        """Wrap angle to [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute(self, robot_pose, target):
        """
        robot_pose: (x, y, yaw)
        target: (tx, ty)
        returns: geometry_msgs/Twist command
        """
        rx, ry, ryaw = robot_pose
        tx, ty = target

        # distance and heading error
        dx, dy = tx - rx, ty - ry
        rho = np.hypot(dx, dy)
        angle_to_target = np.arctan2(dy, dx)
        alpha = self.normalize(angle_to_target - ryaw)

        # linear and angular velocities
        v = min(self.max_lin, self.k_rho * rho)
        omega = max(-self.max_ang, min(self.max_ang, self.k_alpha * alpha))

        # build command
        cmd = Twist()
        if abs(alpha) > self.ang_thresh:
            cmd.angular.z = omega
        else:
            cmd.linear.x = v
            cmd.angular.z = omega
        return cmd
