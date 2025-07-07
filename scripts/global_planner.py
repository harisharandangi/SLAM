#!/usr/bin/env python3
import rospy
import math
import tf
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import heapq

class GlobalPlannerDijkstra:
    def __init__(self):
        rospy.init_node('global_planner_dijkstra')

        self.goal = None  # Set dynamically from RViz

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        self.map = None
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.resolution = None
        self.origin = None

        self.robot_pos = None

        rospy.loginfo("Global Planner started. Waiting for map, odom, and RViz goal...")

    def map_callback(self, msg):
        self.map = msg
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        rospy.loginfo_once("Map received")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.robot_pos = (pos.x, pos.y)
        self.try_plan_path()

    def goal_callback(self, msg):
        pos = msg.pose.position
        self.goal = (pos.x, pos.y)
        rospy.loginfo(f"New goal received from RViz: {self.goal}")
        self.try_plan_path()

    def try_plan_path(self):
        if self.map is not None and self.robot_pos is not None and self.goal is not None:
            path = self.plan_path()
            if path:
                self.publish_path(path)

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return (mx, my)
        return None

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0] + self.resolution / 2.0
        y = my * self.resolution + self.origin[1] + self.resolution / 2.0
        return (x, y)

    def neighbors(self, node):
        x, y = node
        candidates = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        valid = []
        for cx, cy in candidates:
            if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                if self.map_data[cy][cx] == 0:
                    valid.append((cx, cy))
        return valid

    def plan_path(self):
        start_map = self.world_to_map(*self.robot_pos)
        goal_map = self.world_to_map(*self.goal)

        if start_map is None or goal_map is None:
            rospy.logwarn("Start or goal out of map bounds")
            return None

        dist = {start_map: 0}
        prev = {}
        queue = [(0, start_map)]
        visited = set()

        while queue:
            current_dist, current_node = heapq.heappop(queue)
            if current_node in visited:
                continue
            visited.add(current_node)

            if current_node == goal_map:
                break

            for neighbor in self.neighbors(current_node):
                if neighbor in visited:
                    continue
                new_dist = current_dist + 1
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = current_node
                    heapq.heappush(queue, (new_dist, neighbor))

        if goal_map not in prev and start_map != goal_map:
            rospy.logwarn("No path found to goal")
            return None

        path = []
        node = goal_map
        while node != start_map:
            path.append(node)
            node = prev.get(node, start_map)
            if node == start_map:
                path.append(node)
                break
        path.reverse()

        path_poses = []
        for mx, my in path:
            x, y = self.map_to_world(mx, my)
            pose = PoseStamped()
            pose.header.frame_id = self.map.header.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = Point(x, y, 0)
            pose.pose.orientation.w = 1.0
            path_poses.append(pose)

        return path_poses

    def publish_path(self, poses):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map.header.frame_id
        path_msg.poses = poses
        self.path_pub.publish(path_msg)
        rospy.loginfo_throttle(5, "Global path published")

if __name__ == '__main__':
    planner = GlobalPlannerDijkstra()
    rospy.spin()

