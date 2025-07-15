#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import heapq
import math

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header

class astar_node(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.get_logger().info("Astar Active.")
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.grid = None
        self.start = None
        self.goal = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.start_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width, height = msg.info.width, msg.info.height

        # Original map grid
        raw_grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # ✅ Inflate obstacles to make safer path for TurtleBot3
        self.grid = self.inflate_obstacles(raw_grid, radius=4)
        self.get_logger().info('Map received and inflated.')

    def start_callback(self, msg):
        pose = msg.pose.pose
        self.start = self.world_to_grid(pose.position.x, pose.position.y)

    def goal_callback(self, msg):
        pose = msg.pose
        self.goal = self.world_to_grid(pose.position.x, pose.position.y)
        if self.start and self.goal and self.grid is not None:
            path = self.a_star(self.start, self.goal)
            self.publish_path(path)

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (gx, gy)

    def grid_to_world(self, x, y):
        wx = x * self.resolution + self.origin[0]
        wy = y * self.resolution + self.origin[1]
        return (wx, wy)

    def is_valid(self, x, y):
        h, w = self.grid.shape
        return 0 <= x < w and 0 <= y < h and self.grid[y, x] == 0

    def a_star(self, start, goal):
        hq = []
        heapq.heappush(hq, (0 + self.heuristic(start, goal), 0, start, [start]))
        visited = set()

        while hq:
            est_total, cost, current, path = heapq.heappop(hq)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return path
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
                nx, ny = current[0] + dx, current[1] + dy
                if self.is_valid(nx, ny):
                    step_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    heapq.heappush(hq, (
                        cost + step_cost + self.heuristic((nx, ny), goal),
                        cost + step_cost,
                        (nx, ny),
                        path + [(nx, ny)]
                    ))
        return []

    def heuristic(self, a, b):
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2) * 0.5
    
    def smooth_path(self, path):
    # Basic moving average smoother
        if len(path) < 3:
            return path

        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next = path[i + 1]
            avg_x = (prev[0] + curr[0] + next[0]) / 3.0
            avg_y = (prev[1] + curr[1] + next[1]) / 3.0
            smoothed.append((avg_x, avg_y))
        smoothed.append(path[-1])
        return smoothed


    def publish_path(self, path):
        msg = Path()
        msg.header = Header()
        msg.header.frame_id = 'map'
        
        path = self.smooth_path(path)

        for (x, y) in path:
            pose = PoseStamped()
            pose.header = msg.header
            wx, wy = self.grid_to_world(x, y)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        self.get_logger().info(f"Published path with {len(path)} points.")

    def inflate_obstacles(self, grid, radius = 4):
        inflated = grid.copy()
        height, width = grid.shape
        for y in range(height):
            for x in range(width):
                if grid[y, x] == 100:
                    for dy in range(-radius, radius + 1):
                        for dx in range(-radius, radius + 1):
                            ny = y + dy
                            nx = x + dx
                            if 0 <= ny < height and 0 <= nx < width:
                                if grid[ny, nx] == 0:
                                    inflated[ny, nx] = 100
        return inflated

def main(args=None):
    rclpy.init(args=args)
    node = astar_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
