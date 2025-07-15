#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import transforms3d

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        
        # Parameters for DWA
        self.max_speed = 0.5  # Maximum speed m/s
        self.min_speed = 0.0  # Minimum speed m/s
        self.max_yawrate = 1.0  # Maximum yaw rate rad/s
        self.max_accel = 0.2  # Maximum acceleration m/s^2
        self.max_dyawrate = math.pi/4  # Maximum angular acceleration rad/s^2
        self.v_resolution = 0.01  # Velocity resolution m/s
        self.yawrate_resolution = 0.1  # Yaw rate resolution rad/s
        self.dt = 0.1  # Time step for simulation
        self.predict_time = 3.0  # Time to predict ahead [s]
        
        # Robot configuration
        self.robot_radius = 0.2  # Robot radius in meters
        
        # Store map and current state
        self.map_data = None
        self.current_pose = None
        self.global_path = None
        self.laser_data = None
        self.resolution = None
        self.origin = None
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for TF to be available
        self.get_logger().info('Waiting for TF tree to become available...')
        self._wait_for_tf_tree()
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        
        # Publisher
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('DWA Planner initialized')

    def _wait_for_tf_tree(self):
        """Wait for the TF tree to become available"""
        rate = self.create_rate(1)  # 1Hz
        max_attempts = 30  # 30 seconds max wait time
        attempt = 0
        
        self.get_logger().info('Waiting for transform tree to become available...')
        
        while rclpy.ok() and attempt < max_attempts:
            try:
                # Try to get transforms - if it works, we're good
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
                self.get_logger().info('Transform tree is available!')
                return True
            except Exception as e:
                pass
            
            self.get_logger().info(f'Waiting for transforms... ({attempt+1}/{max_attempts})')
            attempt += 1
            try:
                rate.sleep()
            except:
                pass
        
        self.get_logger().error('Could not find transform tree after maximum attempts')
        return False

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def laser_callback(self, msg):
        self.laser_data = msg
        if self.map_data is not None:
            self.process_laser_data()

    def path_callback(self, msg):
        self.global_path = msg

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (gx, gy)

    def is_unmapped_obstacle(self, x, y):
        """Check if an obstacle at (x,y) is unmapped"""
        if self.map_data is None:
            return False
            
        grid_x, grid_y = self.world_to_grid(x, y)
        
        # Check if coordinates are within map bounds
        height, width = self.map_array.shape
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Return True if map shows free space (0) but laser detects obstacle
            return self.map_array[grid_y, grid_x] == 0
        return False

    def process_laser_data(self):
        """Process laser scan data to identify unmapped obstacles"""
        if self.laser_data is None:
            return

        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
                
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, robot_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
            
        except Exception as e:
            self.get_logger().debug('Could not get robot transform')
            return

        unmapped_obstacles = []
        angles = np.arange(
            self.laser_data.angle_min,
            self.laser_data.angle_max + self.laser_data.angle_increment,
            self.laser_data.angle_increment
        )

        for angle, range_value in zip(angles, self.laser_data.ranges):
            if range_value < self.laser_data.range_max:
                # Convert laser reading to world coordinates
                obstacle_x = robot_x + range_value * math.cos(angle + robot_yaw)
                obstacle_y = robot_y + range_value * math.sin(angle + robot_yaw)
                
                if self.is_unmapped_obstacle(obstacle_x, obstacle_y):
                    unmapped_obstacles.append((obstacle_x, obstacle_y))

        if unmapped_obstacles:
            self.plan_local_path(unmapped_obstacles, robot_x, robot_y, robot_yaw)

    def plan_local_path(self, unmapped_obstacles, robot_x, robot_y, robot_yaw):
        """Generate local path to avoid unmapped obstacles"""
        if not unmapped_obstacles or self.global_path is None:
            return

        # Find closest point on global path
        min_dist = float('inf')
        closest_point = None
        for pose in self.global_path.poses:
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_point = pose.pose.position

        if closest_point is None:
            return

        # Simple avoidance: Generate circular arc around obstacles
        # This is a simplified version - you might want to implement full DWA here
        cmd = Twist()
        
        # Check if obstacles are in front of the robot
        obstacle_in_front = False
        for ox, oy in unmapped_obstacles:
            dx = ox - robot_x
            dy = oy - robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx) - robot_yaw
            
            # If obstacle is within 1 meter and in front of robot (+/- 30 degrees)
            if dist < 1.0 and abs(angle) < math.pi/6:
                obstacle_in_front = True
                break

        if obstacle_in_front:
            # Basic obstacle avoidance
            cmd.linear.x = 0.1  # Slow down
            cmd.angular.z = 0.5  # Turn to avoid obstacle
        else:
            # Return to global path
            target_angle = math.atan2(
                closest_point.y - robot_y,
                closest_point.x - robot_x
            )
            angle_diff = target_angle - robot_yaw
            
            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
                
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3 * angle_diff

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()