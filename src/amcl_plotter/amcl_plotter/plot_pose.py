#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from rclpy.time import Time
from rclpy.duration import Duration
import math
import numpy as np
import transforms3d  # Add this import to match AMCL's quaternion handling

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')
        
        # Controller parameters
        self.k_linear = 0.3  # Linear velocity gain
        self.k_angular = 0.8  # Angular velocity gain
        self.goal_tolerance = 0.1  # Distance to goal tolerance
        self.lookahead_distance = 0.5  # Distance to look ahead on path
        
        # Store paths and current state
        self.global_path = None
        self.current_goal_index = 0
        self.is_avoiding_obstacle = False
        self.map_received = False
        
        # TF buffer and listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster for updating robot pose
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.dwa_cmd_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            self.initial_pose_callback, 
            10)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # Last DWA command and timestamp
        self.last_dwa_cmd = None
        self.last_dwa_time = None
        self.dwa_timeout = 0.5  # Timeout for DWA commands (seconds)
        
        # Wait for map frame
        self.wait_for_map_frame()
        
        self.get_logger().info('Path Controller initialized')

    def wait_for_map_frame(self):
        """Wait for the map frame to become available"""
        self.get_logger().info('Waiting for map frame...')
        rate = self.create_rate(1)  # 1 Hz
        max_attempts = 10
        attempt = 0

        while rclpy.ok() and attempt < max_attempts and not self.map_received:
            self.get_logger().info(f'Waiting for map... Attempt {attempt + 1}/{max_attempts}')
            try:
                rate.sleep()
            except:
                pass
            attempt += 1
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.map_received:
            self.get_logger().error('Map not received! Make sure map_server is running and properly configured/activated')
        else:
            self.get_logger().info('Map received successfully')

    def map_callback(self, msg):
        """Called when map is received"""
        if not self.map_received:
            self.get_logger().info('Received map')
            self.map_received = True

    def global_path_callback(self, msg):
        self.global_path = msg
        self.current_goal_index = 0
        self.get_logger().info('Received new global path')

    def dwa_cmd_callback(self, msg):
        """Callback for DWA obstacle avoidance commands"""
        self.last_dwa_cmd = msg
        self.last_dwa_time = self.get_clock().now()
        self.is_avoiding_obstacle = True

    def initial_pose_callback(self, msg):
        """Handle 2D Pose Estimate from RViz"""
        if not self.map_received:
            self.get_logger().warn('Received pose estimate but map is not available yet!')
            return

        self.get_logger().info('Received new pose estimate')
        
        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = 0.0
        
        # Set rotation
        t.transform.rotation = orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Updated robot pose: x={position.x:.2f}, y={position.y:.2f}')

    def get_robot_pose(self):
        """Get current robot pose from TF"""
        if not self.map_received:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
            
            return x, y, yaw
            
        except Exception as e:
            self.get_logger().debug(f'Transform error: {str(e)}')
            return None

    def get_target_point(self, robot_x, robot_y):
        """Get target point on global path using lookahead distance"""
        if not self.global_path or self.current_goal_index >= len(self.global_path.poses):
            return None

        # Start from last target point
        for i in range(self.current_goal_index, len(self.global_path.poses)):
            point = self.global_path.poses[i].pose.position
            distance = math.sqrt((point.x - robot_x)*2 + (point.y - robot_y)*2)
            
            if distance >= self.lookahead_distance:
                self.current_goal_index = i
                return point

        # If we're near the end of the path, return the final point
        return self.global_path.poses[-1].pose.position

    def is_dwa_command_valid(self):
        """Check if we have a recent DWA command"""
        if self.last_dwa_cmd is None or self.last_dwa_time is None:
            return False
            
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_dwa_time).nanoseconds / 1e9
        
        return time_diff < self.dwa_timeout

    def control_loop(self):
        """Main control loop"""
        if not self.global_path:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, robot_yaw = robot_pose
        cmd = Twist()

        # Check if we have a valid DWA command for obstacle avoidance
        if self.is_dwa_command_valid():
            # Use DWA command directly
            cmd = self.last_dwa_cmd
            self.get_logger().debug('Using DWA command for obstacle avoidance')
        else:
            # Reset obstacle avoidance flag
            self.is_avoiding_obstacle = False
            
            # Follow global path
            target = self.get_target_point(robot_x, robot_y)
            if target is None:
                return

            # Calculate distance and angle to target
            dx = target.x - robot_x
            dy = target.y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - robot_yaw
            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi

            # Generate velocity commands
            if distance > self.goal_tolerance:
                # Linear velocity proportional to distance, but capped
                cmd.linear.x = min(self.k_linear * distance, 0.5)
                # Angular velocity proportional to angle difference
                cmd.angular.z = self.k_angular * angle_diff
            else:
                # If we're at the final goal, stop
                if self.current_goal_index >= len(self.global_path.poses) - 1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info('Reached final goal')

        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()