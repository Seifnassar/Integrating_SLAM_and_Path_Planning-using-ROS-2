#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller activated.")

        # Publishers and Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Path tracking variables
        self.path = []
        self.current_goal_index = 0
        self.force_final_target = False

        # Controller parameters - tuned for TurtleBot3 Burger
        self.k_rho = 0.3     # Reduced for smoother linear velocity
        self.k_alpha = 0.6   # Reduced for smoother turning
        self.k_beta = -0.1   # Reduced for final orientation control
        self.k_final_rotation = 1.2  # Increased for more aggressive rotation
        
        # Speed limits
        self.max_linear_speed = 0.22  # Max linear velocity for TurtleBot3 Burger
        self.max_angular_speed = 2.84  # Max angular velocity for TurtleBot3 Burger
        
        # Goal tolerance
        self.position_tolerance = 0.1  # meters
        self.orientation_tolerance = 0.05  # radians - reduced for more precision
        self.lookahead_base = 0.3     # Base lookahead distance
        self.lookahead_final = 0.15   # Reduced lookahead when approaching final goal
        self.min_rotation_speed = 0.15  # Minimum rotation speed for final orientation
        self.final_approach_distance = 0.3  # Distance to switch to final approach mode

        # State variables
        self.final_goal_reached = False
        self.heading_attempts = 0
        self.max_heading_attempts = 100  # Increased for more attempts
        self.final_orientation_start_time = None
        
        # Create timer for debugging
        self.create_timer(1.0, self.debug_status)

    def debug_status(self):
        if self.path:
            self.get_logger().info(f"Current goal index: {self.current_goal_index}/{len(self.path)}")

    def find_nearest_waypoint_index(self, x, y, lookahead):
        if self.force_final_target:
            return len(self.path) - 1

        # Calculate distance to final waypoint
        final_x, final_y, _ = self.path[-1]
        dist_to_final = math.hypot(final_x - x, final_y - y)

        # If we're close to the final point, force targeting it
        if dist_to_final < self.final_approach_distance:
            self.force_final_target = True
            return len(self.path) - 1

        min_dist = float('inf')
        best_index = self.current_goal_index

        for i in range(self.current_goal_index, len(self.path)):
            px, py, _ = self.path[i]
            dist = math.hypot(px - x, py - y)
            
            if dist < min_dist and dist >= lookahead * 0.5:
                min_dist = dist
                best_index = i

        return best_index

    def path_callback(self, msg):
        self.path = []
        self.force_final_target = False
        
        # Process waypoints
        step = max(1, len(msg.poses) // 50)
        
        for i in range(0, len(msg.poses), step):
            pose = msg.poses[i].pose
            x = pose.position.x
            y = pose.position.y
            theta = self.quaternion_to_yaw(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w)
            self.path.append((x, y, theta))

        # Always include the final pose
        if msg.poses:
            final = msg.poses[-1].pose
            x = final.position.x
            y = final.position.y
            theta = self.quaternion_to_yaw(final.orientation.x, final.orientation.y,
                                         final.orientation.z, final.orientation.w)
            if not self.path or (x, y, theta) != self.path[-1]:
                self.path.append((x, y, theta))

        self.current_goal_index = 0
        self.final_goal_reached = False
        self.heading_attempts = 0
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints")

    def pose_callback(self, msg):
        if not self.path or self.current_goal_index >= len(self.path):
            self.stop_robot()
            return

        # Get current pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        # Get final goal point
        final_x, final_y, final_theta = self.path[-1]
        dist_to_final = math.hypot(final_x - x, final_y - y)

        # If we're in final approach mode
        if self.force_final_target:
            angle_error = self.normalize_angle(final_theta - yaw)
            
            if dist_to_final < self.position_tolerance:
                if abs(angle_error) < self.orientation_tolerance:
                    self.stop_robot()
                    self.get_logger().info(f"Final goal reached! Position error: {dist_to_final:.3f}m, Heading error: {math.degrees(angle_error):.2f}°")
                    self.final_goal_reached = True
                    return
                else:
                    # Pure rotation to achieve final heading
                    cmd = Twist()
                    rotation_speed = self.k_final_rotation * angle_error
                    
                    # Ensure minimum rotation speed
                    if abs(rotation_speed) < self.min_rotation_speed:
                        rotation_speed = self.min_rotation_speed if angle_error > 0 else -self.min_rotation_speed
                    
                    cmd.angular.z = max(min(rotation_speed, self.max_angular_speed * 0.7), -self.max_angular_speed * 0.7)
                    self.cmd_vel_pub.publish(cmd)
                    self.get_logger().info(f"Adjusting final heading: error={math.degrees(angle_error):.2f}°, speed={cmd.angular.z:.3f}")
                    return

        # Update goal index
        self.current_goal_index = self.find_nearest_waypoint_index(x, y, self.lookahead_base)
        
        # Get current goal point
        goal_x, goal_y, goal_theta = self.path[self.current_goal_index]
        dx = goal_x - x
        dy = goal_y - y
        rho = math.hypot(dx, dy)
        alpha = self.normalize_angle(math.atan2(dy, dx) - yaw)
        beta = self.normalize_angle(goal_theta - yaw - alpha)

        # Calculate control commands
        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta

        # Reduce speed when turning sharply or near final goal
        turn_factor = max(0.0, 1.0 - abs(alpha) / math.pi)
        v *= turn_factor

        if dist_to_final < self.final_approach_distance:
            v *= 0.5  # Reduce speed during final approach

        # Apply velocity limits
        v = max(min(v, self.max_linear_speed), 0.0)
        omega = max(min(omega, self.max_angular_speed), -self.max_angular_speed)

        # Create and publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()