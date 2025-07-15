#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import random

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Quaternion
from std_msgs.msg import Header
import transforms3d
from tf2_ros import TransformBroadcaster       #need to know both
from geometry_msgs.msg import TransformStamped

class Particle:
    def __init__(self, x, y, theta, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class AmclNode(Node):
    def __init__(self):
        super().__init__('amcl_node')
        self.get_logger().info('AMCL node started.')

        #parameters
        self.num_particles = 100 #number of particles we will use in PF
        self.particles = [] #store particles in this form (x, y, theta, weight)
        self.map = None 
        self.laser_received = False
        self.odom_received = False
        self.last_odom = None


        #topics we subscribe to
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_pose_callback,10)


        #topics we publish to
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        
        # to call the update function every 0.5 sec for updatin localization.
        self.create_timer(0.2, self.update)

        self.tf_broadcaster = TransformBroadcaster(self) #need to study

           
    def odom_callback(self,msg): #called automatially when a new message is on /odom topic
        if not self.odom_received: # we check if we recieved odometry before if not we move in, self.odom_received starts as false in paramters, used for the first message because we need new and old data to calculate.
            self.last_odom = msg
            self.odom_received = True
            return 
        
        current = msg
        previous = self.last_odom #now we have two poses so we can calculate the movment of the robot

        #subtracting the new position with old one in both x and y to know how far the robot moved.
        dx = current.pose.pose.position.x - previous.pose.pose.position.x
        dy = current.pose.pose.position.y - previous.pose.pose.position.y

        def get_yaw(q): # take quaternion message and returns yaw
            _, _, yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z]) # _, _, yaw is don becuse there is roll and pitch also but we dont need them
            return yaw
        
        # To calculate how much the robot rotated
        current_theta = get_yaw(current.pose.pose.orientation) # take the orientation given in quaternion and pass it to get_yaw to get it in radians
        previous_theta = get_yaw(previous.pose.pose.orientation) # same with previous
        dtheta = current_theta - previous_theta

        self.last_odom = current # save the current odometry to be the previous for the next time

        for p in self.particles:
            #add noise
            noisy_dx = dx + random.gauss(0, 0.02)
            noisy_dy = dy + random.gauss(0, 0.02)
            noisy_dtheta = dtheta + random.gauss(0, 0.01)

            # update the particles pose
            p.x += noisy_dx * math.cos(p.theta) - noisy_dy * math.sin(p.theta)
            p.y += noisy_dx * math.sin(p.theta) + noisy_dy * math.cos(p.theta) # need to come back to this part for understanding the math
            p.theta += noisy_dtheta


    def scan_callback(self,msg): # called after receiving laser scan from /scan
        # check if we have a map and odometry first (need both)
        if self.map is None or not self.odom_received:
            return
        
        self.laser_received = True

        angles = []
        ranges = [] 

        angle_min = msg.angle_min # starting angle of the first laser beam
        angle_increment = msg.angle_increment # angle between beams
        
        for i in range(0, len(msg.ranges), 15): # i represents each beam, 15 represents the range of beams that we take out of the ~360 beams that are avalible to reduce processing to run faster
            angle = angle_min + i * angle_increment
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max: # closest and farthest the lidar can read (makng sure its in the range to filter out errors)
                angles.append(angle)
                ranges.append(r)
        #standard deviation
        sigma = 0.5 # was 0.2 changed to lossen calculations
        sigma_sq = sigma ** 2

        # map info
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        # converting map sanme as map_callback
        map_data = np.array(self.map.data, dtype=np.int8).reshape((height, width))

        for p in self.particles:
            total_weight = 1.0

            for angle, actual_range in zip(angles, ranges):
                #calculate global angle
                scan_angle = p.theta + angle
                x_end = p.x + actual_range * math.cos(scan_angle)
                y_end = p.y + actual_range * math.sin(scan_angle)

                # convert to map coordinates
                mx = int((x_end - origin_x) / resolution)
                my = int((y_end - origin_y) / resolution)

                #check bounds
                if 0 <= mx < width and 0 <= my < height :
                    val = map_data[my, mx]
                    if val == 100:
                        distance = 0.0
                    elif val == 0:
                        distance = 0.2 #changed from 0.1 to 0.2 to be less harsh
                    else:
                        distance = 1.0
                else:
                    distance = 1.0 # from 2 to 1 

                prob = math.exp(-(distance ** 2) / (2 * sigma_sq))
                prob = max(prob, 1e-10) # avoid total weight = 0 
                total_weight *= prob

            p.weight = total_weight
            self.get_logger().info(f"Particle weight: {p.weight:.5f}")


    def map_callback(self,msg): #called when we recieve the map
        self.get_logger().info('Map received.')
        self.map = msg #storing the map

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution #size of each cell of the map in meters
        origin = msg.info.origin 
    
        self.particles = []

         # Try odometry-based initialization first (NEED TO STUDY, for fast localization without 2D pose estimate.)
        if self.last_odom:
            self.get_logger().info("Using odometry-based auto initialization.")

            q = self.last_odom.pose.pose.orientation
            _, _, init_theta = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

            init_x = self.last_odom.pose.pose.position.x
            init_y = self.last_odom.pose.pose.position.y

            for _ in range(self.num_particles):
                noisy_x = init_x + random.gauss(0, 0.2)
                noisy_y = init_y + random.gauss(0, 0.2)
                noisy_theta = init_theta + random.gauss(0, 0.1)
                p = Particle(noisy_x, noisy_y, noisy_theta, weight=1.0 / self.num_particles)
                self.particles.append(p)

        else:
            self.get_logger().warn("No odometry yet! Falling back to random map initialization.")

            #msg.data is 1D list of all the map values so convert it to a 2D NumPy array so we can index it like a normal image: map[y][x]
            map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            free_indices = np.argwhere(map_data == 0) #finds all free cells = 0

            #for checking in case of no free cells
            if len(free_indices) == 0:
                self.get_logger().error('No free space in the map to place particles.')
                return

            for _ in range(self.num_particles):
                idx = random.choice(free_indices)
                map_y, map_x = idx
                real_x = map_x * resolution + origin.position.x
                real_y = map_y * resolution + origin.position.y
                theta = random.uniform(-math.pi, math.pi)
            
                p = Particle(real_x, real_y, theta, weight=1.0/self.num_particles)
                self.particles.append(p)

        self.get_logger().info(f'Initialized {len(self.particles)} particles.')
        self.publish_particle_cloud()
        
    def resample_particles(self): # need to study 

        weights = [p.weight for p in self.particles]
        weight_sum = sum(weights)

        if weight_sum == 0:
            self.get_logger().warn('Total particle wight is zero, skipping resampling.')
            return
        
        for p in self.particles:
            p.weight /= weight_sum

        new_particles = []
        N = self.num_particles
        r = random.uniform(0, 1 / N)
        c = self.particles[0].weight
        i = 0

        for m in range(N):
            U = r + m / N
            while U > c and i < N-1:
                i += 1 
                c += self.particles[i].weight
            p = self.particles[i]

            new_p = Particle(
            x=p.x + random.gauss(0, 0.01),
            y=p.y + random.gauss(0, 0.01),
            theta=p.theta + random.gauss(0, 0.005),
            weight=1.0 / N
          )
            new_particles.append(new_p)

        self.particles = new_particles
    
    def estimate_pose(self): #need to study.
        if not self.particles:
            return
        
        x = 0.0
        y = 0.0
        sin_sum = 0.0
        cos_sum = 0.0

        for p in self.particles:
            x += p.weight * p.x
            y += p.weight * p.y
            sin_sum += p.weight * math.sin(p.theta)
            cos_sum += p.weight * math.cos(p.theta)

        avg_x = x
        avg_y = y
        avg_theta = math.atan2(sin_sum, cos_sum)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = avg_x
        t.transform.translation.y = avg_y
        t.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0.0, 0.0, avg_theta)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

        # Fill PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = avg_x
        pose_msg.pose.pose.position.y = avg_y
        pose_msg.pose.pose.position.z = 0.0

        q = transforms3d.euler.euler2quat(0.0, 0.0, avg_theta)  # returns [w, x, y, z]
        pose_msg.pose.pose.orientation.x = q[1]
        pose_msg.pose.pose.orientation.y = q[2]
        pose_msg.pose.pose.orientation.z = q[3]
        pose_msg.pose.pose.orientation.w = q[0]

        self.pose_pub.publish(pose_msg)

    def publish_particle_cloud(self): #need to study
        if not self.particles:
            return
        
        particle_msg = PoseArray()
        particle_msg.header = Header()
        particle_msg.header.stamp = self.get_clock().now().to_msg()
        particle_msg.header.frame_id = "map"

        for p in self.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.position.z = 0.0

            q = transforms3d.euler.euler2quat(0.0, 0.0, p.theta)
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            pose.orientation.w = q[0]

            particle_msg.poses.append(pose)

        self.get_logger().info(f"Publishing {len(self.particles)} particles.")
        self.particles_pub.publish(particle_msg)
    

    def initial_pose_callback(self, msg):
        self.get_logger().info("Received initial pose from RViz.")

        init_x = msg.pose.pose.position.x
        init_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, init_theta = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

        self.particles = []
        for _ in range(self.num_particles):
            noisy_x = init_x + random.gauss(0, 0.2)
            noisy_y = init_y + random.gauss(0, 0.2)
            noisy_theta = init_theta + random.gauss(0, 0.1)
            p = Particle(noisy_x, noisy_y, noisy_theta, weight=1.0 / self.num_particles)
            self.particles.append(p)

        self.get_logger().info(f"Particles reinitialized at ({init_x:.2f}, {init_y:.2f})")
        self.publish_particle_cloud()


    def update(self):
        if not self.laser_received :
            return
        
        self.resample_particles()

        self.estimate_pose()

        self.publish_particle_cloud()


def main(args=None):
    rclpy.init(args=args)
    node = AmclNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()