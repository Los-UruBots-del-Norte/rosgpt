#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from nav_msgs.msg import Odometry


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Odometry, "/husky_velocity_controller/odom", self.pose_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.pose = Odometry()

        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.move_executor = SingleThreadedExecutor()
        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()
        print('ROSGPT Turtlesim Controller Started. Waiting for input commands ...')

        
    
    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        self.theta  = msg.pose.pose.orientation.z
        self.pose = msg
        #print (self.x, self.y) #used for debegging
    
    #this callback represents the ROSGPTParser. It takes a JSON, parses it, and converts it to a ROS 2 command
    def voice_cmd_callback(self, msg):
        #print(msg.data)
        try:
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json']) #we only consider the pure json message. cmd['text'] contains a mix of text and json
            print('JSON command received: \n',cmd,'\n')
            if cmd['action'] == 'go_to_goal':
                location = cmd['params']['location']['value']
                self.go_to_goal(location)
            elif cmd['action'] == 'move':
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                is_forward = cmd['params'].get('is_forward', True)

                #METHOD 1: running an async method as a new task. But method 2 is more straightforward
                #self.move_executor.create_task(self.move_coro(linear_speed, distance, is_forward))
                
                #METHOD 2. create a thread executor
                #we need to run the method on a different thread to avoid blocking rclpy.spin. 
                self.thread_executor.submit(self.move, linear_speed, distance, is_forward)

                #running move on the main thread will generate to error, as it will block rclpy.spin
                #self.move(linear_speed, distance, is_forward)
            elif cmd['action'] == 'rotate':
                angular_velocity = cmd['params'].get('angular_velocity', 1.0)
                angle = cmd['params'].get('angle', 90.0)
                is_clockwise = cmd['params'].get('is_clockwise', True)
                self.thread_executor.submit(self.rotate, angular_velocity, angle, is_clockwise)
                #self.rotate(angular_velocity, angle, is_clockwise)
        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))   

    def go_to_goal(self, location):
        goal_x = location['x']
        goal_y = location['y']
    
        print(f'Going to goal: ({goal_x}, {goal_y})')
    
        while not self.is_at_goal(goal_x, goal_y):
            # Step 1: Calculate the direction to the goal
            direction = self.calculate_direction_to_goal(goal_x, goal_y)
            print(f'Calculated direction: {direction}')
            
            # Step 2: Rotate the robot towards the goal
            self.rotate_towards_goal(direction)
            
            # Step 3: Move the robot towards the goal
            self.move_towards_goal(goal_x, goal_y)

        print('Goal reached!')

    def is_at_goal(self, goal_x, goal_y, tolerance=0.1):
        distance = math.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
        return distance < tolerance

    def calculate_direction_to_goal(self, goal_x, goal_y):
        return math.atan2(goal_y - self.y, goal_x - self.x)

    def rotate_towards_goal(self, goal_direction):
        current_direction = self.theta
        relative_angle = goal_direction - current_direction
        
        # Normalize the angle to the range [-pi, pi]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi
        
        angular_speed = 0.5  # rad/s
        duration = abs(relative_angle) / angular_speed
        
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed if relative_angle > 0 else -angular_speed
        
        start_time = self.get_clock().now().to_sec()
        while self.get_clock().now().to_sec() - start_time < duration:
            self.velocity_publisher.publish(twist_msg)
            self.move_executor.spin_once(timeout_sec=0.1)
        
        # Stop rotation
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)

    def move_towards_goal(self, goal_x, goal_y):
        speed = 0.2  # m/s
        distance = math.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
        
        twist_msg = Twist()
        twist_msg.linear.x = speed
        
        start_time = self.get_clock().now().to_sec()
        duration = distance / speed
        
        while not self.is_at_goal(goal_x, goal_y):
            self.velocity_publisher.publish(twist_msg)
            self.move_executor.spin_once(timeout_sec=0.1)
        
        # Stop movement
        twist_msg.linear.x = 0.0
        self.velocity_publisher.publish(twist_msg)

    async def move_coro(self, linear_speed, distance, is_forward):
        return self.move(linear_speed, distance, is_forward)

    def get_distance(self,start, destination):
        return math.sqrt(((destination.pose.pose.position.x-start.pose.pose.position.x)**2 + (destination.pose.pose.position.y-start.pose.pose.position.y)**2))
    
    def move(self, linear_speed, distance, is_forward): 
        if is_forward:
            direction = 'forward'
        else:
            direction = 'backward'
        print('Start moving the robot ', direction, ' at ', linear_speed, 'm/s and for a distance ', distance, 'meter')
        twist_msg = Twist()

        if (linear_speed > 1.0):
            print('[ERROR]: The speed must be lower than 1.0!')
            return -1

        twist_msg.linear.x = float(abs(linear_speed) if is_forward else -abs(linear_speed))
        twist_msg.linear.x = float(abs(linear_speed) * (1 if is_forward else -1))

        start_pose = copy.copy(self.pose)

        #self.move_executor.add_node(self)#

        while self.get_distance(start_pose, self.pose) < distance:
            #print('start_pose', start_pose, 'self.pose', self.pose, 'moved_distance: ', self.get_distance(start_pose, self.pose))
            #print('self.get_distance (start_pose, self.pose)<distance: ', self.get_distance(start_pose, self.pose) < distance)
            print('distance moved: ', self.get_distance(start_pose, self.pose))
            print(start_pose.pose.pose.position.x)
            self.velocity_publisher.publish(twist_msg)
            self.move_executor.spin_once(timeout_sec=0.1)

        twist_msg.linear.x = 0.0
        self.velocity_publisher.publish(twist_msg)
        print('distance moved: ', self.get_distance(start_pose, self.pose))
        print('The Robot has stopped...')

        #self.move_executor.remove_node(self)


        #return 0



    def rotate (self, angular_speed_degree, desired_relative_angle_degree, clockwise):
        print('Start Rotating the Robot ...')
        #rclpy.spin_once(self)
        twist_msg=Twist()
        angular_speed_degree=abs(angular_speed_degree) #make sure it is a positive relative angle
        if (angular_speed_degree>30) :
            print (angular_speed_degree)
            print('[ERROR]: The rotation speed must be lower than 0.5!')
            return -1
        
        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        twist_msg.angular.z = abs(angular_speed_radians) * (-1 if clockwise else 1)

        start_pose = copy.copy(self.pose)
        
        #rclpy.spin_once(self)

        rotated_related_angle_degree=0.0

        while rotated_related_angle_degree<desired_relative_angle_degree:
            #rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            #print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
            rotated_related_angle_degree = math.degrees(abs(start_pose.pose.pose.position.theta - self.pose.pose.pose.position.theta))
            #rclpy.spin_once(self)
            
            #rclpy.spin_once(self)
            time.sleep(0.01)
        #print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped...')

        #return 0

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
