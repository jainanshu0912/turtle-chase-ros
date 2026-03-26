#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray, Turtle
from my_robot_interfaces.srv import CatchTurtle
from functools import partial
from enum import Enum

class ControllerState(Enum):
    IDLE = 0
    CHASING = 1
    CATCHING = 2

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.declare_parameter('catch_closest_turtle_first', True)
        self.declare_parameter('linear_speed_controller', 2.0)
        self.declare_parameter('angular_speed_controller', 6.0)
        self.catch_closest_turtle_first = self.get_parameter('catch_closest_turtle_first').value
        self.linear_speed_controller = self.get_parameter('linear_speed_controller').value
        self.angular_speed_controller = self.get_parameter('angular_speed_controller').value
        self.turtle_to_catch : Turtle = None
        self.state = ControllerState.IDLE
        self.pose: Pose = None
        self.prev_linear = 0.0
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.alive_turtles_subscriber = self.create_subscription(
            TurtleArray, 'alive_turtles', self.alive_turtles_callback, 10)
        self.catch_turtle_client = self.create_client(CatchTurtle, 'catch_turtle')
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def alive_turtles_callback(self, msg: TurtleArray):
        if self.state != ControllerState.IDLE:
            return
        
        if len(msg.turtles) == 0:
            return
        
        if self.pose is None: 
            return
        
        else:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose.x
                    dist_y = turtle.y - self.pose.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance

                self.turtle_to_catch = closest_turtle
                self.state = ControllerState.CHASING

            else:
                self.turtle_to_catch = msg.turtles[0] 
                self.state = ControllerState.CHASING
    
    def pose_callback(self, pose: Pose):
        self.pose= pose

    def control_loop(self):
        if self.pose is None:
            return
        
        if self.state == ControllerState.IDLE:
            return
        
        cmd = Twist()
        
        self.get_logger().info(f'Target: {self.turtle_to_catch.name}', throttle_duration_sec=1.0)
        if self.state == ControllerState.CHASING:
            dist_x = self.turtle_to_catch.x - self.pose.x
            dist_y = self.turtle_to_catch.y - self.pose.y
            distance = math.sqrt(dist_x**2 + dist_y**2)

            if distance > 0.5:
                max_linear_speed = 8.0
                max_angular_speed = 10.0
                angle_threshold = 0.1
                
                target_angle = math.atan2(dist_y, dist_x)
                angle_diff = target_angle - self.pose.theta

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if abs(angle_diff)> angle_threshold:
                    cmd.linear.x = 0.0
                    cmd.angular.z = min(self.angular_speed_controller * angle_diff, max_angular_speed)
                else:
                    cmd.linear.x = min(self.linear_speed_controller * distance, max_linear_speed)
                    cmd.angular.z = 0.0

            else:
                #target_reached
                cmd.linear.x = max(self.linear_speed_controller * distance, 0.2)
                cmd.angular.z = 0.0
                self.call_catch_turtle_service(self.turtle_to_catch.name)
                self.state = ControllerState.CATCHING

        #Acceleration limiting
        max_accel = 0.3
        delta = cmd.linear.x - self.prev_linear
        delta = max(min(delta, max_accel), -max_accel)
        cmd.linear.x = self.prev_linear + delta
        self.prev_linear = cmd.linear.x

        self.cmd_vel_publisher.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for catch turtle service...')

        request = CatchTurtle.Request()
        request.name = turtle_name
        
        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle_service, turtle_name=turtle_name))
        
    def callback_catch_turtle_service(self, future, turtle_name):
        response: CatchTurtle.Response = future.result()
        if response.success:
            self.state = ControllerState.IDLE
            self.turtle_to_catch = None
        if not response.success:
            self.get_logger().info(f'Failed to catch turtle "{turtle_name}".')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()