#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
import time
import numpy

class CamBotForce(Node):

    def __init__(self):
        super().__init__('force_move_cam_bot')
        
        self.max_linear = 0.5
        self.min_linear = -0.5

        self.max_angular = 0.01
        self.min_angular = -0.01

        self.publisher_ = self.create_publisher(Wrench, '/cam_bot_force', 1)
        self.init_wrench_message()

        self.subscriber= self.create_subscription(
            Twist,
            '/cam_bot_cmd_vel',
            self.listener_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
    

    def listener_callback(self, msg):
        self.update_pose_data(msg)
        self.send_wrench()
        
        


    def update_pose_data(self, msg):
        self.desired_effort = msg

    def get_pose_data(self):
        return self.desired_effort

    def init_wrench_message(self):
        self.wrench_msg = Wrench()

    def send_wrench(self):

        self.wrench_msg.force.x = numpy.clip(self.desired_effort.linear.x,self.min_linear,self.max_linear) 
        self.wrench_msg.force.y = numpy.clip(self.desired_effort.linear.y,self.min_linear,self.max_linear) 
        self.wrench_msg.force.z = numpy.clip(self.desired_effort.linear.z,self.min_linear,self.max_linear) 
        self.wrench_msg.torque.x = numpy.clip(self.desired_effort.angular.x,self.min_angular,self.max_angular) 
        self.wrench_msg.torque.y = numpy.clip(self.desired_effort.angular.y,self.min_angular,self.max_angular)
        self.wrench_msg.torque.z = numpy.clip(self.desired_effort.angular.z,self.min_angular,self.max_angular)

        self.publisher_.publish(self.wrench_msg)

        self.get_logger().warning("Sent Force ="+str(self.wrench_msg))

        time.sleep(0.2)
        #And now we set it again to 0 to avoid build up movement
        self.wrench_msg.force.x = 0.0
        self.wrench_msg.force.y = 0.0
        self.wrench_msg.force.z = 0.0
        self.wrench_msg.torque.x = 0.0
        self.wrench_msg.torque.y = 0.0
        self.wrench_msg.torque.z = 0.0

        self.publisher_.publish(self.wrench_msg)

        self.get_logger().info("Sent Force ZERO="+str(self.wrench_msg))


def main(args=None):
    
    rclpy.init()

    force_obj = CamBotForce()

    rclpy.spin(force_obj)


if __name__ == '__main__':
    main()