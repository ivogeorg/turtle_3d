#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class CamBotOdomToTf(Node):

    def __init__(self, robot_base_frame="camera_bot_base_link"):
        super().__init__('move_laser_actionclient')

        self._robot_base_frame = robot_base_frame
        self.init_tf_message()

        self.subscriber = self.create_subscription(
            Odometry,
            '/cam_bot_odom',
            self.listener_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.br = tf2_ros.TransformBroadcaster(self)

    def listener_callback(self, msg):
        # print the log info in the terminal
        self.update_data(msg)
        self.get_logger().debug('Odom VALUE: "%s"' % str(self.cam_bot_odom))
        self.broadcast_new_tf()

    def update_data(self, msg):
        self.cam_bot_odom = msg

    def get_odom_data(self):

        position = self.cam_bot_odom.pose.pose.position
        orientation = self.cam_bot_odom.pose.pose.orientation

        return position, orientation

    def init_tf_message(self):
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "world"
        self.transform_stamped.child_frame_id = self._robot_base_frame

    def broadcast_new_tf(self):

        position, orientation = self.get_odom_data()
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        self.br.sendTransform(self.transform_stamped)


def main(args=None):

    rclpy.init()

    odom_to_tf_obj = CamBotOdomToTf()

    rclpy.spin(odom_to_tf_obj)


if __name__ == '__main__':
    main()
