#! /usr/bin/env python3
import sys
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_turtle_tf2_broadcaster')

        self.get_logger().info("TEST")
        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms()

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = sys.argv[1]
        static_transformStamped.child_frame_id = sys.argv[2]
        static_transformStamped.transform.translation.x = float(sys.argv[3])
        static_transformStamped.transform.translation.y = float(sys.argv[4])
        static_transformStamped.transform.translation.z = float(sys.argv[5])
        quat = tf_transformations.quaternion_from_euler(
        float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

    


def main():

    rclpy.init()

    node_obj = StaticFramePublisher()
    try:
        rclpy.spin(node_obj)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()