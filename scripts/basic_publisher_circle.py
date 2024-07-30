#! /usr/bin/env python3

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy
import math

class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def print_coordinates(self):
        self.get_logger().info("[ x, y, z] = [" + str(self.x) + ", " +
                      str(self.y) + ", " + str(self.z) + "]")
        self.get_logger().info("[ roll, pitch, yaw] = [" + str(self.roll) +
                      ", " + str(self.pitch) + ", " + str(self.yaw) + "]")

class FramePublisher(Node):

    def __init__(self, num_points=50, timer_period=0.1, radius=1.0, height=1.0):
        super().__init__('basic_tf_publisher_circle')

        self.robot_frame = "carrot"

        self.num_points = num_points
        self.trajectory = self.generate_circle(radius, height)
        self.current_trajectory_index = 0

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.broadcast_robot_tf()
        self.get_logger().info("Published TF Circle")

    def generate_circle(self, radius, height):
        trajectory = []
        min_degrees = 0
        max_degrees = 360
        step_size = (max_degrees - min_degrees)/float(self.num_points)
        for segment in numpy.arange(min_degrees, max_degrees, step_size):
            rad_segment = math.radians(segment)
            x = math.cos(rad_segment)*radius
            y = math.sin(rad_segment)*radius
            z = height
            roll = 0
            pitch = 0
            yaw = rad_segment

            coord_object = Coordinates(x, y, z, roll, pitch, yaw)
            # coord_object.print_coordinates()
            trajectory.append(coord_object)

        return trajectory

    def step_trajectory(self, loop=True):
        """
        It returns the trajectory Coordinates that should be performed and moves one in the index
        :return:
        """
        try:
            coordinates = self.trajectory[self.current_trajectory_index]
            self.current_trajectory_index += 1
        except IndexError:
            if loop:
                # The trajectory doesnt have that index therefore we have finished the trajectory and we restart.
                self.current_trajectory_index = 0
                coordinates = self.trajectory[self.current_trajectory_index]
            else:
                coordinates = self.trajectory[len(self.trajectory)-1]

        return coordinates

    def broadcast_robot_tf(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.robot_frame

        new_coordinates = self.step_trajectory()


        t.transform.translation.x = new_coordinates.x
        t.transform.translation.y = new_coordinates.y
        t.transform.translation.z = new_coordinates.z

        q = tf_transformations.quaternion_from_euler(new_coordinates.roll,
                                                    new_coordinates.pitch,
                                                    new_coordinates.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init()
    node_object = FramePublisher()
    try:
        rclpy.spin(node_object)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()