#! /usr/bin/env python33

from gazebo_msgs.srv import SetEntityState
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as server_client
        super().__init__('server_client')
        # create the service client object
        # defines the name and type of the service server we will work with.
        self.client = self.create_client(SetEntityState, "/cam_bot/set_entity_state")
        # checks once per second if a service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create a Empty request
        self.req = SetEntityState.Request()
        

    def send_request(self):
        """
        state:
            name: 'cam_bot'
            pose:
                position:
                x: 1.0
                y: 0.0
                z: 1.0
                orientation:
                x: 0.0
                y: 0.0
                z: 1.0
                w: 1.0
            twist:
                linear:
                x: 0.0
                y: 0.0
                z: 0.0
                angular:
                x: 0.0
                y: 0.0
                z: 0.0
            reference_frame: 'world'
            "

        """
        self.req.state.name = "cam_bot"
        self.req.state.pose.position.y = 1.0
        self.req.state.pose.position.z = 1.0
        self.req.state.reference_frame = "world"
        # send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the service
                # while the system is running. 
                # If the service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'the robot is moving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()