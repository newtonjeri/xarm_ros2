from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TCPPosePublisherNode(Node):

    def __init__(self):
        super().__init__('tcp_pose_publisher_node')

        self.freq = 10

        self.source_frame_name = 'link_base'
        # Declare and acquire `source_frame` parameter
        self.source_frame = self.declare_parameter(
          'source_frame', self.source_frame_name).get_parameter_value().string_value
        
        self.target_frame_name = 'link_tcp'
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', self.target_frame_name).get_parameter_value().string_value
        
    

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher a publisher to send link_tcp pose relative to 
        self.publisher = self.create_publisher(Pose, '/tcp_pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1/self.freq, self.on_timer)

    def on_timer(self):
        # Look up for the transformation between target_frame and source frame frames
        # and send velocity commands for source frame to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return
        msg = Pose()
        msg.position.x = t.transform.translation.x
        msg.position.y = t.transform.translation.y
        msg.position.z = t.transform.translation.z

        msg.orientation.w = t.transform.rotation.w
        msg.orientation.x = t.transform.rotation.x
        msg.orientation.y = t.transform.rotation.y
        msg.orientation.z = t.transform.rotation.z


        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TCPPosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()