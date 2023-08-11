#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from visualization_msgs.msg import Marker

class Mesh_Marker(Node):
    def __init__(self):
        super().__init__('mesh_marker')
        self.marker_pub = self.create_publisher(Marker, '/avocado', 2)

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = ""

        # Shape (mesh resource type - 10)
        self.marker.type = 10
        self.marker.id = 0
        self.marker.action = 0

        # Note: Must set mesh_resource to a valid URL for a model to appear
        self.marker.mesh_resource = "https://raw.githubusercontent.com/soyhorteconh/foxglove_test/main/ascii_cprazon.stl"
        self.marker.mesh_use_embedded_materials = True

        # Scale
        self.marker.scale.x = 10.0
        self.marker.scale.y = 10.0
        self.marker.scale.z = 10.0

        # Color
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        # Pose
        self.marker.pose.position.x = 3.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)

    mesh_marker = Mesh_Marker()
    mesh_marker.get_logger().info('Mesh marker node started')
    rclpy.spin(mesh_marker)
    mesh_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()