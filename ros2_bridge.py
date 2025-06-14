import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('teleop_ros2_bridge')
        self.image_callback = None
        self.map_callback = None

        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_image(self, msg):
        if self.image_callback:
            self.image_callback(msg)

    def _on_map(self, msg):
        if self.map_callback:
            self.map_callback(msg)

    def set_image_callback(self, cb):
        self.image_callback = cb

    def set_map_callback(self, cb):
        self.map_callback = cb

    def publish_twist(self, twist_msg):
        self.cmd_vel_pub.publish(twist_msg)
