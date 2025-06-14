from geometry_msgs.msg import Twist

class TeleopController:
    def __init__(self, ros2_bridge):
        self.ros2_bridge = ros2_bridge

    def handle_teleop(self, data):
        # data: {'type': 'teleop', 'linear': float, 'angular': float}
        twist = Twist()
        twist.linear.x = float(data.get('linear', 0.0))
        twist.angular.z = float(data.get('angular', 0.0))
        self.ros2_bridge.publish_twist(twist)
