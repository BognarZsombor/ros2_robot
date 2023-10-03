import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion


class Robot(Node):

    def __init__(self):
        super().__init__('robot')
        self.subscription = self.create_subscription(
            String,
            'inputs',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(Pose, 'display', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info(f"Key pressed: {msg.data}")

    def timer_callback(self):
        msg = Pose()
        msg_point = Point()
        msg_quater = Quaternion()

        msg_point.x = 0.0
        msg_point.y = 0.0
        msg_point.z = 0.0

        msg_quater.x = 0.0
        msg_quater.y = 0.0
        msg_quater.z = 0.0
        msg_quater.w = 0.0

        msg.position = msg_point
        msg.orientation = msg_quater

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
