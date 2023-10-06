import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion
import math

from geometry_msgs.msg import PoseStamped, Twist, Vector3


class RvizPose(Node):

    kp = 0.5
    xt = 0.0
    yt = 0.0
    xr = 0
    yr = 0
    theta = 0.0
    sending_dest = False

    def __init__(self):
        super().__init__('rvizpose')
        self.dest_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.destination_pose_listener_callback,
            10)
        self.dest_subscription

        self.robot_subscription = self.create_subscription(
            PoseStamped,
            'display',
            self.robot_pose_listener_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def destination_pose_listener_callback(self, msg):
        self.xt = msg.pose.position.x
        self.yt = msg.pose.position.y
        if (self.xt != 0):
            self.sending_dest = True

    def robot_pose_listener_callback(self, msg):
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw
        self.xr = msg.pose.position.x
        self.yr = msg.pose.position.y

    def timer_callback(self):
        msg = Twist()
        linear = Vector3()
        angular = Vector3()

        if (self.sending_dest):
            linear.x = 0.3
        else:
            linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        angular.x = 0.0
        angular.y = 0.0
        thetat = math.atan2(self.yt - self.yr, self.xt - self.xr)
        if (thetat < math.pi):
            thetat = math.pi * 2 + thetat
        angular.z = self.kp * (thetat - self.theta)


        self.get_logger().info(f"theta: {self.theta}, thetat: {thetat}")


        msg.linear = linear
        msg.angular = angular

        if (abs(self.xt - self.xr) > 0.5 or abs(self.yt - self.yr > 0.5)):
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    rvizpose = RvizPose()

    rclpy.spin(rvizpose)

    rvizpose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()