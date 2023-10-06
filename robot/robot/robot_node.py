import rclpy
from rclpy.node import Node

import math
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist


class Robot(Node):

    v = 0.0
    w = 0.0
    x = 0.0
    y = 0.0
    theta = 0.0
    time = 0.0
    lasttime = 0.0

    def __init__(self):
        super().__init__('robot')
        self.lasttime = self.get_clock().now()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(PoseStamped, 'display', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z


    def matadd(self, A, B):
        return [[A[i][j] + B[i][j]  for j in range(len(A[0]))] for i in range(len(A))]

    def matmul(self, A, B):
        return [[sum(a*b for a,b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]
        


    def timer_callback(self):
        self.time = self.get_clock().now()
        dt = self.time - self.lasttime
        dt_nano = dt.nanoseconds / 1e9

        A = [[1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, 1.0]]
        
        xt = [[self.x],
              [self.y],
              [self.theta]]
        
        B = [[math.cos(self.theta) * dt_nano, 0,0],
             [math.sin(self.theta) * dt_nano, 0,0],
             [0.0, dt_nano]]

        ut = [[self.v],
              [self.w]]
        
        Ax = self.matmul(A, xt)
        Bu = self.matmul(B, ut)

        self.lasttime = self.time

        result = self.matadd(Ax, Bu)

        self.x = result[0][0]
        self.y = result[1][0]
        self.theta = result[2][0]


        msg = PoseStamped()
        msg_point = Point()
        msg_quater = Quaternion()

        msg_point.x = self.x
        msg_point.y = self.y
        msg_point.z = 0.0

        rot = Rotation.from_euler('xyz', [0, 0, self.theta], degrees=False)
        rot_quat = rot.as_quat()

        msg_quater.x = rot_quat[0]
        msg_quater.y = rot_quat[1]
        msg_quater.z = rot_quat[2]
        msg_quater.w = rot_quat[3]

        
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = msg_point
        msg.pose.orientation = msg_quater


        self.get_logger().info(f"Result: {msg}, theta: {self.theta}")


        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
