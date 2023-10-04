import rclpy
from rclpy.node import Node

import math
from scipy.spatial.transform import Rotation

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion


class Robot(Node):

    R = 1.0
    L = 3.0
    x = 0.0
    y = 0.0
    theta = 0.0
    wr = 0.0
    wl = 0.0

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
        if (msg.data == "\'w\'"):
            self.wr = 1.0
            self.wl = 1.0
        elif (msg.data == "\'s\'"):
            self.wr = -1.0
            self.wl = -1.0
        elif (msg.data == "\'a\'"):
            self.wr = 1.0
            self.wl = -1.0
        elif (msg.data == "\'d\'"):
            self.wr = -1.0
            self.wl = 1.0

        A = [[1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, 1.0]]
        
        xt = [[self.x],
              [self.y],
              [self.theta]]
        
        B = [[math.cos(self.theta), 0,0],
             [math.sin(self.theta), 0,0],
             [0.0, 1.0]]
        
        v = (self.wr + self.wl) * self.R / 2
        w = (self.wr - self.wl) * self.R / 2

        ut = [[v],
              [w]]
        
        Ax = self.matmul(A, xt)
        Bu = self.matmul(B, ut)

        result = self.matadd(Ax, Bu)

        self.x = result[0][0]
        self.y = result[1][0]
        self.theta = result[2][0]


    def matadd(self, A, B):
        return [[A[i][j] + B[i][j]  for j in range(len(A[0]))] for i in range(len(A))]

    def matmul(self, A, B):
        return [[sum(a*b for a,b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]
        


    def timer_callback(self):
        msg = Pose()
        msg_point = Point()
        msg_quater = Quaternion()

        msg_point.x = self.x
        msg_point.y = self.y
        msg_point.z = 0.0

        rot = Rotation.from_euler('xyz', [0, 0, self.theta], degrees=True)
        rot_quat = rot.as_quat()

        msg_quater.x = rot_quat[0]
        msg_quater.y = rot_quat[1]
        msg_quater.z = rot_quat[2]
        msg_quater.w = rot_quat[3]

        msg.position = msg_point
        msg.orientation = msg_quater

        self.get_logger().info(f"Result: {msg}")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
