import rclpy
from rclpy.node import Node

from pynput import keyboard

from std_msgs.msg import String


class KeyboardPublisher(Node):
    
    def __init__(self):
        super().__init__('keyboard_input')
        self.publisher_ = self.create_publisher(String, 'inputs', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            while (rclpy.ok() and listener.running):
                self.i += 1

    def on_press(self, key):
        msg = String()
        msg.data = str(key)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    rclpy.spin(keyboard_publisher)

    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
