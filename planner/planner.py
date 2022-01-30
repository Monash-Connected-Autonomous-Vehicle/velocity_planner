import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # Kills the node after 3 seconds to test the supervisor
        if self.i >= 6:
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
