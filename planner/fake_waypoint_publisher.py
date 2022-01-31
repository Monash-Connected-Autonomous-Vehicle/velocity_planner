import rclpy
from rclpy.node import Node
import numpy as np

from mcav_interfaces.msg import Waypoint, WaypointArray

class FakeWaypoints(Node):

    def __init__(self):
        super().__init__('fake_waypoints')
        self.publisher_ = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.init_waypoints()

    def init_waypoints(self):
        """ Modifies self.waypoints """
        x = np.arange(0.0, 20.0, 0.1)
        y = np.sin(x)

        for i in range(len(x)):
            waypoint = Waypoint()
            waypoint.frame_id = 'map'
            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            self.waypoints.append(waypoint)

    def timer_callback(self):
        msg = WaypointArray()
        msg.waypoints = self.waypoints
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing "%d" waypoints' % len(msg.waypoints))

def main(args=None):
    rclpy.init(args=args)

    fake_waypoints = FakeWaypoints()
    rclpy.spin(fake_waypoints)
    fake_waypoints.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
