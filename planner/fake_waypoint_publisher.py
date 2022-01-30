import rclpy
from rclpy.node import Node
import numpy as np

from visualization_msgs.msg import Marker
from mcav_interfaces.msg import Waypoint, WaypointArray

class FakeWaypoints(Node):

    def __init__(self):
        super().__init__('fake_waypoints')
        self.publisher_ = self.create_publisher(WaypointArray, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.init_waypoints()

        self.vis_pub_ = self.create_publisher(Marker, 'visualization_marker', 0 );
        self.publish_markers()

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

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.
        marker.pose.position.y = 1.
        marker.pose.position.z = 1.
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.vis_pub_.publish( marker )

    def timer_callback(self):
        msg = WaypointArray()
        msg.waypoints = self.waypoints
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing "%d" waypoints' % len(msg.waypoints))
        self.publish_markers()


def main(args=None):
    rclpy.init(args=args)

    fake_waypoints = FakeWaypoints()
    rclpy.spin(fake_waypoints)
    fake_waypoints.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
