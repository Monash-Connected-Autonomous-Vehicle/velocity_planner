import rclpy
from rclpy.node import Node
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from mcav_interfaces.msg import Waypoint, WaypointArray

class FakeWaypoints(Node):

    def __init__(self):
        super().__init__('fake_waypoints')
        self.publisher_ = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.init_waypoints()

        self.vis_pub_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 0);
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
        markers = []
        for index, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = "my_namespace"
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = waypoint.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.1
            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.append(marker)
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.vis_pub_.publish(marker_array)

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
