import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from mcav_interfaces.msg import WaypointArray

class WaypointVisualiser(Node):

    def __init__(self):
        super().__init__('waypoint_visusaliser')

        self.global_sub = self.create_subscription(WaypointArray,
            'global_waypoints', self.global_callback, 10)
        self.global_sub = self.create_subscription(WaypointArray,
            'local_waypoints', self.local_callback, 10)

        self.vis_pub_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 0)
        self.local_vis_pub_ = self.create_publisher(MarkerArray, 'local_visualization_marker_array', 0)

    def global_callback(self, msg):
        self.publish_markers(msg.waypoints, 'global_waypoints')

    def local_callback(self, msg):
        self.publish_markers(msg.waypoints, 'local_waypoints')

    def publish_markers(self, waypoints, namespace: str):
        is_local = namespace == 'local_waypoints'
        markers = []
        for index, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = namespace
            marker.id = index
            marker.type = Marker.SPHERE if is_local else Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = waypoint.pose
            length = 0.5
            if not is_local:
                marker.pose.position.x -= length/2.
            marker.scale.x = 0.5 if is_local else 0.5
            marker.scale.y = 0.5 if is_local else 0.05
            marker.scale.z = 0.01 if is_local else 0.1
            marker.color.a = 0.3 if is_local else 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0 if is_local else 1.0
            marker.color.g = 1.0 if is_local else 0.0
            marker.color.b = 0.0
            markers.append(marker)
        marker_array = MarkerArray()
        marker_array.markers = markers

        if namespace == 'local_waypoints':
            self.local_vis_pub_.publish(marker_array)
        else:
            self.vis_pub_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    waypoint_visusaliser = WaypointVisualiser()
    rclpy.spin(waypoint_visusaliser)
    waypoint_visusaliser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
