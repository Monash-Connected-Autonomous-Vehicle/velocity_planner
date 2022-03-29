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
            '/velocity_planner/local_waypoints', self.local_callback, 10)

        self.vis_pub_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 0)
        self.local_vis_pub_ = self.create_publisher(MarkerArray, 'local_visualization_marker_array', 0)

        self.local_waypoints_length = 40 # TODO: make into parameter

    def global_callback(self, msg):
        self.publish_global(msg.waypoints)

    def local_callback(self, msg):
        self.publish_local(msg.waypoints)

    def publish_local(self, waypoints):
        markers = []
        for index, waypoint in enumerate(waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = waypoints[0].frame_id
            pose_marker.ns = 'local_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.SPHERE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.4
            pose_marker.scale.y = 0.4
            pose_marker.scale.z = 0.01
            top_speed = 5.5
            pose_marker.color.a = 0.5 # Don't forget to set the alpha!
            pose_marker.color.r = 1.0-waypoint.velocity.linear.x/top_speed
            pose_marker.color.g = waypoint.velocity.linear.x/top_speed
            pose_marker.color.b = 0.0

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = waypoints[0].frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + 0.2
            velocity_marker.scale.x = 0.5
            velocity_marker.scale.y = 0.05
            velocity_marker.scale.z = 0.1
            velocity_marker.color.a = 0.5 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            # changes text colour according to fraction of max speed
            # green->red gradient for fast->slow
            top_speed = 5.5
            velocity_marker.color.r = 1.0-waypoint.velocity.linear.x/top_speed
            velocity_marker.color.g = 1.0*(waypoint.velocity.linear.x/top_speed)
            velocity_marker.color.b = 0.0

            markers.append(velocity_marker)

        # Remove extra markers
        for index in range(len(waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'local_waypoints'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(len(waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'velocity'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.local_vis_pub_.publish(marker_array)

    def publish_global(self, waypoints):
        markers = []
        for index, waypoint in enumerate(waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = waypoints[0].frame_id
            pose_marker.ns = 'global_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.CUBE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.1
            pose_marker.scale.y = 0.1
            pose_marker.scale.z = 0.1
            pose_marker.color.a = 1.0 # Don't forget to set the alpha!
            pose_marker.color.r = 1.0
            pose_marker.color.g = 1.0
            pose_marker.color.b = 0.0

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = waypoints[0].frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + 0.2
            velocity_marker.scale.x = 0.5
            velocity_marker.scale.y = 0.05
            velocity_marker.scale.z = 0.1
            velocity_marker.color.a = 0.5 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            # changes text colour according to fraction of max speed
            # green->red gradient for fast->slow
            top_speed = 5.5
            velocity_marker.color.r = 1.0
            velocity_marker.color.g = 1.0
            velocity_marker.color.b = 0.0

            markers.append(velocity_marker)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.vis_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    waypoint_visusaliser = WaypointVisualiser()
    rclpy.spin(waypoint_visusaliser)
    waypoint_visusaliser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
