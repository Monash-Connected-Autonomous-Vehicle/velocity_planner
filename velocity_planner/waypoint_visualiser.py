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

    def global_callback(self, msg):
        self.publish_markers(msg.waypoints, 'global_waypoints')

    def local_callback(self, msg):
        self.publish_markers(msg.waypoints, 'local_waypoints')
        print("received waypoints")

    def publish_markers(self, waypoints, namespace: str):
        is_local = namespace == 'local_waypoints'
        markers = []
        for index, waypoint in enumerate(waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = waypoints[0].frame_id
            pose_marker.ns = namespace
            pose_marker.id = index
            pose_marker.type = Marker.SPHERE if is_local else Marker.CUBE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            # length = 0.5
            # if not is_local:
            #     pose_marker.pose.position.x -= length/2.
            pose_marker.scale.x = 0.2 if is_local else 0.05
            pose_marker.scale.y = 0.2 if is_local else 0.05
            pose_marker.scale.z = 0.01 if is_local else 0.05
            pose_marker.color.a = 0.5 if is_local else 1.0 # Don't forget to set the alpha!
            pose_marker.color.r = 0.0 if is_local else 1.0
            pose_marker.color.g = 1.0 if is_local else 0.0
            pose_marker.color.b = 0.0
            markers.append(pose_marker)

            if is_local:
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
                velocity_marker.color.r = 0.0
                velocity_marker.color.g = 1.0
                velocity_marker.color.b = 0.0
                velocity_marker.text = str(waypoint.velocity.linear.x)
                markers.append(velocity_marker)

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
