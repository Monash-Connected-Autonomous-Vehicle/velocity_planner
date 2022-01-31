import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcav_interfaces.msg import WaypointArray
import numpy as np

class VelocityPlanner(Node):

    def __init__(self):
        super().__init__('velocity_planner')
        timer_period = 0.01  # seconds
        self.spinner = self.create_timer(timer_period, self.spin)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            '/initialpose', self.initial_pose_callback, 10)
        self.initial_pose_sub  # prevent unused variable warning
        self.waypoints_sub = self.create_subscription(WaypointArray,
            '/global_waypoints', self.waypoints_callback, 10)
        self.waypoints_sub  # prevent unused variable warning
        self.waypoints_pub = self.create_publisher(WaypointArray, '~/local_waypoints', 10)

        self.local_wp_max_length = 40 # TODO: make ros param

        self.position = np.array([])
        self.global_waypoints = []
        self.global_wp_coords = np.array([])

    def initial_pose_callback(self, pose_msg: PoseWithCovarianceStamped):
        self.position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.get_logger().info('Received: "%s"' % pose_msg.pose.pose)

    def waypoints_callback(self, msg: WaypointArray):
        # TODO: transform so that they can be in different coordinate systems
        self.global_waypoints = msg.waypoints
        self.global_wp_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in msg.waypoints])

    def find_nearest_waypoint(self) -> int:
        deltas = self.global_wp_coords - self.position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

    def spin(self):
        if len(self.position) > 0 and len(self.global_wp_coords) > 0:
            nearest_index = self.find_nearest_waypoint()
            max_index = min(len(self.global_waypoints)-1, nearest_index + self.local_wp_max_length)
            if max_index != nearest_index:
                local_waypoints = self.global_waypoints[nearest_index:max_index]
                local_wp_msg = WaypointArray()
                local_wp_msg.waypoints = local_waypoints
                self.waypoints_pub.publish(local_wp_msg)


def main(args=None):
    rclpy.init(args=args)

    planner = VelocityPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
