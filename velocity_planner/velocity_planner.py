import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcav_interfaces.msg import WaypointArray
import numpy as np
import math

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
        # algorithm from https://codereview.stackexchange.com/a/28210
        deltas = self.global_wp_coords - self.position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

    def slow_to_stop(self, waypoints, stopping_index):
        main_speed = max(wp.velocity.linear.x for wp in waypoints)
        max_accel = 0.5 # m/s/waypoint
        slowing_wp_count = math.ceil(main_speed / max_accel)

        slowed_waypoints = waypoints.copy()

        for i in range(stopping_index, stopping_index-slowing_wp_count, -1):
            curr_speed = (stopping_index - i)*max_accel
            slowed_waypoints[i].velocity.linear.x = curr_speed

        return slowed_waypoints

    def spin(self):
        if len(self.position) > 0 and len(self.global_wp_coords) > 0:
            nearest_index = self.find_nearest_waypoint()
            max_index = min(len(self.global_waypoints)-1, nearest_index + self.local_wp_max_length)
            if max_index != nearest_index:
                local_waypoints = self.global_waypoints[nearest_index:max_index]
                local_waypoints = self.slow_to_stop(local_waypoints, len(local_waypoints)-1)
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
