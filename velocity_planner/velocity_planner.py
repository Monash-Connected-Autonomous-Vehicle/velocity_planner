import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcav_interfaces.msg import WaypointArray, DetectedObjectArray
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
        self.objects_sub = self.create_subscription(DetectedObjectArray,
            'detected_objects', self.objects_callback, 10)
        self.objects_sub  # prevent unused variable warning
        self.waypoints_pub = self.create_publisher(WaypointArray, '~/local_waypoints', 10)

        self.declare_parameter('local_plan_max_length', 20) # number of waypoints to plan ahead
        self.declare_parameter('max_acceleration', 0.5) # m/s/waypoint
        self.declare_parameter('obj_waypoint_distance_threshold', 0.6) # if an object is within this distance of a path,
        # it will be considered as blocking the path
        self.declare_parameter('obj_stopping_waypoint_count', 3) # number of waypoints before object to stop at

        self.position = np.array([])
        self.global_waypoints = []
        self.global_wp_coords = np.array([])
        self.objects = []

    def initial_pose_callback(self, pose_msg: PoseWithCovarianceStamped):
        self.position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])

    def waypoints_callback(self, msg: WaypointArray):
        # TODO: transform so that they can be in different coordinate systems
        self.global_waypoints = msg.waypoints
        self.global_wp_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in msg.waypoints])

    def objects_callback(self, msg: DetectedObjectArray):
        self.objects = msg.detected_objects

    def find_nearest_waypoint(self, waypoint_coords, position) -> int:
        """ Inputs: position 1x2 numpy array [x, y] """
        # algorithm from https://codereview.stackexchange.com/a/28210
        deltas = waypoint_coords - position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

    def find_object_waypoints(self, waypoint_coords):
        # TODO: only if object is in front, not behind
        stopping_indices = []
        distance_threshold = self.get_parameter('obj_waypoint_distance_threshold').get_parameter_value().double_value
        stopping_wp_count = self.get_parameter('obj_stopping_waypoint_count').get_parameter_value().integer_value

        for obj in self.objects:
            position = np.array([obj.pose.position.x, obj.pose.position.y])
            nearest_wp = self.find_nearest_waypoint(waypoint_coords, position)
            distance_to_path = np.linalg.norm(waypoint_coords[nearest_wp, :] - position)
            if distance_to_path < distance_threshold:
                stopping_index = max(nearest_wp-stopping_wp_count, 0)
                stopping_indices.append(stopping_index)

        return stopping_indices

    def slow_to_stop(self, waypoints, stopping_index):
        main_speed = max(wp.velocity.linear.x for wp in waypoints)
        max_accel = self.get_parameter('max_acceleration').get_parameter_value().double_value
        slowing_wp_count = min(math.ceil(main_speed / max_accel), len(waypoints))

        slowed_waypoints = waypoints.copy()

        slowing_indices = range(stopping_index, max(-1, stopping_index-slowing_wp_count), -1)
        for i in slowing_indices:
            curr_speed = slowed_waypoints[i].velocity.linear.x
            slower_speed = (stopping_index - i)*max_accel
            slowed_waypoints[i].velocity.linear.x = min(slower_speed, curr_speed, main_speed)

        return slowed_waypoints

    def spin(self):
        if len(self.position) > 0 and len(self.global_wp_coords) > 0:
            nearest_index = self.find_nearest_waypoint(self.global_wp_coords, self.position)

            # Stop at the end of the global waypoints
            slowed_global = self.slow_to_stop(self.global_waypoints, len(self.global_waypoints)-1)

            # Extract up to local_plan_max_length waypoints as the local plan
            local_plan_max_length = self.get_parameter('local_plan_max_length').get_parameter_value().integer_value
            final_wp_index = min(len(self.global_waypoints)-1, nearest_index + local_plan_max_length - 1)
            local_waypoints = slowed_global[nearest_index:final_wp_index+1]

            # Stop for detected objects
            local_waypoint_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in local_waypoints])
            stopping_indices = self.find_object_waypoints(local_waypoint_coords)
            for wp_index in stopping_indices:
                local_waypoints = self.slow_to_stop(local_waypoints, wp_index)

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
