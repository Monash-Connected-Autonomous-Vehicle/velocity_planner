import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcav_interfaces.msg import WaypointArray, DetectedObjectArray
import numpy as np
import math
import tf2_ros
from velocity_planner.vector_ops import q_normalise, qv_mult

class VelocityPlanner(Node):

    def __init__(self):
        super().__init__('velocity_planner')
        timer_period = 0.01  # seconds
        self.spinner = self.create_timer(timer_period, self.spin)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            '/current_pose', self.initial_pose_callback, 10)
        self.initial_pose_sub  # prevent unused variable warning
        self.waypoints_sub = self.create_subscription(WaypointArray,
            '/global_waypoints', self.waypoints_callback, 10)
        self.waypoints_sub  # prevent unused variable warning
        self.objects_sub = self.create_subscription(DetectedObjectArray,
            'detected_objects', self.objects_callback, 10)
        self.objects_sub  # prevent unused variable warning
        self.waypoints_pub = self.create_publisher(WaypointArray, '~/local_waypoints', 10)

        self.declare_parameter('local_plan_max_length', 25) # number of waypoints to plan ahead
        self.declare_parameter('max_acceleration', 0.5) # m/s/waypoint
        self.declare_parameter('obj_waypoint_distance_threshold', 0.4) # if an object is within this distance of a path,
        # it will be considered as blocking the path
        self.declare_parameter('obj_stopping_waypoint_count', 3) # number of waypoints before object to stop at

        # Initialise tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

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
        # Algorithm from https://codereview.stackexchange.com/a/28210
        deltas = waypoint_coords - position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

    def find_object_waypoints(self, waypoints):
        """ Checks if path is blocked by an object. Path is considered blocked if the closest waypoint
        on the path to an object is within a distance threshold, and the object is in front of that waypoint,
        not behind. """
        waypoint_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in waypoints])
        stopping_indices = []
        distance_threshold = self.get_parameter('obj_waypoint_distance_threshold').get_parameter_value().double_value
        stopping_wp_count = self.get_parameter('obj_stopping_waypoint_count').get_parameter_value().integer_value

        for obj in self.objects:
            position = np.array([obj.pose.position.x, obj.pose.position.y])
            nearest_wp = self.find_nearest_waypoint(waypoint_coords, position)
            wp_obj_vec = position - waypoint_coords[nearest_wp, :]
            distance_to_path = np.linalg.norm(wp_obj_vec)
            if distance_to_path < distance_threshold:
                # Check that object is in front of and not behind the nearest waypoint
                # Get vector of direction of the wp
                x_axis = (1.0, 0.0, 0.0)
                wp_quat = waypoints[nearest_wp].pose.orientation
                wp_quat = (wp_quat.w, wp_quat.x, wp_quat.y, wp_quat.z)
                wp_quat = q_normalise(wp_quat)
                wp_x_axis = qv_mult(wp_quat, x_axis)
                # Check that they point the same direction (i.e. dot product is positive)
                dot_prod = np.dot(np.array(wp_x_axis[0:2]), np.array(wp_obj_vec))
                if dot_prod >= 0:
                    stopping_index = max(nearest_wp-stopping_wp_count, 0)
                    stopping_indices.append(stopping_index)

        return stopping_indices

    def slow_to_stop(self, waypoints, stopping_index):
        main_speed = max(wp.velocity.linear.x for wp in waypoints)
        max_accel = self.get_parameter('max_acceleration').get_parameter_value().double_value

        # Gradually slow to a stop with constant deceleration before the stopping waypoint
        slowing_wp_count = min(math.ceil(main_speed / max_accel), len(waypoints))
        slowed_waypoints = waypoints.copy()
        slowing_indices = range(stopping_index, max(-1, stopping_index-slowing_wp_count), -1)
        for i in slowing_indices:
            curr_speed = slowed_waypoints[i].velocity.linear.x
            slower_speed = (stopping_index - i)*max_accel
            slowed_waypoints[i].velocity.linear.x = min(slower_speed, curr_speed, main_speed)
        
        # Zero everything past the stopping waypoint
        for i in range(stopping_index, len(slowed_waypoints)):
            slowed_waypoints[i].velocity.linear.x = 0.0

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

            # Stop for the first detected object that blocks path
            stopping_indices = self.find_object_waypoints(local_waypoints)
            if len(stopping_indices) > 0:
                local_waypoints = self.slow_to_stop(local_waypoints, min(stopping_indices))

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
