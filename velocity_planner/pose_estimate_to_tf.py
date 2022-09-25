from tarfile import LENGTH_NAME
import rclpy
import logging
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from mcav_interfaces.msg import WaypointArray, Waypoint, DetectedObjectArray
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
from velocity_planner.vector_ops import q_normalise, qv_mult

class PoseEstimateToTf(Node):

    def __init__(self):
        super().__init__('pose_estimate_to_tf')

        # Subscriber
        timer_period = 0.01  # seconds
        self.spinner = self.create_timer(timer_period, self.spin)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            'initialpose', self.initial_pose_callback, 10)
        self.initial_pose_sub  # prevent unused variable warning

        # Publisher
        self.current_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'current_pose', 10)
        
        self.get_logger().set_level(logging.DEBUG)


    def initial_pose_callback(self, pose_msg: PoseWithCovarianceStamped):
        t = TransformStamped() # Create an empty message of type TransformStamped

        # Read message content and assign it to
        # corresponding tf variables
        t.header = pose_msg.header
        # Publish a transform from map to base_link
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Set transform translation to msg pose position
        t.transform.translation = pose_msg.pose.pose.position

        # Set transform rotation to pose msg orientation
        t.transform.rotation = pose_msg.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # Publish the pose msg of type PoseWithCovarianceStamped
        self.current_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    poseTf = PoseEstimateToTf()
    rclpy.spin(poseTf)
    poseTf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

