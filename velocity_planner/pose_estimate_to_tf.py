"""
Takes input from the "2D Pose Estimate" button in rviz2 and publishes this as a tf2 transform to the "base_link" frame,
as well as to the /current_pose topic. This is helpful for debugging the planner.
"""
import rclpy
import logging
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class PoseEstimateToTf(Node):

    def __init__(self):
        super().__init__('pose_estimate_to_tf')

        # Subscriber
        timer_period = 0.01  # seconds
        self.spinner = self.create_timer(timer_period, self.broadcast_tf)
        self.initial_pose_sub = self.create_subscription(PoseStamped,
            'initialpose', self.initial_pose_callback, 10)
        self.initial_pose_sub  # prevent unused variable warning

        # Publisher
        self.current_pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_pose = None
        self.current_tf = None
        
        self.get_logger().set_level(logging.DEBUG)

    def broadcast_tf(self):
        if self.current_tf is not None:
            self.current_tf.header.stamp = self.get_clock().now().to_msg()
            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            # Send the transformation
            self.tf_broadcaster.sendTransform(self.current_tf)
            # Publish the pose msg of type PoseStamped
            self.current_pose_pub.publish(self.current_pose)


    def initial_pose_callback(self, pose_msg: PoseStamped):
        t = TransformStamped() # Create an empty message of type TransformStamped

        # Read message content and assign it to
        # corresponding tf variables
        # Publish a transform from map to base_link
        t.header.frame_id = pose_msg.header.frame_id
        t.child_frame_id = 'base_link'

        # Set transform translation to msg pose position
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        # Set transform rotation to pose msg orientation
        t.transform.rotation = pose_msg.pose.orientation

        self.current_tf = t
        self.current_pose = pose_msg


def main(args=None):
    rclpy.init(args=args)
    poseTf = PoseEstimateToTf()
    rclpy.spin(poseTf)
    poseTf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

