import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPose(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose_with_covariance', 10)
        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # self.get_logger().info('Publishing "%d" detected_objects' % len(self.detected_objects))

        self.start_time = self.get_clock().now().nanoseconds


    def timer_callback(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(pose)
        

def main(args=None):
    rclpy.init(args=args)

    initial_pose_publisher = InitPose()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
