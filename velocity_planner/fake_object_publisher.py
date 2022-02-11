import rclpy
from rclpy.node import Node
import numpy as np

from mcav_interfaces.msg import DetectedObject, DetectedObjectArray

class FakeObjects(Node):

    def __init__(self):
        super().__init__('fake_object_publisher')
        self.publisher_ = self.create_publisher(DetectedObjectArray, 'detected_objects', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.detected_objects = []
        self.init_detected_objects()

    def init_detected_objects(self):
        """ Modifies self.detected_objects """
        x = np.arange(0.0, 5.0, 1.0)
        y = -1.5*np.sin(2*x)

        for i in range(len(x)):
            obj = DetectedObject()
            obj.object_class = DetectedObject.CLASS_PERSON
            obj.dimensions.x = 0.2
            obj.dimensions.y = 0.3
            obj.dimensions.z = 1.5
            obj.frame_id = 'map'
            obj.pose.position.x = x[i]
            obj.pose.position.y = y[i]
            self.detected_objects.append(obj)

    def timer_callback(self):
        msg = DetectedObjectArray()
        msg.detected_objects = self.detected_objects
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing "%d" detected_objects' % len(msg.detected_objects))

def main(args=None):
    rclpy.init(args=args)

    fake_object_publisher = FakeObjects()
    rclpy.spin(fake_object_publisher)
    fake_object_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
