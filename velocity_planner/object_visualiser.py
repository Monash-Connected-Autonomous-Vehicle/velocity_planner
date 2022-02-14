import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from mcav_interfaces.msg import DetectedObjectArray

class ObjectVisualiser(Node):

    def __init__(self):
        super().__init__('object_visualiser')

        self.global_sub = self.create_subscription(DetectedObjectArray,
            'detected_objects', self.object_callback, 10)

        self.objects_vis_pub_ = self.create_publisher(MarkerArray, 'detected_objects_marker_array', 0)

    def object_callback(self, msg):
        self.publish_markers(msg.detected_objects)

    def publish_markers(self, objects):
        # TODO: expand this to differentiate between classes of objects
        markers = []
        for index, object in enumerate(objects):
            obj_marker = Marker()
            obj_marker.header.frame_id = objects[0].frame_id
            obj_marker.ns = 'detected_objects'
            obj_marker.id = index
            obj_marker.type = Marker.CUBE
            obj_marker.action = Marker.ADD
            obj_marker.pose = object.pose
            obj_marker.pose.position.z += object.dimensions.z/4
            obj_marker.scale.x = object.dimensions.x/2
            obj_marker.scale.y = object.dimensions.y/2
            obj_marker.scale.z = object.dimensions.z/2
            obj_marker.color.a = 1.0 # Don't forget to set the alpha!
            obj_marker.color.r = 1.0
            obj_marker.color.g = 1.0
            obj_marker.color.b = 1.0

            markers.append(obj_marker)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.objects_vis_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    object_visualiser = ObjectVisualiser()
    rclpy.spin(object_visualiser)
    object_visualiser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
