import rclpy
import numpy as np
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from mcav_interfaces.msg import WaypointArray

from std_msgs.msg import Float32, String

class WaypointVisualiser(Node):

    def __init__(self):
        super().__init__('waypoint_visusaliser')

        self.global_sub = self.create_subscription(WaypointArray,
            'global_waypoints', self.global_callback, 10)
        self.global_sub = self.create_subscription(WaypointArray,
            'local_map_waypoints', self.local_callback, 10)

        self.vis_pub_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 0)
        self.local_vis_pub_ = self.create_publisher(MarkerArray, 'local_visualization_marker_array', 0)
        self.test_pub = self.create_publisher(Float32, 'test_data2', 10)
        self.test_pub3 = self.create_publisher(String, 'test_data3', 10)
        self.test_pub4 = self.create_publisher(String, 'test_data4', 10)

        self.waypointArrayArray = []

        # these keep track of the length of the visualisation marker arrays published
        # so that we can publish the correct number of DELETE markers
        self.local_waypoints_length = 0
        self.global_waypoints_length = 0

        self.text_scale = 0.15
        self.text_offset = self.text_scale * 3 # used to move text so it doesn't overlap waypoints

    def global_callback(self, msg):
        self.publish_global(msg.waypoints)

    def local_callback(self, msg):
        self.publish_local(msg.waypoints)

    def waypoints_filter(self, new_local_waypoints):
        filter_size = 5
        msg = String()
        msg.data = str(new_local_waypoints[0].velocity.linear.x)
        self.test_pub3.publish(msg)
        self.waypointArrayArray.insert(0, new_local_waypoints) # insert new local waypoints into first index of the array
        
        if len(self.waypointArrayArray) == filter_size + 1:
            self.waypointArrayArray.pop(filter_size)
        else:
            return new_local_waypoints

        shifted_waypointArrayArray = [new_local_waypoints]
        x = new_local_waypoints[0].pose.position.x
        y = new_local_waypoints[0].pose.position.y
        for i in range(filter_size - 1):
            wp_arr = self.waypointArrayArray[i+1]
            shifted_wpArr = []
            for j in range(len(wp_arr)):
                if wp_arr[j].pose.position.x == x and wp_arr[j].pose.position.y == y:
                    shifted_wpArr = wp_arr[j:]
                    break
            shifted_waypointArrayArray.append(shifted_wpArr)

        index = 0
        for i in range(len(new_local_waypoints)):
            waypoint_linear_velocity = np.zeros((1,0))
            filter = True
            for j in range(filter_size):
                try:
                    vel = shifted_waypointArrayArray[j][i].velocity.linear.x
                except:
                    filter = False
                    break
                waypoint_linear_velocity = np.append(waypoint_linear_velocity, vel)

            if filter:
                if np.unique(waypoint_linear_velocity).size != 1:
                    index = np.where(waypoint_linear_velocity == np.median(waypoint_linear_velocity))[0][0]
                    msg = String()
                    msg.data = ','.join(str(vel) for vel in waypoint_linear_velocity.tolist()) + "/" + str(index)
                    self.test_pub4.publish(msg)
                    break
            else:
                break

        return self.waypointArrayArray[index]

    def publish_local(self, waypoints):
        waypoints = self.waypoints_filter(waypoints)

        markers = []
        for index, waypoint in enumerate(waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = waypoints[0].frame_id
            pose_marker.ns = 'local_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.SPHERE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.2
            pose_marker.scale.y = 0.2
            pose_marker.scale.z = 0.01
            top_speed = 5.5
            pose_marker.color.a = 0.5 # Don't forget to set the alpha!
            pose_marker.color.r = 1.0-waypoint.velocity.linear.x/top_speed
            pose_marker.color.g = waypoint.velocity.linear.x/top_speed
            pose_marker.color.b = 0.0

            if index == 6:
                msg = Float32()
                msg.data = pose_marker.color.r
                self.test_pub.publish(msg)

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = waypoints[0].frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + self.text_offset 
            velocity_marker.scale.x = self.text_scale
            velocity_marker.scale.y = self.text_scale
            velocity_marker.scale.z = self.text_scale
            velocity_marker.color.a = 0.5 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            # changes text colour according to fraction of max speed
            # green->red gradient for fast->slow
            top_speed = 5.5
            velocity_marker.color.r = 1.0-waypoint.velocity.linear.x/top_speed
            velocity_marker.color.g = 1.0*(waypoint.velocity.linear.x/top_speed)
            velocity_marker.color.b = 0.0

            markers.append(velocity_marker)

        # Remove extra markers
        for index in range(len(waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'local_waypoints'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(len(waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'velocity'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)

        # update record of number of markers published so they can be deleted next time
        self.local_waypoints_length = len(waypoints) 
        
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.local_vis_pub_.publish(marker_array)

    def publish_global(self, waypoints):
        markers = []
        for index, waypoint in enumerate(waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = waypoints[0].frame_id
            pose_marker.ns = 'global_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.CUBE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.1
            pose_marker.scale.y = 0.1
            pose_marker.scale.z = 0.02
            pose_marker.color.a = 1.0 # Don't forget to set the alpha!
            pose_marker.color.r = 1.0
            pose_marker.color.g = 1.0
            pose_marker.color.b = 0.0

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = waypoints[0].frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + self.text_offset
            velocity_marker.scale.x = self.text_scale
            velocity_marker.scale.y = self.text_scale
            velocity_marker.scale.z = self.text_scale
            velocity_marker.color.a = 0.5 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            velocity_marker.color.r = 1.0
            velocity_marker.color.g = 1.0
            velocity_marker.color.b = 0.0

            markers.append(velocity_marker)

        # Remove extra markers
        for index in range(self.global_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'global_waypoints'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(self.global_waypoints_length):
            marker = Marker()
            marker.header.frame_id = waypoints[0].frame_id
            marker.ns = 'velocity'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)

        # update record of number of markers published so they can be deleted next time
        self.local_waypoints_length = len(waypoints) 

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.vis_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    waypoint_visusaliser = WaypointVisualiser()
    rclpy.spin(waypoint_visusaliser)
    waypoint_visusaliser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()