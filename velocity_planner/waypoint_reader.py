""" Node to read waypoints in from a csv file and publish them """
import rclpy
from rclpy.node import Node
import sys
import csv

from mcav_interfaces.msg import Waypoint, WaypointArray

class WaypointReader(Node):

    def __init__(self, waypoint_filename):
        super().__init__('waypoint_reader')
        self.publisher_ = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.init_waypoints(waypoint_filename)
        self.get_logger().info('Publishing "%d" waypoints' % len(self.waypoints))

    def init_waypoints(self, waypoint_filename):
        """ Reads in waypoints from the given csv file and stores them """
        try:
            with open(waypoint_filename) as csvfile:
                csv_reader = csv.DictReader(csvfile, delimiter=',')

                for row in csv_reader:
                    waypoint = Waypoint()
                    waypoint.frame_id = 'map'
                    waypoint.pose.position.x = float(row['x'])
                    waypoint.pose.position.y = -float(row['y'])
                    waypoint.velocity.linear.x = float(row['velocity']) # m/s
                    self.waypoints.append(waypoint)
        except Exception as e:
            print("Unable to read csv file: ")
            print(e)
            sys.exit()

    def timer_callback(self):
        msg = WaypointArray()
        msg.waypoints = self.waypoints
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Error. Syntax is: waypoint_reader <waypoint_file.csv>")
        rclpy.shutdown()
        sys.exit()

    waypoint_reader = WaypointReader(sys.argv[1])
    rclpy.spin(waypoint_reader)
    waypoint_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
