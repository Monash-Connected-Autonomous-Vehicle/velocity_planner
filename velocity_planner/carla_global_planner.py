#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
import carla
import csv
from rclpy.node import Node
from carla import Transform, Location, Rotation, World
from mcav_interfaces.msg import WaypointArray, Waypoint
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

total_waypoints = 210 # variable
waypoint_distance = 2 # distance between waypoints (m)
target_vel = 4.3 # target velocity (m/s) TODO: are these units correct?

class WaypointPublisher(Node):


    def __init__(self):
        super().__init__('carla_global_planner')
        # subscribers
        self.local_wp_sub = self.create_subscription(
                             WaypointArray, 'local_map_waypoints', self.show_local_wp, 10)
        self.local_wp_sub  # prevent 'unused variable' warning
        self.global_wp_sub = self.create_subscription(
                             WaypointArray, 'global_waypoints', self.show_global_wp, 10)
        self.global_wp_sub  # prevent 'unused variable' warning
        
        # publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'current_pose', 1)
        self.waypoints_pub = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.spinner = self.create_timer(timer_period, self.waypoints_callback)
        
        # function variables
        self.debug = None
        self.map = None
        self.vehicle = None
        self.vehicle_pos = None
        self.path = None


    # Main callback function for publishing waypoints
    def waypoints_callback(self):
        self.vehicle_pos = self.vehicle.get_transform() # record vehicle pose on each loop 
        pose_msg = self.pose2msg()
        (self.pose_pub).publish(pose_msg)
        
        # TODO: comment this section out if using other means of publishing global waypoints
        # wp_list_msg = WaypointArray()
        # wp_list_msg.waypoints = self.waypoints2msg(self.path) # converts to WaypointArray message
        # (self.waypoints_pub).publish(wp_list_msg)


    # Generates waypoint path in CARLA
    def gen_waypoints(self):      
        wp_start = (self.map).get_waypoint(self.vehicle.get_transform().location) # generates a waypoint at vehicle start location
        wp_next = wp_start
        wp_list = [None]*(total_waypoints)
        
        for i in range(len(wp_list)):
            wp_list[i] = wp_next
            # chooses the first waypoint in a list of waypoints that are a distance away
            wp = wp_next.next(waypoint_distance)
            wp_next = wp[-1]

        return wp_list
    
    
    # Converts waypoint to Waypoint message
    def pose2msg(self):
        rad_factor = math.pi/180
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.vehicle_pos.location.x
        pose_msg.pose.pose.position.y = -self.vehicle_pos.location.y
        pose_msg.pose.pose.position.z = self.vehicle_pos.location.z 
        pose_msg.pose.pose.orientation.z = -self.vehicle_pos.rotation.yaw*rad_factor
          
        return pose_msg
    
    
    # Converts waypoint to Waypoint message
    def waypoints2msg(self, waypoints):
        rad_factor = math.pi/180
        wp_msg_list = [None]*len(waypoints)

        for i in range(len(wp_msg_list)):
            v_yaw = self.vehicle_pos.rotation.yaw
            v_x = self.vehicle_pos.location.x
            v_y = self.vehicle_pos.location.y
            wp_x = waypoints[i].transform.location.x
            wp_y = waypoints[i].transform.location.y
            wp_z = waypoints[i].transform.location.z
            wp_yaw = waypoints[i].transform.rotation.yaw
            
            # transform waypoint coordinates to vehicle frame        
            wp_msg = Waypoint()
            wp_msg.frame_id = "map"
            wp_msg.pose.position.x = wp_x
            wp_msg.pose.position.y = -wp_y
            wp_msg.pose.position.z = wp_z
            wp_msg.pose.orientation.z = -wp_yaw*rad_factor
            
            # waypoint velocity is zero upon reaching last waypoint
            if i == len(wp_msg_list)-1:
                wp_msg.velocity.linear.x = 0.0
            else:
                wp_msg.velocity.linear.x = target_vel
                
            wp_msg_list[i] = wp_msg  
          
        return wp_msg_list


    # Visualise local waypoints in CARLA    
    def show_local_wp(self, wp_msg_list):
        self.draw_waypoints(wp_msg_list, carla.Color(255, 0, 0)) 


    # Visualise global waypoints in CARLA
    def show_global_wp(self, wp_msg_list):
        self.draw_waypoints(wp_msg_list, carla.Color(0, 0, 255)) 
        
        
    # Visualises waypoint paths    
    def draw_waypoints(self, wp_msg_list, color):
        wp_list = []
        for wp_msg in wp_msg_list.waypoints:
            wp_location = Location(x=wp_msg.pose.position.x,        
                                   y=-wp_msg.pose.position.y, 
                                   z=wp_msg.pose.position.z)
            wp_list.append(wp_location)
    
        for i in wp_list:
            (self.debug).draw_point(i + carla.Location(z=0.25), 0.05, color, False)
            
          
    # Save path as .csv  
    def save_path(self, waypoints):
        with open('./paths/town02_path.csv', 'w', encoding='UTF8', newline='') as f:
            fieldnames = ['x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for wp in waypoints:
                wp_x = wp.transform.location.x
                wp_y = -wp.transform.location.y
                wp_z = wp.transform.location.z
                wp_yaw = -wp.transform.rotation.yaw
                wp_v = target_vel
                writer.writerow({'x': wp_x, 
                                 'y': wp_y, 
                                 'z': wp_z, 
                                 'yaw': wp_yaw, 
                                 'velocity': wp_v, 
                                 'change_flag': 0})


def main(args=None):
    # node initialisation
    rclpy.init(args=args)
    wp_pub = WaypointPublisher()

    try:
        client = carla.Client('localhost', 2000)  # create a client to communicate with the server
        world = client.get_world()
        vehicle = None
        while vehicle == None:
            for actor in world.get_actors():
                try:
                    if actor.attributes["role_name"] == "ego_vehicle": # finds main car
                        vehicle = actor
                        break 
                    else:
                        print("Unable to find vehicle. Make sure you have spawned a vehicle with role_name 'ego_vehicle'.")
                except KeyError:
                    print("Unable to find vehicle. Make sure you have spawned a vehicle with role_name 'ego_vehicle'.")
                
        #set node parameters and spin
        wp_pub.debug = world.debug
        wp_pub.map = world.get_map()
        wp_pub.vehicle = vehicle
        wp_pub.path = wp_pub.gen_waypoints()
        wp_pub.save_path(wp_pub.path)
        rclpy.spin(wp_pub)

    except KeyboardInterrupt:
        # https://github.com/carla-simulator/carla/issues/1346
        print("User Cancelled")
    finally:
        print("shutting down node")
        wp_pub.destroy_node()
        rclpy.shutdown()
        print("done.")


if __name__ == '__main__':
    main()
