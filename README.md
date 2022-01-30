# Planner

## Inputs
- lanelet2 semantic map
- current location (`geometry_msgs/Pose` on topic `/planner/current_pose`)

## Outputs
- Array of upcoming waypoints beginning at the point closest to the vehicle (`mcav_interfaces/msg/WaypointArray` on topic `/planner/map_waypoints`)