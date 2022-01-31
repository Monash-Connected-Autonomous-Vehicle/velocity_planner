# 🧶 Velocity Planner

## Inputs
- current location (`geometry_msgs/Pose` on topic `/planner/current_pose`)
- global waypoints (`mcav_interfaces/msg/WaypointArray` on topic `/planner/global_waypoints`)

## Outputs
- Array of upcoming waypoints beginning at the point closest to the vehicle (`mcav_interfaces/msg/WaypointArray` on topic `/planner/local_waypoints`)

![View in rviz of the planner output](screenshots/planner.png)
Red: global waypoints, Purple: current pose, Green: local planned waypoints