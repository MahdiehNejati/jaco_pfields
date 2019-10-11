# object charge (point obstacle: 0 or goal: 1 table: 2, wall: 3, ceiling: 4)
int32 charge

# object center position and orientation 
geometry_msgs/Pose object_pose

# grasp pose orientation 
geometry_msgs/Pose grasp_pose

# object shape type CUBE:1, SPHERE:2, CYLINDER:3
# (See ros visualization_msgs/Marker types http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html) 
int32 type

# shape size
geometry_msgs/Vector3 scale

# marker id
int32 id


