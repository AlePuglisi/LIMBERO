std_msgs/Header header

# motion_duration have to be larger than 100, and have to be able to devided by 100.
# If you want to know the reason, see baseMotionCallback function in Low Level Controller.
int64 motion_duration  # [ms]

# Displacement for (x, y, z) in Base coordinate
geometry_msgs/Vector3 base_displacement  # [m]

# Displacement for (roll, pitch, yaw) in Base coordinate
geometry_msgs/Vector3 base_angle_displacement  # [rad]
