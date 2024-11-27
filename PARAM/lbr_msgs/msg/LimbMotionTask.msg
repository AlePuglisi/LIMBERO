std_msgs/Header header

int64 limb_id  # LF = 0, LH = 1, RH = 2, RF = 3

# swing_duration have to be larger than 100, and have to be able to devided by 100.
# If you want to know the reason, see limbMotionCallback function in Low Level Controller.
int64 swing_duration  # [ms]

# Midpoint of swing.
# If all elements swing_mid_point is zero, it equals to linear motion.
geometry_msgs/Point swing_mid_point # [m]

# Displacement for (x, y, z) in Base coordinate
geometry_msgs/Vector3 end_effector_displacement  # [m]

# Displacement for EE pitch angle
float64 pitch_angle_displacement  # [rad]
