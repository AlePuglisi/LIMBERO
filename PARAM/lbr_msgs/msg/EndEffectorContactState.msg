std_msgs/Header header

# End-effector contact state
bool[] is_contact  # true -> contact, false -> not contact

# End-effector position
geometry_msgs/Point[] end_effector_position  # (x, y, z)