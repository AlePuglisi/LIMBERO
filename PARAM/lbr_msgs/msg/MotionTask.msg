std_msgs/Header header

# This array contains motion task IDs to specify the type of motion task and its order.
int64[] task_id_array

LimbMotionTask limb_motion_task  # ID = 0

BaseMotionTask base_motion_task  # ID = 1

# Limb motion task array is used when executing multiple limb motion tasks simultaniously.
LimbMotionTask[] limb_motion_task_array  # ID = 2;
