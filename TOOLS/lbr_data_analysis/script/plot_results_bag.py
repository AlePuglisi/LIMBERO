import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_idl

def read_bag_data(bag_path, topic_name):
    # Open the ROS 2 bag file for reading
    with Reader(bag_path) as reader:
        # Iterate through all connections in the bag file
        messages = []
        for connection, timestamp, raw_data in reader.messages():
            # Check if the connection is for the desired topic
            if connection.topic == topic_name:
                # Deserialize messages from the topic
                messages.append(deserialize_cdr(raw_data, connection.msgtype))

        return messages

def plot_limb_controller_joint_states(control_messages, joint_states, limb):

    positions_limb_control = []
    positions_grieel_control = []

    limb_states = []
    limb_grieel_states = []

    t0 = control_messages[0].header.stamp.sec + control_messages[0].header.stamp.nanosec * 1e-9
    for msg in control_messages:
        # positions, and velocities from each point
        positions_limb_control.append(msg.position[:4])
        positions_grieel_control.append(msg.position[4:7])

    t0 = joint_states[0].header.stamp.sec + joint_states[0].header.stamp.nanosec * 1e-9    
    times = []

    for msg in joint_states: 
        i_B2C = msg.name.index(limb+"_B2C")
        i_C2F = msg.name.index(limb+"_C2F")
        i_F2T = msg.name.index(limb+"_F2T")
        i_T2E = msg.name.index(limb+"_T2E")
        i_wristH = msg.name.index(limb+"_wristH")
        i_wristV = msg.name.index(limb+"_wristV")
        i_driving = msg.name.index(limb+"_driving")
        time = ((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)- t0)
        times.append(time)

        index = [i_B2C, i_C2F, i_F2T, i_T2E, i_wristH, i_wristV, i_driving]

        limb_joint = []
        grieel_joint = []
        for i in index[0:4]: 
            limb_joint.append(msg.position[i])
        for i in index[4:7]: 
            grieel_joint.append(msg.position[i])

        limb_states.append(limb_joint)
        limb_grieel_states.append(grieel_joint)


    # Convert lists to numpy arrays for easy manipulation
    times = np.array(times)
    positions_limb_control = np.array(positions_limb_control)
    positions_grieel_control = np.array(positions_grieel_control)
    limb_states = np.array(limb_states)
    limb_grieel_states = np.array(limb_grieel_states)

    print(positions_limb_control)
    print(len(positions_limb_control))
    print(limb_states)
    print(len(limb_states))
  

    # Plotting positions and velocities
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(times, positions_limb_control, limb_states, label=["B2C_control","C2F_control", "F2T_control", "T2E_control"])#, "B2C_state", "C2F_state", "F2T_state", "T2E_state"])
    plt.title('Limb Joint Positions control')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, positions_grieel_control,limb_grieel_states, label=["wristH_control", "wristV_control", "driving_control"])#,"wristH_state", "wristV_state", "driving_state" ])
    plt.title('Grieel Joint Positions control')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Define the bag file path and topic name
bag_path = 'bag/grieel_bag'
topic_LF = '/lbr_data_analysis/current_joint'
topic_joint_states = "/lbr_data_analysis/reference_joint"

# Read data from the bag file
LF_limb_controller_joint_state = read_bag_data(bag_path, topic_LF)
joint_states = read_bag_data(bag_path, topic_joint_states)

# Plot the joint trajectory data
plot_limb_controller_joint_states(LF_limb_controller_joint_state, joint_states, "LF")
