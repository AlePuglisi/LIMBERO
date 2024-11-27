import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def plot_joint_states(file_path):

    limb = sys.argv[1]

    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Extract and normalize the 'time' column
    time = df['time'].tolist()
    time = [t - time[0] for t in time]  # Normalize time by subtracting the first element
    limb_contact = df['limb_in_contact'].tolist()

    # Extract joint states and references, ensuring correct parsing
    joints = ["B2C", "C2F", "F2T", "T2E", "wristH", "wristV", "driving"]
    position = dict()
    position_references_filtered = dict()
    position_references = dict()
    position_error = dict()
    velocity = dict()
    velocity_references = dict()
    velocity_feed_forward = dict()
    velocity_error = dict()
    integral = dict()
    #proportional = dict()
    control_torque = dict()
    gravity_torque = dict()

    for joint in joints:
        position[joint] = df[ joint + '_current_position'].tolist()
        position_references_filtered[joint] = df[ joint + '_reference_position_filtered'].tolist()
        position_references[joint] = df[ joint + '_reference_position'].tolist()
        position_error[joint] = df[ joint + '_position_error'].tolist()
        velocity[joint] = df[ joint + '_current_velocity'].tolist()
        velocity_references[joint] = df[ joint + '_reference_velocity'].tolist()
        velocity_feed_forward[joint] = df[ joint + '_feed_forward_velocity'].tolist()
        velocity_error[joint] = df[ joint + '_velocity_error'].tolist()
        integral[joint] = df[ joint + '_integral_action'].tolist()
        # proportional[joint] = df[ joint + '_proportional_action'].tolist()
        control_torque[joint] = df[ joint + '_control_torque'].tolist()
        gravity_torque[joint] = df[ joint + '_gravity_torque'].tolist()



        plt.figure( "limb: " + limb + " joint: " + joint + ' Joint State tracking' )

        plt.subplot(3,3,1)
        plt.plot(time, position[joint],'b', label= joint + ' joint position')
        plt.plot(time, position_references[joint], 'r--', label= joint + ' joint position reference')
        plt.plot(time, position_references_filtered[joint], 'g--', label= joint + ' joint position reference filtered')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' Joint angle [rad]')
        plt.title(joint + ' Motor Joint Position States and References')
        plt.grid(True)

        plt.subplot(3,3,2)
        plt.plot(time, velocity[joint],'b', label= joint + ' joint velocity')
        plt.plot(time, velocity_references[joint], 'r--', label= joint + ' joint velocity reference')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( joint + ' Joint velocity [rad/s]')
        plt.title(joint + ' Motor Joint Velocity States and References')
        plt.grid(True)

        plt.subplot(3,3,3)
        plt.plot(time, velocity_feed_forward[joint], 'g', label= joint + ' joint velocity feed forward')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' Joint velocity feed forward [rad/s]')
        plt.title( joint + ' Motor Joint Velocity Feed Forward')
        plt.grid(True)

        plt.subplot(3,3,4)
        plt.plot(time,position_error[joint], 'r', label= joint + ' joint position error')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( joint + ' position error [rad]')
        plt.title( joint + ' Motor Joint Position error')
        plt.grid(True)

        plt.subplot(3,3,5)
        plt.plot(time,velocity_error[joint],'r', label= joint + ' joint velocity error')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel('velocity error [rad/s]')
        plt.title('Motor Joint Velocity error')
        plt.grid(True)

        plt.subplot(3,3,6)
        plt.plot(time,integral[joint],'b', label= joint + ' integral action')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' integral action torque [Nm]')
        plt.title(joint + ' Motor Joint integral torque')
        plt.grid(True)

        plt.subplot(3,3,7)
        plt.plot(time,control_torque[joint],'b', label= joint + ' joint torque')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' torque [Nm]')
        plt.title(joint + ' Motor Joint torque')
        plt.grid(True)

        plt.subplot(3,3,8)
        plt.plot(time,gravity_torque[joint],'b', label= joint + ' gravity compensation action')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( joint + ' gravity compensation [Nm]')
        plt.title(joint + ' Motor Joint gravitational torque')
        plt.grid(True)

        plt.subplot(3,3,9)
        plt.plot(time,limb_contact,'b', label= 'limb contact state')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( ' limb contact [1:true, 0:false]')
        plt.title('limb contact state')
        plt.grid(True)


    plt.show()

if __name__ == '__main__':
    limb = sys.argv[1]
    file_path = "/home/ale/lbr_ws/src/LIMBERO/CONTROL/lbr_indipendent_joint_controller/" + limb + "_data_record.csv"  # Update this path if necessary
    plot_joint_states(file_path)
