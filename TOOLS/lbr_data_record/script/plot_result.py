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

    # Extract joint states and references, ensuring correct parsing
    joints = ["B2C", "C2F", "F2T", "T2E", "wristH", "wristV", "driving"]
    position_reference_pub = dict()
    position_feedback = dict()
    position_references = dict()
    position_error = dict()
    velocity_feedback = dict()
    velocity_references = dict()
    velocity_error = dict()
    #integral = dict()
    #proportional = dict()
    control_torque = dict()
    #gravity_torque = dict()

    for joint in joints:
        position_reference_pub[joint] = df[ joint + '_joint_pub'].tolist()
        position_feedback[joint] = df[ joint + '_feedback_position'].tolist()
        position_references[joint] = df[ joint + '_reference_position'].tolist()
        position_error[joint] = df[ joint + '_position_error'].tolist()

        velocity_feedback[joint] = df[ joint + '_feedback_velocity'].tolist()
        velocity_references[joint] = df[ joint + '_reference_velocity'].tolist()
        velocity_error[joint] = df[ joint + '_velocity_error'].tolist()

        # proportional[joint] = df[ joint + '_proportional_action'].tolist()
        control_torque[joint] = df[ joint + '_torque'].tolist()



        plt.figure( "limb: " + limb + " joint: " + joint + ' Joint State tracking' )

        plt.subplot(2,3,1)
        plt.plot(time, position_reference_pub[joint], 'g', label= joint + ' joint position from lbr')
        plt.plot(time, position_feedback[joint],'b', label= joint + ' joint position feedback')
        plt.plot(time, position_references[joint], 'r--', label= joint + ' joint position reference')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' Joint angle [rad]')
        plt.title(joint + ' Motor Joint Position States and References')
        plt.grid(True)

        plt.subplot(2,3,2)
        plt.plot(time, velocity_feedback[joint],'b', label= joint + ' joint velocity feedback')
        plt.plot(time, velocity_references[joint], 'r--', label= joint + ' joint velocity reference')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( joint + ' Joint velocity [rad/s]')
        plt.title(joint + ' Motor Joint Velocity States and References')
        plt.grid(True)

        plt.subplot(2,3,4)
        plt.plot(time,position_error[joint], 'r', label= joint + ' joint position error')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel( joint + ' position error [rad]')
        plt.title( joint + ' Motor Joint Position error')
        plt.grid(True)

        plt.subplot(2,3,5)
        plt.plot(time,velocity_error[joint],'r', label= joint + ' joint velocity error')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel('velocity error [rad/s]')
        plt.title('Motor Joint Velocity error')
        plt.grid(True)

        plt.subplot(2,3,3)
        plt.plot(time,control_torque[joint],'b', label= joint + ' joint torque')
        plt.legend()
        plt.xlabel('Time [s]')
        plt.ylabel(joint + ' torque [Nm]')
        plt.title(joint + ' Motor Joint torque')
        plt.grid(True)



    plt.show()

if __name__ == '__main__':
    limb = sys.argv[1]
    file_path = "/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_record/script" + "/data_record_" + limb + ".csv"  # Update this path if necessary
    plot_joint_states(file_path)