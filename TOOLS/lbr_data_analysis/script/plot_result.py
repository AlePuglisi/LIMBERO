import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_joint_states(file_path):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)
    
    # Extract and normalize the 'time' column
    time = df['time'].tolist()
    time = [t - time[0] for t in time]  # Normalize time by subtracting the first element
    
    # Extract joint names from the first row
    joint_names = df['joint_name'].iloc[0].split(', ')
    
    # Extract joint states and references, ensuring correct parsing
    positions = df['joint_state'].apply(lambda x: list(map(float, x.split(', '))))
    references = df['joint_reference'].apply(lambda x: list(map(float, x.split(','))))

    torques = df['joint_torque'].apply(lambda x: list(map(float, x.split(','))))
    
    # Convert positions and references into DataFrames
    positions_df = pd.DataFrame(positions.tolist(), columns=joint_names)
    references_df = pd.DataFrame(references.tolist(), columns=joint_names)
    torques_df = pd.DataFrame(torques.tolist(), columns=joint_names)
    
    # Plot each joint's state and reference
    [i_LF,i_LH,i_RH,i_RF] = [1,1,1,1]
    for joint_name in joint_names:
        if joint_name[0:2] == 'LF':
            plt.figure('LF Joints')
            plt.subplot(2,4,i_LF)
            plt.plot(time, positions_df[joint_name].to_numpy(), label=f'{joint_name} (State)')
            plt.plot(time, references_df[joint_name].to_numpy(), 'r--', label=f'{joint_name} (Reference)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Joint')
            plt.title(joint_name + ' Joint States and References')
            plt.grid(True)
            
            plt.figure('LF Torque')
            plt.subplot(2,4,i_LF)
            plt.plot(time, torques_df[joint_name].to_numpy(), label=f'{joint_name} (Torque)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Torque [Nm]')
            plt.title(joint_name + ' control Torque ')
            plt.grid(True)

            i_LF+=1
            
        if joint_name[0:2] == 'LH':
            plt.figure('LH Joints')
            plt.subplot(2,4,i_LH)
            plt.plot(time, positions_df[joint_name].to_numpy(), label=f'{joint_name} (State)')
            plt.plot(time, references_df[joint_name].to_numpy(), 'r--', label=f'{joint_name} (Reference)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Joint')
            plt.title(joint_name + ' Joint States and References')
            plt.grid(True)

            plt.figure('LH Torque')
            plt.subplot(2,4,i_LH)
            plt.plot(time, torques_df[joint_name].to_numpy(), label=f'{joint_name} (Torque)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Torque [Nm]')
            plt.title(joint_name + ' control Torque ')
            plt.grid(True)

            i_LH+=1

        if joint_name[0:2] == 'RH':
            plt.figure('RH Joints')
            plt.subplot(2,4,i_RH)
            plt.plot(time, positions_df[joint_name].to_numpy(), label=f'{joint_name} (State)')
            plt.plot(time, references_df[joint_name].to_numpy(), 'r--', label=f'{joint_name} (Reference)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Joint')
            plt.title(joint_name + ' Joint States and References')
            plt.grid(True)

            plt.figure('RH Torque')
            plt.subplot(2,4,i_RH)
            plt.plot(time, torques_df[joint_name].to_numpy(), label=f'{joint_name} (Torque)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Torque [Nm]')
            plt.title(joint_name + ' control Torque ')
            plt.grid(True)

            i_RH+=1

        if joint_name[0:2] == 'RF':
            plt.figure('RF Joints')
            plt.subplot(2,4,i_RF)
            plt.plot(time, positions_df[joint_name].to_numpy(), label=f'{joint_name} (State)')
            plt.plot(time, references_df[joint_name].to_numpy(), 'r--', label=f'{joint_name} (Reference)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Joint')
            plt.title(joint_name + ' Joint States and References')
            plt.grid(True)

            plt.figure('RF Torque')
            plt.subplot(2,4,i_RF)
            plt.plot(time, torques_df[joint_name].to_numpy(), label=f'{joint_name} (Torque)')
            plt.legend()
            plt.xlabel('Time (seconds)')
            plt.ylabel('Torque [Nm]')
            plt.title(joint_name + ' control Torque ')
            plt.grid(True)

            i_RF+=1

    plt.show()

if __name__ == '__main__':
    file_path = '/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_analysis/script/lbr_data.csv'  # Update this path if necessary
    plot_joint_states(file_path)
