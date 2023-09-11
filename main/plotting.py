import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import *
import os
from scipy.spatial.distance import euclidean
from scipy.spatial.distance import cdist
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
from fastdtw import fastdtw


script_path = os.path.dirname(os.path.abspath(__file__))
folder_path = script_path+'/log/us2/contactsos/'
dataframes = []
subdirectories = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]
sorted_subdirectories = sorted(subdirectories, key=lambda x: int(x))
avg_power_list = []
avg_curr_list = []
avg_vol_list = []
avg_torque_list =[]
combined_df =[]
index = '40'
# Iterate through the sorted subdirectories
combine = False
loop_all =False
if loop_all:
    for folder_name in sorted_subdirectories:
        folder_dir = os.path.join(folder_path, folder_name)
        files = os.listdir(folder_dir)
        csv_files = [f for f in files if f.endswith('.csv')]
        
        if csv_files:
            csv_file_path = os.path.join(folder_dir, csv_files[0])
            df = pd.read_csv(csv_file_path,index_col=False)
            dataframes.append(df)
            df = df.astype(float)
            if combine:
            # Combine all dataframes into a single dataframe if needed
                combined_df = pd.concat(dataframes, ignore_index=True)

        # avg_power_list.append(data['actuar_voltage_1'] * data['actuar_current_1'])
        # df['actuar_voltage_1'].mean(axis=0,skipna=True)
else:
    folder_dir = folder_path + index
    files = os.listdir(folder_dir)
    csv_file_path = os.path.join(folder_dir ,[f for f in files if f.endswith('.csv')][0])
    
    df = pd.read_csv(csv_file_path,index_col=False)
    dataframes.append(df)
    df = df.astype(float)
# avg_power_df = combined_df.actuar_voltage_0 * combined_df.actuar_current_0
# avg_curr_list.append(combined_df['actuar_current_0'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_0'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_0'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_1 * combined_df.actuar_current_1
# avg_curr_list.append(combined_df['actuar_current_1'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_1'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_1'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_2 * combined_df.actuar_current_2
# avg_curr_list.append(combined_df['actuar_current_2'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_2'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_2'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_3 * combined_df.actuar_current_3
# avg_curr_list.append(combined_df['actuar_current_3'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_3'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_3'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_4 * combined_df.actuar_current_4
# avg_curr_list.append(combined_df['actuar_current_4'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_4'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_4'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_5 * combined_df.actuar_current_5
# avg_curr_list.append(combined_df['actuar_current_5'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_5'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_5'].mean(axis=0,skipna=True))

# avg_power_df = combined_df.actuar_voltage_6 * combined_df.actuar_current_6
# avg_curr_list.append(combined_df['actuar_current_6'].mean(axis=0,skipna=True))
# avg_vol_list.append(combined_df['actuar_voltage_6'].mean(axis=0,skipna=True))
# avg_power_list.append(avg_power_df.mean(axis=0,skipna=True))
# avg_torque_list.append(combined_df['joint_torque_6'].mean(axis=0,skipna=True))
# # Combine all dataframes into a single dataframe if needed
# # combined_df = pd.concat(dataframes, ignore_index=True)

# print("power: ",avg_power_list)
# print("avg power",np.array(avg_power_list).mean())
# print("current:",avg_curr_list)
# print("voltage: ",avg_vol_list)
# print("torque: ",avg_torque_list)
# print("avg torque",np.array(avg_torque_list).mean())
# # csvfile = pd.read_csv(csv_file_path,index_col=False)
# # combined_df['actuar_voltage_1'] = savgol_filter(combined_df['actuar_voltage_1'], 10000,2)
# # combined_df.plot(x = 'Index',y='actuar_voltage_1')
# # plt.show()
# # actuar_voltage_6
# # print(combined_df['actuar_voltage_6'])


measured_trajectory_contact = np.array(df[['current_pos_x', 'current_pos_y']].values.tolist())
window_size = 500  # Adjust as needed
poly_order = 2   # Polynomial order, typically 2 or 3
# reference_trajectory = np.array(df[['target_pos_x', 'target_pos_y']].values.tolist())
folder_path2 = script_path+'/log/us2/contactless/'
subdirectories2 = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]
sorted_subdirectories = sorted(subdirectories, key=lambda x: int(x))
dataframes2 = []
combined_df2 = []
combine = False
loop_all = False
if loop_all:
    for folder_name in sorted_subdirectories:
        folder_dir = os.path.join(folder_path2, folder_name)
        files = os.listdir(folder_dir)
        csv_files = [f for f in files if f.endswith('.csv')]
        
        if csv_files:
            csv_file_path = os.path.join(folder_dir, csv_files[0])
            df2 = pd.read_csv(csv_file_path,index_col=False)
            dataframes2.append(df)
            df2 = df2.astype(float)
            if combine:
            # Combine all dataframes into a single dataframe if needed
                combined_df2 = pd.concat(dataframes2, ignore_index=True)
else:
    folder_dir = folder_path2 + index
    files = os.listdir(folder_dir)
    csv_file_path = os.path.join(folder_dir ,[f for f in files if f.endswith('.csv')][0])
    
    df2 = pd.read_csv(csv_file_path,index_col=False)
    dataframes2.append(df2)
    df2 = df2.astype(float)

folder_path3 = script_path+'/log/us2/contactlessos/'
folder_dir = folder_path3 + index
files = os.listdir(folder_dir)
csv_file_path = os.path.join(folder_dir ,[f for f in files if f.endswith('.csv')][0])
dataframes3 = []
df3 = pd.read_csv(csv_file_path,index_col=False)
dataframes3.append(df3)
df3 = df3.astype(float)

measured_trajectory_contactless = np.array(df3[['current_pos_x', 'current_pos_y']].values.tolist())
print(df2[['target_pos_x', 'target_pos_y']])
reference_trajectory = np.array(df2[['target_pos_x', 'target_pos_y']].values.tolist())
reference_trajectory[0][0] = reference_trajectory[0][0] -0.1
reference_trajectory[:,1] = reference_trajectory[:,1] +0.0005
x_reference, y_reference = reference_trajectory.T
x_measured_contact, y_measured_contact = measured_trajectory_contact.T
x_measured2, y_measured2 = measured_trajectory_contactless.T

smoothed_x = savgol_filter(x_measured_contact, window_size, poly_order)
smoothed_y = savgol_filter(y_measured_contact, window_size, poly_order)

smoothed_x2 = savgol_filter(x_measured2, window_size, poly_order)
smoothed_y2 = savgol_filter(y_measured2, window_size, poly_order)

min_length_contact= min(len(reference_trajectory), len(measured_trajectory_contact))
min_length_contactless= min(len(reference_trajectory), len(measured_trajectory_contactless))
shorter_reference_trajectory_contact = reference_trajectory[:min_length_contact]
shorter_measured_trajectory_contact = measured_trajectory_contact[:min_length_contact]
shorter_reference_trajectory_contactless = reference_trajectory[:min_length_contactless]
shorter_measured_trajectory_contactless = measured_trajectory_contactless[:min_length_contactless]
# Calculate the displacement for each point pair

displacements_contactess = shorter_measured_trajectory_contactless[:, 1] - shorter_reference_trajectory_contactless[:, 1]
displacements_contact = shorter_measured_trajectory_contact[:, 1] - shorter_reference_trajectory_contact[:, 1]

# Compute the average displacement
average_displacement= [np.mean(displacements_contact),np.mean(displacements_contactess)]


max_displacement_index = [np.argmax(np.abs(displacements_contact)),np.argmax(np.abs(displacements_contactess))]
# point1 = shorter_measured_trajectory[max_displacement_index]
# point2 = shorter_reference_trajectory[max_displacement_index]
print(max_displacement_index)
pointrefcontact = reference_trajectory[max_displacement_index[0]]
pointrefcontactless = reference_trajectory[max_displacement_index[1]]
pointcontactless = measured_trajectory_contactless[max_displacement_index[1]]
pointcontact = measured_trajectory_contact[-1]
print('maxium displacement contactless: ',displacements_contactess.max())
print('maxium displacement contact: ',displacements_contact.max())
print('average displacement contactless:',average_displacement[1])
print('average displacement contact:',average_displacement[0])
# print('furthest point:',max_displacement_index)

# plt.figure(figsize=(8, 6)) 
# plt.plot(x_reference, y_reference, label='Reference Trajectory', marker='o')
plt.plot(y_reference, x_reference, label='Reference Trajectory')
plt.plot(smoothed_y, smoothed_x, label='Measured Trajectory')
# plt.plot(smoothed_y2, smoothed_x2, label='Measured Trajectory',c='g')
plt.xlim(-.05,0.05)
plt.ylim(0.385,0.505)
plt.plot( [pointrefcontactless[1], pointcontactless[1]],[pointcontactless[0], pointcontactless[0]], 'k--', label="Max Displacement in contactless case")
plt.plot( [pointrefcontact[1], pointcontact[1]],[pointrefcontact[0], pointcontact[0]], 'r--', label="Max Displacement in contact case")
print([pointrefcontactless[1], pointcontactless[1]],[pointcontactless[0], pointcontactless[0]])
print([pointrefcontact[1], pointcontact[1]],[pointrefcontact[0], pointcontact[0]])
plt.xlabel('current Y position (m)')
plt.ylabel('current X position (m)')
plt.legend()
plt.title('Trajectory comparision of pen writing between contact and contactless')
plt.grid(True)
plt.show()
