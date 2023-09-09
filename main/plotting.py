# import matplotlib.pyplot as plt
# import numpy as np
# import pandas as pd
# from scipy.signal import *
# import os

# folder_path = 'main/log/us1/contact/'

# dataframes = []
# subdirectories = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]
# sorted_subdirectories = sorted(subdirectories, key=lambda x: int(x))
# avg_power_list = []
# avg_curr_list = []
# avg_vol_list = []
# avg_torque_list =[]
# combined_df =[]
# # Iterate through the sorted subdirectories
# combine = True
# for folder_name in sorted_subdirectories:
#     folder_dir = os.path.join(folder_path, folder_name)
#     files = os.listdir(folder_dir)
#     csv_files = [f for f in files if f.endswith('.csv')]
    
#     if csv_files:
#         csv_file_path = os.path.join(folder_dir, csv_files[0])
#         df = pd.read_csv(csv_file_path,index_col=False)
#         dataframes.append(df)
#         df = df.astype(float)
#         if combine:
#         # Combine all dataframes into a single dataframe if needed
#             combined_df = pd.concat(dataframes, ignore_index=True)

#         # avg_power_list.append(data['actuar_voltage_1'] * data['actuar_current_1'])
#         # df['actuar_voltage_1'].mean(axis=0,skipna=True)

    
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
import numpy as np
from scipy.spatial.distance import euclidean
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

# Generate example trajectories (replace these with your actual trajectories)
reference_trajectory = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]])
measured_trajectory = np.array([[0, 0], [1, 1], [1.5, 2], [3, 3.5], [4, 4]])

# Define a custom distance function (Euclidean distance in this example)
def distance_function(x, y):
    return euclidean(x, y)

# Compute the DTW distance
distance_matrix = cdist(reference_trajectory, measured_trajectory, metric=distance_function)
dtw_distance = np.min(distance_matrix)

# Calculate the alignment path using the accumulated cost matrix
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
from fastdtw import fastdtw
alignment_path = np.array(list(fastdtw(reference_trajectory, measured_trajectory, dist=distance_function)[1]))

# Print the DTW distance (a lower distance indicates higher similarity)
print(f"DTW Distance: {dtw_distance}")

# Visualize the alignment path
x_reference, y_reference = reference_trajectory.T
x_measured, y_measured = measured_trajectory.T

plt.figure(figsize=(8, 6))
plt.plot(x_reference, y_reference, label='Reference Trajectory', marker='o')
plt.plot(x_measured, y_measured, label='Measured Trajectory', marker='x')
for i, j in alignment_path:
    plt.plot([x_reference[i], x_measured[j]], [y_reference[i], y_measured[j]], 'k--')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('DTW Alignment')
plt.grid(True)
plt.show()
