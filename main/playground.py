import numpy as np
from fastdtw import fastdtw
import matplotlib.pyplot as plt

# Define your reference and measured trajectories as NumPy arrays
ref_trajectory = np.array([1, 2, 4, 3, 6, 8, 9])  # Replace with your data
measured_trajectory = np.array([1, 3, 3, 4, 7, 9, 10])  # Replace with your data

# Subsample the trajectories by a factor (e.g., every nth data point)
subsampling_factor = 2  # Adjust as needed

# Apply subsampling
subsampled_ref_trajectory = ref_trajectory[::subsampling_factor]
subsampled_measured_trajectory = measured_trajectory[::subsampling_factor]

# Calculate DTW distance and alignment using subsampled data
distance, path = fastdtw(subsampled_ref_trajectory, subsampled_measured_trajectory)

# Find the longest aligned segment in the DTW path
start, end = path[0][0], path[0][1]
longest_distance = np.abs(subsampled_ref_trajectory[start] - subsampled_measured_trajectory[end])

for i, j in path:
    segment_distance = np.abs(subsampled_ref_trajectory[i] - subsampled_measured_trajectory[j])
    if segment_distance > longest_distance:
        start, end = i, j
        longest_distance = segment_distance

# Create a plot for the longest DTW path
plt.figure(figsize=(8, 6))
plt.plot(subsampled_ref_trajectory, label='Reference Trajectory', marker='o', markersize=5, linestyle='-', color='blue')
plt.plot(subsampled_measured_trajectory, label='Measured Trajectory', marker='o', markersize=5, linestyle='-', color='red')

# Highlight the longest DTW path
plt.plot([start, end], [subsampled_ref_trajectory[start], subsampled_measured_trajectory[end]], color='green', linewidth=2)

plt.title('Longest DTW Path with Subsampling')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

print("Longest DTW Distance:", longest_distance)