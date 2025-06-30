import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Assuming 'data.json' is a file containing JSON data

conflict_matrix_filename = 'log/conflict_matrix.log'
parsed_data = []

conflict_matrix_data = {}
with open(conflict_matrix_filename, 'r') as f:
    for line in f:
        try:
            json_object = json.loads(line.strip())  # Parse each line as JSON
            for device, app_info in json_object.items():
                for app, value in app_info.items():
                    time_stamp = value[0]
                    setpoint = value[1]
                    if isinstance(setpoint, list):
                        setpoint = abs(complex(setpoint[0], setpoint[1]))
                    
                    if abs(setpoint) > 1e3:
                        setpoint = setpoint / 1e3
                        
                    if time_stamp not in conflict_matrix_data:
                        conflict_matrix_data[time_stamp] = {}
                    if device not in conflict_matrix_data[time_stamp]:
                       conflict_matrix_data[time_stamp][device] = {}
                    
                    conflict_matrix_data[time_stamp][device][app] = setpoint
                    # print(f"app: {app}, Value: {value}")
            parsed_data.append(json_object)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON on line: {line.strip()} - {e}")



conflict_matrix_dfs = {} 
conflict_matrix_sizes = {}
for time, data_dict in conflict_matrix_data.items():
    conflict_matrix_dfs[time] = pd.DataFrame.from_dict(data_dict)
    print(time, conflict_matrix_dfs[time].size)
    


# Create a sample matrix
# Create a sample matrix
plt_interval  = 1751318021
matrix_data = conflict_matrix_dfs[plt_interval].to_numpy()
apps = list(conflict_matrix_dfs[plt_interval].index.values)
# Plot the matrix with a colormap
fig, ax = plt.subplots()
img = ax.imshow(matrix_data, cmap='inferno') # 'viridis' is a common colormap
fig.colorbar(img, label='Value') # Add a colorbar to interpret the colors
ax.set_title('Conflcit Matrix')
ax.set_yticks(range(0,len(apps)), range(1,len(apps)+1))
ax.set_xlabel('Device #')
ax.set_ylabel('App #')
fig.savefig('plots/Conflict_Matrix_{}.png'.format(plt_interval))
fig.show()
# fig.close()