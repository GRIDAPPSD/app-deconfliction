import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import importlib
import os
import sys
if (os.path.isdir('../shared')):
  sys.path.append('../shared')
# Assuming 'data.json' is a file containing JSON data
from AppUtil import AppUtil
from gridappsd import GridAPPSD

SPARQLManager = getattr(importlib.import_module('sparql'), 'SPARQLManager')
feeder_mrid = "_E3D03A27-B988-4D79-BFAB-F9D37FB289F7"
simulation_id = "12345678"
os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-conflict-matrix'
os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
os.environ['GRIDAPPSD_USER'] = 'app_user'
os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

gapps = GridAPPSD(simulation_id)
assert gapps.connected
sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

SolarPVsInfo, SolarPVs = AppUtil.getSolarPVs(sparql_mgr)
BatteriesInfo, BatteriesBus = AppUtil.getBatteries(sparql_mgr)
RegulatorsInfo, RegulatorsIdx = AppUtil.getCombineRegulators(sparql_mgr)
device_rating_map =  {}
for key in RegulatorsInfo:
    device_rating_map[key] = [-16, 16]
for key in BatteriesInfo:
    device_rating_map[key] = [-BatteriesInfo[key]['ratedkW']*1000, BatteriesInfo[key]['ratedkW']*1000]
for key in SolarPVs:
    device_rating_map[key] = [0, SolarPVs[key]['ratedS']]


case_name =  '5_apps'
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

                    ## new_x = ((x - min_x) * 2 / (max_x - min_x)) - 1
                    setpoint_norm = -1 + (2*(setpoint - device_rating_map[device][0]) /  (device_rating_map[device][1] - device_rating_map[device][0]))
                        
                    if time_stamp not in conflict_matrix_data:
                        conflict_matrix_data[time_stamp] = {}
                    if device not in conflict_matrix_data[time_stamp]:
                       conflict_matrix_data[time_stamp][device] = {}
                    
                    conflict_matrix_data[time_stamp][device][app] = setpoint_norm
                    # print(f"app: {app}, Value: {value}")
            parsed_data.append(json_object)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON on line: {line.strip()} - {e}")



conflict_matrix_dfs = {} 
conflict_matrix_sizes = {}
for time, data_dict in conflict_matrix_data.items():
    conflict_matrix_dfs[time] = pd.DataFrame.from_dict(data_dict)
    print('Conflict Matrix Size at Time stamp {} is {}'.format(time, conflict_matrix_dfs[time].size))
    


# Create a sample matrix
# Create a sample matrix
plt_interval  = 1751321808
matrix_data = conflict_matrix_dfs[plt_interval].to_numpy()
apps = list(conflict_matrix_dfs[plt_interval].index.values)
# Plot the matrix with a colormap
fig, ax = plt.subplots()
img = ax.imshow(matrix_data, cmap='inferno') # 'viridis' is a common colormap
fig.colorbar(img, label='Normalized App Setpoints') # Add a colorbar to interpret the colors
ax.set_title('Conflcit Matrix')
ax.set_yticks(range(0,len(apps)), range(1,len(apps)+1))
ax.set_xlabel('Device #')
ax.set_ylabel('App #')
fig.savefig('plots/Conflict_Matrix_{}_{}.png'.format(case_name,plt_interval))
fig.show()
# fig.close()