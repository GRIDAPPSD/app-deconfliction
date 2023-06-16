# -*- coding: utf-8 -*-
"""
Created on Wed Jun 14 19:09:22 2023

@author: mukh915
"""


import cvxpy as cp
import numpy as np
import pandas as pd
import competing_apps as CA
import json

def update_conflict_matrix(matrix, data, col_name):
    
    if matrix.empty:
        matrix= pd.DataFrame([data]).T.rename(columns= {0:col_name}) 

    else: 
        col_names = list(conflict_matrix.keys())
        row_names = list(conflict_matrix.index)
        
        if col_name not in col_names:
             matrix[col_name] = np.nan
  
        for device in data:
            if device not in row_names:
                 matrix.loc[device] = np.nan    
            matrix[col_name][device] = data[device]
         
    return matrix

def check_conflict(matrix, time):
    
    conflict_status = False
    for device in conflict_matrix.index:
       max_val = max(conflict_matrix.loc[device].values)
       min_val = min(conflict_matrix.loc[device].values)
       diff = max_val - min_val
       if diff > 0:
           print("conflict exists for device:"+ device + "; At time:" + str(time)) 
           conflict_status = True
         
    return conflict_status


if __name__ == "__main__":

    case_name = 'test'
    Battery = {
        'E_rated'    : [500, 500],
        'Pch_max'    : [250, 250],
        'Pdisch_max' : [-250, -250],
        'Ch_eff'     : [0.8385, 0.8385],
        'Disch_eff'  : [0.8385, 0.8385],
        }
    
    data = pd.read_csv('time-series.csv', index_col = 0)
    base_load= 12302.35
    base_pv = 2454.66
    
    Measurement = pd.DataFrame(columns= ['Time','Psub', 'PBatt1', 'PBatt2', 'socBatt1', 'socBatt2'])
    conflict_matrix = pd.DataFrame()
    conflict_matrix_store = {}
    
    time = 0
    Measurement.loc[0] = [time, 0, 0 ,0, 0.5, 0.5]
    
    
    # t = 1
    for i in data.index:
       
        deltaT = 15/60
        time = i*deltaT
        load_now  = base_load * data['Loadshape'][i]
        solar_now = base_pv * data['Solar'][i]
        
        
    
        # conflict_matrix[row]
    
        soc_now = list(Measurement.loc[i-1][4:6])
        Pbatt_preffered = CA.Resilience(load_now, solar_now, soc_now, Battery, deltaT)
        conflict_matrix= update_conflict_matrix(conflict_matrix, Pbatt_preffered, 'App1')
        conflict_matrix_store[time]= conflict_matrix.T.to_dict()
        
        Pbatt_preffered = CA.Decarbonization(load_now, solar_now, soc_now, Battery, deltaT)
        conflict_matrix = update_conflict_matrix(conflict_matrix, Pbatt_preffered, 'App2')
        conflict_matrix_store[time] = conflict_matrix.T.to_dict()
        
        conflict_status = check_conflict(conflict_matrix, time)
        
        if conflict_status: 
            
            resol_vector = {}
            for device in conflict_matrix.index:
                resol_vector[device] = np.mean(conflict_matrix.loc[device])
        
        print(resol_vector)
        Psub, SOC_new = CA.Simulate(load_now, solar_now, resol_vector, soc_now, Battery, deltaT)   
        
        
        meas_idx =  len(Measurement)
        Measurement.loc[meas_idx] = [time, Psub, resol_vector['batt1'], resol_vector['batt2'], SOC_new[0], SOC_new[1]]
        
        print(conflict_status, resol_vector)
        
    Measurement.to_csv('outputs\\'+case_name + '_meas.csv', index=False) 
    with open('outputs\\'+case_name + '_conflict_store.json', "w") as outfile:
        json.dump(conflict_matrix_store, outfile)
        
        
    ################## Plotting ##################
    import matplotlib.pyplot as plt 
    fig = plt.figure()
    plt.plot(Measurement['Time'][1:], Measurement['Psub'][1:])
    plt.xlabel('Time of Day')
    plt.ylabel('Substation Demand (kW)')
    plt.grid()
    
    fig, ax = plt.subplots(2,1)
    ax[0].plot(Measurement['Time'][1:], Measurement['PBatt1'][1:])
    ax[0].plot(Measurement['Time'][1:], Measurement['PBatt2'][1:])
    ax[0].set_xlabel('Time of Day')
    ax[0].set_ylabel('Battery Dispatch (kW)')
    ax[0].grid()
    ax[1].plot(Measurement['Time'][1:], Measurement['socBatt1'][1:], '--')
    ax[1].plot(Measurement['Time'][1:], Measurement['socBatt2'][1:],'--')
    ax[1].set_ylabel('Battery SOC')
    ax[1].legend(['Batt1', 'Batt2'])
    ax[1].grid()