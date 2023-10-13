# -*- coding: utf-8 -*-
"""
Created on Thu Jun 29 20:26:21 2023

@author: mukh915
"""


import cvxpy as cp
import numpy as np
import pandas as pd
import competing_apps as CA
import copy
import json
import math

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
       max_val = max(conflict_matrix.loc[device].values[:2])
       min_val = min(conflict_matrix.loc[device].values[:2])
       diff = max_val - min_val
       if diff > 5:
           print("conflict exists for device:"+ device + "; At time:" + str(time)) 
           conflict_status = True
         
    return conflict_status


if __name__ == "__main__":

    case_name = 'No_block_test_2'
    Battery = {
        'E_rated'    : [500, 500],
        'Pch_max'    : [250, 250],
        'Pdisch_max' : [-250, -250],
        'Ch_eff'     : [0.8385, 0.8385],
        'Disch_eff'  : [0.8385, 0.8385],
        }
    
    data = pd.read_csv('time-series.csv', index_col = 0)
    data['Time'] = np.linspace(0,24, len(data)+1)[:-1] 
    data = data.set_index(['Time'])
    
    base_load= 12302.35
    base_pv = 2454.66
    
    Measurement = pd.DataFrame(columns= ['Time','Psub', 'PBatt1', 'PBatt2', 'socBatt1', 'socBatt2'])
    conflict_matrix = pd.DataFrame()
    conflict_matrix_store = {}
    
    time = 0
    Measurement.loc[0] = [time, 0, 0 ,0, 0.2, 0.2]
    
    
    # t = 1
    # for i in data.index:
    Res_Naive = False
    Decarb_Naive = False   
    
    deltaT_sim = 0.5/60 
    deltaT_app =  15/60 # minutes
    time = 0 
    
    app_preference = pd.DataFrame(columns=['Time', 'Res-Batt1', 'Res-Batt2', 'Decarb-Batt1', 'Decarb-Batt2'])
    
    while time < 24-deltaT_sim:
       
        print('Current Simulation Time: ', round(time,3))
        # deltaT = 15/60
        # time = i*deltaT
        
        
        
        # conflict_matrix[row]
        meas_idx =  len(Measurement)
        soc_now = list(Measurement.loc[meas_idx-1][4:6])
        
        rounded_time = round(time,3)
        if rounded_time in  data.index: 
            print("Change in data: ", time)
            load_now  = base_load * data['Loadshape'][rounded_time]
            solar_now = base_pv * data['Solar'][rounded_time]
            
            if 18 <= rounded_time <= 21:
                emergency = True
            else:
                emergency = False
                
            
            Pbatt_preffered_Res = CA.Resilience(load_now, solar_now, soc_now, Battery, deltaT_app, [0,0], [0,0], 0, [0,0], emergency, True)
            conflict_matrix= update_conflict_matrix(conflict_matrix, Pbatt_preffered_Res, 'App1')
            conflict_matrix_store[time]= conflict_matrix.T.to_dict()
            
            Pbatt_preffered_Decarb = CA.Decarbonization(load_now, solar_now, soc_now, Battery, deltaT_app, [0,0], [0,0], 0, [0,0], emergency, True)
            conflict_matrix = update_conflict_matrix(conflict_matrix, Pbatt_preffered_Decarb, 'App2')
            conflict_matrix_store[time] = conflict_matrix.T.to_dict()
            
            app_preference.loc[len(app_preference.index)] = [rounded_time, Pbatt_preffered_Res['batt1'], Pbatt_preffered_Res['batt2'], \
                                                             Pbatt_preffered_Decarb['batt1'], Pbatt_preffered_Decarb['batt2']]
            
            conflict_status = check_conflict(conflict_matrix, time)
            
            ### Sample Incentive Design ###
            incentive= copy.deepcopy(conflict_matrix)
            incentive.loc[:,:] = 0
            iteration = 0 
            
            weight = copy.deepcopy(conflict_matrix)
            weight.loc[:,:] = 0.5
        
            preffered = copy.deepcopy(conflict_matrix)
            conflict_matrix['advise'] = [0, 0 ]
            weights = (1/(conflict_matrix.shape[1]-1))*np.ones(conflict_matrix.shape[1]-1)
            

        
        # while conflict_status: 
        if conflict_status:
            iteration = iteration + 1
            # time =  time + 1/60 ## incremental time steps for iterative cooperation
            no_devices = conflict_matrix.shape[0]
            no_apps = conflict_matrix.shape[1]-1
            ### Sample Press Advise Signal Design ###
            rho = 0.5
           
            for j in range(no_apps):
                del_weight = 0
                for device in conflict_matrix.index:
                    del_weight += ((preffered.loc[device][j] - conflict_matrix.loc[device][j])/500)**2
                del_weight = math.sqrt(del_weight) 
                weights[j] = weights[j] + del_weight
            
            
            for device in conflict_matrix.index:
                temp_advise = 0 
                for j in range(no_apps):
                    temp_advise += (weights[j]* preffered.loc[device][j])/ sum(weights)
                    # print(temp_advise)
                conflict_matrix['advise'].loc[device] = temp_advise
            
           
            for device in conflict_matrix.index:
                for j in range(no_apps):
                    # incentive.loc[device][j] = incentive.loc[device][j] + round(rho*(conflict_matrix.loc[device][j] - conflict_matrix.loc[device][no_devices-1-j])/500,2)
                    incentive.loc[device][j] = incentive.loc[device][j] + round(rho*(conflict_matrix.loc[device][j] - conflict_matrix.loc[device].advise)/500,2)
                    
            
        
            press = incentive['App1'].values ### Sample Press - Linear based on other App's Preffered Values ###
            # advise = conflict_matrix['App2'].values ### Sample Advise - Other Apps' Preffered Values ###
            advise = conflict_matrix['advise'].values ### Sample Advise - Other Apps' Preffered Values ###
            Pbatt_preffered = CA.Resilience(load_now, solar_now, soc_now, Battery, deltaT_app, press, advise, rho, emergency, Res_Naive)
            conflict_matrix= update_conflict_matrix(conflict_matrix, Pbatt_preffered, 'App1')
            conflict_matrix_store[time]= conflict_matrix.T.to_dict()
            
            press = incentive['App2'].values ### Sample Press - Linear based on other App's Preffered Values ###
            # advise = conflict_matrix['App1'].values ### Sample Advise - Other App's Preffered Values ###
            advise = conflict_matrix['advise'].values ### Sample Advise - Other Apps' Preffered Values ###
            Pbatt_preffered = CA.Decarbonization(load_now, solar_now, soc_now, Battery, deltaT_app, press, advise, rho, emergency, Decarb_Naive)
            conflict_matrix = update_conflict_matrix(conflict_matrix, Pbatt_preffered, 'App2')
            conflict_matrix_store[time] = conflict_matrix.T.to_dict()
            # print(iteration, conflict_matrix)
            
            conflict_status = check_conflict(conflict_matrix, time)
        
        
        ### Sample Compromise Design ###
        resol_vector = {}
        for device in conflict_matrix.index:
            resol_vector[device] = round(np.mean(conflict_matrix.loc[device]),2)
        
        ### Grid Emulation ###
        print(conflict_status, resol_vector)
        Psub, SOC_new = CA.Simulate(load_now, solar_now, resol_vector, soc_now, Battery, deltaT_sim)   
        
        time = time + deltaT_sim
        meas_idx =  len(Measurement)
        Measurement.loc[meas_idx] = [time, Psub, resol_vector['batt1'], resol_vector['batt2'], SOC_new[0], SOC_new[1]]
        

    app_preference.to_csv('outputs\\'+case_name + '_app_preferences.csv', index=False)    
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
    ax[0].legend(['Batt1', 'Batt2'])
    ax[0].grid()
    ax[1].plot(Measurement['Time'][1:], Measurement['socBatt1'][1:], '--')
    ax[1].plot(Measurement['Time'][1:], Measurement['socBatt2'][1:],'--')
    ax[1].set_ylabel('Battery SOC')
    ax[1].legend(['Batt1', 'Batt2'])
    ax[1].grid()