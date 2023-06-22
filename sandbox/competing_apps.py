# -*- coding: utf-8 -*-
"""
Created on Wed Jun 14 13:33:12 2023

@author: mukh915
"""

import cvxpy as cp
import numpy as np
import pandas as pd




deltaT = 15/60
soc_0 = [0.5, 0.5]

def Resilience(Pload, P_PV, soc_0, Battery, deltaT, press = [0, 0], advise= [0,0], rho =0, emergency=False, Naive = False):

    E_rated    = Battery['E_rated']
    pch_max    =   Battery['Pch_max']
    pdisch_max =  Battery['Pdisch_max']
    Ch_eff     = Battery['Ch_eff']
    Disch_eff  = Battery['Disch_eff']
    no_batt = len(E_rated)
    
    #       SOC      Pbatt   Pbatt_Ch  Pbatt_DisCh   Pload  Psub
    n1 = no_batt + no_batt  + no_batt  + no_batt       +  1  + 1
    x = cp.Variable(n1, integer=False)
    
    socbatt_idx    = 0
    pbatt_idx      = socbatt_idx + no_batt
    pchbatt_idx    = pbatt_idx + no_batt
    pdischbatt_idx = pchbatt_idx + no_batt
    pload_idx = pdischbatt_idx + no_batt
    psub_idx = pload_idx + 1
    
    #  Lchbatt  Ldischbatt 
    n2 = no_batt + no_batt 
    y = cp.Variable(n2, integer=True)
    
    Lchbatt_idx = 0
    Ldischbatt_idx = Lchbatt_idx + no_batt
    
    pren = P_PV
    p_load_req = Pload ## 4733.3
    
    
    obj = 0
    if emergency:
        for batt_idx in range(no_batt):
            obj += 1*x[socbatt_idx+batt_idx] 
    else:
        if Naive: 
            for batt_idx in range(no_batt):
                obj += -1*x[socbatt_idx+batt_idx] 
        else:
            for batt_idx in range(no_batt):
                obj += -1*x[socbatt_idx+batt_idx]/(1.8) + press[batt_idx]*(x[pbatt_idx+batt_idx] - advise[batt_idx])/500 + (rho/2)*((x[pbatt_idx+batt_idx] - advise[batt_idx])/500 )**2
        
    
    constraints = []
    for batt_idx in range(no_batt):
        constraints.append(x[socbatt_idx+batt_idx] -1*(Ch_eff[batt_idx] * deltaT / E_rated[batt_idx])*x[pchbatt_idx+batt_idx] \
                                                   -1*((1/Disch_eff[batt_idx]) * deltaT / E_rated[batt_idx])*x[pdischbatt_idx+batt_idx] == soc_0[batt_idx])
        constraints.append( x[socbatt_idx+batt_idx]   <= 0.9 )    
        constraints.append( x[socbatt_idx+batt_idx]   >= 0.2 ) 
        
        constraints.append( x[pbatt_idx+batt_idx] -1*x[pchbatt_idx+batt_idx] -1*x[pdischbatt_idx+batt_idx] == 0 )
        constraints.append( x[pchbatt_idx+batt_idx] -1*y[Lchbatt_idx+batt_idx]*pch_max[batt_idx]  <= 0 )
        constraints.append( x[pchbatt_idx+batt_idx] >= 0 )    
        constraints.append( x[pdischbatt_idx+batt_idx] -1*y[Ldischbatt_idx+batt_idx]*pdisch_max[batt_idx]  >= 0 )
        constraints.append( x[pdischbatt_idx+batt_idx] <= 0 )                
        constraints.append( y[Lchbatt_idx+batt_idx] + y[Ldischbatt_idx+batt_idx]  <= 1 )     
       
     
    constraints.append( x[psub_idx] -1*x[pload_idx] + 1* pren - x[pbatt_idx+0] - x[pbatt_idx+1]  == 0 )   
    
    if emergency:
        constraints.append( x[pload_idx]  <= p_load_req)
    else:       
        constraints.append( x[pload_idx]  == p_load_req)   
    
    
    prob = cp.Problem(cp.Minimize(obj), constraints)
    prob.solve()
    
    # Print result.
    # print("\nOptimization Status: ", prob.status)
    # # print("The optimal x is", prob.value)

    # print("The norm of the residual is ", cp.norm(A @ x - b, p=2).value)
    
    pbatt_sol = {}
    for batt_idx in range(no_batt):
        pbatt_sol['batt'+str(batt_idx+1)] = x[pbatt_idx+batt_idx].value
    
    return pbatt_sol


def Decarbonization(Pload, P_PV, soc_0, Battery, deltaT, press = [0, 0], advise= [0,0], rho =0, emergency=False, Naive = False):

    E_rated    = Battery['E_rated']
    pch_max    =   Battery['Pch_max']
    pdisch_max =  Battery['Pdisch_max']
    Ch_eff     = Battery['Ch_eff']
    Disch_eff  = Battery['Disch_eff']
    no_batt = len(E_rated)
    
    #       SOC      Pbatt   Pbatt_Ch  Pbatt_DisCh   Pload  Psub
    n1 = no_batt + no_batt  + no_batt  + no_batt       +  1  + 1
    x = cp.Variable(n1, integer=False)
    
    socbatt_idx    = 0
    pbatt_idx      = socbatt_idx + no_batt
    pchbatt_idx    = pbatt_idx + no_batt
    pdischbatt_idx = pchbatt_idx + no_batt
    pload_idx = pdischbatt_idx + no_batt
    psub_idx = pload_idx + 1
    
    #  Lchbatt  Ldischbatt 
    n2 = no_batt + no_batt 
    y = cp.Variable(n2, integer=True)
    
    Lchbatt_idx = 0
    Ldischbatt_idx = Lchbatt_idx + no_batt
    
    pren = P_PV
    p_load_req = Pload #4733.3
    
    
    obj = 0
    if emergency:
        for batt_idx in range(no_batt):
            obj += 1*x[socbatt_idx+batt_idx]
    else:
        if Naive: 
            obj = cp.abs(x[psub_idx])
        else:
            obj = 1*cp.abs(x[psub_idx])/(p_load_req)
            for batt_idx in range(no_batt):
                obj = obj + press[batt_idx]*(x[pbatt_idx+batt_idx] - advise[batt_idx])/500 + (rho/2)*((x[pbatt_idx+batt_idx] - advise[batt_idx])/500 )**2
    
    constraints = []
    for batt_idx in range(no_batt):
        constraints.append(x[socbatt_idx+batt_idx] -1*(Ch_eff[batt_idx] * deltaT / E_rated[batt_idx])*x[pchbatt_idx+batt_idx] \
                                                   -1*((1/Disch_eff[batt_idx]) * deltaT / E_rated[batt_idx])*x[pdischbatt_idx+batt_idx] == soc_0[batt_idx])
        
        constraints.append( x[socbatt_idx+batt_idx]   <= 0.9 )    
        constraints.append( x[socbatt_idx+batt_idx]   >= 0.2 ) 
        
        constraints.append( x[pbatt_idx+batt_idx] -1*x[pchbatt_idx+batt_idx] -1*x[pdischbatt_idx+batt_idx] == 0 )
        constraints.append( x[pchbatt_idx+batt_idx] -1*y[Lchbatt_idx+batt_idx]*pch_max[batt_idx]  <= 0 )
        constraints.append( x[pchbatt_idx+batt_idx] >= 0 )    
        constraints.append( x[pdischbatt_idx+batt_idx] -1*y[Ldischbatt_idx+batt_idx]*pdisch_max[batt_idx]  >= 0 )
        constraints.append( x[pdischbatt_idx+batt_idx] <= 0 )                
        constraints.append( y[Lchbatt_idx+batt_idx] + y[Ldischbatt_idx+batt_idx]  <= 1 )     
       
     
    constraints.append( x[psub_idx] -1*x[pload_idx] + 1* pren - x[pbatt_idx+0] - x[pbatt_idx+1]  == 0 )    
    constraints.append( x[pload_idx]  == p_load_req)   
    
    
    prob = cp.Problem(cp.Minimize(obj), constraints)
    prob.solve()
    
    # Print result.
    # print("\nOptimization Status: ", prob.status)
    # print("The optimal x is", prob.value)

    # print("The norm of the residual is ", cp.norm(A @ x - b, p=2).value)
    
    pbatt_sol = {}
    for batt_idx in range(no_batt):
        pbatt_sol['batt'+str(batt_idx+1)] = x[pbatt_idx+batt_idx].value

    
    return pbatt_sol

def Simulate(Pload, P_PV, resol_vector, soc_0, Battery, deltaT):
    
    Pgrid = Pload -P_PV + sum(resol_vector.values())
    
    E_rated    = Battery['E_rated']
    Ch_eff     = Battery['Ch_eff']
    Disch_eff  = Battery['Disch_eff']
    
    SOC_new = []
    for batt_idx, batt in enumerate(resol_vector):
        if resol_vector[batt] > 0:
            Pbatt_Ch = resol_vector[batt]
            Pbatt_Disch = 0
        else:
            Pbatt_Ch = 0
            Pbatt_Disch = resol_vector[batt]
    
    
        SOC_new.append(soc_0[batt_idx] + 1 * (Ch_eff[batt_idx] * deltaT / E_rated[batt_idx])*Pbatt_Ch \
                                        + 1 * ((1/Disch_eff[batt_idx]) * deltaT / E_rated[batt_idx])*Pbatt_Disch) 
                                        
                                        
    return  Pgrid, SOC_new   

                         
# if __name__ == "__main__":

    
#     Battery = {
#         'E_rated'    : [500, 500],
#         'Pch_max'    : [250, 250],
#         'Pdisch_max' : [-250, -250],
#         'Ch_eff'     : [0.8385, 0.8385],
#         'Disch_eff'  : [0.8385, 0.8385],
#         }
    
#     data = pd.read_csv('time-series.csv', index_col = 0)
#     base_load= 12302.35
#     base_pv = 2454.66
    
#     t = 1
    
#     load_now  = base_load * data['Loadshape'][1]
#     solar_now = base_pv * data['Solar'][1]
#     deltaT = 15/60
    
#     soc_now = [0.5, 0.5]
#     Pbatt_preffered = Resilience(load_now, solar_now, soc_now, Battery, deltaT)
#     print(Pbatt_preffered)
#     Pbatt_preffered1 = Decarbonization(load_now, solar_now, soc_now, Battery, deltaT)
#     print(Pbatt_preffered1)