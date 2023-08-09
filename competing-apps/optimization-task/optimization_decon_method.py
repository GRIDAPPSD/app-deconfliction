#TODO: Copyright text

import csv
import itertools
import json
import os
from pathlib import Path
import sys
from typing import Dict, List

import numpy as np
import pulp

import copy
import importlib
from time import sleep

# find and add shared directory to path hopefully wherever it is from here
if (os.path.isdir('../../shared')):
  sys.path.append('../../shared')
elif (os.path.isdir('../../competing-apps/shared')):
  sys.path.append('../../competing-apps/shared')
elif (os.path.isdir('../../../competing-apps/shared')):
  sys.path.append('../../../competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')

from AppUtil import AppUtil
import MethodUtil

class DeconflictionMethod:
  def __init__(self, conflictMatrix: Dict): 
    self.conflictMatrix = MethodUtil.ConflictSubMatrix 
    
    self.sourceFolder = "../optimization-apps/output" 
    self.constraintSourceFile = "decarbonization" 
    self.resilienceUtilitySourceFile = "resilience"
    self.decarbonizationUtilitySourceFile = "decarbonization"

    self.conflictTime = -1 
    self.max_exch_capacity = 5e6
    
    self.decision_var = {} 
    self.opt_prob = {}

    self.Batteries = AppUtil.getBatteries(MethodUtil.sparql_mgr) 
    self.Regulators = AppUtil.getRegulators(MethodUtil.sparql_mgr)



  def addResilienceUtility(self, inputDict): 
    #utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.resilienceUtilitySourceFile}_{self.conflictTime}.json") 
    utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.resilienceUtilitySourceFile}_latest.json") 
    json_f = open(utilitySourceJSON, "r") 
    source_data = json.load(json_f) 
    
    n = len(source_data["objective"]["coefficients"]) 
    
    for myDict in source_data["objective"]["coefficients"]: 
      myDict["value"] = 0.01 * myDict["value"] / n 
      inputDict["objective"]["coefficients"].append(myDict) 
    
    return

  def addDecarbonizationUtility(self, inputDict, Pmax): 
    #utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.decarbonizationUtilitySourceFile}_{self.conflictTime}.json") 
    utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.decarbonizationUtilitySourceFile}_latest.json") 
    json_f = open(utilitySourceJSON, "r") 
    source_data = json.load(json_f) 
    
    myDict = source_data["objective"]["coefficients"][0] 
    myDict["value"] = 1 / Pmax 
    inputDict["objective"]["coefficients"].append(myDict) 
    
    return 

  def optimizationResolveConflict(self, constraintSourceJSON): 
    json_f = open(constraintSourceJSON, "r") 
    constraint_data = json.load(json_f) 

    optDict = {
      "objective": {"name": "Anything", "coefficients": []}, 
      "constraints": copy.deepcopy(constraint_data["constraints"]),
      "variables": copy.deepcopy(constraint_data["variables"]), 
      "parameters": copy.deepcopy(constraint_data["parameters"]),
      "sos1": [],
      "sos2": []
    } 
    
    self.addDecarbonizationUtility(optDict, self.max_exch_capacity) 
    self.addResilienceUtility(optDict) 
    
    #print("FIXME: Debugging.") 
    #print(optDict.objective.coefficients) 
    #return 
  
    json_bi = open(f"opt_prob_{self.conflictTime}.json", 'w') 
    json.dump(optDict, json_bi, indent = 4) 
    json_bi.close()

    self.decision_var, self.opt_prob = pulp.LpProblem.from_dict(optDict) 
    self.opt_prob.solve(pulp.PULP_CBC_CMD(msg = 0, gapRel = 0.01, timeLimit = 8)) 
    print('Optimization-based Deconfliction: Status:', pulp.LpStatus[self.opt_prob.status], flush = True) 

    # FIXME: The things below are hard-wired. Change in the future.
    outputDict = {}
    outputDict[f"{self.conflictTime}"] = {
      "obj_value": pulp.value(self.opt_prob.objective),
      "decarbonization_utility": (1 / self.max_exch_capacity) * self.decision_var['P_sub_mod'].value(),
      "resilience_utility": (1 / 5) * (self.decision_var['soc_0'].value() + self.decision_var['soc_1'].value() + \
               self.decision_var['soc_2'].value() + self.decision_var['soc_3'].value() + self.decision_var['soc_4'].value()),
      "solution_time": self.opt_prob.solutionTime
    }
  
    with open(f"debug_opt_prob.txt", 'a+') as txtFile: 
      txtFile.write(json.dumps(outputDict))
      txtFile.write("\n")
    
    # Maybe the thing below will be useful in the future.  
    # #decision_var["reg_tap"].value() 

    ResolutionVector = {} 
    setpoints = {} 
    timestamps = {} 
    for i in self.Regulators: 
      idx = self.Regulators[i]['idx'] 
      for k in range(32): 
        reg = 'reg_tap_('+ str(idx) + ',_' + str(k) + ')' 
        #if self.reg_taps[(idx, k)].varValue >= 0.5: 
        if self.decision_var[reg].value() >= 0.5: 
          setpoints[i] = k-16 
          timestamps[i] = self.conflictTime 
          
    for i in self.Batteries: 
      idx = self.Batteries[i]['idx'] 
      batt = 'p_batt_' + str(idx) 
      setpoints[i] = self.decision_var[batt].value() 
      timestamps[i] = self.conflictTime

    ResolutionVector["setpoints"] = setpoints 
    ResolutionVector["timestamps"] = timestamps 
    
    return ResolutionVector


  def deconflict(self, currentTimestamp) -> Dict: 
    sleep(2)
    self.conflictTime = currentTimestamp 
    #constraintSourceJSON = os.path.join(self.sourceFolder, f"{self.constraintSourceFile}_{self.conflictTime}.json") 
    constraintSourceJSON = os.path.join(self.sourceFolder, f"{self.constraintSourceFile}_latest.json") 
    
    return self.optimizationResolveConflict(constraintSourceJSON)