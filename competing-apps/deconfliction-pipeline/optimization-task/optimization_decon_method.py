#TODO: Copyright text

import csv
import copy
import itertools
import json
import os
from pathlib import Path
import sys
from typing import Dict, List

import numpy as np
import pulp

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
  def __init__(self, conflictMatrix: Dict, fullResolutionFlag: bool = True): 
    self.conflictMatrix = MethodUtil.ConflictSubMatrix 
    self.setpointSetVector = None 
    self.numberOfSets = 0 
    
    self.sourceFolder = "output" 
    self.constraintSourceFile = "decarbonization" 
    self.resilienceUtilitySourceFile = "resilience"
    self.decarbonizationUtilitySourceFile = "decarbonization"

    self.conflictTime = -1 
    self.max_exch_capacity = 5e6
    
    self.decision_var = {} 
    self.opt_prob = {}

    SPARQLManager = getattr(importlib.import_module('sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id) 
    
    self.Batteries = AppUtil.getBatteries(sparql_mgr) 
    self.Regulators = AppUtil.getRegulators(sparql_mgr)



    def addResilienceUtility(self, inputDict): 
      utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.resilienceUtilitySourceFile}_{self.conflictTime}.json") 
      json_f = open(utilitySourceJSON, "r") 
      source_data = json.load(json_f) 

      n = len(source_data.objective.coefficients)

      for myDict in source_data.objective.coefficients:
        myDict["value"] = myDict["value"] / n 
        inputDict.objective.coefficients.append(myDict)

      return

    def addDecarbonizationUtility(self, inputDict, Pmax): 
      utilitySourceJSON = os.path.join(self.sourceFolder, f"{self.decarbonizationUtilitySourceFile}_{self.conflictTime}.json") 
      json_f = open(utilitySourceJSON, "r") 
      source_data = json.load(json_f) 
      
      myDict = source_data.objective.coefficients[0]
      myDict["value"] = 1 / Pmax
      inputDict.objective.coefficients.append(myDict)

      return 

    def optimizationResolveConflict(self, constraintSourceJSON): 
      optDict = {}

      json_f = open(constraintSourceJSON, "r") 
      constraint_data = json.load(json_f) 

      optDict.objective = {"coefficients": []}
      optDict.constraints = copy.deepcopy(constraint_data.constraints)
      optDict.variables = copy.deepcopy(constraint_data.variables)
      optDict.parameters = copy.deepcopy(constraint_data.parameters)

      addDecarbonizationUtility(optDict, self.max_exch_capacity) 
      addResilienceUtility(optDict) 

      print("FIXME: Debugging.")
      print(optDict.objective.coefficients)
      return 
      
      self.decision_var, self.opt_prob = pulp.LpProblem.from_dict(optDict) 

      self.opt_prob.solve(pulp.PULP_CBC_CMD(msg = 0, gapRel = 0.01)) 
      print('Optimization-based Deconfliction: Status:', pulp.LpStatus[self.opt_prob.status], flush = True) 

      # Maybe the thing below will be useful in the future.
      #decision_var["reg_tap"].value() 
      
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

      ResolutionVector.setpoints = setpoints
      ResolutionVector.timestamps = timestamps

      return ResolutionVector

    def deconflict(self, currentTimestamp) -> Dict: 
      self.conflictTime = currentTimestamp
      constraintSourceJSON = os.path.join(self.sourceFolder, f"{self.constraintSourceFile}_{self.conflictTime}.json") 

      
      return optimizationResolveConflict(self, constraintSourceJSON) 