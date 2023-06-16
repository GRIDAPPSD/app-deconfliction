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

# find and add shared directory to path hopefully wherever it is from here
if (os.path.isdir('../../shared')):
  sys.path.append('../../shared')
elif (os.path.isdir('../../competing-apps/shared')):
  sys.path.append('../../competing-apps/shared')
elif (os.path.isdir('../../../competing-apps/shared')):
  sys.path.append('../../../competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')

import MethodUtil

class DeconflictionMethod:
    def __init__(self, conflictMatrix: Dict, fullResolutionFlag: bool = True):
        self.conflictMatrix = MethodUtil.ConflictSubMatrix
        self.setpointSetVector = None
        self.numberOfSets = 0

        self.constraintJSONFileName = f"output/resilience"
        self.conflictTime = -1

    def getDecarbonizationUtility(self):
       return 0

    def getResilienceUtility(self):
       return 0

    def optimizationResolveConflict(self): 
      json_constr_f = open(f"{self.constraintJSONFileName}_{self.conflictTime}.json", "r") 
      input_data = json.load(json_constr_f) 
      
      decision_var, opt_prob = pulp.LpProblem.from_dict(input_data) 
      
      opt_prob.solve() 
      return 0

    def deconflict(self) -> Dict:
        for timeVal in self.conflictMatrix.get("timestamps", {}):
            self.conflictMatrix = max(self.conflictTime, timeVal)

        self.setpointSetVector = self.buildSetpointsVector(self.conflictMatrix)
        self.numberOfSets = len(self.setpointSetVector.get("setpointSets",[]))
        self.uMatrix = np.empty((self.numberOfSets, DeconflictionMethod.numberOfMetrics))
        resolutionVector = {
            "setpoints": {},
            "timestamps": {}
        }
        return resolutionVector