#TODO: Copyright text

import csv
import itertools
import json
import os
from pathlib import Path
import sys
from typing import Dict, List

import numpy as np
import opendssdirect as dss
from opendssdirect.utils import Iterator

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
    maxAnalogSetpoints = 5 #default range length for analog setpoints
    numberOfMetrics = 4 #defines the number of columns in the U matrix.

    def __init__(self, conflictMatrix: Dict, fullResolutionFlag: bool = True):
        self.conflictMatrix = MethodUtil.ConflictSubMatrix
        self.setpointSetVector = None
        self.numberOfSets = 0
        self.uMatrix = None
        self.pMatrix = None
        self.cMatrix = None
        self.invalidSetpoints = []
        self.conflictTime = -1
        # The following constants should be read from a deconfliction configuration file to allow for buisness variability.
        self.cost_transmission = 0.02817 #transmission price $/kWh
        self.cost_retail = 0.1090 #retail rate
        self.cost_net_metering = 0.01786 #feed-in tariff
        self.cost_diesel = 0.34 #diesel fuel cost
        self.cost_lng = 0.25 #lng fuel cost
        self.emissions_transmission = 1.205 #grid co2 lb/kWh
        self.emissions_diesel = 2.44
        self.emissions_lng = 0.97
        csvDataFile = Path(__file__).parent.parent.resolve() / 'sim-starter' / 'time-series.csv'
        self.csvData = []
        with open(csvDataFile, 'r', newline='') as cf:
            csvReader = csv.reader(cf)
            self.csvData = list(csvReader)
            del self.csvData[0]
        openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        dss.Text.Command(f'Redirect {openDssFile}')
        dss.Text.Command(f'Compile {openDssFile}')
        self.rVector = np.empty((DeconflictionMethod.numberOfMetrics, 1))
        self.calculateWeightVector()

    def deconflict(self) -> Dict:
        #TODO: call the opendss solver for each alternative setpoint Set and create the U matrix
        for timeVal in self.conflictMatrix.get["timestamps"]:
            self.conflictMatrix = max(self.conflictMatrix, timeVal)
        self.setpointSetVector = self.buildSetpointsVector(self.conflictMatrix)
        self.numberOfSets = len(self.setpointSetVector.get("setpointSets",[]))
        self.uMatrix = np.empty((self.numberOfSets, DeconflictionMethod.numberOfMetrics))
        #TODO: replace simulationData with device setpoint measurements
        simulationData = {}
        for i in self.csvData:
            if int(i[0]) == self.conflictTime:
                simulationData["load"] = float(i[1])
                simulationData["solar"] = float(i[2])
                simulationData["price"] = float(i[3])
                break
        for i in self.numberOfSets:
            self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"], i, simulationData)
        self.normalizeUMatrix()
        self.calculateCriteriaPriorityMatrix()
        self.calculateSetpointAlternativesPriorityMatrix()
        fVector = self.pMatrix @ self.uMatrix @ self.cMatrix @ self.rVector
        resolutionIndex = fVector.argmax()
        resolutionSetpointSet = self.setpointSetVector["setpointSets"][resolutionIndex]
        resolutionVector = {
            "setpoints": {},
            "timestamps": {}
        }
        for i in range(len(resolutionSetpointSet)):
            resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = resolutionSetpointSet[i]
            resolutionVector["timestamps"][[self.setpointSetVector["setpointIndexMap"][i]]] = self.conflictTime
        return resolutionVector


    
#for testing purposes only must be removed for actual implementation
if __name__ == "__main__":
    cM = {}
    with open("/home/vale/git/app-deconfliction/competing-apps/deconfliction-pipeline/rbdm/ConflictMatrix.json","r") as cmf:
        cM = json.load(cmf)
    rbdm = DeconflictionMethod(cM)
    dss.Text.Command('Redirect ./123Bus/Run_IEEE123Bus.dss')
    dss.Text.Command('Compile Run_IEEE123Bus.dss')