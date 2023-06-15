#TODO: Copyright text

import csv
import itertools
import json
import math
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



    def buildSetpointsVector(self, conflictMatrix:Dict) -> Dict:
        setpointSetVector = {
            "setpointIndexMap": [],
            "setpointSets": []
        }
        setpointRanges = []
        for setpoint in conflictMatrix.get("setpoints",{}).keys():
            values = []
            setpointSetVector["setpointIndexMap"].append(setpoint)
            for setpointValue in conflictMatrix.get("setpoints",{}).get(setpoint,{}).values():
                values.append(setpointValue)
            maxValue = max(values)
            minValue = min(values)
            if "BatteryUnit." in setpoint:
                step = (maxValue - minValue) / (DeconflictionMethod.maxAnalogSetpoints - 1)
                setpointRange = []
                for i in range(DeconflictionMethod.maxAnalogSetpoints):
                    setpointRange.append(minValue + (i * step))
                setpointRanges.append(setpointRange)
            elif "RatioTapChanger." in setpoint:
                setpointRanges.append(range(minValue, maxValue + 1))
            else:
                raise RuntimeError(f"Unrecognized setpoint in the Conflict Matrix: {setpoint}")
        setIter = itertools.product(*setpointRanges)
        for i in setIter:
            setpointSetVector["setpointSets"].append(i)
        return setpointSetVector
    

    def buildUMatrix(self, setpoints: List, setpointNames: List, index: int, simulationData: Dict):
        def checkForViolations() -> bool:
            bus_volt = dss.Circuit.AllBusMagPu()
            print(min(bus_volt), max(bus_volt))
            if min(bus_volt) < 0.95:
                return True
            elif max(bus_volt) > 1.10:
                return True
            sub = dss.Circuit.TotalPower()
            sub_kva = math.sqrt((sub[0]*sub[0]) + (sub[1]*sub[1]))
            if sub_kva > 5000:
                return True
            dss.Circuit.SetActiveClass('Transformer')
            device = dss.Circuit.FirstElement()
            while device:
                a = dss.CktElement.CurrentsMagAng()
                if a[0] > 668:
                    return True
                device = dss.Circuit.NextElement()
            return False
        

        def runPowerFlowSnapshot(setpoints: List, setpointNames: List, simulationData: Dict):
            if len(setpoints) != len(setpointNames):
                raise RuntimeError("The number of setpoints does not match the number of setpoint names")
            #TODO: update model with current measurements not data from this csvfile.
            #-----------------------------------------------------------------------
            dss.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
            dss.run_command(f'set loadmult = {simulationData["load"]}')
            #-----------------------------------------------------------------------
            #end model update with measurements
            for i in range(setpoints):
                setpointNameSplit = setpointNames[i].split(".")
                if "BatteryUnit." in setpointNames[i]:
                    #TODO: Figure out opendss command to change storage ouptput
                    dss.run_command(f"Storage.{setpointNameSplit[1]}.kw={setpoints[i]}")
                elif "RatioTapChanger." in setpointNames[i]:
                    reg = dss.Transformers.First()
                    while reg:
                        if dss.Transformers.Name() == setpointNameSplit[1]:
                            break
                        else:
                            dss.Transformers.Next()
                    regulationPerStep = (dss.Transformers.MaxTap() - dss.Transformers.MinTap()) / dss.Transformers.NumTaps()
                    tapStep = 1.0 + (setpoints[i]*regulationPerStep)
                    dss.Transformers.Tap(tapStep)
            dss.Solution.SolveNoControl()
        

        def extractPowerLosses() -> float:
            loss = dss.Circuit.Losses()
            return loss[0]/1000
        

        def extractProfitAndEmissions() -> tuple:
            bus_shunt_p = {}
            total_load = 0
            total_pv = 0
            total_dg = 0
            profit = 0
            emissions = 0
            sub = dss.Circuit.TotalPower()
            p_sub = sub[0]
            dss.Circuit.SetActiveClass('Load')
            device = dss.Circuit.FirstElement()
            while device:
                output = dss.CktElement.TotalPowers()
                bus = dss.CktElement.BusNames()
                bus_shunt_p[bus[0]] = output[0]
                total_load = total_load + output[0]
                device = dss.Circuit.NextElement()  
            dss.Circuit.SetActiveClass('PVSystem')
            device = dss.Circuit.FirstElement()
            while device:
                output = dss.CktElement.TotalPowers()
                bus = dss.CktElement.BusNames()
                total_pv = total_pv + output[0]
                if bus[0] in bus_shunt_p.keys():
                    bus_shunt_p[bus[0]] = bus_shunt_p[bus[0]] + output[0]
                else:
                    bus_shunt_p[bus[0]] = output[0]
                device = dss.Circuit.NextElement() 
            dss.Circuit.SetActiveClass('Generator')
            device = dss.Circuit.FirstElement()
            while device:
                output = dss.CktElement.TotalPowers()
                p_dg = output[0]
                total_dg = total_dg + p_dg
                if 'dies' in dss.CktElement.Name():
                    profit = profit + p_dg*self.cost_diesel
                    emissions = emissions - p_dg*self.emissions_diesel
                elif 'lng' in dss.CktElement.Name():
                    profit = profit + p_dg*self.cost_lng
                    emissions = emissions - p_dg*self.emissions_lng
                device = dss.Circuit.NextElement()
            emissions = emissions - p_sub*self.emissions_transmission
            profit = profit + p_sub*self.cost_transmission
            for bus in list(bus_shunt_p.keys()):
                p = bus_shunt_p[bus]
                if  p > 0:
                    profit = profit + p*self.cost_retail
                else:
                    profit = profit + p*self.cost_net_metering
            return (profit, emissions)
        

        def extractBatteryOutput() -> float:
            dss.Circuit.SetActiveClass('Storage')
            device = dss.Circuit.FirstElement()
            total_batt = 0
            while device:
                output = dss.CktElement.TotalPowers()
                total_batt = total_batt + output[0]
                device = dss.Circuit.NextElement()
            return total_batt


        runPowerFlowSnapshot()
        violations = checkForViolations()
        if not violations:
            p_loss = extractPowerLosses()
            profit, emissions = extractProfitAndEmissions()
            total_batt = extractBatteryOutput()
            self.uMatrix[index,0] = profit
            self.uMatrix[index,1] = -p_loss
            self.uMatrix[index,2] = -emissions
            self.uMatrix[index,3] = total_batt
        else:
            self.invalidSetpoints.append(index)
            self.uMatrix[index,0] = np.NaN
            self.uMatrix[index,1] = np.NaN
            self.uMatrix[index,2] = np.NaN
            self.uMatrix[index,3] = np.NaN
    

    def normalizeUMatrix(self):
        #delete all rows with NaN in them and remove invalid setpoint alternatives
        self.uMatrix= self.uMatrix[~np.isnan(self.uMatrix).any(axis=1),:]
        if self.uMatrix.size == 0: #Throw error if all setpoint alternaitives caused powerflow operation violations
            raise RuntimeError("All the setpoints requested by the applications are causing powerflow operation violations!")
        for i in range(-len(self.invalidSetpoints),0):
            del self.setpointSetVector["setpointSets"][self.invalidSetpoints[i]]
        #normalize uMatrix
        for i in DeconflictionMethod.numberOfMetrics:
            maxVal = self.uMatrix[:,i].max()
            minVal = self.uMatrix[:,i].min()
            trueMax = maxVal - minVal
            for j in self.numberOfSets:
                self.uMatrix[j,i] = (self.uMatrix[j,i] - minVal) / trueMax

    
    def calculateCriteriaPreferenceWeightVector(self):
        def calculateWeight(sum, start, end):
            if start < end:
                return calculateWeight(sum+1.0/float(start), start+1, end)
            elif start == end:
                return (sum + 1.0/float(end))*(1.0/float(end))
        for i in range(DeconflictionMethod.numberOfMetrics):
            self.rVector[i,1] = calculateWeight(0.0, i+1, DeconflictionMethod.numberOfMetrics)
    

    def calculateSetpointAlternativesPriorityMatrix(self):
        self.pMatrix = np.identity(self.uMatrix.shape[0])
        #TODO: formulate algorithm for forming the setpoint alternatives priority matrix


    def calculateCriteriaPriorityMatrix(self):
        self.cMatrix = np.identity(self.uMatrix.shape[1])
        #TODO: formulate algorithm for forming the criteria priority matrix
        
            
    def deconflict(self) -> Dict:
        #TODO: call the opendss solver for each alternative setpoint Set and create the U matrix
        for timeVal in self.conflictMatrix.get("timestamps",{}):
            self.conflictTime = max(self.conflictTime, timeVal)
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
            resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
        return (False, resolutionVector)


    
#for testing purposes only must be removed for actual implementation
if __name__ == "__main__":
    cM = {}
    with open("/home/vale/git/app-deconfliction/competing-apps/deconfliction-pipeline/rbdm/ConflictMatrix.json","r") as cmf:
        cM = json.load(cmf)
    rbdm = DeconflictionMethod(cM)
    dss.Text.Command('Redirect ./123Bus/Run_IEEE123Bus.dss')
    dss.Text.Command('Compile Run_IEEE123Bus.dss')