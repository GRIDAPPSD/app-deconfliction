#TODO: Copyright text

import csv
import itertools
import json
import math
from pathlib import Path
import sys
import time
from typing import Dict, List

import numpy as np
import opendssdirect as dss
from opendssdirect.utils import Iterator

# find and add shared directory to path hopefully wherever it is from here
sharedDir = Path(__file__).parent.parent.parent.resolve() / "shared"
sys.path.append(str(sharedDir))

import MethodUtil

class DeconflictionMethod:
    maxAnalogSetpoints = 15 #default range length for analog setpoints.
    numberOfMetrics = 4 #defines the number of columns in the U matrix.
    maxTapBudget = 3 #defines the maximum tap budget allowed in a deconfliction scenario.

    def __init__(self, conflictMatrix: Dict = {}):
        #self.conflictMatrix = MethodUtil.ConflictSubMatrix
        self.conflictMatrix = conflictMatrix
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
        csvDataFile = Path(__file__).parent.parent.parent.resolve() / 'sim-starter' / 'time-series.csv'
        self.csvData = []
        with open(csvDataFile, 'r', newline='') as cf:
            csvReader = csv.reader(cf)
            self.csvData = list(csvReader)
            del self.csvData[0]
        #intialize OpenDSS model.
        openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        dss.Text.Command(f'Redirect {str(openDssFile)}')
        dss.Text.Command(f'Compile {str(openDssFile)}')
        dss.Solution.SolveNoControl()
        self.initializeDistributedConflictMatrix()
        self.rVector = np.empty((DeconflictionMethod.numberOfMetrics, 1))
        self.calculateCriteriaPreferenceWeightVector()
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
        self.criteriaResults = {}

    def initializeDistributedConflictMatrix(self):
        distributedAreasEquipmentFile = Path(__file__).parent.resolve() / "AddressableEquipment.json"
        self.distributedEquipment = {}
        with open(distributedAreasEquipmentFile,"r") as df:
            self.distributedEquipment = json.load(df)
    

    def buildSetpointsVector(self, conflictMatrix:Dict) -> Dict:
        def getTapPosition() -> Dict:
            #TODO: don't hardcode tap conversion
            xfmr_id = dss.Transformers.First()
            tap_vals = {}
            while xfmr_id:
                regulationPerStep = (dss.Transformers.MaxTap() - dss.Transformers.MinTap()) / dss.Transformers.NumTaps()
                tap_vals[dss.Transformers.Name()] = round((dss.Transformers.Tap()-1)/regulationPerStep)
                xfmr_id = dss.Transformers.Next()
            return tap_vals
        

        setpointSetVector = {
            "setpointIndexMap": [],
            "setpointSets": []
        }
        tapVals = getTapPosition()
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
                tapVal = tapVals.get(setpoint.split(".")[1])
                printStr = {
                    "device": setpoint,
                    "tapPosition": tapVal,
                    "maxValue": maxValue,
                    "minValue": minValue
                }
#                print(f"{json.dumps(printStr, indent=4)}")
                if tapVal is not None:
                    if maxValue > tapVal + DeconflictionMethod.maxTapBudget:
                        maxValue = tapVal + DeconflictionMethod.maxTapBudget
                    elif maxValue < tapVal - DeconflictionMethod.maxTapBudget:
                        maxValue = tapVal - DeconflictionMethod.maxTapBudget
                        minValue = maxValue
                    if minValue > tapVal + DeconflictionMethod.maxTapBudget:
                        minValue = tapVal + DeconflictionMethod.maxTapBudget
                        maxValue = minValue
                    elif minValue < tapVal - DeconflictionMethod.maxTapBudget:
                        minValue = tapVal - DeconflictionMethod.maxTapBudget
                else:
                    raise RuntimeError(f"no tap postion was found for RatioTapChanger, {setpoint.split('.')[1]}")
                if minValue < maxValue:
                    setpointRanges.append(range(minValue, maxValue + 1))
                elif minValue == maxValue:
                    setpointRanges.append([maxValue])
                else:
                    raise RuntimeError("minValue is somehow greater than maxValue!")

            else:
                raise RuntimeError(f"Unrecognized setpoint in the Conflict Matrix: {setpoint}")
        setIter = itertools.product(*setpointRanges)
        for i in setIter:
            setpointSetVector["setpointSets"].append(i)
        return setpointSetVector
    

    
    def checkForViolations(self) -> bool:
        bus_volt = dss.Circuit.AllBusMagPu()
        #print(min(bus_volt), max(bus_volt))
        if min(bus_volt) < 0.90:
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


    def extractPowerLosses(self) -> float:
        loss = dss.Circuit.Losses()
        return loss[0]/1000


    def extractProfitAndEmissions(self) -> tuple:
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


    def extractBatteryOutput(self) -> float:
        dss.Circuit.SetActiveClass('Storage')
        device = dss.Circuit.FirstElement()
        total_batt = 0
        while device:
            output = dss.CktElement.TotalPowers()
            total_batt = total_batt + output[0]
            device = dss.Circuit.NextElement()
        return total_batt

    def buildUMatrix(self, setpoints: List, setpointNames: List, index: int, simulationData: Dict):
        if index % 100 == 0:
            print(f"processing setpoint Alternative {index} of {self.uMatrix.shape[0]}...")
        self.runPowerFlowSnapshot(setpoints, setpointNames, simulationData)
        violations = self.checkForViolations()
        if not violations:
            p_loss = self.extractPowerLosses()
            profit, emissions = self.extractProfitAndEmissions()
            total_batt = self.extractBatteryOutput()
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
             print("!\n!\n!WARNING: All the setpoints requested by the applications are causing powerflow operation violations! Sending Resolution from previous time.\n!\n!")
             return
        for i in range(-1,-len(self.invalidSetpoints)-1,-1):
            idx = self.invalidSetpoints[i]
            del self.setpointSetVector["setpointSets"][idx]
        #normalize uMatrix
        for i in range(DeconflictionMethod.numberOfMetrics):
            maxVal = self.uMatrix[:,i].max()
            minVal = self.uMatrix[:,i].min()
            trueMax = maxVal - minVal
            for j in range(self.uMatrix.shape[0]):
                self.uMatrix[j,i] = (self.uMatrix[j,i] - minVal) / trueMax

    
    def calculateCriteriaPreferenceWeightVector(self):
        def calculateWeight(sum, start, end):
            if start < end:
                return calculateWeight(sum+1.0/float(start), start+1, end)
            elif start == end:
                return (sum + 1.0/float(end))*(1.0/float(end))
        for i in range(DeconflictionMethod.numberOfMetrics):
            self.rVector[i,0] = calculateWeight(0.0, i+1, DeconflictionMethod.numberOfMetrics)
    

    def calculateSetpointAlternativesPriorityMatrix(self):
        self.pMatrix = np.identity(self.uMatrix.shape[0])
        #TODO: formulate algorithm for forming the setpoint alternatives priority matrix


    def calculateCriteriaPriorityMatrix(self):
        self.cMatrix = np.identity(self.uMatrix.shape[1])
        #TODO: formulate algorithm for forming the criteria priority matrix
        
    def runPowerFlowSnapshot(self, setpoints: List, setpointNames: List, simulationData: Dict):
            if len(setpoints) != len(setpointNames):
                raise RuntimeError("The number of setpoints does not match the number of setpoint names")
            #end model update with measurements
            for i in range(len(setpoints)):
                setpointNameSplit = setpointNames[i].split(".")
                if "BatteryUnit." in setpointNames[i]:
                    #TODO: Figure out opendss command to change storage ouptput
                    batt_kW = -0.001*setpoints[i]
                    dss.run_command(f"Storage.{setpointNameSplit[1]}.kw={batt_kW}")
                elif "RatioTapChanger." in setpointNames[i]:
                    reg = dss.Transformers.First()
                    while reg:
                        if dss.Transformers.Name() == setpointNameSplit[1]:
                            break
                        else:
                            reg = dss.Transformers.Next()
                    regulationPerStep = (dss.Transformers.MaxTap() - dss.Transformers.MinTap()) / dss.Transformers.NumTaps()
                    tapStep = 1.0 + (setpoints[i]*regulationPerStep)
#                     dss.Transformers.Tap(tapStep)
                    dss.run_command(f"Transformer.{setpointNameSplit[1]}.Taps=[1.0 {tapStep}")
            dss.Solution.SolveNoControl()


    def deconflict(self, currentTime: int) -> Dict:
        deconflictStart = time.perf_counter()
        for timeVal in self.conflictMatrix.get("timestamps",{}).values():
            self.conflictTime = max(self.conflictTime, timeVal)
        distributedConflictMatrix = {}
        for area in self.distributedEquipment.keys():
            distributedConflictMatrix[area] = {
                "setpoints": {}
            }
            for equipment in self.distributedEquipment.get(area,[]):
                if equipment in self.conflictMatrix["setpoints"].keys():
                    distributedConflictMatrix[area]["setpoints"][equipment] = self.conflictMatrix["setpoints"][equipment]
            if len(distributedConflictMatrix[area]["setpoints"]) == 0:
                   del distributedConflictMatrix[area]
        #TODO: replace simulationData with device setpoint measurements
        simulationData = {}
        for i in self.csvData:
            if int(i[0]) == self.conflictTime:
                simulationData["load"] = float(i[1])
                simulationData["solar"] = float(i[2])
                simulationData["price"] = float(i[3])
                break
        #TODO: update model with current measurements not data from this csvfile.
        #-----------------------------------------------------------------------
        dss.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
        dss.run_command(f'set loadmult = {simulationData["load"]}')
        #-----------------------------------------------------------------------
        #update OpenDSSModel with 
        #resolve per distributed area
        distributedResolutionVector = {}
        resolutionVector = {
            "setpoints": {},
            "timestamps": {}
        }
        deconflictionFailed = False
        for area in distributedConflictMatrix.keys():
            print(f"deconflicting area {area}...")
            self.setpointSetVector = self.buildSetpointsVector(distributedConflictMatrix[area])
            self.numberOfSets = len(self.setpointSetVector.get("setpointSets",[]))
            self.uMatrix = np.empty((self.numberOfSets, DeconflictionMethod.numberOfMetrics))
            self.invalidSetpoints = []
            for i in range(self.numberOfSets):
                self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"], i, simulationData)
            print(f"{len(self.invalidSetpoints)} setpoint alternatives eliminated due to operational violations.")
            self.normalizeUMatrix()
            if self.uMatrix.size > 0:
                self.calculateCriteriaPriorityMatrix()
                self.calculateSetpointAlternativesPriorityMatrix()
                fVector = self.pMatrix @ self.uMatrix @ self.cMatrix @ self.rVector
                resolutionIndex = fVector.argmax()
                resolutionSetpointSet = self.setpointSetVector["setpointSets"][resolutionIndex]
                distributedResolutionVector = {
                    "setpoints": {},
                    "timestamps": {}
                }
                for i in range(len(resolutionSetpointSet)):
                    distributedResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = resolutionSetpointSet[i]
                    distributedResolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
            else:
                #TODO: figure out a more sophisticated alternative than giving the last resolution when current conflict can't be resolved.
                deconflictionFailed = True
                pastResolutionVector = {
                    "setpoints": {}
                }
                for device in self.setpointSetVector["setpointIndexMap"]:
                    for i in range(self.conflictTime-1, 0, -1):
                        if device in self.resolutionDict[i]["setpoints"].keys():
                            pastResolutionVector["setpoints"][device] = self.resolutionDict[i]["setpoints"][device]
                            break
                resolutionSetpointSet = [0]*len(self.setpointSetVector["setpointIndexMap"])
                for i in range(len(self.setpointSetVector["setpointIndexMap"])):
                    resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
                    resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
                    resolutionSetpointSet[i] = pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
            #update opendssmodel with resolved setpoints for next conflict
            self.runPowerFlowSnapshot(resolutionSetpointSet, self.setpointSetVector["setpointIndexMap"], simulationData)
        deconflictTime = time.perf_counter() - deconflictStart
        p_loss = self.extractPowerLosses()
        profit, emissions = self.extractProfitAndEmissions()
        total_batt = self.extractBatteryOutput()
        self.criteriaResults[self.conflictTime] = [profit, p_loss, emissions, total_batt]
        print(f"deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
        self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
        
        self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
        
        if self.conflictTime == 96:
            #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
            print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
            resultFile = Path(__file__).parent.resolve() / 'resolutionResults.json'
            with resultFile.open(mode="w") as rf:
                json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
                
            conflictFile = Path(__file__).parent.resolve() / 'conflictResults.json'
            with conflictFile.open(mode="w") as rf:
                json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
    
            criteriaFile = Path(__file__).parent.resolve() / 'criteriaResults.json'
            with criteriaFile.open(mode="w") as rf:
                json.dump(self.criteriaResults, rf, indent=4, sort_keys=True)
        return (False, resolutionVector)
    

    def __del__(self):
        resultFile = Path(__file__).parent.resolve() / 'resolutionResults.json'
        with resultFile.open(mode="w") as rf:
            json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)


    
#for testing purposes only must be removed for actual implementation
if __name__ == "__main__":
    cM = {}
    centralConflictMatrixFile = Path(__file__).parent.resolve() / "ConflictMatrix.json"
    centralConflictMatrix = {}
    with open(centralConflictMatrixFile,"r") as cmf:
        centralConflictMatrix = json.load(cmf)
    rbdm = DeconflictionMethod(cM)
    MethodUtil.ConflictSubMatrix["setpoints"] = centralConflictMatrix["setpoints"]
    MethodUtil.ConflictSubMatrix["timestamps"] = centralConflictMatrix["timestamps"]
    fullResFlag, resolutionVector = rbdm.deconflict(12)
    print(f'resolution vector:\n{json.dumps(resolutionVector, indent=4, sort_keys=True)}')