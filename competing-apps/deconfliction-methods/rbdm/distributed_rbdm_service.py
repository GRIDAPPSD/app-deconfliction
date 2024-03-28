from argparse import ArgumentParser
from copy import deepcopy
import itertools
import json
import logging
import math
import os
from pathlib import Path
import time
from typing import Dict, List, Optional

from cimgraph.data_profile import CIM_PROFILE
from cimgraph.models import DistributedArea
import cimgraph.utils as cimUtils
import gridappsd.field_interface.agents.agents as agents_mod
from gridappsd.field_interface.agents import FeederAgent, SwitchAreaAgent, SecondaryAreaAgent
from gridappsd.field_interface.context import LocalContext
from gridappsd.field_interface.interfaces import FieldMessageBus, MessageBusDefinition
import gridappsd.topics as gt
import numpy as np
import opendssdirect as dss
import pandas as pd

#TODO: query gridappsd-python for correct cim_profile instead of hardcoding it.
cim_profile = CIM_PROFILE.RC4_2021.value
agents_mod.set_cim_profile(cim_profile, iec61970_301=7)
# cim_profile = CIM_PROFILE.CIMHUB_2023.value
# agents_mod.set_cim_profile(cim_profile, iec61970_301=8)
cim = agents_mod.cim
logging.basicConfig(format='%(asctime)s::%(levelname)s::%(name)s::%(filename)s::%(lineno)d::%(message)s',
                    filename='DistributedRBDMService.log',
                    filemode='w',
                    level=logging.INFO,
                    encoding='utf-8')
logger = logging.getLogger(__name__)
# dss.Basic.AllowChangeDir(False)
# dss.Basic.AllowEditor(False)
# dss.Basic.AllowForms(False)
dssContext = dss
dssContext.run_command("Clear")
openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
dssContext.run_command(f'Redirect {str(openDssFile)}')
dssContext.run_command(f'Compile {str(openDssFile)}')
dssContext.Solution.SolveNoControl()



class FeederAgentLevelRdbmService(FeederAgent):
    maxAnalogSetpoints = 15  #default range length for analog setpoints.
    numberOfMetrics = 4  #defines the number of columns in the U matrix.
    maxTapBudget = 3  #defines the maximum tap budget allowed in a deconfliction scenario.

    def __init__(self,
                 upstream_message_bus_def: MessageBusDefinition,
                 downstream_message_bus_def: MessageBusDefinition,
                 service_config: Dict,
                 feeder_dict: Optional[Dict] = None,
                 simulation_id: Optional[str] = None):
        super().__init__(upstream_message_bus_def, downstream_message_bus_def,
                         service_config, feeder_dict, simulation_id)
        self.isServiceInitialized = False
        self.conflictMatrix = {}
        self.setpointSetVector = None
        self.numberOfSets = 0
        self.uMatrix = None
        self.pMatrix = None
        self.cMatrix = None
        self.invalidSetpoints = []
        self.conflictTime = -1
        # The following constants should be read from a deconfliction configuration file to allow for buisness variability.
        self.cost_transmission = 0.02817  #transmission price $/kWh
        self.cost_retail = 0.1090  #retail rate
        self.cost_net_metering = 0.01786  #feed-in tariff
        self.cost_diesel = 0.34  #diesel fuel cost
        self.cost_lng = 0.25  #lng fuel cost
        self.emissions_transmission = 1.205  #grid co2 lb/kWh
        self.emissions_diesel = 2.44
        self.emissions_lng = 0.97
        csvDataFile = Path(__file__).parent.parent.parent.resolve() / 'sim-starter' / 'time-series.csv'
        self.csvData = pd.read_csv(csvDataFile)
        # dssContext = dss.NewContext()
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()
        self.rVector = np.empty((FeederAgentLevelRdbmService.numberOfMetrics, 1))
        self.calculateCriteriaPreferenceWeightVector()
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
        self.criteriaResults = {}
        self.addressableEquipmentNames = []
        if self.feeder_area is not None:
            populateAddressableEquipment(self.feeder_area)
            for c, objDict in self.feeder_area.addressable_equipment.items():
                for obj in objDict.values():
                    logger.info(f"Found {c.__name__} called {obj.name} in addressable equipment for feeder area "
                                f"{self.feeder_area.container.mRID}.")
                    self.addressableEquipmentNames.append(obj.name)
            self.isServiceInitialized = True

    def on_request(self, message_bus: FieldMessageBus, headers: Dict, message: Dict):
        if message.get("requestType", "") == "Deconflict":
            resolutionFailed, resolutionVector = self.deconflict(message.get("conflictMatrix", {}))
            returnMessage = {"resolutionFailed": resolutionFailed, "resolutionVector": resolutionVector}
            message_bus.send(headers.get('reply-to'), returnMessage)
        if message.get("requestType", "") == "is_initialized":
            response = {"is_initialized": self.isServiceInitialized}
            message_bus.send(headers.get('reply-to'), response)

    def calculateCriteriaPreferenceWeightVector(self):

        def calculateWeight(sum, start, end):
            if start < end:
                return calculateWeight(sum + 1.0 / float(start), start + 1, end)
            elif start == end:
                return (sum + 1.0 / float(end)) * (1.0 / float(end))

        for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
            self.rVector[i, 0] = calculateWeight(0.0, i + 1, FeederAgentLevelRdbmService.numberOfMetrics)

    def buildSetpointsVector(self, conflictMatrix: Dict) -> Dict:

        def getTapPosition() -> Dict:
            #TODO: don't hardcode tap conversion
            xfmr_id = dssContext.Transformers.First()
            tap_vals = {}
            while xfmr_id:
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tap_vals[dssContext.Transformers.Name()] = round((dssContext.Transformers.Tap() - 1) 
                                                                      / regulationPerStep)
                xfmr_id = dssContext.Transformers.Next()
            return tap_vals

        setpointSetVector = {"setpointIndexMap": [], "setpointSets": []}
        tapVals = getTapPosition()
        setpointRanges = []
        for setpoint in conflictMatrix.get("setpoints", {}).keys():
            values = []
            setpointSetVector["setpointIndexMap"].append(setpoint)
            for setpointValue in conflictMatrix.get("setpoints", {}).get(setpoint, {}).values():
                values.append(setpointValue)
            maxValue = max(values)
            minValue = min(values)
            if "BatteryUnit." in setpoint:
                step = (maxValue - minValue) / (FeederAgentLevelRdbmService.maxAnalogSetpoints - 1)
                setpointRange = []
                for i in range(FeederAgentLevelRdbmService.maxAnalogSetpoints):
                    setpointRange.append(minValue + (i * step))
                setpointRanges.append(setpointRange)
            elif "RatioTapChanger." in setpoint:
                tapVal = tapVals.get(setpoint.split(".")[1])
                printStr = {"device": setpoint, "tapPosition": tapVal, "maxValue": maxValue, "minValue": minValue}
                # print(f"{json.dumps(printStr, indent=4)}")
                if tapVal is not None:
                    if maxValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
                        maxValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
                    elif maxValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
                        maxValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
                        minValue = maxValue
                    if minValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
                        minValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
                        maxValue = minValue
                    elif minValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
                        minValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
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

    def runPowerFlowSnapshot(self, setpoints: List, setpointNames: List, simulationData: Dict):
        if len(setpoints) != len(setpointNames):
            raise RuntimeError("The number of setpoints does not match the number of setpoint names")
        # end model update with measurements
        for i in range(len(setpoints)):
            setpointNameSplit = setpointNames[i].split(".")
            if "BatteryUnit." in setpointNames[i]:
                #TODO: Figure out opendss command to change storage ouptput
                batt_kW = -0.001 * setpoints[i]
                dssContext.run_command(f"Storage.{setpointNameSplit[1]}.kw={batt_kW}")
            elif "RatioTapChanger." in setpointNames[i]:
                reg = dssContext.Transformers.First()
                while reg:
                    if dssContext.Transformers.Name() == setpointNameSplit[1]:
                        break
                    else:
                        reg = dssContext.Transformers.Next()
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tapStep = 1.0 + (setpoints[i] * regulationPerStep)
                # dssContext.Transformers.Tap(tapStep)
                dssContext.run_command(f"Transformer.{setpointNameSplit[1]}.Taps=[1.0 {tapStep}")
        dssContext.Solution.SolveNoControl()

    def checkForViolations(self) -> bool:
        bus_volt = dssContext.Circuit.AllBusMagPu()
        # print(min(bus_volt), max(bus_volt))
        if min(bus_volt) < 0.90:
            return True
        elif max(bus_volt) > 1.10:
            return True
        sub = dssContext.Circuit.TotalPower()
        sub_kva = math.sqrt((sub[0] * sub[0]) + (sub[1] * sub[1]))
        if sub_kva > 5000:
            return True
        dssContext.Circuit.SetActiveClass('Transformer')
        device = dssContext.Circuit.FirstElement()
        while device:
            a = dssContext.CktElement.CurrentsMagAng()
            if a[0] > 668:
                return True
            device = dssContext.Circuit.NextElement()
        return False

    def extractPowerLosses(self) -> float:
        loss = dssContext.Circuit.Losses()
        return loss[0] / 1000

    def extractProfitAndEmissions(self) -> tuple:
        bus_shunt_p = {}
        total_load = 0
        total_pv = 0
        total_dg = 0
        profit = 0
        emissions = 0
        sub = dssContext.Circuit.TotalPower()
        p_sub = sub[0]
        dssContext.Circuit.SetActiveClass('Load')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            bus_shunt_p[bus[0]] = output[0]
            total_load = total_load + output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('PVSystem')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            total_pv = total_pv + output[0]
            if bus[0] in bus_shunt_p.keys():
                bus_shunt_p[bus[0]] = bus_shunt_p[bus[0]] + output[0]
            else:
                bus_shunt_p[bus[0]] = output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('Generator')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            p_dg = output[0]
            total_dg = total_dg + p_dg
            if 'dies' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_diesel
                emissions = emissions - p_dg * self.emissions_diesel
            elif 'lng' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_lng
                emissions = emissions - p_dg * self.emissions_lng
            device = dssContext.Circuit.NextElement()
        emissions = emissions - p_sub * self.emissions_transmission
        profit = profit + p_sub * self.cost_transmission
        for bus in list(bus_shunt_p.keys()):
            p = bus_shunt_p[bus]
            if p > 0:
                profit = profit + p * self.cost_retail
            else:
                profit = profit + p * self.cost_net_metering
        return (profit, emissions)

    def extractBatteryOutput(self) -> float:
        dssContext.Circuit.SetActiveClass('Storage')
        device = dssContext.Circuit.FirstElement()
        total_batt = 0
        while device:
            output = dssContext.CktElement.TotalPowers()
            total_batt = total_batt + output[0]
            device = dssContext.Circuit.NextElement()
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
            self.uMatrix[index, 0] = profit
            self.uMatrix[index, 1] = -p_loss
            self.uMatrix[index, 2] = -emissions
            self.uMatrix[index, 3] = total_batt
        else:
            self.invalidSetpoints.append(index)
            self.uMatrix[index, 0] = np.NaN
            self.uMatrix[index, 1] = np.NaN
            self.uMatrix[index, 2] = np.NaN
            self.uMatrix[index, 3] = np.NaN

    def normalizeUMatrix(self):
        # delete all rows with NaN in them and remove invalid setpoint alternatives
        self.uMatrix = self.uMatrix[~np.isnan(self.uMatrix).any(axis=1), :]
        if self.uMatrix.size == 0:  #Throw error if all setpoint alternaitives caused powerflow operation violations
            print("!\n!\n!WARNING: All the setpoints requested by the applications are causing powerflow operation "
                  "violations! Sending Resolution from previous time.\n!\n!")
            return
        for i in range(-1, -len(self.invalidSetpoints) - 1, -1):
            idx = self.invalidSetpoints[i]
            del self.setpointSetVector["setpointSets"][idx]
        #normalize uMatrix
        for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
            maxVal = self.uMatrix[:, i].max()
            minVal = self.uMatrix[:, i].min()
            trueMax = maxVal - minVal
            for j in range(self.uMatrix.shape[0]):
                self.uMatrix[j, i] = (self.uMatrix[j, i] - minVal) / trueMax

    def calculateCriteriaPriorityMatrix(self):
        self.cMatrix = np.identity(self.uMatrix.shape[1])
        #TODO: formulate algorithm for forming the criteria priority matrix

    def calculateSetpointAlternativesPriorityMatrix(self):
        self.pMatrix = np.identity(self.uMatrix.shape[0])
        #TODO: formulate algorithm for forming the setpoint alternatives priority matrix

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = deepcopy(conflictMatrix)
        for timeVal in self.conflictMatrix.get("timestamps", {}).values():
            self.conflictTime = max(self.conflictTime, timeVal)
        distributedConflictMatrix = {"setpoints": {}}
        for device in self.conflictMatrix.get("setpoints", {}).keys():
            deviceName = (device.split("."))[1]
            if deviceName in self.addressableEquipmentNames:
                distributedConflictMatrix["setpoints"][device] = deepcopy(
                    self.conflictMatrix.get("setpoints", {}).get(device, {})
                )
                # add distributed peak_shaving app setpoint
                distributedConflictMatrix["setpoints"][device].update(getPeakShavingSetpoint(self.conflictTime, device))
        for device in distributedConflictMatrix.get("setpoints", {}).keys():
            del self.conflictMatrix["setpoints"][device]
        #TODO: replace simulationData with device setpoint measurements
        simulationData = {}
        for i in self.csvData.index:
            if int(self.csvData.at[i, "Time"]) == self.conflictTime:
                simulationData["load"] = float(self.csvData.at[i, "Loadshape"])
                simulationData["solar"] = float(self.csvData.at[i, "Solar"])
                simulationData["price"] = float(self.csvData.at[i, "Price"])
                break
        distributedResolutionVector = {}
        resolutionVector = {"setpoints": {}, "timestamps": {}}
        deconflictionFailed = False
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()

        if len(distributedConflictMatrix.get("setpoints", {}).keys()) > 0:
            
            #TODO: update model with current measurements not data from this csvfile.
            #-----------------------------------------------------------------------
            dssContext.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
            dssContext.run_command(f'set loadmult = {simulationData["load"]}')
            #-----------------------------------------------------------------------
            print(f"deconflicting area {self.downstream_message_bus_def.id}...")
            self.setpointSetVector = self.buildSetpointsVector(distributedConflictMatrix)
            self.numberOfSets = len(self.setpointSetVector.get("setpointSets", []))
            self.uMatrix = np.empty((self.numberOfSets, FeederAgentLevelRdbmService.numberOfMetrics))
            self.invalidSetpoints = []
            for i in range(self.numberOfSets):
                self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"],
                                  i, simulationData)
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
                    distributedResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        resolutionSetpointSet[i]
                    distributedResolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        self.conflictTime
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
            else:
                #TODO: figure out a more sophisticated alternative than giving the last resolution when current conflict can't be resolved.
                deconflictionFailed = True
                pastResolutionVector = {"setpoints": {}}
                for device in self.setpointSetVector["setpointIndexMap"]:
                    for i in range(self.conflictTime - 1, 0, -1):
                        if device in self.resolutionDict[i]["setpoints"].keys():
                            pastResolutionVector["setpoints"][device] = self.resolutionDict[i]["setpoints"][device]
                            break
                resolutionSetpointSet = [0] * len(self.setpointSetVector["setpointIndexMap"])
                for i in range(len(self.setpointSetVector["setpointIndexMap"])):
                    resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
                    resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
                    resolutionSetpointSet[i] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
            # update opendssmodel with resolved setpoints for next conflict
            self.runPowerFlowSnapshot(resolutionSetpointSet, self.setpointSetVector["setpointIndexMap"], simulationData)
        # localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
        # for agent_id, agent_details in localContextResponse.items():
        #     if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
        #         switchAreaRequestTopic = gt.field_agent_request_queue(self.downstream_message_bus.id, agent_id)
        #         switchAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
        #         deconflictionResponse = self.downstream_message_bus.get_response(switchAreaRequestTopic,
        #                                                                          switchAreaRequestMessage,
        #                                                                          timeout=60)
        #         distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
        #                                                                 {"setpoints": {}, "timestamps": {}})
        #         resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
        #         resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - deconflictStart
        p_loss = self.extractPowerLosses()
        profit, emissions = self.extractProfitAndEmissions()
        total_batt = self.extractBatteryOutput()
        self.criteriaResults[self.conflictTime] = [profit, p_loss, emissions, total_batt]
        print(f"{self.agent_id}:deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
        self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
        self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
        if self.conflictTime == 96:
            #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
            print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
            resultFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
                                                            'resolutionResults.json'
            with resultFile.open(mode="w") as rf:
                json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
            conflictFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
                                                                'conflictResults.json'
            with conflictFile.open(mode="w") as rf:
                json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
            criteriaFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
                                                                'criteriaResults.json'
            with criteriaFile.open(mode="w") as rf:
                json.dump(self.criteriaResults, rf, indent=4, sort_keys=True)
        return (False, resolutionVector)


class SwitchAreaAgentLevelRbdmService(SwitchAreaAgent):
    maxAnalogSetpoints = 15  #default range length for analog setpoints.
    numberOfMetrics = 4  #defines the number of columns in the U matrix.
    maxTapBudget = 3  #defines the maximum tap budget allowed in a deconfliction scenario.

    def __init__(self,
                 upstream_message_bus_def: MessageBusDefinition,
                 downstream_message_bus_def: MessageBusDefinition,
                 service_config: Dict,
                 switch_area_dict: Optional[Dict] = None,
                 simulation_id: Optional[str] = None):
        super().__init__(upstream_message_bus_def, downstream_message_bus_def,
                         service_config, switch_area_dict, simulation_id)
        self.isServiceInitialized = False
        self.conflictMatrix = {}
        self.setpointSetVector = None
        self.numberOfSets = 0
        self.uMatrix = None
        self.pMatrix = None
        self.cMatrix = None
        self.invalidSetpoints = []
        self.conflictTime = -1
        # The following constants should be read from a deconfliction configuration file to allow for buisness variability.
        self.cost_transmission = 0.02817  #transmission price $/kWh
        self.cost_retail = 0.1090  #retail rate
        self.cost_net_metering = 0.01786  #feed-in tariff
        self.cost_diesel = 0.34  #diesel fuel cost
        self.cost_lng = 0.25  #lng fuel cost
        self.emissions_transmission = 1.205  #grid co2 lb/kWh
        self.emissions_diesel = 2.44
        self.emissions_lng = 0.97
        csvDataFile = Path(__file__).parent.parent.parent.resolve() / 'sim-starter' / 'time-series.csv'
        self.csvData = pd.read_csv(csvDataFile)
        # dssContext = dss.NewContext()
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()
        self.rVector = np.empty((SwitchAreaAgentLevelRbdmService.numberOfMetrics, 1))
        self.calculateCriteriaPreferenceWeightVector()
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
        self.criteriaResults = {}
        self.addressableEquipmentNames = []
        if self.switch_area is not None:
            populateAddressableEquipment(self.switch_area)
            for c, objDict in self.switch_area.addressable_equipment.items():
                for obj in objDict.values():
                    logger.info(f"Found {c.__name__} called {obj.name} in addressable equipment for switch area {self.switch_area.container.mRID}.")
                    self.addressableEquipmentNames.append(obj.name)
            self.isServiceInitialized = True

    def on_request(self, message_bus: FieldMessageBus, headers: Dict, message: Dict):
        if message.get("requestType", "") == "Deconflict":
            resolutionFailed, resolutionVector = self.deconflict(message.get("conflictMatrix", {}))
            returnMessage = {"resolutionFailed": resolutionFailed, "resolutionVector": resolutionVector}
            message_bus.send(headers.get('reply-to'), returnMessage)
        if message.get("requestType", "") == "is_initialized":
            response = {"is_initialized": self.isServiceInitialized}
            message_bus.send(headers.get('reply-to'), response)

    def calculateCriteriaPreferenceWeightVector(self):

        def calculateWeight(sum, start, end):
            if start < end:
                return calculateWeight(sum + 1.0 / float(start), start + 1, end)
            elif start == end:
                return (sum + 1.0 / float(end)) * (1.0 / float(end))

        for i in range(SwitchAreaAgentLevelRbdmService.numberOfMetrics):
            self.rVector[i, 0] = calculateWeight(0.0, i + 1, SwitchAreaAgentLevelRbdmService.numberOfMetrics)

    def buildSetpointsVector(self, conflictMatrix: Dict) -> Dict:

        def getTapPosition() -> Dict:
            #TODO: don't hardcode tap conversion
            xfmr_id = dssContext.Transformers.First()
            tap_vals = {}
            while xfmr_id:
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tap_vals[dssContext.Transformers.Name()] = round((dssContext.Transformers.Tap() - 1) \
                                                                      / regulationPerStep)
                xfmr_id = dssContext.Transformers.Next()
            return tap_vals

        setpointSetVector = {"setpointIndexMap": [], "setpointSets": []}
        tapVals = getTapPosition()
        setpointRanges = []
        for setpoint in conflictMatrix.get("setpoints", {}).keys():
            values = []
            setpointSetVector["setpointIndexMap"].append(setpoint)
            for setpointValue in conflictMatrix.get("setpoints", {}).get(setpoint, {}).values():
                values.append(setpointValue)
            maxValue = max(values)
            minValue = min(values)
            if "BatteryUnit." in setpoint:
                step = (maxValue - minValue) / (SwitchAreaAgentLevelRbdmService.maxAnalogSetpoints - 1)
                setpointRange = []
                for i in range(SwitchAreaAgentLevelRbdmService.maxAnalogSetpoints):
                    setpointRange.append(minValue + (i * step))
                setpointRanges.append(setpointRange)
            elif "RatioTapChanger." in setpoint:
                tapVal = tapVals.get(setpoint.split(".")[1])
                printStr = {"device": setpoint, "tapPosition": tapVal, "maxValue": maxValue, "minValue": minValue}
                # print(f"{json.dumps(printStr, indent=4)}")
                if tapVal is not None:
                    if maxValue > tapVal + SwitchAreaAgentLevelRbdmService.maxTapBudget:
                        maxValue = tapVal + SwitchAreaAgentLevelRbdmService.maxTapBudget
                    elif maxValue < tapVal - SwitchAreaAgentLevelRbdmService.maxTapBudget:
                        maxValue = tapVal - SwitchAreaAgentLevelRbdmService.maxTapBudget
                        minValue = maxValue
                    if minValue > tapVal + SwitchAreaAgentLevelRbdmService.maxTapBudget:
                        minValue = tapVal + SwitchAreaAgentLevelRbdmService.maxTapBudget
                        maxValue = minValue
                    elif minValue < tapVal - SwitchAreaAgentLevelRbdmService.maxTapBudget:
                        minValue = tapVal - SwitchAreaAgentLevelRbdmService.maxTapBudget
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

    def runPowerFlowSnapshot(self, setpoints: List, setpointNames: List, simulationData: Dict):
        if len(setpoints) != len(setpointNames):
            raise RuntimeError("The number of setpoints does not match the number of setpoint names")
        # end model update with measurements
        for i in range(len(setpoints)):
            setpointNameSplit = setpointNames[i].split(".")
            if "BatteryUnit." in setpointNames[i]:
                #TODO: Figure out opendss command to change storage ouptput
                batt_kW = -0.001 * setpoints[i]
                dssContext.run_command(f"Storage.{setpointNameSplit[1]}.kw={batt_kW}")
            elif "RatioTapChanger." in setpointNames[i]:
                reg = dssContext.Transformers.First()
                while reg:
                    if dssContext.Transformers.Name() == setpointNameSplit[1]:
                        break
                    else:
                        reg = dssContext.Transformers.Next()
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tapStep = 1.0 + (setpoints[i] * regulationPerStep)
                # dssContext.Transformers.Tap(tapStep)
                dssContext.run_command(f"Transformer.{setpointNameSplit[1]}.Taps=[1.0 {tapStep}")
        dssContext.Solution.SolveNoControl()

    def checkForViolations(self) -> bool:
        bus_volt = dssContext.Circuit.AllBusMagPu()
        # print(min(bus_volt), max(bus_volt))
        if min(bus_volt) < 0.90:
            return True
        elif max(bus_volt) > 1.10:
            return True
        sub = dssContext.Circuit.TotalPower()
        sub_kva = math.sqrt((sub[0] * sub[0]) + (sub[1] * sub[1]))
        if sub_kva > 5000:
            return True
        dssContext.Circuit.SetActiveClass('Transformer')
        device = dssContext.Circuit.FirstElement()
        while device:
            a = dssContext.CktElement.CurrentsMagAng()
            if a[0] > 668:
                return True
            device = dssContext.Circuit.NextElement()
        return False

    def extractPowerLosses(self) -> float:
        loss = dssContext.Circuit.Losses()
        return loss[0] / 1000

    def extractProfitAndEmissions(self) -> tuple:
        bus_shunt_p = {}
        total_load = 0
        total_pv = 0
        total_dg = 0
        profit = 0
        emissions = 0
        sub = dssContext.Circuit.TotalPower()
        p_sub = sub[0]
        dssContext.Circuit.SetActiveClass('Load')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            bus_shunt_p[bus[0]] = output[0]
            total_load = total_load + output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('PVSystem')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            total_pv = total_pv + output[0]
            if bus[0] in bus_shunt_p.keys():
                bus_shunt_p[bus[0]] = bus_shunt_p[bus[0]] + output[0]
            else:
                bus_shunt_p[bus[0]] = output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('Generator')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            p_dg = output[0]
            total_dg = total_dg + p_dg
            if 'dies' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_diesel
                emissions = emissions - p_dg * self.emissions_diesel
            elif 'lng' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_lng
                emissions = emissions - p_dg * self.emissions_lng
            device = dssContext.Circuit.NextElement()
        emissions = emissions - p_sub * self.emissions_transmission
        profit = profit + p_sub * self.cost_transmission
        for bus in list(bus_shunt_p.keys()):
            p = bus_shunt_p[bus]
            if p > 0:
                profit = profit + p * self.cost_retail
            else:
                profit = profit + p * self.cost_net_metering
        return (profit, emissions)

    def extractBatteryOutput(self) -> float:
        dssContext.Circuit.SetActiveClass('Storage')
        device = dssContext.Circuit.FirstElement()
        total_batt = 0
        while device:
            output = dssContext.CktElement.TotalPowers()
            total_batt = total_batt + output[0]
            device = dssContext.Circuit.NextElement()
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
            self.uMatrix[index, 0] = profit
            self.uMatrix[index, 1] = -p_loss
            self.uMatrix[index, 2] = -emissions
            self.uMatrix[index, 3] = total_batt
        else:
            self.invalidSetpoints.append(index)
            self.uMatrix[index, 0] = np.NaN
            self.uMatrix[index, 1] = np.NaN
            self.uMatrix[index, 2] = np.NaN
            self.uMatrix[index, 3] = np.NaN

    def normalizeUMatrix(self):
        # delete all rows with NaN in them and remove invalid setpoint alternatives
        self.uMatrix = self.uMatrix[~np.isnan(self.uMatrix).any(axis=1), :]
        if self.uMatrix.size == 0:  # Throw error if all setpoint alternaitives caused powerflow operation violations
            print("!\n!\n!WARNING: All the setpoints requested by the applications are causing powerflow operation "
                  "violations! Sending Resolution from previous time.\n!\n!")
            return
        for i in range(-1, -len(self.invalidSetpoints) - 1, -1):
            idx = self.invalidSetpoints[i]
            del self.setpointSetVector["setpointSets"][idx]
        # normalize uMatrix
        for i in range(SwitchAreaAgentLevelRbdmService.numberOfMetrics):
            maxVal = self.uMatrix[:, i].max()
            minVal = self.uMatrix[:, i].min()
            trueMax = maxVal - minVal
            for j in range(self.uMatrix.shape[0]):
                self.uMatrix[j, i] = (self.uMatrix[j, i] - minVal) / trueMax

    def calculateCriteriaPriorityMatrix(self):
        self.cMatrix = np.identity(self.uMatrix.shape[1])
        #TODO: formulate algorithm for forming the criteria priority matrix

    def calculateSetpointAlternativesPriorityMatrix(self):
        self.pMatrix = np.identity(self.uMatrix.shape[0])
        #TODO: formulate algorithm for forming the setpoint alternatives priority matrix

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = deepcopy(conflictMatrix)
        for timeVal in self.conflictMatrix.get("timestamps", {}).values():
            self.conflictTime = max(self.conflictTime, timeVal)
        distributedConflictMatrix = {"setpoints": {}}
        for device in self.conflictMatrix.get("setpoints", {}).keys():
            deviceName = (device.split("."))[1]
            if deviceName in self.addressableEquipmentNames:
                distributedConflictMatrix["setpoints"][device] = deepcopy(
                    self.conflictMatrix.get("setpoints", {}).get(device, {}))
                # add distributed peak_shaving app setpoint
                distributedConflictMatrix["setpoints"][device].update(getPeakShavingSetpoint(self.conflictTime, device))
        for device in distributedConflictMatrix.get("setpoints", {}).keys():
            del self.conflictMatrix["setpoints"][device]
        simulationData = {}
        for i in self.csvData.index:
            if int(self.csvData.at[i, "Time"]) == self.conflictTime:
                simulationData["load"] = float(self.csvData.at[i, "Loadshape"])
                simulationData["solar"] = float(self.csvData.at[i, "Solar"])
                simulationData["price"] = float(self.csvData.at[i, "Price"])
                break
        distributedResolutionVector = {}
        resolutionVector = {"setpoints": {}, "timestamps": {}}
        deconflictionFailed = False
        # dssContext = dss.NewContext()
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()

        if len(distributedConflictMatrix.get("setpoints", {}).keys()) > 0:
            #TODO: update model with current measurements not data from this csvfile.
            #-----------------------------------------------------------------------
            dssContext.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
            dssContext.run_command(f'set loadmult = {simulationData["load"]}')
            #-----------------------------------------------------------------------
            print(f"deconflicting area {self.downstream_message_bus_def.id}...")
            self.setpointSetVector = self.buildSetpointsVector(distributedConflictMatrix)
            self.numberOfSets = len(self.setpointSetVector.get("setpointSets", []))
            self.uMatrix = np.empty((self.numberOfSets, FeederAgentLevelRdbmService.numberOfMetrics))
            self.invalidSetpoints = []
            for i in range(self.numberOfSets):
                self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"],
                                  i, simulationData)
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
                    distributedResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        resolutionSetpointSet[i]
                    distributedResolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        self.conflictTime
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
            else:
                #TODO: figure out a more sophisticated alternative than giving the last resolution when current conflict can't be resolved.
                deconflictionFailed = True
                pastResolutionVector = {"setpoints": {}}
                for device in self.setpointSetVector["setpointIndexMap"]:
                    for i in range(self.conflictTime - 1, 0, -1):
                        if device in self.resolutionDict[i]["setpoints"].keys():
                            pastResolutionVector["setpoints"][device] = self.resolutionDict[i]["setpoints"][device]
                            break
                resolutionSetpointSet = [0] * len(self.setpointSetVector["setpointIndexMap"])
                for i in range(len(self.setpointSetVector["setpointIndexMap"])):
                    resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
                    resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
                    resolutionSetpointSet[i] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
            # update opendssmodel with resolved setpoints for next conflict
            self.runPowerFlowSnapshot(resolutionSetpointSet, self.setpointSetVector["setpointIndexMap"], simulationData)
        # localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
        # for agent_id, agent_details in localContextResponse.items():
        #     if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
        #         secondaryAreaRequestTopic = gt.field_agent_request_queue(self.downstream_message_bus.id, agent_id)
        #         secondaryAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
        #         deconflictionResponse = self.downstream_message_bus.get_response(secondaryAreaRequestTopic,
        #                                                                          secondaryAreaRequestMessage,
        #                                                                          timeout=60)
        #         distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
        #                                                                 {"setpoints": {}, "timestamps": {}})
        #         resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
        #         resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - deconflictStart
        p_loss = self.extractPowerLosses()
        profit, emissions = self.extractProfitAndEmissions()
        total_batt = self.extractBatteryOutput()
        self.criteriaResults[self.conflictTime] = [profit, p_loss, emissions, total_batt]
        print(f"{self.agent_id}:deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
        self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
        self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
        if self.conflictTime == 96:
            #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
            print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
            resultFile = Path(__file__).parent.resolve() \
                         / f'switch_area_{self.downstream_message_bus_def.id}_resolutionResults.json'
            with resultFile.open(mode="w") as rf:
                json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
            conflictFile = Path(__file__).parent.resolve() \
                           / f'switch_area_{self.downstream_message_bus_def.id}_conflictResults.json'
            with conflictFile.open(mode="w") as rf:
                json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
            criteriaFile = Path(__file__).parent.resolve() \
                           / f'switch_area_{self.downstream_message_bus_def.id}_criteriaResults.json'
            with criteriaFile.open(mode="w") as rf:
                json.dump(self.criteriaResults, rf, indent=4, sort_keys=True)
        return (False, resolutionVector)


class SecondaryAreaAgentLevelRbdmService(SecondaryAreaAgent):
    maxAnalogSetpoints = 15  #default range length for analog setpoints.
    numberOfMetrics = 4  #defines the number of columns in the U matrix.
    maxTapBudget = 3  #defines the maximum tap budget allowed in a deconfliction scenario.

    def __init__(self,
                 upstream_message_bus_def: MessageBusDefinition,
                 downstream_message_bus_def: MessageBusDefinition,
                 service_config: Dict,
                 secondary_area_dict: Optional[Dict] = None,
                 simulation_id: Optional[str] = None):
        super().__init__(upstream_message_bus_def, downstream_message_bus_def,
                         service_config, secondary_area_dict, simulation_id)
        self.isServiceInitialized = False
        self.conflictMatrix = {}
        self.setpointSetVector = None
        self.numberOfSets = 0
        self.uMatrix = None
        self.pMatrix = None
        self.cMatrix = None
        self.invalidSetpoints = []
        self.conflictTime = -1
        # The following constants should be read from a deconfliction configuration file to allow for buisness variability.
        self.cost_transmission = 0.02817  #transmission price $/kWh
        self.cost_retail = 0.1090  #retail rate
        self.cost_net_metering = 0.01786  #feed-in tariff
        self.cost_diesel = 0.34  #diesel fuel cost
        self.cost_lng = 0.25  #lng fuel cost
        self.emissions_transmission = 1.205  #grid co2 lb/kWh
        self.emissions_diesel = 2.44
        self.emissions_lng = 0.97
        csvDataFile = Path(__file__).parent.parent.parent.resolve() / 'sim-starter' / 'time-series.csv'
        self.csvData = pd.read_csv(csvDataFile)
        # dssContext = dss.NewContext()
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()
        self.rVector = np.empty((SecondaryAreaAgentLevelRbdmService.numberOfMetrics, 1))
        self.calculateCriteriaPreferenceWeightVector()
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
        self.criteriaResults = {}
        self.addressableEquipmentNames = []
        if self.secondary_area is not None:
            populateAddressableEquipment(self.secondary_area)
            for c, objDict in self.secondary_area.addressable_equipment.items():
                for obj in objDict.values():
                    logger.info(f"Found {c.__name__} called {obj.name} in addressable equipment for secondary area "
                                f"{self.secondary_area.container.mRID}.")
                    self.addressableEquipmentNames.append(obj.name)
            self.isServiceInitialized = True

    def on_request(self, message_bus: FieldMessageBus, headers: Dict, message: Dict):
        if message.get("requestType", "") == "Deconflict":
            resolutionFailed, resolutionVector = self.deconflict(message.get("conflictMatrix", {}))
            returnMessage = {"resolutionFailed": resolutionFailed, "resolutionVector": resolutionVector}
            message_bus.send(headers.get('reply-to'), returnMessage)
        if message.get("requestType", "") == "is_initialized":
            response = {"is_initialized": self.isServiceInitialized}
            message_bus.send(headers.get('reply-to'), response)

    def calculateCriteriaPreferenceWeightVector(self):

        def calculateWeight(sum, start, end):
            if start < end:
                return calculateWeight(sum + 1.0 / float(start), start + 1, end)
            elif start == end:
                return (sum + 1.0 / float(end)) * (1.0 / float(end))

        for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
            self.rVector[i, 0] = calculateWeight(0.0, i + 1, FeederAgentLevelRdbmService.numberOfMetrics)

    def buildSetpointsVector(self, conflictMatrix: Dict) -> Dict:

        def getTapPosition() -> Dict:
            #TODO: don't hardcode tap conversion
            xfmr_id = dssContext.Transformers.First()
            tap_vals = {}
            while xfmr_id:
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tap_vals[dssContext.Transformers.Name()] = round((dssContext.Transformers.Tap() - 1) \
                                                                      / regulationPerStep)
                xfmr_id = dssContext.Transformers.Next()
            return tap_vals

        setpointSetVector = {"setpointIndexMap": [], "setpointSets": []}
        tapVals = getTapPosition()
        setpointRanges = []
        for setpoint in conflictMatrix.get("setpoints", {}).keys():
            values = []
            setpointSetVector["setpointIndexMap"].append(setpoint)
            for setpointValue in conflictMatrix.get("setpoints", {}).get(setpoint, {}).values():
                values.append(setpointValue)
            maxValue = max(values)
            minValue = min(values)
            if "BatteryUnit." in setpoint:
                step = (maxValue - minValue) / (FeederAgentLevelRdbmService.maxAnalogSetpoints - 1)
                setpointRange = []
                for i in range(FeederAgentLevelRdbmService.maxAnalogSetpoints):
                    setpointRange.append(minValue + (i * step))
                setpointRanges.append(setpointRange)
            elif "RatioTapChanger." in setpoint:
                tapVal = tapVals.get(setpoint.split(".")[1])
                printStr = {"device": setpoint, "tapPosition": tapVal, "maxValue": maxValue, "minValue": minValue}
                # print(f"{json.dumps(printStr, indent=4)}")
                if tapVal is not None:
                    if maxValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
                        maxValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
                    elif maxValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
                        maxValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
                        minValue = maxValue
                    if minValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
                        minValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
                        maxValue = minValue
                    elif minValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
                        minValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
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

    def runPowerFlowSnapshot(self, setpoints: List, setpointNames: List, simulationData: Dict):
        if len(setpoints) != len(setpointNames):
            raise RuntimeError("The number of setpoints does not match the number of setpoint names")
        # end model update with measurements
        for i in range(len(setpoints)):
            setpointNameSplit = setpointNames[i].split(".")
            if "BatteryUnit." in setpointNames[i]:
                #TODO: Figure out opendss command to change storage ouptput
                batt_kW = -0.001 * setpoints[i]
                dssContext.run_command(f"Storage.{setpointNameSplit[1]}.kw={batt_kW}")
            elif "RatioTapChanger." in setpointNames[i]:
                reg = dssContext.Transformers.First()
                while reg:
                    if dssContext.Transformers.Name() == setpointNameSplit[1]:
                        break
                    else:
                        reg = dssContext.Transformers.Next()
                regulationPerStep = (dssContext.Transformers.MaxTap() - dssContext.Transformers.MinTap()) \
                                    / dssContext.Transformers.NumTaps()
                tapStep = 1.0 + (setpoints[i] * regulationPerStep)
                # dssContext.Transformers.Tap(tapStep)
                dssContext.run_command(f"Transformer.{setpointNameSplit[1]}.Taps=[1.0 {tapStep}")
        dssContext.Solution.SolveNoControl()

    def checkForViolations(self) -> bool:
        bus_volt = dssContext.Circuit.AllBusMagPu()
        #print(min(bus_volt), max(bus_volt))
        if min(bus_volt) < 0.90:
            return True
        elif max(bus_volt) > 1.10:
            return True
        sub = dssContext.Circuit.TotalPower()
        sub_kva = math.sqrt((sub[0] * sub[0]) + (sub[1] * sub[1]))
        if sub_kva > 5000:
            return True
        dssContext.Circuit.SetActiveClass('Transformer')
        device = dssContext.Circuit.FirstElement()
        while device:
            a = dssContext.CktElement.CurrentsMagAng()
            if a[0] > 668:
                return True
            device = dssContext.Circuit.NextElement()
        return False

    def extractPowerLosses(self) -> float:
        loss = dssContext.Circuit.Losses()
        return loss[0] / 1000

    def extractProfitAndEmissions(self) -> tuple:
        bus_shunt_p = {}
        total_load = 0
        total_pv = 0
        total_dg = 0
        profit = 0
        emissions = 0
        sub = dssContext.Circuit.TotalPower()
        p_sub = sub[0]
        dssContext.Circuit.SetActiveClass('Load')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            bus_shunt_p[bus[0]] = output[0]
            total_load = total_load + output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('PVSystem')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            bus = dssContext.CktElement.BusNames()
            total_pv = total_pv + output[0]
            if bus[0] in bus_shunt_p.keys():
                bus_shunt_p[bus[0]] = bus_shunt_p[bus[0]] + output[0]
            else:
                bus_shunt_p[bus[0]] = output[0]
            device = dssContext.Circuit.NextElement()
        dssContext.Circuit.SetActiveClass('Generator')
        device = dssContext.Circuit.FirstElement()
        while device:
            output = dssContext.CktElement.TotalPowers()
            p_dg = output[0]
            total_dg = total_dg + p_dg
            if 'dies' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_diesel
                emissions = emissions - p_dg * self.emissions_diesel
            elif 'lng' in dssContext.CktElement.Name():
                profit = profit + p_dg * self.cost_lng
                emissions = emissions - p_dg * self.emissions_lng
            device = dssContext.Circuit.NextElement()
        emissions = emissions - p_sub * self.emissions_transmission
        profit = profit + p_sub * self.cost_transmission
        for bus in list(bus_shunt_p.keys()):
            p = bus_shunt_p[bus]
            if p > 0:
                profit = profit + p * self.cost_retail
            else:
                profit = profit + p * self.cost_net_metering
        return (profit, emissions)

    def extractBatteryOutput(self) -> float:
        dssContext.Circuit.SetActiveClass('Storage')
        device = dssContext.Circuit.FirstElement()
        total_batt = 0
        while device:
            output = dssContext.CktElement.TotalPowers()
            total_batt = total_batt + output[0]
            device = dssContext.Circuit.NextElement()
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
            self.uMatrix[index, 0] = profit
            self.uMatrix[index, 1] = -p_loss
            self.uMatrix[index, 2] = -emissions
            self.uMatrix[index, 3] = total_batt
        else:
            self.invalidSetpoints.append(index)
            self.uMatrix[index, 0] = np.NaN
            self.uMatrix[index, 1] = np.NaN
            self.uMatrix[index, 2] = np.NaN
            self.uMatrix[index, 3] = np.NaN

    def normalizeUMatrix(self):
        # delete all rows with NaN in them and remove invalid setpoint alternatives
        self.uMatrix = self.uMatrix[~np.isnan(self.uMatrix).any(axis=1), :]
        if self.uMatrix.size == 0:  # Throw error if all setpoint alternaitives caused powerflow operation violations
            print("!\n!\n!WARNING: All the setpoints requested by the applications are causing powerflow operation "
                  "violations! Sending Resolution from previous time.\n!\n!")
            return
        for i in range(-1, -len(self.invalidSetpoints) - 1, -1):
            idx = self.invalidSetpoints[i]
            del self.setpointSetVector["setpointSets"][idx]
        # normalize uMatrix
        for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
            maxVal = self.uMatrix[:, i].max()
            minVal = self.uMatrix[:, i].min()
            trueMax = maxVal - minVal
            for j in range(self.uMatrix.shape[0]):
                self.uMatrix[j, i] = (self.uMatrix[j, i] - minVal) / trueMax

    def calculateCriteriaPriorityMatrix(self):
        self.cMatrix = np.identity(self.uMatrix.shape[1])
        #TODO: formulate algorithm for forming the criteria priority matrix

    def calculateSetpointAlternativesPriorityMatrix(self):
        self.pMatrix = np.identity(self.uMatrix.shape[0])
        #TODO: formulate algorithm for forming the setpoint alternatives priority matrix

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = deepcopy(conflictMatrix)
        for timeVal in self.conflictMatrix.get("timestamps", {}).values():
            self.conflictTime = max(self.conflictTime, timeVal)
        distributedConflictMatrix = {"setpoints": {}}
        for device in self.conflictMatrix.get("setpoints", {}).keys():
            deviceName = (device.split("."))[1]
            if deviceName in self.addressableEquipmentNames:
                distributedConflictMatrix["setpoints"][device] = deepcopy(
                    self.conflictMatrix.get("setpoints", {}).get(device, {}))
                # add distributed peak_shaving app setpoint
                distributedConflictMatrix["setpoints"][device].update(getPeakShavingSetpoint(self.conflictTime, device))
        for device in distributedConflictMatrix.get("setpoints", {}).keys():
            del self.conflictMatrix["setpoints"][device]
        #TODO: replace simulationData with device setpoint measurements
        simulationData = {}
        for i in self.csvData.index:
            if int(self.csvData.at[i, "Time"]) == self.conflictTime:
                simulationData["load"] = float(self.csvData.at[i, "Loadshape"])
                simulationData["solar"] = float(self.csvData.at[i, "Solar"])
                simulationData["price"] = float(self.csvData.at[i, "Price"])
                break
        distributedResolutionVector = {}
        resolutionVector = {"setpoints": {}, "timestamps": {}}
        deconflictionFailed = False
        # dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
        # dssContext.run_command(f'Redirect {str(openDssFile)}')
        # dssContext.run_command(f'Compile {str(openDssFile)}')
        # dssContext.Solution.SolveNoControl()

        if len(distributedConflictMatrix.get("setpoints", {}).keys()) > 0:
            #TODO: update model with current measurements not data from this csvfile.
            #-----------------------------------------------------------------------
            dssContext.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
            dssContext.run_command(f'set loadmult = {simulationData["load"]}')
            #-----------------------------------------------------------------------
            print(f"deconflicting area {self.downstream_message_bus_def.id}...")
            self.setpointSetVector = self.buildSetpointsVector(distributedConflictMatrix)
            self.numberOfSets = len(self.setpointSetVector.get("setpointSets", []))
            self.uMatrix = np.empty((self.numberOfSets, FeederAgentLevelRdbmService.numberOfMetrics))
            self.invalidSetpoints = []
            for i in range(self.numberOfSets):
                self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"],
                                  i, simulationData)
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
                    distributedResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        resolutionSetpointSet[i]
                    distributedResolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        self.conflictTime
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
            else:
                #TODO: figure out a more sophisticated alternative than giving the last resolution when current conflict can't be resolved.
                deconflictionFailed = True
                pastResolutionVector = {"setpoints": {}}
                for device in self.setpointSetVector["setpointIndexMap"]:
                    for i in range(self.conflictTime - 1, 0, -1):
                        if device in self.resolutionDict[i]["setpoints"].keys():
                            pastResolutionVector["setpoints"][device] = self.resolutionDict[i]["setpoints"][device]
                            break
                resolutionSetpointSet = [0] * len(self.setpointSetVector["setpointIndexMap"])
                for i in range(len(self.setpointSetVector["setpointIndexMap"])):
                    resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
                    resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
                    resolutionSetpointSet[i] = \
                        pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
            # update opendssmodel with resolved setpoints for next conflict
            self.runPowerFlowSnapshot(resolutionSetpointSet, self.setpointSetVector["setpointIndexMap"], simulationData)
        deconflictTime = time.perf_counter() - deconflictStart
        p_loss = self.extractPowerLosses()
        profit, emissions = self.extractProfitAndEmissions()
        total_batt = self.extractBatteryOutput()
        self.criteriaResults[self.conflictTime] = [profit, p_loss, emissions, total_batt]
        print(f"{self.agent_id}:deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
        self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
        self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
        if self.conflictTime == 96:
            #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
            print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
            resultFile = Path(__file__).parent.resolve() \
                         / f'secondary_area_{self.downstream_message_bus_def.id}_resolutionResults.json'
            with resultFile.open(mode="w") as rf:
                json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
            conflictFile = Path(__file__).parent.resolve() \
                           / f'secondary_area_{self.downstream_message_bus_def.id}_conflictResults.json'
            with conflictFile.open(mode="w") as rf:
                json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
            criteriaFile = Path(__file__).parent.resolve() \
                           / f'secondary_area_{self.downstream_message_bus_def.id}_criteriaResults.json'
            with criteriaFile.open(mode="w") as rf:
                json.dump(self.criteriaResults, rf, indent=4, sort_keys=True)
        return (False, resolutionVector)


def populateAddressableEquipment(distributedArea: DistributedArea):
    cimUtils.get_all_transformer_data(distributedArea)
    cimUtils.get_all_load_data(distributedArea)
    cimUtils.get_all_inverter_data(distributedArea)
    cimUtils.get_all_switch_data(distributedArea)


def getMessageBusDefinition(areaId: str) -> MessageBusDefinition:
    if not isinstance(areaId, str):
        raise TypeError(f"area id is not a string type.\ntype: {type(areaId)}")
    connectionArgs = {
        "GRIDAPPSD_ADDRESS": os.environ.get('GRIDAPPSD_ADDRESS', "tcp://gridappsd:61613"),
        "GRIDAPPSD_USER": os.environ.get('GRIDAPPSD_USER'),
        "GRIDAPPSD_PASSWORD": os.environ.get('GRIDAPPSD_PASSWORD'),
        "GRIDAPPSD_APPLICATION_ID": os.environ.get('GRIDAPPSD_APPLICATION_ID')
    }
    bus = MessageBusDefinition(id=areaId,
                               is_ot_bus=True,
                               connection_type="GRIDAPPSD_TYPE_GRIDAPPSD",
                               conneciton_args=connectionArgs)
    return bus


# distributed peak shaving conflict function
def getPeakShavingSetpoint(timestep: int, deviceName: str) -> Dict:
    rv = {}
    dataFile = Path(__file__).parent.resolve() / "peak_shaving.csv"
    data = pd.read_csv(dataFile)
    for i in data.index:
        if data.at[i, "time"] == timestep:
            rv["peak_shaving-app"] = float(data.at[i, deviceName])
            break
    return rv


def main(**kwargs):
    # global dssContext
    servicesAreRunning = False
    runningServiceInfo = []
    runningServiceInstances = []
    systemMessageBusDef = None
    feederMessageBusDef = None
    switchAreaMessageBusDef = None
    serviceMetadata = {
        "app_id": "rules_based_deconfliction_service",
        "description": "A GridAPPS-D distributed rules based deconfliction service agent."
    }
    modelMrid = kwargs.get("model_mrid")
    systemMessageBusConfigFile = kwargs.get("system_bus_config_file")
    feederMessageBusConfigFile = kwargs.get("feeder_bus_config_file")
    switchAreaMessageBusConfigFile = kwargs.get("switch_bus_config_file")
    secondaryAreaMessageBusConfigFile = kwargs.get("secondary_bus_config_file")
    if not isinstance(systemMessageBusConfigFile, Path) and systemMessageBusConfigFile is not None:
        errorStr = f"system_bus_config_file isn't a Path type.\ntype: {type(systemMessageBusConfigFile)}"
        logger.error(errorStr)
        raise TypeError(errorStr)
    if not isinstance(feederMessageBusConfigFile, Path) and feederMessageBusConfigFile is not None:
        errorStr = f"feeder_bus_config_file isn't a Path type.\ntype: {type(feederMessageBusConfigFile)}"
        logger.error(errorStr)
        raise TypeError(errorStr)
    if not isinstance(switchAreaMessageBusConfigFile, Path) and switchAreaMessageBusConfigFile is not None:
        errorStr = f"switch_bus_config_file isn't a Path type.\ntype: {type(switchAreaMessageBusConfigFile)}"
        logger.error(errorStr)
        raise TypeError(errorStr)
    if not isinstance(secondaryAreaMessageBusConfigFile, Path) and secondaryAreaMessageBusConfigFile is not None:
        errorStr = f"secondary_bus_config_file isn't a Path type.\ntype: {type(secondaryAreaMessageBusConfigFile)}"
        logger.error(errorStr)
        raise TypeError(errorStr)
    if not isinstance(modelMrid, str) and modelMrid is not None:
        errorStr = f"model_mrid is not a string.\ntype: {type(modelMrid)}"
        logger.error(errorStr)
        raise TypeError(errorStr)
    
    if modelMrid is None:
        if systemMessageBusConfigFile is not None and feederMessageBusConfigFile is not None:
            systemMessageBusDef = MessageBusDefinition.load(systemMessageBusConfigFile)
            feederMessageBusDef = MessageBusDefinition.load(feederMessageBusConfigFile)
            logger.info(f"Creating Feeder Area Rules Based Deconfliction Service for area id: {feederMessageBusDef.id}")
            feederService = FeederAgentLevelRdbmService(systemMessageBusDef, feederMessageBusDef, serviceMetadata)
            runningServiceInfo.append(f"{type(feederService).__name__}:{feederMessageBusDef.id}")
            runningServiceInstances.append(feederService)
        if switchAreaMessageBusConfigFile is not None and feederMessageBusConfigFile is not None:
            if feederMessageBusDef is None:
                feederMessageBusDef = MessageBusDefinition.load(feederMessageBusConfigFile)
            switchAreaMessageBusDef = MessageBusDefinition.load(switchAreaMessageBusConfigFile)
            logger.info("Creating Switch Area Rules Based Deconfliction Service for area id: "
                        f"{switchAreaMessageBusDef.id}")
            switchAreaService = SwitchAreaAgentLevelRbdmService(feederMessageBusDef, switchAreaMessageBusDef,
                                                                serviceMetadata)
            runningServiceInfo.append(f"{type(switchAreaService).__name__}:{switchAreaMessageBusDef.id}")
            runningServiceInstances.append(switchAreaService)
        if secondaryAreaMessageBusConfigFile is not None and switchAreaMessageBusConfigFile is not None:
            if switchAreaMessageBusDef is None:
                switchAreaMessageBusDef = MessageBusDefinition.load(switchAreaMessageBusConfigFile)
            secondaryAreaMessageBusDef = MessageBusDefinition.load(secondaryAreaMessageBusConfigFile)
            logger.info("Creating Secondary Area Rules Based Deconfliction Service for area id: "
                        f"{secondaryAreaMessageBusDef.id}")
            secondaryAreaService = SecondaryAreaAgentLevelRbdmService(switchAreaMessageBusDef,
                                                                      secondaryAreaMessageBusDef, serviceMetadata)
            if len(secondaryAreaService.agent_area_dict['addressable_equipment']) == 0 and \
                    len(secondaryAreaService.agent_area_dict['unaddressable_equipment']) == 0:
                secondaryAreaService.disconnect()
            else:
                runningServiceInfo.append(f"{type(secondaryAreaService).__name__}:{secondaryAreaMessageBusDef.id}")
                runningServiceInstances.append(secondaryAreaService)
    else:
        systemMessageBusDef = getMessageBusDefinition(modelMrid)
        feederMessageBusDef = getMessageBusDefinition(modelMrid)
        logger.info(f"Creating Feeder Area Rules Based Deconfliction Service for area id: {feederMessageBusDef.id}")
        feederService = FeederAgentLevelRdbmService(systemMessageBusDef, feederMessageBusDef, serviceMetadata)
        runningServiceInfo.append(f"{type(feederService).__name__}:{feederMessageBusDef.id}")
        runningServiceInstances.append(feederService)
        for switchArea in feederService.agent_area_dict.get('switch_areas', []):
            switchAreaId = switchArea.get('message_bus_id')
            # if switchAreaId is not None:
            if switchAreaId == '_E3D03A27-B988-4D79-BFAB-F9D37FB289F7.1':
                switchAreaMessageBusDef = getMessageBusDefinition(switchAreaId)
                logger.info(f"Creating Switch Area Rules Based Deconfliction Service for area id: {switchAreaMessageBusDef.id}")
                switchAreaService = SwitchAreaAgentLevelRbdmService(feederMessageBusDef, switchAreaMessageBusDef,
                                                                    serviceMetadata)
                runningServiceInfo.append(f"{type(switchAreaService).__name__}:{switchAreaMessageBusDef.id}")
                runningServiceInstances.append(switchAreaService)
                for secondaryArea in switchArea.get('secondary_areas', []):
                    secondaryAreaId = secondaryArea.get('message_bus_id')
                    if secondaryAreaId is not None:
                        secondaryAreaMessageBusDef = getMessageBusDefinition(secondaryAreaId)
                        logger.info(
                            f"Creating Secondary Area Rules Based Deconfliction Service for area id: {secondaryAreaMessageBusDef.id}")
                        secondaryAreaService = SecondaryAreaAgentLevelRbdmService(
                            switchAreaMessageBusDef, secondaryAreaMessageBusDef, serviceMetadata)
                        if len(secondaryAreaService.agent_area_dict['addressable_equipment']) == 0 and \
                                len(secondaryAreaService.agent_area_dict['unaddressable_equipment']) == 0:
                            secondaryAreaService.disconnect()
                        else:
                            runningServiceInfo.append(f"{type(secondaryAreaService).__name__}:"
                                                          f"{secondaryAreaMessageBusDef.id}")
                            runningServiceInstances.append(secondaryAreaService)
    if len(runningServiceInstances) > 0:
        servicesAreRunning = True
        print("rules based deconfliction services are running!")
        centralizedConflictMatrixFile = Path(__file__).parent.resolve() / "ConflictMatrix.json"
        centralizedConflictMatrix = {}
        with centralizedConflictMatrixFile.open(mode='r') as fh:
            centralizedConflictMatrix = json.load(fh)
        aggregatedResolutionVector = {"setpoints": {}, "timestamps": {}}
        for service in runningServiceInstances:
            rv = service.deconflict(centralizedConflictMatrix)
            aggregatedResolutionVector.update(rv[1])
        dssContext.run_command("Clear")
    while servicesAreRunning:
        try:
            for service in runningServiceInstances:
                # TODO: check to see if service instance is still running
                servicesAreRunning = True
        except KeyboardInterrupt:
            exitStr = "Manually exiting distributed rules based deconfliction service for the following areas:"
            for area in runningServiceInfo:
                exitStr += f"\n{area}"
            logger.info(exitStr)
            break


if __name__ == "__main__":
    parser = ArgumentParser()
    serviceConfigHelpStr = "Variable keyword arguments that provide user defined distributed area agent " \
                           "configuration files. Valid keywords are as follows:SYSTEM_BUS_CONFIG_FILE=<full " \
                           "path of the system bus configuration file>. FEEDER_BUS_CONFIG_FILE=<full path of the " \
                           "feeder bus configuration file>. SWITCH_BUS_CONFIG_FILE=<full path to the " \
                           "directory containing the switch bus configuration file(s)>. " \
                           "SECONDARY_BUS_CONFIG_FILE=<full path to the directory containing the secondary bus " \
                           "configuration file(s)>. MODEL_MRID=<The desired model mrid to automatically start a " \
                           "rdbm deconfliction service for all the distributed areas for that model id>. This " \
                           "variable should not be specified in combination with SYSTEM_BUS_CONFIG_FILE, " \
                           "FEEDER_BUS_CONFIG_FILE, SWITCH_BUS_CONFIG_FILE_DIR, and SECONDARY_BUS_CONFIG_FILE_DIR, " \
                           "as it will override those values. The directory needs to contain at least one of the " \
                           "following folder names: feeder_level, secondary_level, switch_level, and/or system_level."
    parser.add_argument("service_configurations", nargs="+", help=serviceConfigHelpStr)
    args = parser.parse_args()
    validKeywords = [
        "MODEL_MRID", "SYSTEM_BUS_CONFIG_FILE", "FEEDER_BUS_CONFIG_FILE", "SWITCH_BUS_CONFIG_FILE",
        "SECONDARY_BUS_CONFIG_FILE"
    ]
    mainArgs = {}
    for arg in args.service_configurations:
        argSplit = arg.split("=", maxsplit=1)
        if argSplit[0] in validKeywords:
            mainArgs[argSplit[0]] = argSplit[1]
        else:
            logger.error(f"Invalid keyword argument, {argSplit[0]}, was given by the user. Valid keywords are "
                         f"{validKeywords}.")
            raise RuntimeError(f"Invalid keyword argument, {argSplit[0]}, was given by the user. Valid keywords are "
                               f"{validKeywords}.")
    if len(mainArgs) == 0:
        logger.error(f"No arguments were provided by the user. Valid keywords are {validKeywords}.")
        raise RuntimeError(f"No arguments were provided by the user. Valid keywords are {validKeywords}.")
    systemBusConfigFile = None
    feederBusConfigFile = None
    switchBusConfigFile = None
    secondaryBusConfigFile = None
    modelMrid = mainArgs.get("MODEL_MRID")
    if modelMrid is not None:
        main(model_mrid=modelMrid)
    else:
        systemBusConfigFile = mainArgs.get("SYSTEM_BUS_CONFIG_FILE")
        feederBusConfigFile = mainArgs.get("FEEDER_BUS_CONFIG_FILE")
        switchBusConfigFile = mainArgs.get("SWITCH_BUS_CONFIG_FILE")
        secondaryBusConfigFile = mainArgs.get("SECONDARY_BUS_CONFIG_FILE")
        if systemBusConfigFile is not None:
            systemBusConfigFile = Path(systemBusConfigFile)
        if feederBusConfigFile is not None:
            feederBusConfigFile = Path(feederBusConfigFile)
        if switchBusConfigFile is not None:
            switchBusConfigFile = Path(switchBusConfigFile)
        if secondaryBusConfigFile is not None:
            secondaryBusConfigFile = Path(secondaryBusConfigFile)
        main(system_bus_config_file=systemBusConfigFile,
            feeder_bus_config_file=feederBusConfigFile,
            switch_bus_config_file=switchBusConfigFile,
            secondary_bus_config_file=secondaryBusConfigFile)
