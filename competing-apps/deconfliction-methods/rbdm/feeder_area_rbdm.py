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
# import opendssdirect as dss
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
        # self.dssContext = dss.NewContext()
        # self.dssContext.run_command("Clear")
        # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'distributed_123' / 'Run_IEEE123Bus.dss'
        # self.dssContext.run_command(f'Redirect {str(openDssFile)}')
        # self.dssContext.run_command(f'Compile {str(openDssFile)}')
        # self.dssContext.Solution.SolveNoControl()
        self.rVector = np.empty((FeederAgentLevelRdbmService.numberOfMetrics, 1))
        # self.calculateCriteriaPreferenceWeightVector()
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.deconflictionCounter = 0
        self.conflictDump = {}
        self.criteriaResults = {}
        self.addressableEquipmentNames = []
        upstream_field_agent_topic = gt.field_message_bus_agent_topic("ot_bus",self.agent_id)
        self.upstream_message_bus.subscribe(upstream_field_agent_topic, self.on_upstream_message)
        if self.feeder_area is not None:
            populateAddressableEquipment(self.feeder_area)
            for c, objDict in self.feeder_area.addressable_equipment.items():
                for obj in objDict.values():
                    logger.info(f"Found {c.__name__} called {obj.name} in addressable equipment for feeder area "
                                f"{self.feeder_area.container.mRID}.")
                    self.addressableEquipmentNames.append(obj.name)
            self.isServiceInitialized = True
        
        
        



    def on_upstream_message(self, headers: Dict, message: Dict):
        if message.get("requestType", "") == "Deconflict":
            resolutionFailed, resolutionVector = self.deconflict(message.get("conflictMatrix", {}))
            returnMessage = {"resolutionFailed": resolutionFailed, "resolutionVector": resolutionVector}
            # print(headers.get('reply-to'))
            self.upstream_message_bus.send('goss.gridappsd.request.data.resolution',returnMessage)
            print('returned resolution vector message')

    def on_downstream_message(self, headers: Dict, message: Dict):
        distributedResolutionVector = message.get("resolutionVector")
        if distributedResolutionVector is not None:
            self.resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
            self.resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
            self.deconflictionCounter -= 1

    def on_request(self, message_bus: FieldMessageBus, headers: Dict, message: Dict):
        if message.get("requestType", "") == "Deconflict":
            resolutionFailed, resolutionVector = self.deconflict(message.get("conflictMatrix", {}))
            returnMessage = {"resolutionFailed": resolutionFailed, "resolutionVector": resolutionVector}
            print(headers.get('reply-to'))
            message_bus.send(headers.get('reply-to'), returnMessage)
            print('returned resolution vector message')
        if message.get("requestType", "") == "is_initialized":
            response = {"is_initialized": self.isServiceInitialized}
            message_bus.send(headers.get('reply-to'), response)

    def deconflict(self, conflictMatrix: Dict):
        print(f'entered deconflictor for SA {self.feeder_area.container.mRID}')

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
        distributedResolutionVector = {}
        self.resolutionVector = {"setpoints": {}, "timestamps": {}}
        deconflictionFailed = False
        if len(distributedConflictMatrix.get("setpoints", {}).keys()) > 0:
            distributedResolutionVector = {"setpoints": {}, "timestamps": {}}
            for device in distributedConflictMatrix['setpoints'].keys():
                appCount = len(distributedConflictMatrix['setpoints'][device].keys())
                setpointSum = 0.0
                for app in distributedConflictMatrix['setpoints'][device].keys():
                    setpointSum += distributedConflictMatrix['setpoints'][device][app]
                distributedResolutionVector["setpoints"][device] = setpointSum / appCount
                distributedResolutionVector["timestamps"][device] = self.conflictTime
            self.resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
            self.resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        # localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
        # for agent_id, agent_details in localContextResponse.items():
        for switchArea in self.agent_area_dict.get("switch_areas",[]):
            switchAreaId = switchArea.get('message_bus_id')
            if switchAreaId is not None:
                self.deconflictionCounter += 1
                agent_id = f'da_rules_based_deconfliction_service_{switchAreaId}'
            # if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
                switchAreaRequestTopic = gt.field_message_bus_agent_topic(self.downstream_message_bus.id, agent_id)
                switchAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
                self.downstream_message_bus.send(switchAreaRequestTopic, switchAreaRequestMessage)
                # deconflictionResponse = self.downstream_message_bus.get_response(switchAreaRequestTopic,
                #                                                                  switchAreaRequestMessage,
                #                                                                  timeout=60)
                # distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
                #                                                         {"setpoints": {}, "timestamps": {}})
                # self.resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                # self.resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        while self.deconflictionCounter > 0:
            pass
        deconflictTime = time.perf_counter() - deconflictStart
        print(f"{self.agent_id}:deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = self.resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
        self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
        self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
        if self.conflictTime == 96:
            #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
            print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
            resultFile = Path(__file__).parent.resolve() / f'feeder_area_{self.agent_id}_' \
                                                            'resolutionResults.json'
            with resultFile.open(mode="w") as rf:
                json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
            conflictFile = Path(__file__).parent.resolve() / f'feeder_area_{self.agent_id}_' \
                                                                'conflictResults.json'
            with conflictFile.open(mode="w") as rf:
                json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
        return (False, self.resolutionVector)


    # def calculateCriteriaPreferenceWeightVector(self):

    #     def calculateWeight(sum, start, end):
    #         if start < end:
    #             return calculateWeight(sum + 1.0 / float(start), start + 1, end)
    #         elif start == end:
    #             return (sum + 1.0 / float(end)) * (1.0 / float(end))

    #     for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
    #         self.rVector[i, 0] = calculateWeight(0.0, i + 1, FeederAgentLevelRdbmService.numberOfMetrics)

    # def buildSetpointsVector(self, conflictMatrix: Dict) -> Dict:

    #     def getTapPosition() -> Dict:
    #         #TODO: don't hardcode tap conversion
    #         xfmr_id = self.dssContext.Transformers.First()
    #         tap_vals = {}
    #         while xfmr_id:
    #             regulationPerStep = (self.dssContext.Transformers.MaxTap() - self.dssContext.Transformers.MinTap()) \
    #                                 / self.dssContext.Transformers.NumTaps()
    #             tap_vals[self.dssContext.Transformers.Name()] = round((self.dssContext.Transformers.Tap() - 1) 
    #                                                                   / regulationPerStep)
    #             xfmr_id = self.dssContext.Transformers.Next()
    #         return tap_vals

    #     setpointSetVector = {"setpointIndexMap": [], "setpointSets": []}
    #     tapVals = getTapPosition()
    #     setpointRanges = []
    #     for setpoint in conflictMatrix.get("setpoints", {}).keys():
    #         values = []
    #         setpointSetVector["setpointIndexMap"].append(setpoint)
    #         for setpointValue in conflictMatrix.get("setpoints", {}).get(setpoint, {}).values():
    #             values.append(setpointValue)
    #         maxValue = max(values)
    #         minValue = min(values)
    #         if "BatteryUnit." in setpoint:
    #             step = (maxValue - minValue) / (FeederAgentLevelRdbmService.maxAnalogSetpoints - 1)
    #             setpointRange = []
    #             for i in range(FeederAgentLevelRdbmService.maxAnalogSetpoints):
    #                 setpointRange.append(minValue + (i * step))
    #             setpointRanges.append(setpointRange)
    #         elif "RatioTapChanger." in setpoint:
    #             tapVal = tapVals.get(setpoint.split(".")[1])
    #             printStr = {"device": setpoint, "tapPosition": tapVal, "maxValue": maxValue, "minValue": minValue}
    #             # print(f"{json.dumps(printStr, indent=4)}")
    #             if tapVal is not None:
    #                 if maxValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
    #                     maxValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
    #                 elif maxValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
    #                     maxValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
    #                     minValue = maxValue
    #                 if minValue > tapVal + FeederAgentLevelRdbmService.maxTapBudget:
    #                     minValue = tapVal + FeederAgentLevelRdbmService.maxTapBudget
    #                     maxValue = minValue
    #                 elif minValue < tapVal - FeederAgentLevelRdbmService.maxTapBudget:
    #                     minValue = tapVal - FeederAgentLevelRdbmService.maxTapBudget
    #             else:
    #                 raise RuntimeError(f"no tap postion was found for RatioTapChanger, {setpoint.split('.')[1]}")
    #             if minValue < maxValue:
    #                 setpointRanges.append(range(minValue, maxValue + 1))
    #             elif minValue == maxValue:
    #                 setpointRanges.append([maxValue])
    #             else:
    #                 raise RuntimeError("minValue is somehow greater than maxValue!")
    #         else:
    #             raise RuntimeError(f"Unrecognized setpoint in the Conflict Matrix: {setpoint}")
    #     setIter = itertools.product(*setpointRanges)
    #     for i in setIter:
    #         setpointSetVector["setpointSets"].append(i)
    #     return setpointSetVector

    # def runPowerFlowSnapshot(self, setpoints: List, setpointNames: List, simulationData: Dict):
    #     if len(setpoints) != len(setpointNames):
    #         raise RuntimeError("The number of setpoints does not match the number of setpoint names")
    #     # end model update with measurements
    #     for i in range(len(setpoints)):
    #         setpointNameSplit = setpointNames[i].split(".")
    #         if "BatteryUnit." in setpointNames[i]:
    #             #TODO: Figure out opendss command to change storage ouptput
    #             batt_kW = -0.001 * setpoints[i]
    #             self.dssContext.run_command(f"Storage.{setpointNameSplit[1]}.kw={batt_kW}")
    #         elif "RatioTapChanger." in setpointNames[i]:
    #             reg = self.dssContext.Transformers.First()
    #             while reg:
    #                 if self.dssContext.Transformers.Name() == setpointNameSplit[1]:
    #                     break
    #                 else:
    #                     reg = self.dssContext.Transformers.Next()
    #             regulationPerStep = (self.dssContext.Transformers.MaxTap() - self.dssContext.Transformers.MinTap()) \
    #                                 / self.dssContext.Transformers.NumTaps()
    #             tapStep = 1.0 + (setpoints[i] * regulationPerStep)
    #             # self.dssContext.Transformers.Tap(tapStep)
    #             self.dssContext.run_command(f"Transformer.{setpointNameSplit[1]}.Taps=[1.0 {tapStep}")
    #     self.dssContext.Solution.SolveNoControl()

    # def checkForViolations(self) -> bool:
    #     bus_volt = self.dssContext.Circuit.AllBusMagPu()
    #     # print(min(bus_volt), max(bus_volt))
    #     if min(bus_volt) < 0.90:
    #         return True
    #     elif max(bus_volt) > 1.10:
    #         return True
    #     sub = self.dssContext.Circuit.TotalPower()
    #     sub_kva = math.sqrt((sub[0] * sub[0]) + (sub[1] * sub[1]))
    #     if sub_kva > 5000:
    #         return True
    #     self.dssContext.Circuit.SetActiveClass('Transformer')
    #     device = self.dssContext.Circuit.FirstElement()
    #     while device:
    #         a = self.dssContext.CktElement.CurrentsMagAng()
    #         if a[0] > 668:
    #             return True
    #         device = self.dssContext.Circuit.NextElement()
    #     return False

    # def extractPowerLosses(self) -> float:
    #     loss = self.dssContext.Circuit.Losses()
    #     return loss[0] / 1000

    # def extractProfitAndEmissions(self) -> tuple:
    #     bus_shunt_p = {}
    #     total_load = 0
    #     total_pv = 0
    #     total_dg = 0
    #     profit = 0
    #     emissions = 0
    #     sub = self.dssContext.Circuit.TotalPower()
    #     p_sub = sub[0]
    #     self.dssContext.Circuit.SetActiveClass('Load')
    #     device = self.dssContext.Circuit.FirstElement()
    #     while device:
    #         output = self.dssContext.CktElement.TotalPowers()
    #         bus = self.dssContext.CktElement.BusNames()
    #         bus_shunt_p[bus[0]] = output[0]
    #         total_load = total_load + output[0]
    #         device = self.dssContext.Circuit.NextElement()
    #     self.dssContext.Circuit.SetActiveClass('PVSystem')
    #     device = self.dssContext.Circuit.FirstElement()
    #     while device:
    #         output = self.dssContext.CktElement.TotalPowers()
    #         bus = self.dssContext.CktElement.BusNames()
    #         total_pv = total_pv + output[0]
    #         if bus[0] in bus_shunt_p.keys():
    #             bus_shunt_p[bus[0]] = bus_shunt_p[bus[0]] + output[0]
    #         else:
    #             bus_shunt_p[bus[0]] = output[0]
    #         device = self.dssContext.Circuit.NextElement()
    #     self.dssContext.Circuit.SetActiveClass('Generator')
    #     device = self.dssContext.Circuit.FirstElement()
    #     while device:
    #         output = self.dssContext.CktElement.TotalPowers()
    #         p_dg = output[0]
    #         total_dg = total_dg + p_dg
    #         if 'dies' in self.dssContext.CktElement.Name():
    #             profit = profit + p_dg * self.cost_diesel
    #             emissions = emissions - p_dg * self.emissions_diesel
    #         elif 'lng' in self.dssContext.CktElement.Name():
    #             profit = profit + p_dg * self.cost_lng
    #             emissions = emissions - p_dg * self.emissions_lng
    #         device = self.dssContext.Circuit.NextElement()
    #     emissions = emissions - p_sub * self.emissions_transmission
    #     profit = profit + p_sub * self.cost_transmission
    #     for bus in list(bus_shunt_p.keys()):
    #         p = bus_shunt_p[bus]
    #         if p > 0:
    #             profit = profit + p * self.cost_retail
    #         else:
    #             profit = profit + p * self.cost_net_metering
    #     return (profit, emissions)

    # def extractBatteryOutput(self) -> float:
    #     self.dssContext.Circuit.SetActiveClass('Storage')
    #     device = self.dssContext.Circuit.FirstElement()
    #     total_batt = 0
    #     while device:
    #         output = self.dssContext.CktElement.TotalPowers()
    #         total_batt = total_batt + output[0]
    #         device = self.dssContext.Circuit.NextElement()
    #     return total_batt

    # def buildUMatrix(self, setpoints: List, setpointNames: List, index: int, simulationData: Dict):
    #     if index % 100 == 0:
    #         print(f"processing setpoint Alternative {index} of {self.uMatrix.shape[0]}...")
    #     self.runPowerFlowSnapshot(setpoints, setpointNames, simulationData)
    #     violations = self.checkForViolations()
    #     if not violations:
    #         p_loss = self.extractPowerLosses()
    #         profit, emissions = self.extractProfitAndEmissions()
    #         total_batt = self.extractBatteryOutput()
    #         self.uMatrix[index, 0] = profit
    #         self.uMatrix[index, 1] = -p_loss
    #         self.uMatrix[index, 2] = -emissions
    #         self.uMatrix[index, 3] = total_batt
    #     else:
    #         self.invalidSetpoints.append(index)
    #         self.uMatrix[index, 0] = np.NaN
    #         self.uMatrix[index, 1] = np.NaN
    #         self.uMatrix[index, 2] = np.NaN
    #         self.uMatrix[index, 3] = np.NaN

    # def normalizeUMatrix(self):
    #     # delete all rows with NaN in them and remove invalid setpoint alternatives
    #     self.uMatrix = self.uMatrix[~np.isnan(self.uMatrix).any(axis=1), :]
    #     if self.uMatrix.size == 0:  #Throw error if all setpoint alternaitives caused powerflow operation violations
    #         print("!\n!\n!WARNING: All the setpoints requested by the applications are causing powerflow operation "
    #               "violations! Sending Resolution from previous time.\n!\n!")
    #         return
    #     for i in range(-1, -len(self.invalidSetpoints) - 1, -1):
    #         idx = self.invalidSetpoints[i]
    #         del self.setpointSetVector["setpointSets"][idx]
    #     #normalize uMatrix
    #     for i in range(FeederAgentLevelRdbmService.numberOfMetrics):
    #         maxVal = self.uMatrix[:, i].max()
    #         minVal = self.uMatrix[:, i].min()
    #         trueMax = maxVal - minVal
    #         for j in range(self.uMatrix.shape[0]):
    #             self.uMatrix[j, i] = (self.uMatrix[j, i] - minVal) / trueMax

    # def calculateCriteriaPriorityMatrix(self):
    #     self.cMatrix = np.identity(self.uMatrix.shape[1])
    #     #TODO: formulate algorithm for forming the criteria priority matrix

    # def calculateSetpointAlternativesPriorityMatrix(self):
    #     self.pMatrix = np.identity(self.uMatrix.shape[0])
    #     #TODO: formulate algorithm for forming the setpoint alternatives priority matrix

    # def old_deconflict(self, conflictMatrix: Dict):
    #     deconflictStart = time.perf_counter()
    #     self.conflictMatrix = deepcopy(conflictMatrix)
    #     for timeVal in self.conflictMatrix.get("timestamps", {}).values():
    #         self.conflictTime = max(self.conflictTime, timeVal)
    #     distributedConflictMatrix = {"setpoints": {}}
    #     for device in self.conflictMatrix.get("setpoints", {}).keys():
    #         deviceName = (device.split("."))[1]
    #         if deviceName in self.addressableEquipmentNames:
    #             distributedConflictMatrix["setpoints"][device] = deepcopy(
    #                 self.conflictMatrix.get("setpoints", {}).get(device, {})
    #             )
    #             # add distributed peak_shaving app setpoint
    #             distributedConflictMatrix["setpoints"][device].update(getPeakShavingSetpoint(self.conflictTime, device))
    #     for device in distributedConflictMatrix.get("setpoints", {}).keys():
    #         del self.conflictMatrix["setpoints"][device]
    #     #TODO: replace simulationData with device setpoint measurements
    #     simulationData = {}
    #     for i in self.csvData.index:
    #         if int(self.csvData.at[i, "Time"]) == self.conflictTime:
    #             simulationData["load"] = float(self.csvData.at[i, "Loadshape"])
    #             simulationData["solar"] = float(self.csvData.at[i, "Solar"])
    #             simulationData["price"] = float(self.csvData.at[i, "Price"])
    #             break
    #     distributedResolutionVector = {}
    #     resolutionVector = {"setpoints": {}, "timestamps": {}}
    #     deconflictionFailed = False
    #     # self.dssContext.run_command("Clear")
    #     # openDssFile = Path(__file__).parent.resolve() / '123Bus' / 'Run_IEEE123Bus.dss'
    #     # self.dssContext.run_command(f'Redirect {str(openDssFile)}')
    #     # self.dssContext.run_command(f'Compile {str(openDssFile)}')
    #     # self.dssContext.Solution.SolveNoControl()

    #     if len(distributedConflictMatrix.get("setpoints", {}).keys()) > 0:
            
    #         #TODO: update model with current measurements not data from this csvfile.
    #         #-----------------------------------------------------------------------
    #         self.dssContext.run_command(f'BatchEdit PVsystem..* irradiance={simulationData["solar"]}')
    #         self.dssContext.run_command(f'set loadmult = {simulationData["load"]}')
    #         #-----------------------------------------------------------------------
    #         print(f"deconflicting area {self.downstream_message_bus_def.id}...")
    #         self.setpointSetVector = self.buildSetpointsVector(distributedConflictMatrix)
    #         self.numberOfSets = len(self.setpointSetVector.get("setpointSets", []))
    #         self.uMatrix = np.empty((self.numberOfSets, FeederAgentLevelRdbmService.numberOfMetrics))
    #         self.invalidSetpoints = []
    #         for i in range(self.numberOfSets):
    #             self.buildUMatrix(self.setpointSetVector["setpointSets"][i], self.setpointSetVector["setpointIndexMap"],
    #                               i, simulationData)
    #         print(f"{len(self.invalidSetpoints)} setpoint alternatives eliminated due to operational violations.")
    #         self.normalizeUMatrix()
    #         if self.uMatrix.size > 0:
    #             self.calculateCriteriaPriorityMatrix()
    #             self.calculateSetpointAlternativesPriorityMatrix()
    #             fVector = self.pMatrix @ self.uMatrix @ self.cMatrix @ self.rVector
    #             resolutionIndex = fVector.argmax()
    #             resolutionSetpointSet = self.setpointSetVector["setpointSets"][resolutionIndex]
    #             distributedResolutionVector = {
    #                 "setpoints": {},
    #                 "timestamps": {}
    #             }
    #             for i in range(len(resolutionSetpointSet)):
    #                 distributedResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
    #                     resolutionSetpointSet[i]
    #                 distributedResolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = \
    #                     self.conflictTime
    #             resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
    #             resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
    #         else:
    #             #TODO: figure out a more sophisticated alternative than giving the last resolution when current conflict can't be resolved.
    #             deconflictionFailed = True
    #             pastResolutionVector = {"setpoints": {}}
    #             for device in self.setpointSetVector["setpointIndexMap"]:
    #                 for i in range(self.conflictTime - 1, 0, -1):
    #                     if device in self.resolutionDict[i]["setpoints"].keys():
    #                         pastResolutionVector["setpoints"][device] = self.resolutionDict[i]["setpoints"][device]
    #                         break
    #             resolutionSetpointSet = [0] * len(self.setpointSetVector["setpointIndexMap"])
    #             for i in range(len(self.setpointSetVector["setpointIndexMap"])):
    #                 resolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]] = \
    #                     pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
    #                 resolutionVector["timestamps"][self.setpointSetVector["setpointIndexMap"][i]] = self.conflictTime
    #                 resolutionSetpointSet[i] = \
    #                     pastResolutionVector["setpoints"][self.setpointSetVector["setpointIndexMap"][i]]
    #         # update opendssmodel with resolved setpoints for next conflict
    #         self.runPowerFlowSnapshot(resolutionSetpointSet, self.setpointSetVector["setpointIndexMap"], simulationData)
    #     # localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
    #     # for agent_id, agent_details in localContextResponse.items():
    #     #     if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
    #     #         switchAreaRequestTopic = gt.field_agent_request_queue(self.downstream_message_bus.id, agent_id)
    #     #         switchAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
    #     #         deconflictionResponse = self.downstream_message_bus.get_response(switchAreaRequestTopic,
    #     #                                                                          switchAreaRequestMessage,
    #     #                                                                          timeout=60)
    #     #         distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
    #     #                                                                 {"setpoints": {}, "timestamps": {}})
    #     #         resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
    #     #         resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
    #     deconflictTime = time.perf_counter() - deconflictStart
    #     p_loss = self.extractPowerLosses()
    #     profit, emissions = self.extractProfitAndEmissions()
    #     total_batt = self.extractBatteryOutput()
    #     self.criteriaResults[self.conflictTime] = [profit, p_loss, emissions, total_batt]
    #     print(f"{self.agent_id}:deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
    #     self.resolutionDict[self.conflictTime] = resolutionVector
    #     self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
    #     self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed
    #     self.maxDeconflictionTime = max(self.maxDeconflictionTime, deconflictTime)
    #     self.conflictDump[self.conflictTime] = json.loads(json.dumps(self.conflictMatrix))
    #     if self.conflictTime == 96:
    #         #TODO: move all this to a __del__() method when deconfliction pipeline service properly implements a clean exit
    #         print(f"Maximum deconfliction time was {self.maxDeconflictionTime}")
    #         resultFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
    #                                                         'resolutionResults.json'
    #         with resultFile.open(mode="w") as rf:
    #             json.dump(self.resolutionDict, rf, indent=4, sort_keys=True)
    #         conflictFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
    #                                                             'conflictResults.json'
    #         with conflictFile.open(mode="w") as rf:
    #             json.dump(self.conflictDump, rf, indent=4, sort_keys=True)
    #         criteriaFile = Path(__file__).parent.resolve() / f'feeder_area_{self.downstream_message_bus_def.id}_' \
    #                                                             'criteriaResults.json'
    #         with criteriaFile.open(mode="w") as rf:
    #             json.dump(self.criteriaResults, rf, indent=4, sort_keys=True)
    #     return (False, resolutionVector)



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