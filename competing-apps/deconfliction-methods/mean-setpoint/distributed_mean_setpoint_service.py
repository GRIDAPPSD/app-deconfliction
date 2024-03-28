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
import pandas as pd

#TODO: query gridappsd-python for correct cim_profile instead of hardcoding it.
cim_profile = CIM_PROFILE.RC4_2021.value
agents_mod.set_cim_profile(cim_profile, iec61970_301=7)
# cim_profile = CIM_PROFILE.CIMHUB_2023.value
# agents_mod.set_cim_profile(cim_profile, iec61970_301=8)
cim = agents_mod.cim
logging.basicConfig(format='%(asctime)s::%(levelname)s::%(name)s::%(filename)s::%(lineno)d::%(message)s',
                    filename='DistributedAverageSetpointsService.log',
                    filemode='w',
                    level=logging.INFO,
                    encoding='utf-8')
logger = logging.getLogger(__name__)


class FeederAgentLevelAverageSetpointsService(FeederAgent):

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
        self.conflictTime = -1
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
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

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = conflictMatrix
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
        resolutionVector = {"setpoints": {}, "timestamps": {}}
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
            resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
            resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
        for agent_id, agent_details in localContextResponse.items():
            if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
                switchAreaRequestTopic = gt.field_agent_request_queue(self.downstream_message_bus.id, agent_id)
                switchAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
                deconflictionResponse = self.downstream_message_bus.get_response(switchAreaRequestTopic,
                                                                                 switchAreaRequestMessage,
                                                                                 timeout=60)
                distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
                                                                        {"setpoints": {}, "timestamps": {}})
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - deconflictStart
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
        return (False, resolutionVector)


class SwitchAreaAgentLevelAverageSetpointsService(SwitchAreaAgent):

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
        self.conflictTime = -1
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
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

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = conflictMatrix
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
        distributedResolutionVector = {}
        resolutionVector = {"setpoints": {}, "timestamps": {}}
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
            resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
            resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        localContextResponse = LocalContext.get_agents(self.downstream_message_bus)
        for agent_id, agent_details in localContextResponse.items():
            if agent_details.get("app_id") == self.app_id and agent_id != self.agent_id:
                secondaryAreaRequestTopic = gt.field_agent_request_queue(self.downstream_message_bus.id, agent_id)
                secondaryAreaRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
                deconflictionResponse = self.downstream_message_bus.get_response(secondaryAreaRequestTopic,
                                                                                 secondaryAreaRequestMessage,
                                                                                 timeout=60)
                distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
                                                                        {"setpoints": {}, "timestamps": {}})
                resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
                resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - deconflictStart
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
        return (False, resolutionVector)


class SecondaryAreaAgentLevelAverageSetpointsService(SecondaryAreaAgent):

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
        self.conflictTime = -1
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
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

    def deconflict(self, conflictMatrix: Dict):
        deconflictStart = time.perf_counter()
        self.conflictMatrix = conflictMatrix
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
        distributedResolutionVector = {}
        resolutionVector = {"setpoints": {}, "timestamps": {}}
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
            resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
            resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - deconflictStart
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
        "app_id": "mean_setpoint_deconfliction_service",
        "description": "A GridAPPS-D distributed mean setpoint deconfliction service agent."
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
            logger.info(f"Creating Feeder Area Mean Setpoint Deconfliction Service for area id: {feederMessageBusDef.id}")
            feederService = FeederAgentLevelAverageSetpointsService(systemMessageBusDef, feederMessageBusDef, serviceMetadata)
            runningServiceInfo.append(f"{type(feederService).__name__}:{feederMessageBusDef.id}")
            runningServiceInstances.append(feederService)
        if switchAreaMessageBusConfigFile is not None and feederMessageBusConfigFile is not None:
            if feederMessageBusDef is None:
                feederMessageBusDef = MessageBusDefinition.load(feederMessageBusConfigFile)
            switchAreaMessageBusDef = MessageBusDefinition.load(switchAreaMessageBusConfigFile)
            logger.info("Creating Switch Area Mean Setpoint Deconfliction Service for area id: "
                        f"{switchAreaMessageBusDef.id}")
            switchAreaService = SwitchAreaAgentLevelAverageSetpointsService(feederMessageBusDef, switchAreaMessageBusDef,
                                                                serviceMetadata)
            runningServiceInfo.append(f"{type(switchAreaService).__name__}:{switchAreaMessageBusDef.id}")
            runningServiceInstances.append(switchAreaService)
        if secondaryAreaMessageBusConfigFile is not None and switchAreaMessageBusConfigFile is not None:
            if switchAreaMessageBusDef is None:
                switchAreaMessageBusDef = MessageBusDefinition.load(switchAreaMessageBusConfigFile)
            secondaryAreaMessageBusDef = MessageBusDefinition.load(secondaryAreaMessageBusConfigFile)
            logger.info("Creating Secondary Area Mean Setpoint Deconfliction Service for area id: "
                        f"{secondaryAreaMessageBusDef.id}")
            secondaryAreaService = SecondaryAreaAgentLevelAverageSetpointsService(switchAreaMessageBusDef,
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
        feederService = FeederAgentLevelAverageSetpointsService(systemMessageBusDef, feederMessageBusDef, serviceMetadata)
        runningServiceInfo.append(f"{type(feederService).__name__}:{feederMessageBusDef.id}")
        runningServiceInstances.append(feederService)
        for switchArea in feederService.agent_area_dict.get('switch_areas', []):
            switchAreaId = switchArea.get('message_bus_id')
            if switchAreaId is not None:
            # if switchAreaId == '_E3D03A27-B988-4D79-BFAB-F9D37FB289F7.1':
                switchAreaMessageBusDef = getMessageBusDefinition(switchAreaId)
                logger.info(f"Creating Switch Area Rules Based Deconfliction Service for area id: {switchAreaMessageBusDef.id}")
                switchAreaService = SwitchAreaAgentLevelAverageSetpointsService(feederMessageBusDef, switchAreaMessageBusDef,
                                                                    serviceMetadata)
                runningServiceInfo.append(f"{type(switchAreaService).__name__}:{switchAreaMessageBusDef.id}")
                runningServiceInstances.append(switchAreaService)
                for secondaryArea in switchArea.get('secondary_areas', []):
                    secondaryAreaId = secondaryArea.get('message_bus_id')
                    if secondaryAreaId is not None:
                        secondaryAreaMessageBusDef = getMessageBusDefinition(secondaryAreaId)
                        logger.info(
                            f"Creating Secondary Area Rules Based Deconfliction Service for area id: {secondaryAreaMessageBusDef.id}")
                        secondaryAreaService = SecondaryAreaAgentLevelAverageSetpointsService(
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
        print("mean setpoint deconfliction services are running!")
        centralizedConflictMatrixFile = Path(__file__).parent.resolve() / "ConflictMatrix.json"
        centralizedConflictMatrix = {}
        with centralizedConflictMatrixFile.open(mode='r') as fh:
            centralizedConflictMatrix = json.load(fh)
        if f"{FeederAgentLevelAverageSetpointsService.__name__}:_E3D03A27-B988-4D79-BFAB-F9D37FB289F7" in runningServiceInfo:
            rv = feederService.deconflict(centralizedConflictMatrix)
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