#TODO: Copyright text

import csv
import itertools
import json
import math
from pathlib import Path
import sys
import time
from typing import Dict

from gridappsd import GridAPPSD
from gridappsd import topics as gt

# find and add shared directory to path hopefully wherever it is from here
sharedDir = Path(__file__).parent.parent.parent.resolve() / "shared"
sys.path.append(str(sharedDir))

import MethodUtil

class DeconflictionMethod:

    def __init__(self, gapps: GridAPPSD, conflictMatrix: Dict = {}):
        #self.conflictMatrix = MethodUtil.ConflictSubMatrix
        self.gapps = gapps
        self.conflictMatrix = conflictMatrix
        self.conflictTime = -1
        self.resolutionDict = {}
        self.maxDeconflictionTime = -1
        self.conflictDump = {}
        self.gapps.subscribe('goss.gridappsd.request.data.resolution', self.on_resolution_message)
    
    def deconflict(self, app_name: str, currentTime: int) -> Dict:
        self.deconflictStart = time.perf_counter()
        for timeVal in self.conflictMatrix.get("timestamps",{}).values():
            self.conflictTime = max(self.conflictTime, timeVal)
        self.deconflictionFailed = False
        self.distributedResolutionVector = {}
        self.resolutionVector = {
            "setpoints": {},
            "timestamps": {}
        }
        systemBusRequestTopic = gt.field_agent_request_queue("_E3D03A27-B988-4D79-BFAB-F9D37FB289F7",
                                                             'da_mean_setpoint_deconfliction_service_'
                                                             '_E3D03A27-B988-4D79-BFAB-F9D37FB289F7')
        print('SYSTEM BUS', systemBusRequestTopic)
        systemBusRequestMessage = {"requestType": "Deconflict","conflictMatrix": self.conflictMatrix}
        deconflictionResponse = self.gapps.send(systemBusRequestTopic, systemBusRequestMessage)#, timeout=30)
        


    def on_resolution_message(self, headers, message):
        deconflictionResponse = message
        print('received resolution vector')
        distributedResolutionVector = deconflictionResponse.get("resolutionVector", 
                                                                {"setpoints": {}, "timestamps": {}})
        self.resolutionVector["setpoints"].update(distributedResolutionVector["setpoints"])
        self.resolutionVector["timestamps"].update(distributedResolutionVector["timestamps"])
        deconflictTime = time.perf_counter() - self.deconflictStart
        print(f"deconfliction for timestamp, {self.conflictTime}, took {deconflictTime}s.\n\n\n")
        self.resolutionDict[self.conflictTime] = self.resolutionVector
        self.resolutionDict[self.conflictTime]["deconflictTime"] = deconflictTime
        self.resolutionDict[self.conflictTime]["deconflictionFailed"] = deconflictionFailed = False
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
        print('returned resolution vector')
        self.gapps.send('goss.gridappsd.request.data.resolution_vector', self.resolutionVector)
    
