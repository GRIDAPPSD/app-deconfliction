#TODO: Copyright text

import itertools
import json
from typing import Dict

import numpy as np
import opendssdirect as dss
from opendssdirect.utils import Iterator

class DeconflictionMethod:
    maxAnalogSetpoints = 5
    def __init__(self, conflictMatrix:Dict):
        self.conflictMatrix = conflictMatrix
        self.setpointSetVector = self.buildSetpointsVector(self.conflictMatrix)
    
    def buildSetpointsVector(self, conflictMatrix:Dict) -> Dict:
        setpointSetVector = {
            "setpointIndexMap": [],
            "setpointSets": None
        }
        setpointRanges = []
        for setpoint in conflictMatrix.get("setpoints",{}).keys():
            values = []
            setpointSetVector["setpointIndexMap"].append(setpoint)
            for setpointValue in conflictMatrix.get("setpoints",{}).get(setpoint,{}).values():
                values.append(setpointValue)
            maxValue = max(values)
            minValue = min(values)
            if "BatteryUnit:" in setpoint:
                step = (maxValue - minValue) / (DeconflictionMethod.maxAnalogSetpoints - 1)
                setpointRange = []
                for i in range(DeconflictionMethod.maxAnalogSetpoints):
                    setpointRange.append(minValue + (i * step))
                setpointRanges.append(setpointRange)
            elif "RatioTapChanger:" in setpoint:
                setpointRanges.append(range(minValue, maxValue + 1))
            else:
                raise RuntimeError(f"Unrecognized setpoint in the Conflict Matrix: {setpoint}")
        setIter = itertools.product(*setpointRanges)
        for i in setIter:
            setpointSetVector["setpointSets"].append(i)
        return setpointSetVector


    def deconflict(self):
        #TODO: call the opendss solver for each alternative setpoint Set and create the U matrix
        pass
    
#for testing purposes only must be removed for actual implementation
if __name__ == "__main__":
    cM = {}
    with open("/home/vale/git/app-deconfliction/competing-apps/deconfliction-pipeline/rbdm/ConflictMatrix.json","r") as cmf:
        cM = json.load(cmf)
    rbdm = DeconflictionMethod(cM)