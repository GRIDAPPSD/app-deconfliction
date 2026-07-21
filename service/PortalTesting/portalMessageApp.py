import json
import logging
import os
import sys
import time
from argparse import ArgumentParser
from pathlib import Path
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

import cimgraph.data_profile.cimhub_2023 as cim
from cimgraph.databases import BlazegraphConnection
from cimgraph.models import FeederModel
import cimgraph.utils as cimUtils
from gridappsd import GridAPPSD, topics as t
from gridappsd.simulation import Simulation

class PortalMessenger(object):
    def __init__(self):
        simFile = Path("/home/vale/git/app-deconfliction/service/sim-starter/123apps_12hr-config.json").resolve()
        gad_user = os.environ.get('GRIDAPPSD_USER')
        if gad_user is None:
            os.environ['GRIDAPPSD_USER'] = 'system'
        gad_password = os.environ.get('GRIDAPPSD_PASSWORD')
        if gad_password is None:
            os.environ['GRIDAPPSD_PASSWORD'] = 'manager'
        self.gapps = GridAPPSD()
        with simFile.open(mode="r", encoding="utf-8") as sfh:
            simDict = json.load(sfh)
        feederMRID = simDict["power_system_configs"][0]["Line_name"]
        simObj = Simulation(gapps=self.gapps, run_config=simDict)
        
        os.environ['CIMG_CIM_PROFILE'] = 'cimhub_2023'
        os.environ['CIMG_URL'] = 'http://localhost:8889/bigdata/namespace/kb/sparql'
        os.environ['CIMG_NAMESPACE'] = 'http://iec.ch/TC57/CIM100#'
        os.environ['CIMG_IEC61970_552'] = '552-NEW'
        os.environ['CIMG_USE_UNITS'] = 'false'
        self.databaseConnection = BlazegraphConnection()
        self.feederModels = {}
        feeder = cim.Feeder(mRID=feederMRID)
        self.feederModel = FeederModel(container=feeder, connection=self.databaseConnection, distributed=False)
        cimUtils.get_all_data(self.feederModel)
        measurements = self.feederModel.graph.get(cim.Discrete, {})
        self.ratioTapChangerMeasurements = []
        self.measurementValues = {}
        for m in measurements.values():
            if isinstance(m.PowerSystemResource, cim.PowerTransformer):
                self.ratioTapChangerMeasurements.append(m)
        simObj.start_simulation(timeout=90)
        self.simId = simObj.simulation_id
        input(f"Simulation {self.simId} started. Press Enter to continue...")
        self.gapps.subscribe(t.simulation_output_topic(f"{self.simId}"), self.on_message)
        simObj.add_oncomplete_callback(self.simulationComplete)
        self.external_control_command = {
            "command": "update",
            "priority_level": "PARTICIPANT",
            "app_name": "MGO",
            "local_topic": "local.agent.topic",
            "input": {    
                "simulation_id": "123453245",
                "message": {
                    "timestamp": 1704207060,
                    "difference_mrid": "123a456b-789c-012d-345e-678f901a235c",
                    "reverse_differences": [
                        {
                            "object": "E5D09901-5395-42F0-97B1-0E243CA16F6A",
                            "attribute": "TapChanger.step",
                            "value": 4
                        }
                    ],
                    "forward_differences": [
                        {
                            "object": "E5D09901-5395-42F0-97B1-0E243CA16F6A",
                            "attribute": "TapChanger.step",
                            "value": 6
                        }
                    ]
                }
            }
        }
        self.internal_control_command = {
            "command": "update",
            "app_name": "cvr",
            "local_topic": "local.agent.topic",
            "input": {    
                "simulation_id": f"{self.simId}",
                "message": {
                    "timestamp": 1704207060,
                    "difference_mrid": "123a456b-789c-012d-345e-678f901a235c",
                    "reverse_differences": [
                        {
                            "object": "E5D09901-5395-42F0-97B1-0E243CA16F6A",
                            "attribute": "TapChanger.step",
                            "value": 4
                        }
                    ],
                    "forward_differences": [
                        {
                            "object": "E5D09901-5395-42F0-97B1-0E243CA16F6A",
                            "attribute": "TapChanger.step",
                            "value": 3
                        }
                    ]
                }
            }
        }
        self.recievedFirstMessage = False
        self.internalMessageSent = False
        self.isSimulationComplete = False
        while not self.isSimulationComplete:
            time.sleep(1)
        

    def simulationComplete(self, sim):
        self.isSimulationComplete = True


    def on_message(self, headers, message):
        measurementValues = {}
        timestamp = message["message"]["timestamp"]
        for meas in self.ratioTapChangerMeasurements:
            measValue = message["message"]["measurements"].get(meas.mRID)
            if measValue is not None:
                if meas.PowerSystemResource.name not in measurementValues:
                    measurementValues[meas.PowerSystemResource.name] = {}
                measurementValues[meas.PowerSystemResource.name][meas.phases.value] = measValue["value"]
        if measurementValues != self.measurementValues:
            print(f"New measurement values for ratio tap changers: {json.dumps(measurementValues, indent=4, sort_keys=True)}")
            self.measurementValues = measurementValues
        if self.recievedFirstMessage and not self.internalMessageSent and timestamp >= self.timeToSendExternalCommand:
            self.internalMessageSent = True
            self.external_control_command["input"]["message"]["timestamp"] = timestamp
            self.gapps.send(t.simulation_input_topic(f"{self.simId}"), self.internal_control_command)
        if not self.recievedFirstMessage:
            self.recievedFirstMessage = True
            self.internal_control_command["input"]["message"]["timestamp"] = timestamp
            self.timeToSendExternalCommand = timestamp + 12
            self.gapps.send(t.simulation_input_topic(f"{self.simId}"), self.external_control_command)

if __name__ == "__main__":
    messenger = PortalMessenger()