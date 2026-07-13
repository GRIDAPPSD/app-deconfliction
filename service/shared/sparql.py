"""Module for querying and parsing SPARQL through GridAPPS-D"""
import logging
import pandas as pd
import numpy as np
import os
import re
from operator import itemgetter
import cimgraph.data_profile.cimhub_2023 as cim
from cimgraph.databases import BlazegraphConnection, GridappsdConnection
from cimgraph.models import GraphModel, FeederModel
import cimgraph.utils as cimUtils
from gridappsd import GridAPPSD, topics, utils

class SPARQLManager:
    """Class for querying SPARQL in GridAPPS-D Toolbox tools/services
    """
    
    def __init__(self, gapps, feeder_mrid, simulation_id=None,
                 logFile=None, timeout=60):
        """Connect to the platform.

        :param feeder_mrid: unique identifier for the feeder in
            question. Since PyVVO works on a per feeder basis, this is
            required, and all queries will be executed for the specified
            feeder.
        :param gapps: gridappsd_object
        :param timeout: timeout for querying the blazegraph database.
        """

        # Connect to the platform.
        self.gad = gapps
       
        # Assign feeder mrid.
        self.feeder_mrid = feeder_mrid

        # Timeout for SPARQL queries.
        self.timeout = timeout

        # Assign simulation id
        self.simulation_id = simulation_id

        #self.topic = "goss.gridappsd.process.request.data.powergridmodel"

        self.logFile = logFile

        os.environ['CIMG_CIM_PROFILE'] = 'cimhub_2023'
        os.environ['CIMG_URL'] = 'http://localhost:8889/bigdata/namespace/kb/sparql'
        os.environ['CIMG_NAMESPACE'] = 'http://iec.ch/TC57/CIM100#'
        os.environ['CIMG_IEC61970_552'] = '552-NEW'
        os.environ['CIMG_USE_UNITS'] = 'false'
        self.databaseConnection = BlazegraphConnection()
        self.feederModels = {}
        feeder = cim.Feeder(mRID=feeder_mrid)
        self.feederModel = FeederModel(container=feeder, connection=self.databaseConnection, distributed=False)
        cimUtils.get_all_data(self.feederModel)



# Start of Common Competing Apps queries

    def obj_meas_export(self, objectType):
        # message = {
        #   "requestType": "QUERY_OBJECT_MEASUREMENTS",
        #   "modelId": self.feeder_mrid,
        #   "objectType": objectType,
        #   "resultFormat": "JSON",
        # }

        # results = self.gad.get_response(topics.REQUEST_POWERGRID_DATA, message, timeout=1200)
        measurements = []
        for obj in self.feederModel.graph.get(objectType, {}).values():
            for measObj in obj.Measurements:
                measDict = {
                    "measid": measObj.mRID,
                    "type": measObj.measurementType,
                    "class": type(measObj).__name__,
                    "bus": measObj.Terminal.ConnectivityNode.name,
                    "phases": measObj.phases.value,
                    "eqtype": type(obj).__name__,
                    "eqname": obj.name,
                    "eqid": obj.mRID,
                    "trmid": measObj.Terminal.mRID    
                }
                measurements.append(measDict)
        return measurements


    def battery_query(self):
        # VALUES_QUERY = """
        # PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        # PREFIX c:  <http://iec.ch/TC57/CIM100#>
        # SELECT ?name ?bus ?ratedE ?storedE ?ratedS ?ratedU ?ipu ?p ?q ?fdrid ?id ?pecid (group_concat(distinct ?phs;separator="\\n") as ?phases) WHERE {
        #  ?s r:type c:BatteryUnit.
        #  ?s c:IdentifiedObject.name ?name.
        #  ?s c:IdentifiedObject.mRID ?id.
        #  ?s c:BatteryUnit.ratedE ?ratedE.
        #  ?s c:BatteryUnit.storedE ?storedE.
        #  ?pec c:PowerElectronicsConnection.PowerElectronicsUnit ?s.
        # VALUES ?fdrid {"%s"}
        #  ?pec c:IdentifiedObject.mRID ?pecid.
        #  ?pec c:Equipment.EquipmentContainer ?fdr.
        #  ?fdr c:IdentifiedObject.mRID ?fdrid.
        #  ?pec c:PowerElectronicsConnection.ratedS ?ratedS.
        #  ?pec c:PowerElectronicsConnection.ratedU ?ratedU.
        #  ?pec c:PowerElectronicsConnection.maxIFault ?ipu.
        #  ?pec c:PowerElectronicsConnection.p ?p.
        #  ?pec c:PowerElectronicsConnection.q ?q.
        #  OPTIONAL {?pecp c:PowerElectronicsConnectionPhase.PowerElectronicsConnection ?pec.
        #  ?pecp c:PowerElectronicsConnectionPhase.phase ?phsraw.
        #    bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
        #  ?t c:Terminal.ConductingEquipment ?pec.
        #  ?t c:Terminal.ConnectivityNode ?cn. 
        #  ?cn c:IdentifiedObject.name ?bus
        # }
        # GROUP by ?name ?bus ?ratedE ?storedE ?ratedS ?ratedU ?ipu ?p ?q ?fdrid ?id ?pecid
        # ORDER by ?name
        # """% self.feeder_mrid

        # results = self.gad.query_data(VALUES_QUERY)
        # bindings = results['data']['results']['bindings']
        bindings = []
        for batteryUnit in self.feederModel.graph.get(cim.BatteryUnit, {}).values():
            batteryDict = {}
            batteryDict['id'] = batteryUnit.mRID
            batteryDict['pecid'] = batteryUnit.PowerElectronicsConnection.mRID
            batteryDict['name'] = batteryUnit.name
            batteryDict['bus'] = batteryUnit.PowerElectronicsConnection.Terminals[0].ConnectivityNode.name
            batteryDict['phases'] = []
            for pecp in batteryUnit.PowerElectronicsConnection.PowerElectronicsConnectionPhases:
                batteryDict['phases'].append(pecp.phase.value)
            if len(batteryDict['phases']) == 0: # implied 3 phase inverter
                batteryDict['phases'] = ['A', 'B', 'C']
            batteryDict['ratedS'] = batteryUnit.PowerElectronicsConnection.ratedS
            batteryDict['ratedE'] = batteryUnit.ratedE
            batteryDict['storedE'] = batteryUnit.storedE
            bindings.append(batteryDict)
        bindingsSorted = sorted(bindings, key=itemgetter('name'))
        return bindingsSorted


    def pv_query(self):
        # VALUES_QUERY = """
        # PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        # PREFIX c:  <http://iec.ch/TC57/CIM100#>
        # SELECT ?name ?bus ?ratedS ?ratedU ?ipu ?p ?q ?id ?fdrid (group_concat(distinct ?phs;separator="\\n") as ?phases) WHERE {
        #  ?s r:type c:PhotovoltaicUnit.
        #  ?s c:IdentifiedObject.name ?name.
        #  ?s c:IdentifiedObject.mRID ?id.
        #  ?pec c:PowerElectronicsConnection.PowerElectronicsUnit ?s.
        # VALUES ?fdrid {"%s"}
        #  ?pec c:Equipment.EquipmentContainer ?fdr.
        #  ?fdr c:IdentifiedObject.mRID ?fdrid.
        #  ?pec c:PowerElectronicsConnection.ratedS ?ratedS.
        #  ?pec c:PowerElectronicsConnection.ratedU ?ratedU.
        #  ?pec c:PowerElectronicsConnection.maxIFault ?ipu.
        #  ?pec c:PowerElectronicsConnection.p ?p.
        #  ?pec c:PowerElectronicsConnection.q ?q.
        #  OPTIONAL {?pecp c:PowerElectronicsConnectionPhase.PowerElectronicsConnection ?pec.
        #  ?pecp c:PowerElectronicsConnectionPhase.phase ?phsraw.
        #  bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
        #  ?t c:Terminal.ConductingEquipment ?pec.
        #  ?t c:Terminal.ConnectivityNode ?cn. 
        #  ?cn c:IdentifiedObject.name ?bus
        # }
        # GROUP by ?name ?bus ?ratedS ?ratedU ?ipu ?p ?q ?id ?fdrid
        # ORDER by ?name
        # """% self.feeder_mrid

        # results = self.gad.query_data(VALUES_QUERY)
        # bindings = results['data']['results']['bindings']
        bindings = []
        for pvUnit in self.feederModel.graph.get(cim.PhotovoltaicUnit, {}).values():
            pvDict = {}
            pvDict['id'] = pvUnit.mRID
            pvDict['pecid'] = pvUnit.PowerElectronicsConnection.mRID
            pvDict['name'] = pvUnit.name
            pvDict['bus'] = pvUnit.PowerElectronicsConnection.Terminals[0].ConnectivityNode.name
            pvDict['phases'] = []
            for pecp in pvUnit.PowerElectronicsConnection.PowerElectronicsConnectionPhases:
                pvDict['phases'].append(pecp.phase.value)
            if len(pvDict['phases']) == 0: # implied 3 phase inverter
                pvDict['phases'] = ['A', 'B', 'C']
            pvDict['ratedS'] = pvUnit.PowerElectronicsConnection.ratedS
            pvDict['p'] = pvUnit.PowerElectronicsConnection.p
            pvDict['q'] = pvUnit.PowerElectronicsConnection.q
            bindings.append(pvDict)
        bindingsSorted = sorted(bindings, key=itemgetter('name'))
        return bindingsSorted


    def regulator_query(self):
        # VALUES_QUERY = """
        # PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        # PREFIX c:  <http://iec.ch/TC57/CIM100#>
        # SELECT ?rname ?rid ?pname ?pid ?tname ?tid ?wnum ?phs ?incr ?mode ?enabled ?highStep ?lowStep ?step
        # WHERE {
        # VALUES ?fdrid {"%s"}
        #  ?pxf c:Equipment.EquipmentContainer ?fdr.
        #  ?fdr c:IdentifiedObject.mRID ?fdrid.
        #  ?rtc r:type c:RatioTapChanger.
        #  ?rtc c:IdentifiedObject.name ?rname.
        #  ?rtc c:IdentifiedObject.mRID ?rid.
        #  ?rtc c:RatioTapChanger.TransformerEnd ?end.
        #  ?end c:TransformerEnd.endNumber ?wnum.
        # {?end c:PowerTransformerEnd.PowerTransformer ?pxf.}
        #   UNION
        # {?end c:TransformerTankEnd.TransformerTank ?tank.
        #  ?tank c:IdentifiedObject.name ?tname.
        #  ?tank c:IdentifiedObject.mRID ?tid.
        #  OPTIONAL {?end c:TransformerTankEnd.phases ?phsraw.
        #   bind(strafter(str(?phsraw),"PhaseCode.") as ?phs)}
        #  ?tank c:TransformerTank.PowerTransformer ?pxf.}
        #  ?pxf c:IdentifiedObject.name ?pname.
        #  ?pxf c:IdentifiedObject.mRID ?pid.
        #  ?rtc c:RatioTapChanger.stepVoltageIncrement ?incr.
        #  ?rtc c:RatioTapChanger.tculControlMode ?moderaw.
        #   bind(strafter(str(?moderaw),"TransformerControlMode.") as ?mode)
        #  ?rtc c:TapChanger.controlEnabled ?enabled.
        #  ?rtc c:TapChanger.highStep ?highStep.
        #  ?rtc c:TapChanger.initialDelay ?initDelay.
        #  ?rtc c:TapChanger.lowStep ?lowStep.
        #  ?rtc c:TapChanger.ltcFlag ?ltc.
        #  ?rtc c:TapChanger.neutralStep ?neutralStep.
        #  ?rtc c:TapChanger.neutralU ?neutralU.
        #  ?rtc c:TapChanger.normalStep ?normalStep.
        #  ?rtc c:TapChanger.step ?step.
        # }
        # ORDER BY ?pname ?tname ?rname ?wnum
        # """% self.feeder_mrid

        # # GDB 5/17/24: Bumped up timeout to work with 9500 node model
        # results = self.gad.query_data(VALUES_QUERY, timeout=1200)
        # bindings = results['data']['results']['bindings']
        bindings = []
        for ratioTapChanger in self.feederModel.graph.get(cim.RatioTapChanger, {}).values():
            regDict = {}
            regDict['rid'] = ratioTapChanger.mRID
            regDict['rname'] = ratioTapChanger.name
            regDict['step'] = ratioTapChanger.step
            regDict['highStep'] = ratioTapChanger.highStep
            regDict['lowStep'] = ratioTapChanger.lowStep
            regDict['incr'] = ratioTapChanger.stepVoltageIncrement
            tEnd = ratioTapChanger.TransformerEnd
            if isinstance(tEnd, cim.PowerTransformerEnd):
                regDict['pid'] = tEnd.PowerTransformer.mRID
                regDict['pname'] = tEnd.PowerTransformer.name
            else:
                regDict['pid'] = tEnd.TransformerTank.PowerTransformer.mRID
                regDict['pname'] = tEnd.TransformerTank.PowerTransformer.name
                regDict['tid'] = tEnd.TransformerTank.mRID
                regDict['tname'] = tEnd.TransformerTank.name
                regDict['phs'] = tEnd.orderedPhases.value
            regDict['wnum'] = tEnd.endNumber
            bindings.append(regDict)
        bindingsSorted = sorted(bindings, key=lambda x:(x.get('pname', ""),
                                                        x.get('tname', ""),
                                                        x.get('rname', ""),
                                                        x.get('wnum', 0)))
        return bindingsSorted


    def regulator_combine_query(self):
        # VALUES_QUERY = """
        # PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        # PREFIX c:  <http://iec.ch/TC57/CIM100#>
        # SELECT ?rname ?rid ?pname ?pid ?tname ?tid ?wnum ?phs ?incr ?mode ?enabled ?highStep ?lowStep
        # WHERE {
        # VALUES ?fdrid {"%s"}
        #  ?pxf c:Equipment.EquipmentContainer ?fdr.
        #  ?fdr c:IdentifiedObject.mRID ?fdrid.
        #  ?rtc r:type c:RatioTapChanger.
        #  ?rtc c:IdentifiedObject.name ?rname.
        #  ?rtc c:IdentifiedObject.mRID ?rid.
        #  ?rtc c:RatioTapChanger.TransformerEnd ?end.
        #  ?end c:TransformerEnd.endNumber ?wnum.
        # {?end c:PowerTransformerEnd.PowerTransformer ?pxf.}
        #   UNION
        # {?end c:TransformerTankEnd.TransformerTank ?tank.
        #  ?tank c:IdentifiedObject.name ?tname.
        #  ?tank c:IdentifiedObject.mRID ?tid.
        #  OPTIONAL {?end c:TransformerTankEnd.phases ?phsraw.
        #   bind(strafter(str(?phsraw),"PhaseCode.") as ?phs)}
        #  ?tank c:TransformerTank.PowerTransformer ?pxf.}
        #  ?pxf c:IdentifiedObject.name ?pname.
        #  ?pxf c:IdentifiedObject.mRID ?pid.
        #  ?rtc c:RatioTapChanger.stepVoltageIncrement ?incr.
        #  ?rtc c:RatioTapChanger.tculControlMode ?moderaw.
        #   bind(strafter(str(?moderaw),"TransformerControlMode.") as ?mode)
        #  ?rtc c:TapChanger.controlEnabled ?enabled.
        #  ?rtc c:TapChanger.highStep ?highStep.
        #  ?rtc c:TapChanger.initialDelay ?initDelay.
        #  ?rtc c:TapChanger.lowStep ?lowStep.
        # }
        # ORDER BY ?pname ?tname ?rname ?phs ?wnum
        # """% self.feeder_mrid

        # results = self.gad.query_data(VALUES_QUERY)
        # bindings = results['data']['results']['bindings']
        bindings = []
        for ratioTapChanger in self.feederModel.graph.get(cim.RatioTapChanger, {}).values():
            regDict = {}
            regDict['rid'] = ratioTapChanger.mRID
            tEnd = ratioTapChanger.TransformerEnd
            if isinstance(tEnd, cim.PowerTransformerEnd):
                regDict['pname'] = tEnd.PowerTransformer.name
            else:
                regDict['pname'] = tEnd.TransformerTank.PowerTransformer.name
                regDict['tname'] = tEnd.TransformerTank.name
                regDict['phs'] = tEnd.orderedPhases.value
            regDict['wnum'] = tEnd.endNumber
            bindings.append(regDict)
        bindingsSorted = sorted(bindings, key=lambda x:(x.get('pname', ""),
                                                        x.get('tname', ""),
                                                        x.get('rname', ""),
                                                        x.get('phs', ""),
                                                        x.get('wnum', 0)))
        return bindingsSorted


    def lines_connectivity_query(self):
        LINES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?bus1 ?bus2 ?id (group_concat(distinct ?phs;separator="") as ?phases) WHERE {
        SELECT ?name ?bus1 ?bus2 ?phs ?id WHERE {
        VALUES ?fdrid {"%s"}
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?s r:type c:ACLineSegment.
         ?s c:Equipment.EquipmentContainer ?fdr.
         ?s c:IdentifiedObject.name ?name.
         ?s c:IdentifiedObject.mRID ?id.
         ?t1 c:Terminal.ConductingEquipment ?s.
         ?t1 c:ACDCTerminal.sequenceNumber "1".
         ?t1 c:Terminal.ConnectivityNode ?cn1.
         ?cn1 c:IdentifiedObject.name ?bus1.
         ?t2 c:Terminal.ConductingEquipment ?s.
         ?t2 c:ACDCTerminal.sequenceNumber "2".
         ?t2 c:Terminal.ConnectivityNode ?cn2.
         ?cn2 c:IdentifiedObject.name ?bus2.
         OPTIONAL {?acp c:ACLineSegmentPhase.ACLineSegment ?s.
           ?acp c:ACLineSegmentPhase.phase ?phsraw.
             bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs)}
         } ORDER BY ?name ?phs
        }
        GROUP BY ?name ?bus1 ?bus2 ?id
        ORDER BY ?name
        """% self.feeder_mrid

        results = self.gad.query_data(LINES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings

    def power_transformer_connectivity_query(self):
        XFMRS_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?xfmr_name ?vector_group ?end_number ?bus ?base_voltage ?connection ?ratedS ?ratedU ?r_ohm ?angle ?grounded ?r_ground ?x_ground
        WHERE {
        VALUES ?fdrid {"%s"}
         ?p c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?p r:type c:PowerTransformer.
         ?p c:IdentifiedObject.name ?xfmr_name.
         ?p c:PowerTransformer.vectorGroup ?vector_group.
         ?end c:PowerTransformerEnd.PowerTransformer ?p.
         ?end c:TransformerEnd.endNumber ?end_number.
         ?end c:PowerTransformerEnd.ratedS ?ratedS.
         ?end c:PowerTransformerEnd.ratedU ?ratedU.
         ?end c:PowerTransformerEnd.r ?r_ohm.
         ?end c:PowerTransformerEnd.phaseAngleClock ?angle.
         ?end c:PowerTransformerEnd.connectionKind ?connraw.
          bind(strafter(str(?connraw),"WindingConnection.") as ?connection)
         ?end c:TransformerEnd.grounded ?grounded.
         OPTIONAL {?end c:TransformerEnd.rground ?r_ground.}
         OPTIONAL {?end c:TransformerEnd.xground ?x_ground.}
         ?end c:TransformerEnd.Terminal ?trm.
         ?trm c:Terminal.ConnectivityNode ?cn.
         ?cn c:IdentifiedObject.name ?bus.
         ?end c:TransformerEnd.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?base_voltage.
        }
        ORDER BY ?xfmr_name ?end_number
        """% self.feeder_mrid

        results = self.gad.query_data(XFMRS_QUERY, timeout=1200)
        bindings = results['data']['results']['bindings']
        return bindings


    def tank_transformer_connectivity_query(self):
        # XFMRS_QUERY = """
        # PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        # PREFIX c:  <http://iec.ch/TC57/CIM100#>
        # SELECT ?xfmr_name ?xfmr_code ?vector_group ?enum ?bus ?baseV ?phase ?grounded ?rground ?xground
        # WHERE {
        # VALUES ?fdrid {"%s"}
        #  ?p c:Equipment.EquipmentContainer ?fdr.
        #  ?fdr c:IdentifiedObject.mRID ?fdrid.
        #  ?p r:type c:PowerTransformer.
        #  ?p c:IdentifiedObject.name ?pname.
        #  ?p c:PowerTransformer.vectorGroup ?vector_group.
        #  ?t c:TransformerTank.PowerTransformer ?p.
        #  ?t c:IdentifiedObject.name ?xfmr_name.
        #  ?asset c:Asset.PowerSystemResources ?t.
        #  ?asset c:Asset.AssetInfo ?inf.
        #  ?inf c:IdentifiedObject.name ?xfmr_code.
        #  ?end c:TransformerTankEnd.TransformerTank ?t.
        #  ?end c:TransformerTankEnd.phases ?phsraw.
        #   bind(strafter(str(?phsraw),"PhaseCode.") as ?phase)
        #  ?end c:TransformerEnd.endNumber ?enum.
        #  ?end c:TransformerEnd.grounded ?grounded.
        #  OPTIONAL {?end c:TransformerEnd.rground ?rground.}
        #  OPTIONAL {?end c:TransformerEnd.xground ?xground.}
        #  ?end c:TransformerEnd.Terminal ?trm.
        #  ?trm c:Terminal.ConnectivityNode ?cn.
        #  ?cn c:IdentifiedObject.name ?bus.
        #  ?end c:TransformerEnd.BaseVoltage ?bv.
        #  ?bv c:BaseVoltage.nominalVoltage ?baseV.
        # }
        # ORDER BY ?xfmr_name ?enum
        # """% self.feeder_mrid

        # results = self.gad.query_data(XFMRS_QUERY)
        # bindings = results['data']['results']['bindings']
        bindings = []
        for xfmrTankEnd in self.feederModel.graph.get(cim.TransformerTankEnd, {}):
            xfmrDict = {}
            xfmrDict['xfmr_name'] = xfmrTankEnd.TransformerTank.PowerTransformer.name
            xfmrDict['bus'] = xfmrTankEnd.Terminal.ConnectivityNode.name
            xfmrDict['phase'] = xfmrTankEnd.orderedPhases.value
            xfmrDict['enum'] = xfmrTankEnd.endNumber
            bindings.append(xfmrDict)
        bindingsSorted = sorted(bindings, key=itemgetter('xfmr_name', 'enum'))
        return bindingsSorted


    def switch_connectivity_query(self):
        SWITCH_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?basev ?open ?bus1 ?bus2 (group_concat(distinct ?phs;separator="\\n") as ?phases)
        WHERE {
         ?s r:type c:LoadBreakSwitch.
        VALUES ?fdrid {"%s"}
         ?s c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?s c:IdentifiedObject.name ?name.
         ?s c:ConductingEquipment.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?base_V.
         ?s c:Switch.normalOpen ?open.
         ?t1 c:Terminal.ConductingEquipment ?s.
         ?t1 c:ACDCTerminal.sequenceNumber "1".
         ?t1 c:Terminal.ConnectivityNode ?cn1.
         ?cn1 c:IdentifiedObject.name ?bus1.
         ?t2 c:Terminal.ConductingEquipment ?s.
         ?t2 c:ACDCTerminal.sequenceNumber "2".
         ?t2 c:Terminal.ConnectivityNode ?cn2.
         ?cn2 c:IdentifiedObject.name ?bus2
           OPTIONAL {?swp c:SwitchPhase.Switch ?s.
           ?swp c:SwitchPhase.phaseSide1 ?phsraw.
           bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
        }
        GROUP BY ?name ?basev ?open ?fdrid ?bus1 ?bus2
        ORDER BY ?name
        """% self.feeder_mrid

        results = self.gad.query_data(SWITCH_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def energyconsumer_query(self):
        """Get information on loads in the feeder."""
        # Perform the query.
        LOAD_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?bus ?basev ?p ?q ?conn ?cnt ?pz ?qz ?pi ?qi ?pp ?qp ?pe ?qe ?fdrid (group_concat(distinct ?phs;separator="\\n") as ?phases) WHERE {
        ?s r:type c:EnergyConsumer.
        VALUES ?fdrid {"%s"}
        ?s c:Equipment.EquipmentContainer ?fdr.
        ?fdr c:IdentifiedObject.mRID ?fdrid.
        ?s c:IdentifiedObject.name ?name.
        ?s c:ConductingEquipment.BaseVoltage ?bv.
        ?bv c:BaseVoltage.nominalVoltage ?basev.
        ?s c:EnergyConsumer.customerCount ?cnt.
        ?s c:EnergyConsumer.p ?p.
        ?s c:EnergyConsumer.q ?q.
        ?s c:EnergyConsumer.phaseConnection ?connraw.
        bind(strafter(str(?connraw),"PhaseShuntConnectionKind.") as ?conn)
        ?s c:EnergyConsumer.LoadResponse ?lr.
        ?lr c:LoadResponseCharacteristic.pConstantImpedance ?pz.
        ?lr c:LoadResponseCharacteristic.qConstantImpedance ?qz.
        ?lr c:LoadResponseCharacteristic.pConstantCurrent ?pi.
        ?lr c:LoadResponseCharacteristic.qConstantCurrent ?qi.
        ?lr c:LoadResponseCharacteristic.pConstantPower ?pp.
        ?lr c:LoadResponseCharacteristic.qConstantPower ?qp.
        ?lr c:LoadResponseCharacteristic.pVoltageExponent ?pe.
        ?lr c:LoadResponseCharacteristic.qVoltageExponent ?qe.
        OPTIONAL {?ecp c:EnergyConsumerPhase.EnergyConsumer ?s.
        ?ecp c:EnergyConsumerPhase.phase ?phsraw.
        bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
        ?t c:Terminal.ConductingEquipment ?s.
        ?t c:Terminal.ConnectivityNode ?cn.
        ?cn c:IdentifiedObject.name ?bus
        }
        GROUP BY ?name ?bus ?basev ?p ?q ?cnt ?conn ?pz ?qz ?pi ?qi ?pp ?qp ?pe ?qe ?fdrid
        ORDER by ?name
        """% self.feeder_mrid

        results = self.gad.query_data(LOAD_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings

    def energysource_query(self):
        """Get information on loads in the feeder."""
        # Perform the query.
        SOURCE_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?bus ?basev ?nomv ?vmag ?vang ?r1 ?x1 ?r0 ?x0 WHERE {
        ?s r:type c:EnergySource.
        VALUES ?fdrid {"%s"}
        ?s c:Equipment.EquipmentContainer ?fdr.
        ?fdr c:IdentifiedObject.mRID ?fdrid.
        ?s c:IdentifiedObject.name ?name.
        ?s c:ConductingEquipment.BaseVoltage ?bv.
        ?bv c:BaseVoltage.nominalVoltage ?basev.
        ?s c:EnergySource.nominalVoltage ?nomv. 
        ?s c:EnergySource.voltageMagnitude ?vmag. 
        ?s c:EnergySource.voltageAngle ?vang. 
        ?s c:EnergySource.r ?r1. 
        ?s c:EnergySource.x ?x1. 
        ?s c:EnergySource.r0 ?r0. 
        ?s c:EnergySource.x0 ?x0. 
        ?t c:Terminal.ConductingEquipment ?s.
        ?t c:Terminal.ConnectivityNode ?cn. 
        ?cn c:IdentifiedObject.name ?bus
        }
        ORDER by ?name
        """% self.feeder_mrid

        results = self.gad.query_data(SOURCE_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


# End of Common Competing Apps queries

# Start of Static/Dynamic Y-bus queries

    def ybus_export(self):
        message = {
        "configurationType": "YBus Export",
        "parameters": {
            "model_id": self.feeder_mrid}
        }

        results = self.gad.get_response("goss.gridappsd.process.request.config", message, timeout=1200)
        return results['data']['yParse'],results['data']['nodeList']


    def vnom_export(self):
        message = {
        "configurationType": "Vnom Export",
        "parameters": {
            "model_id": self.feeder_mrid}
        }

        results = self.gad.get_response("goss.gridappsd.process.request.config", message, timeout=1200)
        return results['data']['vnom']

# End of Static/Dynamic Y-bus queries

