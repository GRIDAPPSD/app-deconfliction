"""Module for querying and parsing SPARQL through GridAPPS-D"""
import logging
import pandas as pd
import numpy as np
import re
from gridappsd import GridAPPSD, topics, utils

class SPARQLManager:
    """Class for querying SPARQL in GridAPPS-D Toolbox tools/services
    """
    
    def __init__(self, gapps, feeder_mrid, simulation_id=None, timeout=30):
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

# Start of Common Competing Apps queries

    def obj_dict_export(self, objectType):
        message = {
          "requestType": "QUERY_OBJECT_DICT",
          "modelId": self.feeder_mrid,
          "objectType": objectType,
          "resultFormat": "JSON",
        }

        results = self.gad.get_response(topics.REQUEST_POWERGRID_DATA, message, timeout=1200)
        return results['data']


    def obj_meas_export(self, objectType):
        message = {
          "requestType": "QUERY_OBJECT_MEASUREMENTS",
          "modelId": self.feeder_mrid,
          "objectType": objectType,
          "resultFormat": "JSON",
        }

        results = self.gad.get_response(topics.REQUEST_POWERGRID_DATA, message, timeout=1200)
        return results['data']


    def battery_query(self):
        VALUES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?bus ?ratedS ?ratedU ?ipu ?ratedE ?storedE ?state ?p ?q ?id ?fdrid (group_concat(distinct ?phs;separator="\\n") as ?phases) WHERE {
         ?s r:type c:BatteryUnit.
         ?s c:IdentifiedObject.name ?name.
         ?pec c:PowerElectronicsConnection.PowerElectronicsUnit ?s.
        VALUES ?fdrid {"%s"}
         ?pec c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?pec c:PowerElectronicsConnection.ratedS ?ratedS.
         ?pec c:PowerElectronicsConnection.ratedU ?ratedU.
         ?pec c:PowerElectronicsConnection.maxIFault ?ipu.
         ?s c:BatteryUnit.ratedE ?ratedE.
         ?s c:BatteryUnit.storedE ?storedE.
         ?s c:BatteryUnit.batteryState ?stateraw.
           bind(strafter(str(?stateraw),"BatteryState.") as ?state)
         ?pec c:PowerElectronicsConnection.p ?p.
         ?pec c:PowerElectronicsConnection.q ?q.
         OPTIONAL {?pecp c:PowerElectronicsConnectionPhase.PowerElectronicsConnection ?pec.
         ?pecp c:PowerElectronicsConnectionPhase.phase ?phsraw.
           bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
         bind(strafter(str(?s),"#_") as ?id).
         ?t c:Terminal.ConductingEquipment ?pec.
         ?t c:Terminal.ConnectivityNode ?cn. 
         ?cn c:IdentifiedObject.name ?bus
        }
        GROUP by ?name ?bus ?ratedS ?ratedU ?ipu ?ratedE ?storedE ?state ?p ?q ?id ?fdrid
        ORDER by ?name
        """% self.feeder_mrid

        results = self.gad.query_data(VALUES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def pv_query(self):
        VALUES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?name ?bus ?ratedS ?ratedU ?ipu ?p ?q ?id ?fdrid (group_concat(distinct ?phs;separator="\\n") as ?phases) WHERE {
         ?s r:type c:PhotovoltaicUnit.
         ?s c:IdentifiedObject.name ?name.
         ?s c:IdentifiedObject.mRID ?id.
         ?pec c:PowerElectronicsConnection.PowerElectronicsUnit ?s.
        VALUES ?fdrid {"%s"}
         ?pec c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?pec c:PowerElectronicsConnection.ratedS ?ratedS.
         ?pec c:PowerElectronicsConnection.ratedU ?ratedU.
         ?pec c:PowerElectronicsConnection.maxIFault ?ipu.
         ?pec c:PowerElectronicsConnection.p ?p.
         ?pec c:PowerElectronicsConnection.q ?q.
         OPTIONAL {?pecp c:PowerElectronicsConnectionPhase.PowerElectronicsConnection ?pec.
         ?pecp c:PowerElectronicsConnectionPhase.phase ?phsraw.
         bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs) }
         ?t c:Terminal.ConductingEquipment ?pec.
         ?t c:Terminal.ConnectivityNode ?cn. 
         ?cn c:IdentifiedObject.name ?bus
        }
        GROUP by ?name ?bus ?ratedS ?ratedU ?ipu ?p ?q ?id ?fdrid
        ORDER by ?name
        """% self.feeder_mrid

        results = self.gad.query_data(VALUES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def regulator_query(self):
        VALUES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?rname ?pname ?tname ?wnum ?phs ?step
        WHERE {
        VALUES ?fdrid {"%s"}
         ?pxf c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?rtc r:type c:RatioTapChanger.
         ?rtc c:IdentifiedObject.name ?rname.
         ?rtc c:RatioTapChanger.TransformerEnd ?end.
         ?end c:TransformerEnd.endNumber ?wnum.
        {?end c:PowerTransformerEnd.PowerTransformer ?pxf.}
          UNION
        {?end c:TransformerTankEnd.TransformerTank ?tank.
         ?tank c:IdentifiedObject.name ?tname.
         OPTIONAL {?end c:TransformerTankEnd.phases ?phsraw.
          bind(strafter(str(?phsraw),"PhaseCode.") as ?phs)}
         ?tank c:TransformerTank.PowerTransformer ?pxf.}
         ?pxf c:IdentifiedObject.name ?pname.
         ?rtc c:RatioTapChanger.stepVoltageIncrement ?incr.
         ?rtc c:RatioTapChanger.tculControlMode ?moderaw.
          bind(strafter(str(?moderaw),"TransformerControlMode.") as ?mode)
         ?rtc c:TapChanger.controlEnabled ?enabled.
         ?rtc c:TapChanger.highStep ?highStep.
         ?rtc c:TapChanger.initialDelay ?initDelay.
         ?rtc c:TapChanger.lowStep ?lowStep.
         ?rtc c:TapChanger.ltcFlag ?ltc.
         ?rtc c:TapChanger.neutralStep ?neutralStep.
         ?rtc c:TapChanger.neutralU ?neutralU.
         ?rtc c:TapChanger.normalStep ?normalStep.
         ?rtc c:TapChanger.step ?step.
        }
        ORDER BY ?pname ?tname ?rname ?wnum
        """% self.feeder_mrid

        results = self.gad.query_data(VALUES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def regulator_combine_query(self):
        VALUES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?rname ?pname ?tname ?wnum ?phs ?incr ?mode ?enabled ?highStep ?lowStep
        WHERE {
        VALUES ?fdrid {"%s"}
         ?pxf c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?rtc r:type c:RatioTapChanger.
         ?rtc c:IdentifiedObject.name ?rname.
         ?rtc c:RatioTapChanger.TransformerEnd ?end.
         ?end c:TransformerEnd.endNumber ?wnum.
        {?end c:PowerTransformerEnd.PowerTransformer ?pxf.}
          UNION
        {?end c:TransformerTankEnd.TransformerTank ?tank.
         ?tank c:IdentifiedObject.name ?tname.
         OPTIONAL {?end c:TransformerTankEnd.phases ?phsraw.
          bind(strafter(str(?phsraw),"PhaseCode.") as ?phs)}
         ?tank c:TransformerTank.PowerTransformer ?pxf.}
         ?pxf c:IdentifiedObject.name ?pname.
         ?rtc c:RatioTapChanger.stepVoltageIncrement ?incr.
         ?rtc c:RatioTapChanger.tculControlMode ?moderaw.
          bind(strafter(str(?moderaw),"TransformerControlMode.") as ?mode)
         ?rtc c:TapChanger.controlEnabled ?enabled.
         ?rtc c:TapChanger.highStep ?highStep.
         ?rtc c:TapChanger.initialDelay ?initDelay.
         ?rtc c:TapChanger.lowStep ?lowStep.
        }
        ORDER BY ?pname ?tname ?rname ?wnum
        """% self.feeder_mrid

        results = self.gad.query_data(VALUES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


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

        results = self.gad.query_data(XFMRS_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def tank_transformer_connectivity_query(self):
        XFMRS_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?xfmr_name ?xfmr_code ?vector_group ?enum ?bus ?baseV ?phase ?grounded ?rground ?xground
        WHERE {
        VALUES ?fdrid {"%s"}
         ?p c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?p r:type c:PowerTransformer.
         ?p c:IdentifiedObject.name ?pname.
         ?p c:PowerTransformer.vectorGroup ?vector_group.
         ?t c:TransformerTank.PowerTransformer ?p.
         ?t c:IdentifiedObject.name ?xfmr_name.
         ?asset c:Asset.PowerSystemResources ?t.
         ?asset c:Asset.AssetInfo ?inf.
         ?inf c:IdentifiedObject.name ?xfmr_code.
         ?end c:TransformerTankEnd.TransformerTank ?t.
         ?end c:TransformerTankEnd.phases ?phsraw.
          bind(strafter(str(?phsraw),"PhaseCode.") as ?phase)
         ?end c:TransformerEnd.endNumber ?enum.
         ?end c:TransformerEnd.grounded ?grounded.
         OPTIONAL {?end c:TransformerEnd.rground ?rground.}
         OPTIONAL {?end c:TransformerEnd.xground ?xground.}
         ?end c:TransformerEnd.Terminal ?trm.
         ?trm c:Terminal.ConnectivityNode ?cn.
         ?cn c:IdentifiedObject.name ?bus.
         ?end c:TransformerEnd.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?baseV.
        }
        ORDER BY ?xfmr_name ?enum
        """% self.feeder_mrid

        results = self.gad.query_data(XFMRS_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


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


# End of Common Competing Apps queries

# Start of Static/Dynamic Y-bus queries

    def TransformerTank_xfmr_names(self):
        XFMRS_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?xfmr_name ?xfmr_code ?vector_group ?enum ?bus ?baseV ?phase ?grounded ?rground ?xground
        WHERE {
        VALUES ?fdrid {"%s"}
         ?p c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?p r:type c:PowerTransformer.
         ?p c:IdentifiedObject.name ?pname.
         ?p c:PowerTransformer.vectorGroup ?vector_group.
         ?t c:TransformerTank.PowerTransformer ?p.
         ?t c:IdentifiedObject.name ?xfmr_name.
         ?asset c:Asset.PowerSystemResources ?t.
         ?asset c:Asset.AssetInfo ?inf.
         ?inf c:IdentifiedObject.name ?xfmr_code.
         ?end c:TransformerTankEnd.TransformerTank ?t.
         ?end c:TransformerTankEnd.phases ?phsraw.
          bind(strafter(str(?phsraw),"PhaseCode.") as ?phase)
         ?end c:TransformerEnd.endNumber ?enum.
         ?end c:TransformerEnd.grounded ?grounded.
         OPTIONAL {?end c:TransformerEnd.rground ?rground.}
         OPTIONAL {?end c:TransformerEnd.xground ?xground.}
         ?end c:TransformerEnd.Terminal ?trm.
         ?trm c:Terminal.ConnectivityNode ?cn.
         ?cn c:IdentifiedObject.name ?bus.
         ?end c:TransformerEnd.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?baseV.
        }
        ORDER BY ?xfmr_name ?enum
        """% self.feeder_mrid

        results = self.gad.query_data(XFMRS_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def SwitchingEquipment_switch_names(self):
        SWITCHES_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?sw_name ?base_V ?is_Open ?rated_Current ?breaking_Capacity ?sw_ph_status ?bus1 ?bus2 (group_concat(distinct ?phs1;separator="") as ?phases_side1) (group_concat(distinct ?phs2;separator="") as ?phases_side2)
        WHERE {
        VALUES ?fdrid {"%s"}
         VALUES ?cimraw {c:LoadBreakSwitch c:Recloser c:Breaker c:Fuse c:Sectionaliser c:Jumper c:Disconnector c:GroundDisconnector}
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?s r:type ?cimraw.
         bind(strafter(str(?cimraw),"#") as ?cimtype)
         ?s c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?s c:IdentifiedObject.name ?sw_name.
         ?s c:ConductingEquipment.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?base_V.
         ?s c:Switch.open ?is_Open.
         ?s c:Switch.ratedCurrent ?rated_Current.
         OPTIONAL {?s c:ProtectedSwitch.breakingCapacity ?breaking_Capacity.}
         ?t1 c:Terminal.ConductingEquipment ?s.
         ?t1 c:ACDCTerminal.sequenceNumber "1".
         ?t1 c:Terminal.ConnectivityNode ?cn1.
         ?cn1 c:IdentifiedObject.name ?bus1.
         ?t2 c:Terminal.ConductingEquipment ?s.
         ?t2 c:ACDCTerminal.sequenceNumber "2".
         ?t2 c:Terminal.ConnectivityNode ?cn2.
         ?cn2 c:IdentifiedObject.name ?bus2.
         OPTIONAL {?swp c:SwitchPhase.Switch ?s.
          ?swp c:SwitchPhase.phaseSide1 ?phsraw.
          ?swp c:SwitchPhase.normalOpen ?sw_ph_status.
          bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phs1)
          ?swp c:SwitchPhase.phaseSide2 ?phsraw2.
          bind(strafter(str(?phsraw2),"SinglePhaseKind.") as ?phs2)}
        }
        GROUP BY ?sw_name ?base_V ?is_Open ?rated_Current ?breaking_Capacity ?sw_ph_status ?bus1 ?bus2
        ORDER BY ?sw_name ?sw_phase_name
        """% self.feeder_mrid

        results = self.gad.query_data(SWITCHES_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


    def ShuntElement_cap_names(self):
        SHUNT_QUERY = """
        PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX c:  <http://iec.ch/TC57/CIM100#>
        SELECT ?cap_name ?base_volt ?nominal_volt ?b_per_section ?bus ?connection ?grnd ?phase ?ctrlenabled ?discrete ?mode ?deadband ?setpoint ?delay ?monitored_class ?monitored_eq ?monitored_bus ?monitored_phs
        WHERE {
         ?s r:type c:LinearShuntCompensator.
        VALUES ?fdrid {"%s"}
         ?s c:Equipment.EquipmentContainer ?fdr.
         ?fdr c:IdentifiedObject.mRID ?fdrid.
         ?s c:IdentifiedObject.name ?cap_name.
         ?s c:ConductingEquipment.BaseVoltage ?bv.
         ?bv c:BaseVoltage.nominalVoltage ?base_volt.
         ?s c:ShuntCompensator.nomU ?nominal_volt.
         ?s c:LinearShuntCompensator.bPerSection ?b_per_section.
         ?s c:ShuntCompensator.phaseConnection ?connraw.
          bind(strafter(str(?connraw),"PhaseShuntConnectionKind.") as ?connection)
         ?s c:ShuntCompensator.grounded ?grnd.
         OPTIONAL {?scp c:ShuntCompensatorPhase.ShuntCompensator ?s.
         ?scp c:ShuntCompensatorPhase.phase ?phsraw.
          bind(strafter(str(?phsraw),"SinglePhaseKind.") as ?phase)}
         OPTIONAL {?ctl c:RegulatingControl.RegulatingCondEq ?s.
          ?ctl c:RegulatingControl.discrete ?discrete.
          ?ctl c:RegulatingControl.enabled ?ctrlenabled.
          ?ctl c:RegulatingControl.mode ?moderaw.
           bind(strafter(str(?moderaw),"RegulatingControlModeKind.") as ?mode)
          ?ctl c:RegulatingControl.monitoredPhase ?monraw.
           bind(strafter(str(?monraw),"PhaseCode.") as ?monitored_phs)
          ?ctl c:RegulatingControl.targetDeadband ?deadband.
          ?ctl c:RegulatingControl.targetValue ?setpoint.
          ?s c:ShuntCompensator.aVRDelay ?delay.
          ?ctl c:RegulatingControl.Terminal ?trm.
          ?trm c:Terminal.ConductingEquipment ?eq.
          ?eq a ?classraw.
           bind(strafter(str(?classraw),"CIM100#") as ?monitored_class)
          ?eq c:IdentifiedObject.name ?monitored_eq.
          ?trm c:Terminal.ConnectivityNode ?moncn.
          ?moncn c:IdentifiedObject.name ?monitored_bus.}
         ?s c:IdentifiedObject.mRID ?id.
         ?t c:Terminal.ConductingEquipment ?s.
         ?t c:Terminal.ConnectivityNode ?cn.
         ?cn c:IdentifiedObject.name ?bus
        }
        ORDER BY ?cap_name
        """% self.feeder_mrid

        results = self.gad.query_data(SHUNT_QUERY)
        bindings = results['data']['results']['bindings']
        return bindings


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


    def cim_export(self):
        message = {
        "configurationType":"CIM Dictionary",
        "parameters": {
            "simulation_id": self.simulation_id }
        }

        results = self.gad.get_response('goss.gridappsd.process.request.config', message, timeout=1200)
        return results['data']['feeders']

# End of Static/Dynamic Y-bus queries

