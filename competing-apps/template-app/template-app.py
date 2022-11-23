
# Copyright (c) 2021, Battelle Memorial Institute All rights reserved.
# Battelle Memorial Institute (hereinafter Battelle) hereby grants permission to any person or entity
# lawfully obtaining a copy of this software and associated documentation files (hereinafter the
# Software) to redistribute and use the Software in source and binary forms, with or without modification.
# Such person or entity may use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and may permit others to do so, subject to the following conditions:
# Redistributions of source code must retain the above copyright notice, this list of conditions and the
# following disclaimers.
# Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
# the following disclaimer in the documentation and/or other materials provided with the distribution.
# Other than as used herein, neither the name Battelle Memorial Institute or Battelle may be used in any
# form whatsoever without the express written consent of Battelle.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# BATTELLE OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
# General disclaimer for use with OSS licenses
#
# This material was prepared as an account of work sponsored by an agency of the United States Government.
# Neither the United States Government nor the United States Department of Energy, nor Battelle, nor any
# of their employees, nor any jurisdiction or organization that has cooperated in the development of these
# materials, makes any warranty, express or implied, or assumes any legal liability or responsibility for
# the accuracy, completeness, or usefulness or any information, apparatus, product, software, or process
# disclosed, or represents that its use would not infringe privately owned rights.
#
# Reference herein to any specific commercial product, process, or service by trade name, trademark, manufacturer,
# or otherwise does not necessarily constitute or imply its endorsement, recommendation, or favoring by the United
# States Government or any agency thereof, or Battelle Memorial Institute. The views and opinions of authors expressed
# herein do not necessarily state or reflect those of the United States Government or any agency thereof.
#
# PACIFIC NORTHWEST NATIONAL LABORATORY operated by BATTELLE for the
# UNITED STATES DEPARTMENT OF ENERGY under Contract DE-AC05-76RL01830
# ------------------------------------------------------------------------------
"""
Created on October 31, 2022

@author: Gary Black
"""""

import sys
import time
import os
import argparse
import json
import importlib
import math
import pprint
import numpy as np
import csv

from gridappsd import GridAPPSD
from gridappsd.topics import simulation_output_topic, simulation_log_topic, service_output_topic, service_input_topic


class SimWrapper(object):
  def __init__(self, gapps, feeder_mrid, simulation_id, Ybus, NodeIndex, SwitchMridToNodes, TransformerMridToNodes, TransformerLastPos, CapacitorMridToNode, CapacitorMridToYbusContrib, CapacitorLastValue):
    self.gapps = gapps
    self.feeder_mrid = feeder_mrid
    self.simulation_id = simulation_id
    self.timestamp = 0
    self.Ybus = Ybus
    self.NodeIndex = NodeIndex
    self.SwitchMridToNodes = SwitchMridToNodes
    self.TransformerMridToNodes = TransformerMridToNodes
    self.TransformerLastPos = TransformerLastPos
    self.CapacitorMridToNode = CapacitorMridToNode
    self.CapacitorMridToYbusContrib = CapacitorMridToYbusContrib
    self.CapacitorLastValue = CapacitorLastValue
    self.keepLoopingFlag = True
    self.publish_to_topic = service_output_topic('gridappsd-dynamic-ybus', simulation_id)
    self.ybusInitFlag = False


  def keepLooping(self):
    return self.keepLoopingFlag


  def checkSwitchOpen(self, nodes):
    try:
      Yval = self.Ybus[nodes[0]][nodes[1]].real
      if abs(Yval) <= 0.001:
        return True
      elif Yval <= -500.0:
        return False
      else:
        print('*** WARNING: Found unexpected switch Ybus value checking for open: ' + str(Yval), flush=True)
        return False
    except:
      return True


  def checkSwitchClosed(self, nodes):
    try:
      Yval = self.Ybus[nodes[0]][nodes[1]].real
      if Yval <= -500.0:
        return True
      elif abs(Yval) <= 0.001:
        return False
      else:
        print('*** WARNING: Found unexpected switch Ybus value checking for closed: ' + str(Yval), flush=True)
        return False
    except:
      return False


  def printLower(self, YbusPrint):
    for noderow in YbusPrint:
      rowFlag = False
      for nodecol,value in YbusPrint[noderow].items():
        if self.NodeIndex[noderow] >= self.NodeIndex[nodecol]:
          rowFlag = True
          #print('['+noderow+','+nodecol+']='+str(value), flush=True)
          print('['+noderow+':'+str(self.NodeIndex[noderow])+','+nodecol+':'+str(self.NodeIndex[nodecol])+']='+str(value), flush=True)
      if rowFlag:
        print('', flush=True)


  def lowerUncomplex(self, YbusComplex):
    YbusUncomplex = {}
    for noderow in YbusComplex:
      for nodecol,value in YbusComplex[noderow].items():
        if self.NodeIndex[noderow] >= self.NodeIndex[nodecol]:
          if noderow not in YbusUncomplex:
            YbusUncomplex[noderow] = {}
          YbusUncomplex[noderow][nodecol] = (value.real, value.imag)

    return YbusUncomplex


  def publishYbus(self, YbusChanges):
    #print('\nYbusChanges lower diagonal:', flush=True)
    #self.printLower(YbusChanges)
    #print('Full Ybus lower diagonal:', flush=True)
    #self.printLower(self.Ybus)

    # JSON can't serialize complex values so convert to a tuple of real and
    # imaginary values while also only populating lower diagonal elements
    # for both ybus versions
    lowerFull = self.lowerUncomplex(self.Ybus)
    lowerChanges = self.lowerUncomplex(YbusChanges)
    message = {
      'feeder_id': self.feeder_mrid,
      'simulation_id': self.simulation_id,
      'timestamp': self.timestamp,
      'ybus': lowerFull,
      'ybusChanges': lowerChanges
    }
    self.gapps.send(self.publish_to_topic, message)
    print('\nYbus published message:', flush=True)
    print(message, flush=True)
    print('')


  def publishStatus(self, status):
    message = {
      'feeder_id': self.feeder_mrid,
      'simulation_id': self.simulation_id,
      'processStatus': status
    }
    self.gapps.send(self.publish_to_topic, message)
    print('\nStatus published message:', flush=True)
    print(message, flush=True)
    print('')


  def process_switches(self, msgdict, YbusChanges):
    switchOpenValue = complex(0,0)
    switchClosedValue = complex(-500,500)

    # Switch processing
    for mrid in self.SwitchMridToNodes:
      try:
        value = msgdict['measurements'][mrid]['value']
        nodes = self.SwitchMridToNodes[mrid] # two endpoints for switch
        #print('Found switch mrid: ' + mrid + ', nodes: ' + str(nodes) + ', value: ' + str(value), flush=True)
        if value == 0: # open
          if not self.checkSwitchOpen(nodes):
            print('Switch value changed from closed to open for nodes: ' + str(nodes), flush=True)
            Yval_diag = -self.Ybus[nodes[0]][nodes[1]]
            self.Ybus[nodes[0]][nodes[1]] = self.Ybus[nodes[1]][nodes[0]] = switchOpenValue
            # Modify diagonal terms for both endpoints
            self.Ybus[nodes[0]][nodes[0]] -= Yval_diag
            self.Ybus[nodes[1]][nodes[1]] -= Yval_diag

            if nodes[0] not in YbusChanges:
              YbusChanges[nodes[0]] = {}
            if nodes[1] not in YbusChanges:
              YbusChanges[nodes[1]] = {}
            YbusChanges[nodes[0]][nodes[1]] = YbusChanges[nodes[1]][nodes[0]] = switchOpenValue

            YbusChanges[nodes[0]][nodes[0]] = self.Ybus[nodes[0]][nodes[0]]
            YbusChanges[nodes[1]][nodes[1]] = self.Ybus[nodes[1]][nodes[1]]

        else: # closed
          if not self.checkSwitchClosed(nodes):
            print('Switch value changed from open to closed for nodes: ' + str(nodes), flush=True)
            self.Ybus[nodes[0]][nodes[1]] = self.Ybus[nodes[1]][nodes[0]] = switchClosedValue
            self.Ybus[nodes[0]][nodes[0]] += -switchClosedValue
            self.Ybus[nodes[1]][nodes[1]] += -switchClosedValue

            if nodes[0] not in YbusChanges:
              YbusChanges[nodes[0]] = {}
            if nodes[1] not in YbusChanges:
              YbusChanges[nodes[1]] = {}
            YbusChanges[nodes[0]][nodes[1]] = YbusChanges[nodes[1]][nodes[0]] = switchClosedValue
            YbusChanges[nodes[0]][nodes[0]] = self.Ybus[nodes[0]][nodes[0]]
            YbusChanges[nodes[1]][nodes[1]] = self.Ybus[nodes[1]][nodes[1]]

      except:
        if mrid not in msgdict['measurements']:
          print('*** WARNING: Did not find switch mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        elif 'value' not in msgdict['measurements'][mrid]:
          print('*** WARNING: Did not find value element for switch mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        else:
          print('*** WARNING: Unknown exception processing switch mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
    return


  def process_transformers(self, msgdict, YbusChanges):
    # Transformer processing
    for mrid in self.TransformerMridToNodes:
      try:
        value = msgdict['measurements'][mrid]['value']
        nodes = self.TransformerMridToNodes[mrid]
        node1 = nodes[0]
        node2 = nodes[1]
        #print('Found transformer mrid: ' + mrid + ', node: ' + noderow + ', value: ' + str(value), flush=True)
        if value != self.TransformerLastPos[nodes[1]]:
          print('Transformer value changed for node: ' + node2 + ', old value: ' + str(self.TransformerLastPos[node2]) + ', new value: ' + str(value), flush=True)
          # calculate the admittance multiplier based on the change in the tap
          # position, last value vs. new value
          old_tap = (1.0 + self.TransformerLastPos[node2]*0.00625)
          new_tap = (1.0 + value*0.00625)
          posMultiplier = old_tap / new_tap
          self.TransformerLastPos[node2] = value

          # Update the entries of system Ybus for the given tap change
          # 1. The off-diagonal element (two terminals of xfmr)
          Yval_offdiag = self.Ybus[node1][node2]
          self.Ybus[node1][node2] = self.Ybus[node2][node1] = Yval_offdiag * posMultiplier
          # 2. The diagonal element of a regulating node
          Yval_diag = - Yval_offdiag / old_tap
          diff = self.Ybus[node2][node2] - Yval_diag
          self.Ybus[node2][node2] = - Yval_offdiag * old_tap / (new_tap ** 2) + diff

          if node1 not in YbusChanges:
            YbusChanges[node1] = {}
          if node2 not in YbusChanges:
            YbusChanges[node2] = {}
          YbusChanges[node1][node2] = YbusChanges[node2][node1] = self.Ybus[node1][node2]
          YbusChanges[node2][node2] = self.Ybus[node2][node2]

      except:
        if mrid not in msgdict['measurements']:
          print('*** WARNING: Did not find transformer mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        elif 'value' not in msgdict['measurements'][mrid]:
          print('*** WARNING: Did not find value element for transformer mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        else:
          print('*** WARNING: Unknown exception processing transformer mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
    return


  def process_capacitors(self, msgdict, YbusChanges):
    # Capacitor (LinearShuntCompensator) processing
    for mrid in self.CapacitorMridToNode:
      try:
        value = msgdict['measurements'][mrid]['value']
        noderow = self.CapacitorMridToNode[mrid]
        #print('Found capacitor mrid: ' + mrid + ', node: ' + noderow + ', value: ' + str(value), flush=True)
        if value == 0: # off
          if self.CapacitorLastValue[noderow] == 1:
            print('Capacitor value changed from on to off for node: ' + noderow, flush=True)
            self.CapacitorLastValue[noderow] = value
            self.Ybus[noderow][noderow] -= self.CapacitorMridToYbusContrib[mrid]
            if noderow not in YbusChanges:
              YbusChanges[noderow] = {}
            YbusChanges[noderow][noderow] = self.Ybus[noderow][noderow]

        elif value == 1: # on
          if self.CapacitorLastValue[noderow] == 0:
            print('Capacitor value changed from off to on for node: ' + noderow, flush=True)
            self.CapacitorLastValue[noderow] = value
            self.Ybus[noderow][noderow] += self.CapacitorMridToYbusContrib[mrid]
            if noderow not in YbusChanges:
              YbusChanges[noderow] = {}
            YbusChanges[noderow][noderow] = self.Ybus[noderow][noderow]

      except:
        if mrid not in msgdict['measurements']:
          print('*** WARNING: Did not find capacitor mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        elif 'value' not in msgdict['measurements'][mrid]:
          print('*** WARNING: Did not find value element for capacitor mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
        else:
          print('*** WARNING: Unknown exception processing capacitor mrid: ' + mrid + ' in measurement for timestamp: ' + str(self.timestamp), flush=True)
    return


  def on_message(self, header, message):
    # TODO workaround for broken unsubscribe method
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      status = message['processStatus']
      if status=='COMPLETE' or status=='CLOSED':
        self.keepLoopingFlag = False
        self.publishStatus(status)

    else:
      msgdict = message['message']
      self.timestamp = msgdict['timestamp']
      print('Processing simulation timestamp: ' + str(self.timestamp), flush=True)

      # Question: Andy mentioned creating a separate Ybus for each feeder and
      #   island.  Right now I have only a monolithic Ybus so need to come
      #   back and get more guidance on this.  Perhaps this is related to
      #   making dynamic YBus aware of Topology Processor as I'm not sure
      #   of the need for this otherwise

      YbusChanges = {} # minimal set of Ybus changes for timestamp

      self.process_switches(msgdict, YbusChanges)

      self.process_transformers(msgdict, YbusChanges)

      self.process_capacitors(msgdict, YbusChanges)

      if len(YbusChanges) > 0: # Ybus changed if there are any entries
        if not self.ybusInitFlag:
          print("*** First Ybus changes to initialize tap positions so don't publish, but allow snapshot responses!", flush=True)
          self.ybusInitFlag = True
        else:
          print('*** Ybus changed so I will publish full Ybus and YbusChanges!', flush=True)
          self.publishYbus(YbusChanges)
      else:
        print('Ybus NOT changed\n', flush=True)


def nodes_to_update(sparql_mgr):
    print('\nFinding dynamic Ybus nodes to track for simulation updates...', flush=True)

    phaseToIdx = {'A': '.1', 'B': '.2', 'C': '.3', 's1': '.1', 's2': '.2'}

    bindings = sparql_mgr.SwitchingEquipment_switch_names()
    switchToBuses = {}
    for obj in bindings:
      sw_name = obj['sw_name']['value']
      bus1 = obj['bus1']['value'].upper()
      bus2 = obj['bus2']['value'].upper()
      switchToBuses[sw_name] = [bus1, bus2]

    bindings = sparql_mgr.TransformerTank_xfmr_names()
    xfmrtoBuses = {}
    Buses = {}
    Phases = {}
    baseV = {}
    for obj in bindings:
      xfmr_name = obj['xfmr_name']['value']
      enum = int(obj['enum']['value'])
      if xfmr_name not in Buses:
        Buses[xfmr_name] = {}
        Phases[xfmr_name] = {}
        baseV[xfmr_name] = {}
      Buses[xfmr_name][enum] = obj['bus']['value'].upper()
      Phases[xfmr_name] = obj['phase']['value']
      baseV[xfmr_name][enum] = obj['baseV']['value']

    Nodes = {}
    for bus in Buses:
      # For regulator, the terminal base voltage should be equal
      if baseV[bus][1] == baseV[bus][2]:
        node1 = Buses[bus][1] + phaseToIdx[Phases[bus]]
        node2 = Buses[bus][2] + phaseToIdx[Phases[bus]]
        Nodes[node1] = Nodes[node2] = {}
        Nodes[node1]['conn'] = node2
        Nodes[node2]['conn'] = node1

    # Get the per capacitor Ybus contributions
    bindings = sparql_mgr.ShuntElement_cap_names()
    CapToYbusContrib = {}
    for obj in bindings:
      cap_name = obj['cap_name']['value']
      b_per_section = float(obj['b_per_section']['value'])
      CapToYbusContrib[cap_name] = complex(0.0, b_per_section)
      #print('cap_name: ' + cap_name + ', b_per_section: ' + str(b_per_section))

    feeders = sparql_mgr.cim_export()

    SwitchMridToNodes = {}
    TransformerMridToNodes = {}
    TransformerLastPos = {}
    CapacitorMridToNode = {}
    CapacitorLastValue = {}
    CapacitorMridToYbusContrib = {}

    for feeder in feeders:
      for meas in feeder['measurements']:
        # Pos measurement type includes both switches and regulators
        if meas['measurementType'] == 'Pos':
          mrid = meas['mRID']
          phase = meas['phases']
          if meas['ConductingEquipment_type'] == 'LoadBreakSwitch':
            sw_name = meas['ConductingEquipment_name']
            if sw_name in switchToBuses:
              buses = switchToBuses[sw_name]
              node1 = buses[0] + phaseToIdx[phase]
              node2 = buses[1] + phaseToIdx[phase]
              SwitchMridToNodes[mrid] = [node1, node2]
              print('Switch mrid: ' + mrid + ', nodes: ' + str(SwitchMridToNodes[mrid]), flush=True)
          elif meas['ConductingEquipment_type'] == 'PowerTransformer':
            node =  meas['ConnectivityNode'] + phaseToIdx[phase]
            node2 = node.upper()
            node1 = Nodes[node2]['conn']
            TransformerMridToNodes[mrid] = [node1, node2]
            TransformerLastPos[node2] = 0
            print('Transformer mrid: ' + mrid + ', nodes: ' + str(TransformerMridToNodes[mrid]), flush=True)
          elif meas['ConductingEquipment_type'] == 'LinearShuntCompensator':
            node = meas['ConnectivityNode'] + phaseToIdx[phase]
            node = node.upper()
            CapacitorMridToNode[mrid] = node
            CapacitorLastValue[node] = 1
            print('Capacitor mrid: ' + mrid + ', node: ' + node, flush=True)
            cap_name = meas['ConductingEquipment_name']
            if cap_name in CapToYbusContrib:
              CapacitorMridToYbusContrib[mrid] = CapToYbusContrib[cap_name]
              #print('Capacitor mrid: ' + mrid + ', node: ' + node + ', Ybus contribution: ' + str(CapToYbusContrib[cap_name]), flush=True)
            else:
              #print('Capacitor mrid: ' + mrid + ', node: ' + node, flush=True)
              print('*** WARNING: CIM dictionary capacitor name not found from b_per_section query: ' + cap_name, flush=True)
            #print(meas)

    #print('Switches:', flush=True)
    #pprint.pprint(SwitchMridToNodes)
    #print('Transformers:', flush=True)
    #pprint.pprint(TransformerMridToNode)
    #print('Capacitors:', flush=True)
    #pprint.pprint(CapacitorMridToNode)
    #pprint.pprint(CapacitorMridToYbusContrib)

    # Hold here for demo
    #text = input('\nWait here...')

    return SwitchMridToNodes,TransformerMridToNodes,TransformerLastPos,CapacitorMridToNode,CapacitorMridToYbusContrib,CapacitorLastValue


class DynamicYbus(GridAPPSD):

  def opendss_ybus(self, sparql_mgr):
    yParse,nodeList = sparql_mgr.ybus_export()

    idx = 1
    Nodes = {}
    NodeIndex = {}
    for obj in nodeList:
      Nodes[idx] = obj.strip('\"')
      NodeIndex[obj.strip('\"')] = idx
      idx += 1
    #pprint.pprint(Nodes)
    #pprint.pprint(NodeIndex)

    Ybus = {}
    for obj in yParse:
      items = obj.split(',')
      if items[0] == 'Row':
        continue
      if Nodes[int(items[0])] not in Ybus:
        Ybus[Nodes[int(items[0])]] = {}
      if Nodes[int(items[1])] not in Ybus:
        Ybus[Nodes[int(items[1])]] = {}
      Ybus[Nodes[int(items[0])]][Nodes[int(items[1])]] = Ybus[Nodes[int(items[1])]][Nodes[int(items[0])]] = complex(float(items[2]), float(items[3]))
    #pprint.pprint(Ybus)

    return NodeIndex, Ybus


  def on_message(self, headers, message):
    reply_to = headers['reply-to']

    if message['requestType'] == 'GET_SNAPSHOT_YBUS':
      # hold up responses until we know the ybus reflects the regulator
      # tap positions for the simulation
      while not self.simRap.ybusInitFlag:
        time.sleep(0.1)

      lowerUncomplex = self.simRap.lowerUncomplex(self.simRap.Ybus)
      message = {
        'feeder_id': self.simRap.feeder_mrid,
        'simulation_id': self.simRap.simulation_id,
        'timestamp': self.simRap.timestamp,
        'ybus': lowerUncomplex
      }
      print('Sending Ybus snapshot response for timestamp: ' + str(self.simRap.timestamp), flush=True)
      self.simRap.gapps.send(reply_to, message)

    else:
      message = "No valid requestType specified"
      self.simRap.gapps.send(reply_to, message)


  def charge_batteries(self, Batteries, eff_c, T):
    for name in Batteries:
      if 'P_batt_c' in Batteries[name]:
        Batteries[name]['SoC'] += eff_c*Batteries[name]['P_batt_c']*T/Batteries[name]['ratedE']


  def discharge_batteries(self, Batteries, eff_d, T):
    for name in Batteries:
      if 'P_batt_d' in Batteries[name]:
        Batteries[name]['SoC'] -= eff_d*Batteries[name]['P_batt_d']*T/Batteries[name]['ratedE']


  def resiliency(self, EnergyConsumers, SynchronousMachines, Batteries, SolarPVs,
                 emergencyState=False):

    P_load = 0.0
    for name in EnergyConsumers:
      P_load += EnergyConsumers[name]['kW']
    print('Total EnergyConsumers P_load: ' + str(P_load))

    P_ren = 0.0
    for name in SolarPVs:
      P_ren += SolarPVs[name]['kW']
    print('Total SolarPVs P_ren: ' + str(P_ren))
    print('\n\nRESILIENCY APP OUTPUT\n---------------------')
    msg = 'Emergency' if emergencyState else 'Alert'
    print('\nSTATE: ' + msg + '\n')

    eff_c = 0.9
    eff_d = 0.9
    T = 0.25
    P_sub = 2.0*P_load

    if not emergencyState: # alert state
      P_batt_total = 0.0
      for name in Batteries:
        if Batteries[name]['SoC'] < 0.9:
          P_batt_c = (0.9 - Batteries[name]['SoC'])*Batteries[name]['ratedE'] / (eff_c * T)
          Batteries[name]['P_batt_c'] = P_batt_c = min(P_batt_c, Batteries[name]['ratedkW'])
          P_batt_total += P_batt_c
          print('Battery name: ' + name + ', P_batt_c: ' + str(P_batt_c) + ', ratedkW: ' + str(Batteries[name]['ratedkW']))

      if P_batt_total > 0.0:
        if P_ren > P_load:
          if P_ren - P_load >= P_batt_total:
            # print('Charging from renewables')
            # YES, Charge ESS
            self.charge_batteries(Batteries, eff_c, T)

          else:
            # NO, Check P_sub
            if P_ren + P_sub > P_load:
              # print('P_ren<P_load Charging from renewable + substation')
              self.charge_batteries(Batteries, eff_c, T)

        else:
          # Check P_sub
          if P_ren + P_sub > P_load:
            # print('P_ren+P_sub>P_load Charging from renewable + substation')
            self.charge_batteries(Batteries, eff_c, T)

    else: # emergency state
      P_batt_total = 0.0
      # Shiva HACK
      P_sub = 0.0
      for name in Batteries:
        if Batteries[name]['SoC'] > 0.2:
          P_batt_d = (Batteries[name]['SoC'] - 0.2)*Batteries[name]['ratedE'] / (eff_d * T)
          Batteries[name]['P_batt_d'] = P_batt_d = min(P_batt_d, Batteries[name]['ratedkW'])
          print('Battery name: ' + name + ', P_batt_d: ' + str(P_batt_d) + ', ratedkW: ' + str(Batteries[name]['ratedkW']))
          P_batt_total += P_batt_d

      if P_batt_total > 0.0:
        self.discharge_batteries(Batteries, eff_d, T)

      if P_batt_total + P_ren + P_sub <= P_load:
        P_def = P_load - (P_batt_total + P_ren + P_sub)
        print('Power deficiency: ' + str(P_def))

        P_sync_available = 0.0
        for name, sync in SynchronousMachines.items():
          #avail = math.sqrt(sync['ratedS']**2 - (sync['kW']**2 + sync['kVar']**2))
          sync['P_available'] = math.sqrt(sync['ratedS']**2 - sync['kVar']**2)
          P_sync_available += sync['P_available']
        print('SynchronousMachines available power: ' + str(P_sync_available))

        if P_sync_available > P_def:
          for name, sync in SynchronousMachines.items():
            P_dis = P_def*sync['P_available']/P_sync_available
            print('SynchronousMachine name: ' + name + ', P_dis: ' + str(P_dis))
        else:
          for name, sync in SynchronousMachines.items():
            P_dis = sync['P_available']
            print('SynchronousMachine name: ' + name + ', P_dis: ' + str(P_dis))

    for name in Batteries:
      print('Updated SoC for Battery: ' + name + ', SoC: ' + str(Batteries[name]['SoC']))

    return


  def __init__(self, gapps, feeder_mrid, simulation_id, state):
    SPARQLManager = getattr(importlib.import_module('shared.sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    # Competing Apps Start

    emergencyState = state.startswith('e') or state.startswith('E')

    Loadshape = {}
    Solar = {}
    Price = {}

    with open('time-series-data.csv', 'r') as f:
      reader = csv.reader(f)
      next(reader) # skip header

      for row in reader:
        time = int(row[0])
        Loadshape[time] = float(row[1])
        Solar[time] = float(row[2])
        Price[time] = float(row[3])
        #print('time series time: ' + str(time) + ', Loadshape: ' + str(Loadshape[time]) + ', Solar: ' + str(Solar[time]) + ', Price: ' + str(Price[time]))

    # select random time between 40-60
    # for now, just take time=50
    t = 50 # 50,0.4159,0.814572,0.28
    pv_mult = Solar[t]
    load_mult = Loadshape[t]

    EnergyConsumers = {}
    objs = sparql_mgr.obj_dict_export('EnergyConsumer')
    print('Count of EnergyConsumers Dict: ' + str(len(objs)))
    for item in objs:
      name = item['IdentifiedObject.name']
      EnergyConsumers[name] = {}
      EnergyConsumers[name]['kW'] = load_mult*float(item['EnergyConsumer.p'])/1000.0
      # Shiva HACK to force battery charging...
      #EnergyConsumers[name]['kW'] = 0.01*float(item['EnergyConsumer.p'])/1000.0
      EnergyConsumers[name]['kVar'] = load_mult*float(item['EnergyConsumer.q'])/1000.0
      #print('EnergyConsumer name: ' + name + ', kW: ' + str(EnergyConsumers[name]['kW']) + ', kVar: ' + str(EnergyConsumers[name]['kVar']))

    #objs = sparql_mgr.obj_meas_export('EnergyConsumer')
    #print('Count of EnergyConsumers Meas: ' + str(len(objs)))
    #for item in objs:
    #  print('EnergyConsumer: ' + str(item))

    #objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    #print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)))
    #for item in objs:
    #  print('PowerElectronicsConnection: ' + str(item))

    #objs = sparql_mgr.obj_dict_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Dict: ' + str(len(objs)))
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item))

    #objs = sparql_mgr.obj_meas_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Meas: ' + str(len(objs)))
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item))

    SynchronousMachines = {}
    objs = sparql_mgr.obj_dict_export('SynchronousMachine')
    print('Count of SynchronousMachines Dict: ' + str(len(objs)))
    for item in objs:
      name = item['IdentifiedObject.name']
      SynchronousMachines[name] = {}
      SynchronousMachines[name]['kW'] = float(item['SynchronousMachine.p'])/1000.0
      SynchronousMachines[name]['kVar'] = float(item['SynchronousMachine.q'])/1000.0
      SynchronousMachines[name]['ratedS'] = float(item['SynchronousMachine.ratedS'])/1000.0
      print('SynchronousMachine name: ' + name + ', kW: ' + str(SynchronousMachines[name]['kW']) + ', kVar: ' + str(SynchronousMachines[name]['kVar']))

    objs = sparql_mgr.obj_meas_export('SynchronousMachine')
    #print('Count of SynchronousMachines Meas: ' + str(len(objs)))
    #for item in objs:
    #  print('SynchronousMachine: ' + str(item))

    Batteries = {}
    bindings = sparql_mgr.battery_query()
    print('Count of Batteries: ' + str(len(bindings)))
    for obj in bindings:
      name = obj['name']['value']
      #bus = obj['bus']['value'].upper()
      Batteries[name] = {}
      Batteries[name]['ratedkW'] = float(obj['ratedS']['value'])/1000.0
      Batteries[name]['ratedE'] = float(obj['ratedE']['value'])/1000.0
      # Shiva HACK
      #Batteries[name]['SoC'] = 0.25
      Batteries[name]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
      print('Battery name: ' + name + ', ratedE: ' + str(Batteries[name]['ratedE']) + ', SoC: ' + str(Batteries[name]['SoC']))

    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    print('Count of SolarPV: ' + str(len(bindings)))
    for obj in bindings:
      name = obj['name']['value']
      #bus = obj['bus']['value'].upper()
      #ratedS = float(obj['ratedS']['value'])
      #ratedU = float(obj['ratedU']['value'])
      SolarPVs[name] = {}
      SolarPVs[name]['kW'] = pv_mult*float(obj['p']['value'])/1000.0
      SolarPVs[name]['kVar'] = float(obj['q']['value'])/1000.0
      #print('SolarPV name: ' + name + ', kW: ' + str(SolarPVs[name]['kW']) + ', kVar: ' + str(SolarPVs[name]['kVar']))

    self.resiliency(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, emergencyState)

    '''
    bindings = sparql_mgr.regulator_query()
    print('Count of Regulators: ' + str(len(bindings)))
    for obj in bindings:
      rname = obj['rname']['value']
      pname = obj['pname']['value']
      if 'phs' in obj:
        phs = obj['phs']['value']
      else:
        phs = 'ABC'
      step = int(obj['step']['value'])
      #print('Regulator rname: ' + rname + ', pname: ' + pname + ', phs: ' + phs + ', step: ' + str(step))

    objs = sparql_mgr.obj_meas_export('PowerTransformer')
    print('Count of PowerTransformer Meas: ' + str(len(objs)))
    #for item in objs:
    #  if item['type'] == 'Pos':
    #    print('PowerTransformer: ' + str(item))
    '''

    sys.exit(0)
    # Competing Apps Finish

    SwitchMridToNodes,TransformerMridToNodes,TransformerLastPos,CapacitorMridToNode,CapacitorMridToYbusContrib,CapacitorLastValue = nodes_to_update(sparql_mgr)

    # Hold here for demo
    #text = input('\nWait here...')

    # Get node to index mapping from OpenDSS
    NodeIndex, Ybus = self.opendss_ybus(sparql_mgr)

    gapps_sim = GridAPPSD()

    self.simRap = SimWrapper(gapps_sim, feeder_mrid, simulation_id, Ybus, NodeIndex, SwitchMridToNodes, TransformerMridToNodes, TransformerLastPos, CapacitorMridToNode, CapacitorMridToYbusContrib, CapacitorLastValue)

    # don't subscribe to handle snapshot requests until we have an initial
    # Ybus to provide from the SimWrapper class
    # TODO figure out if there is a GridAPPS-D compliant topic to use
    topic = 'goss.gridappsd.request.data.dynamic-ybus.' + simulation_id
    #topic = service_input_topic('gridappsd-dynamic-ybus', simulation_id)
    #topic = '/topic/goss.gridappsd.simulation.gridappsd-dynamic-ybus.' + simulation_id + '.input'
    #topic = 'goss.gridappsd.simulation.gridappsd-dynamic-ybus.' + simulation_id + '.input'
    req_id = gapps.subscribe(topic, self)

    out_id = gapps_sim.subscribe(simulation_output_topic(simulation_id), self.simRap)
    log_id = gapps_sim.subscribe(simulation_log_topic(simulation_id), self.simRap)

    print('Starting simulation monitoring loop...\n', flush=True)

    while self.simRap.keepLooping():
      #print('Sleeping...', flush=True)
      time.sleep(0.1)

    print('Finished simulation monitoring loop and Dynamic Ybus.\n', flush=True)

    gapps.unsubscribe(req_id)
    gapps_sim.unsubscribe(out_id)
    gapps_sim.unsubscribe(log_id)

    return


def _main():
  # for loading modules
  if (os.path.isdir('shared')):
    sys.path.append('.')
  elif (os.path.isdir('../shared')):
    sys.path.append('..')
  elif (os.path.isdir('app-deconfliction/competing-apps/shared')):
    sys.path.append('app-deconfliction/competing-apps')
  else:
    sys.path.append('/gridappsd/services/app-deconfliction/competing-apps')
   
  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("state", nargs="?", default="Alert", help="Alert or Emergency State")
  parser.add_argument("--api", action="store_true", help="Invoke static ybus as an API call rather than as a service")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]
  simulation_id = opts.simulation_id
  state = opts.state

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-dynamic-ybus-service'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(simulation_id)
  assert gapps.connected

  dynamic_ybus = DynamicYbus(gapps, feeder_mrid, simulation_id, state)


if __name__ == "__main__":
  _main()

