#!/usr/bin/env python3

# Copyright (c) 2023, Battelle Memorial Institute All rights reserved.
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

@author: Gary Black and Shiva Poudel
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
from pulp import *

from gridappsd import GridAPPSD

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime
from tabulate import tabulate

# 80 column ruler for continuation lines
#0000000011111111112222222222333333333344444444445555555555666666666677777777778
#2345678901234567890123456789012345678901234567890123456789012345678901234567890

class CompetingApp(GridAPPSD):

  def __init__(self, gapps, opt_type, feeder_mrid, simulation_id, state):
    self.gapps = gapps

    self.AppUtil = getattr(importlib.import_module('shared.apputil'), 'AppUtil')

    SPARQLManager = getattr(importlib.import_module('shared.sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    # Competing Apps Start

    emergencyState = state.startswith('e') or state.startswith('E')

    feeder_power = {'p': {'A': 0, 'B': 0, 'C': 0},
                    'q': {'A': 0, 'B': 0, 'C': 0}}
    EnergyConsumers = {}
    bindings = sparql_mgr.energyconsumer_query()
    for obj in bindings:
      bus = obj['bus']['value'].upper()
      if bus not in EnergyConsumers:
        EnergyConsumers[bus] = {}
        EnergyConsumers[bus]['kW'] = {}
        EnergyConsumers[bus]['kVar'] = {}

      phases = obj['phases']['value']
      if phases == '':
        pval = float(obj['p']['value']) / 3.0
        qval = float(obj['q']['value']) / 3.0
        EnergyConsumers[bus]['kW']['A'] = pval
        EnergyConsumers[bus]['kW']['B'] = pval
        EnergyConsumers[bus]['kW']['C'] = pval
        EnergyConsumers[bus]['kVar']['A'] = qval
        EnergyConsumers[bus]['kVar']['B'] = qval
        EnergyConsumers[bus]['kVar']['C'] = qval
        feeder_power['p']['A'] += pval
        feeder_power['p']['B'] += pval
        feeder_power['p']['C'] += pval
        feeder_power['q']['A'] += qval
        feeder_power['q']['B'] += qval
        feeder_power['q']['C'] += qval
      else:
        pval = float(obj['p']['value'])
        qval = float(obj['q']['value'])
        EnergyConsumers[bus]['kW'][phases] = pval
        EnergyConsumers[bus]['kVar'][phases] = qval
        feeder_power['p'][phases] += pval
        feeder_power['q'][phases] += qval

    #print('EnergyConsumers[65]: ' + str(EnergyConsumers['65']), flush=True)
    #print('EnergyConsumers[47]: ' + str(EnergyConsumers['47']), flush=True)
    #print('EnergyConsumers[99]: ' + str(EnergyConsumers['99']), flush=True)

    # objs = sparql_mgr.obj_meas_export('EnergyConsumer')
    # print('Count of EnergyConsumers Meas: ' + str(len(objs)), flush=True)
    # for item in objs:
    #   print('EnergyConsumer: ' + str(item), flush=True)

    # objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    # print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)),
    #       flush=True)
    # for item in objs:
    #   print('PowerElectronicsConnection: ' + str(item), flush=True)

    # objs = sparql_mgr.obj_dict_export('LinearShuntCompensator')
    # print('Count of LinearShuntCompensators Dict: ' + str(len(objs)),
    #       flush=True)
    # for item in objs:
    #   print('LinearShuntCompensator: ' + str(item), flush=True)

    # objs = sparql_mgr.obj_meas_export('LinearShuntCompensator')
    # print('Count of LinearShuntCompensators Meas: ' + str(len(objs)),
    #       flush=True)
    # for item in objs:
    #   print('LinearShuntCompensator: ' + str(item), flush=True)

    SynchronousMachines = self.AppUtil.getSynchronousMachines(sparql_mgr)

    Batteries = self.AppUtil.getBatteries(sparql_mgr)

    # SHIVA HACK for 123 model testing
    Batteries['BatteryUnit:65'] = {'idx': 0, 'prated': 250000, 'phase': 'A',
                       'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.35}
    Batteries['BatteryUnit:52'] = {'idx': 1, 'prated': 250000, 'phase': 'B',
                       'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.275}
    Batteries['BatteryUnit:76'] = {'idx': 2, 'prated': 250000, 'phase': 'C',
                       'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.465}

    SolarPVs = self.AppUtil.getSolarPVs(sparql_mgr)

    # SHIVA HACK for 123 model testing
    SolarPVs['65'] = {'p': 120000, 'phase': 'A'}
    SolarPVs['52'] = {'p': 100000, 'phase': 'B'}
    SolarPVs['76'] = {'p': 165000, 'phase': 'C'}

    vnom = sparql_mgr.vnom_export()

    print('Processing Vnom...', flush=True)

    bus_info = {}
    idx = 0
    p_total = {'A': 0, 'B': 0, 'C': 0}
    for obj in vnom:
      #print(obj)
      phases = []

      items = obj.split(',')
      if items[0] == 'Bus':  # skip header line
        continue

      bus = items[0].strip('"')
      node1 = items[2].strip()
      phases.append(node1)

      node2 = items[6].strip()
      if node2 != '0':
        phases.append(node2)
        node3 = items[10].strip()
        if node3 != '0':
          phases.append(node3)

      bus_info[bus] = {}
      bus_info[bus]['idx'] = idx
      bus_info[bus]['phases'] = phases

      idx += 1

    print('Vnom Processed', flush=True)
    #print('bus_info[65]: ' + str(bus_info['65']), flush=True)
    #print('bus_info[47]: ' + str(bus_info['47']), flush=True)
    #print('bus_info[150]: ' + str(bus_info['150']), flush=True)

    ysparse, nodelist = sparql_mgr.ybus_export()

    print('Processing Ybus...', flush=True)

    node_name = {}
    for idx, obj in enumerate(nodelist):
      node_name[obj.strip('\"')] = idx

    num_nodes = len(node_name)
    ybus = np.zeros((num_nodes, num_nodes), dtype=complex)

    for obj in ysparse:
      items = obj.split(',')
      if items[0] == 'Row': # skip header
        continue
      ybus[int(items[0])-1][int(items[1])-1] = \
      ybus[int(items[1])-1][int(items[0])-1] = \
                            complex(float(items[2]), float(items[3]))

    branch_info = {}

    bindings = sparql_mgr.lines_connectivity_query()
    print('\nCount of ACLineSegments: ' + str(len(bindings)), flush=True)
    idx = 0
    for obj in bindings:
      name = obj['name']['value']
      bus1 = obj['bus1']['value'].upper()
      bus2 = obj['bus2']['value'].upper()
      phases = obj['phases']['value']
      #print('ACLineSegment name: ' + name + ', bus1: ' + bus1 +
      #      ', bus2: ' + bus2 + ', phases: ' + phases, flush=True)

      branch_info[name] = {}
      branch_info[name]['idx'] = idx
      branch_info[name]['phases'] = phases
      branch_info[name]['type'] = 'line'
      branch_info[name]['from_bus'] = bus1
      branch_info[name]['from_bus_idx'] = bus_info[bus1]['idx']
      branch_info[name]['to_bus'] = bus2
      branch_info[name]['to_bus_idx'] = bus_info[bus2]['idx']
      #print(name + ': ' + str(branch_info[name]))
      #print(obj)
      idx += 1

    bindings = sparql_mgr.regulator_combine_query()
    print('\nCount of Combine Regulators: ' + str(len(bindings)), flush=True)
    Regulators = {}
    RegIdx = {}
    reg_idx = 0
    for obj in bindings:
      pname = obj['pname']['value']
      if 'phs' in obj:
        phases = obj['phs']['value']
      else:
        phases = 'ABC'

      if 'tname' in obj:
        tname = obj['tname']['value']
        Regulators['RatioTapChanger:'+tname] = {'pname': pname, 'idx': reg_idx, 'phases': phases}
      else:
        Regulators['RatioTapChanger:'+pname] = {'pname': pname, 'idx': reg_idx, 'phases': phases}

      for char in phases:
        RegIdx[pname+'.'+char] = reg_idx

      reg_idx += 1

    print('Regulators: ' + str(Regulators), flush=True)
    print('RegIdx: ' + str(RegIdx), flush=True)

    bindings = sparql_mgr.power_transformer_connectivity_query()
    print('\nCount of PowerTransformers: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['xfmr_name']['value']
      bus = obj['bus']['value'].upper()
      print('PowerTransformer name: ' + name + ', bus: ' + bus, flush=True)
      #print(obj)

      if name not in branch_info:
        branch_info[name] = {}
        branch_info[name]['idx'] = idx
        branch_info[name]['phases'] = 'ABC'
        if 'RatioTapChanger:'+name in Regulators:
          branch_info[name]['type'] = 'regulator'
        else:
          branch_info[name]['type'] = 'transformer'
        branch_info[name]['from_bus'] = bus
        branch_info[name]['from_bus_idx'] = bus_info[bus]['idx']
      else:
        branch_info[name]['to_bus'] = bus
        branch_info[name]['to_bus_idx'] = bus_info[bus]['idx']
        print(name + ': ' + str(branch_info[name]))
        idx += 1

    bindings = sparql_mgr.tank_transformer_connectivity_query()
    print('\nCount of TankTransformers: ' + str(len(bindings)), flush=True)
    for obj in bindings:
        name = obj['xfmr_name']['value']
        bus = obj['bus']['value'].upper()
        phase = obj['phase']['value']
        print('TankTransformer name: ' + name + ', bus: ' + bus + ', phase: ' +
              phase, flush=True)
        #print(obj)

        pname = Regulators['RatioTapChanger:'+name]['pname']
        if pname not in branch_info:
          branch_info[pname] = {}
          branch_info[pname]['idx'] = idx
          branch_info[pname]['phases'] = phase
          branch_info[pname]['type'] = 'regulator'
          branch_info[pname]['from_bus'] = bus
          branch_info[pname]['from_bus_idx'] = bus_info[bus]['idx']
          idx += 1
        elif bus != branch_info[pname]['from_bus']:
          if phase not in branch_info[pname]['phases']:
            branch_info[pname]['phases'] += phase
          branch_info[pname]['to_bus'] = bus
          branch_info[pname]['to_bus_idx'] = bus_info[bus]['idx']
          print(pname + ': ' + str(branch_info[pname]))

    bindings = sparql_mgr.switch_connectivity_query()
    print('\nCount of Switches: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['name']['value']
      isopen = obj['open']['value'].upper()
      bus1 = obj['bus1']['value'].upper()
      bus2 = obj['bus2']['value'].upper()
      phases = obj['phases']['value']
      if phases == '':
        phases = 'ABC'
      print('Switch name: ' + name + ', open: ' + isopen + ', bus1: ' + bus1 +
            ', bus2: ' + bus2 + ', phases: ' + phases, flush=True)

      if isopen == 'FALSE':
        branch_info[name] = {}
        branch_info[name]['idx'] = idx
        branch_info[name]['phases'] = phases
        branch_info[name]['type'] = 'line'
        branch_info[name]['from_bus'] = bus1
        branch_info[name]['from_bus_idx'] = bus_info[bus1]['idx']
        branch_info[name]['to_bus'] = bus2
        branch_info[name]['to_bus_idx'] = bus_info[bus2]['idx']
        print(name + ': ' + str(branch_info[name]))
        #print(obj)
        idx += 1

    # setup two dictionaries for quick lookup of incident line and
    # outgoing lines for any bus index
    lines_in = {}
    lines_out = {}
    n_line_phase = {}
    for branch in branch_info:
      if branch_info[branch]['to_bus_idx'] not in lines_in:
        lines_in[branch_info[branch]['to_bus_idx']] = \
                                     {'A': [], 'B': [], 'C': []}
      if branch_info[branch]['from_bus_idx'] not in lines_out:
        lines_out[branch_info[branch]['from_bus_idx']] = \
                                     {'A': [], 'B': [], 'C': []}

      phases = branch_info[branch]['phases']
      for char in phases:
        lines_in[branch_info[branch]['to_bus_idx']][char].append(
                                                     branch_info[branch]['idx'])
        lines_out[branch_info[branch]['from_bus_idx']][char].append(
                                                     branch_info[branch]['idx'])
        if char not in n_line_phase:
          n_line_phase[char] = 0
        n_line_phase[char] += 1

      if branch_info[branch]['type'] == 'line':
        fr_bus = branch_info[branch]['from_bus']
        to_bus = branch_info[branch]['to_bus']
        fr_nodes = []
        to_nodes = []
        if 'A' in phases:
          fr_nodes.append(node_name[fr_bus+'.1'])
          to_nodes.append(node_name[to_bus+'.1'])
        if 'B' in phases:
          fr_nodes.append(node_name[fr_bus+'.2'])
          to_nodes.append(node_name[to_bus+'.2'])
        if 'C' in phases:
          fr_nodes.append(node_name[fr_bus+'.3'])
          to_nodes.append(node_name[to_bus+'.3'])

        branch_info[branch]['zprim'] = -1 * \
                            np.linalg.inv(ybus[np.ix_(fr_nodes, to_nodes)])

        #print('added line branch_info for: ' + branch + ', zprim: ' +
        #      str(branch_info[branch]['zprim']), flush=True)
      else:
        branch_info[branch]['zprim'] = np.zeros((3, 3), dtype=complex)
        #print('added non-line branch_info for: ' + branch + ', zprim: empty',
        #      flush=True)

    print('\nbranch_info phase count: ' + str(n_line_phase), flush=True)

    deltaT = 0.25

    # Optimization problem formulation
    # decision variables
    flow_max, flow_min = 5e6, -5e6
    p_flow_A = LpVariable.dicts("p_flow_A",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    p_flow_B = LpVariable.dicts("p_flow_B",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    p_flow_C = LpVariable.dicts("p_flow_C",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')

    q_flow_A = LpVariable.dicts("q_flow_A",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    q_flow_B = LpVariable.dicts("q_flow_B",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    q_flow_C = LpVariable.dicts("q_flow_C",(i for i in range(len(branch_info))),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')

    p_rated = 250e3
    p_batt = LpVariable.dicts("p_batt", (i for i in range(len(Batteries))),
                        lowBound=-p_rated, upBound=p_rated, cat='Continuous')
    p_batt_c = LpVariable.dicts("p_batt_c", (i for i in range(len(Batteries))),
                        lowBound=-p_rated, upBound=p_rated, cat='Continuous')
    p_batt_d = LpVariable.dicts("p_batt_d", (i for i in range(len(Batteries))),
                        lowBound=-p_rated, upBound=p_rated, cat='Continuous')
    soc = LpVariable.dicts("soc", (i for i in range(len(Batteries))),
                        lowBound=0.2, upBound=0.9, cat='Continuous')
    lambda_c = LpVariable.dicts("lambda_c", (i for i in range(len(Batteries))),
                        lowBound=0, upBound=1, cat='Binary')
    lambda_d = LpVariable.dicts("lambda_d", (i for i in range(len(Batteries))),
                        lowBound=0, upBound=1, cat='Binary')

    v_max, v_min = (1.05 * 2401.77) ** 2, (0.95 * 2401.77) ** 2
    # v_max, v_min = 1e9, 1e9
    v_A = LpVariable.dicts("v_A", (i for i in range(len(bus_info))),
                     lowBound=v_min, upBound=v_max, cat='Continuous')
    v_B = LpVariable.dicts("v_B", (i for i in range(len(bus_info))),
                     lowBound=v_min, upBound=v_max, cat='Continuous')
    v_C = LpVariable.dicts("v_C", (i for i in range(len(bus_info))),
                     lowBound=v_min, upBound=v_max, cat='Continuous')

    reg_taps = LpVariable.dicts("reg_tap", [(i, tap) for i in
                          range(len(Regulators)) for tap in range(32)],
                          lowBound=0, upBound=1, cat='Binary')
    b_i = np.arange(0.9, 1.1, 0.00625)

    # define base/static optimization problem that doesn't change with the
    # time-series multiplier values

    # objective
    if opt_type.startswith('d') or opt_type.startswith('D'):
      # Decarbonization
      baseProb = LpProblem("Min_Sub_Flow", LpMinimize)
      baseProb += p_flow_A[118] + p_flow_B[118] + p_flow_C[118]
    elif opt_type.startswith('r') or opt_type.startswith('R'):
      # Resilience
      baseProb = LpProblem("Max_Reserve", LpMinimize)
      baseProb += lpSum(-soc[i] for i in range(len(Batteries)))
    else:
      print('*** Exiting due to unrecognized optimization type: ' + opt_type,
            flush=True)
      exit()

    for branch in branch_info:
      # if branch == 'reg1a':
      if branch_info[branch]['type'] == 'regulator':
        M = 1e9
        if 'A' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.A']

          for k in range(32):
            baseProb += v_A[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_A[branch_info[branch]['from_bus_idx']] - \
                    M * (1 - reg_taps[(reg_idx, k)]) <= 0
            baseProb += v_A[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_A[branch_info[branch]['from_bus_idx']] + \
                    M * (1 - reg_taps[(reg_idx, k)]) >= 0

        if 'B' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.B']

          for k in range(32):
            baseProb += v_B[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_B[branch_info[branch]['from_bus_idx']] - \
                    M * (1 - reg_taps[(reg_idx, k)]) <= 0
            baseProb += v_B[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_B[branch_info[branch]['from_bus_idx']] + \
                    M * (1 - reg_taps[(reg_idx, k)]) >= 0

        if 'C' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.C']

          for k in range(32):
            baseProb += v_C[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_C[branch_info[branch]['from_bus_idx']] - \
                    M * (1 - reg_taps[(reg_idx, k)]) <= 0
            baseProb += v_C[branch_info[branch]['to_bus_idx']] - \
                    b_i[k] ** 2 * v_C[branch_info[branch]['from_bus_idx']] + \
                    M * (1 - reg_taps[(reg_idx, k)]) >= 0

      else:
        zprim = branch_info[branch]['zprim']
        phases = branch_info[branch]['phases']
        z_aa = z_bb = z_cc = z_ab = z_ac = z_bc = complex(0.0, 0.0)

        if zprim.size == 1:
          if phases == 'A':
            z_aa = zprim[0,0]
          elif phases == 'B':
            z_bb = zprim[0,0]
          elif phases == 'C':
            z_cc = zprim[0,0]
          else:
            print('*** Unrecognized single phase for branch: ' + branch +
                  ', phase: ' + phases, flush=True)

        elif zprim.size == 4:
          if 'A' in phases and 'B' in phases:
            z_aa = zprim[0,0]
            z_bb = zprim[1,1]
            z_ab = zprim[0,1]
          elif 'A' in phases and 'C' in phases:
            z_aa = zprim[0,0]
            z_cc = zprim[1,1]
            z_ac = zprim[0,1]
          elif 'B' in phases and 'C' in phases:
            z_bb = zprim[0,0]
            z_cc = zprim[1,1]
            z_bc = zprim[0,1]
          else:
            print('*** Unrecognized two phases for branch: ' + branch + ', phases: ' + phases, flush=True)

        elif zprim.size == 9:
          z_aa = zprim[0,0]
          z_bb = zprim[1,1]
          z_cc = zprim[2,2]
          z_ab = zprim[0,1]
          z_ac = zprim[0,2]
          z_bc = zprim[1,2]

        else:
          print('*** Unrecognized zprim size for branch: ' + branch + ', size: ' + str(zprim.size), flush=True)

        fr_bus_idx = branch_info[branch]['from_bus_idx']
        to_bus_idx = branch_info[branch]['to_bus_idx']
        idx = branch_info[branch]['idx']
        hfsqrt3 = math.sqrt(3.0)/2.0

        baseProb += v_A[to_bus_idx] == v_A[fr_bus_idx] - \
                2.0*(p_flow_A[idx]*z_aa.real + q_flow_A[idx]*z_aa.imag + \
                p_flow_B[idx]*(-0.5*z_ab.real + hfsqrt3*z_ab.imag) + \
                q_flow_B[idx]*(-0.5*z_ab.imag - hfsqrt3*z_ab.real) + \
                p_flow_C[idx]*(-0.5*z_ac.real - hfsqrt3*z_ac.imag) + \
                q_flow_C[idx]*(-0.5*z_ac.imag + hfsqrt3*z_ac.real))

        baseProb += v_B[to_bus_idx] == v_B[fr_bus_idx] - \
                2.0*(p_flow_B[idx]*z_bb.real + q_flow_B[idx]*z_bb.imag + \
                p_flow_A[idx]*(-0.5*z_ab.real - hfsqrt3*z_ab.imag) + \
                q_flow_A[idx]*(-0.5*z_ab.imag + hfsqrt3*z_ab.real) + \
                p_flow_C[idx]*(-0.5*z_bc.real + hfsqrt3*z_bc.imag) + \
                q_flow_C[idx]*(-0.5*z_bc.imag - hfsqrt3*z_bc.real))

        baseProb += v_C[to_bus_idx] == v_C[fr_bus_idx] - \
                2.0*(p_flow_C[idx]*z_cc.real + q_flow_C[idx]*z_cc.imag + \
                p_flow_A[idx]*(-0.5*z_ac.real + hfsqrt3*z_ac.imag) + \
                q_flow_A[idx]*(-0.5*z_ac.imag - hfsqrt3*z_ac.real) + \
                p_flow_B[idx]*(-0.5*z_bc.real - hfsqrt3*z_bc.imag) + \
                q_flow_B[idx]*(-0.5*z_bc.imag + hfsqrt3*z_bc.real))

    # Make source bus to 1.0
    baseProb += v_A[bus_info['150']['idx']] == 2401.77 ** 2
    baseProb += v_B[bus_info['150']['idx']] == 2401.77 ** 2
    baseProb += v_C[bus_info['150']['idx']] == 2401.77 ** 2

    for k in range(len(Regulators)):
      baseProb += lpSum(reg_taps[(k, tap)] for tap in range(32)) == 1
    # done defining base problem

    with open('../sim-starter/time-series.csv', 'r') as f:
      reader = csv.reader(f)
      next(reader)  # skip header

      for row in reader:
        time = int(row[0])
        #load_mult = 0.1728
        #pv_mult = 0.951665
        load_mult = float(row[1])
        pv_mult = float(row[2])
        price = float(row[3])

        print('Timestamp: ' + str(time), end='', flush=True)

        # copy the base LpProblem that doesn't depend on time-series data
        # as a starting point to then add the time-series dependent part on
        prob = LpProblem.deepcopy(baseProb)

        # note that deepcopy assigns references for LpVariables so if any of
        # these are changed with the time-series data then I should google
        # python pulp deepcopy and look at the stackoverflow link for that
    
        # constraints
        for bus in bus_info:
        #for bus in ['106']:
          bus_idx = bus_info[bus]['idx']
    
          #if bus_idx not in lines_in:
          #  print('Source bus: ' + bus, flush=True)
    
          if bus_idx not in lines_out:
            lines_out[bus_idx] = {'A': [], 'B': [], 'C': []}
    
          batname = 'BatteryUnit:' + bus
          if bus_idx in lines_in: # check for source bus
            if '1' in bus_info[bus]['phases']:
              injection_p, injection_q = 0, 0
              if bus in EnergyConsumers and 'A' in EnergyConsumers[bus]['kW']:
                injection_p = load_mult*EnergyConsumers[bus]['kW']['A']
                injection_q = load_mult*EnergyConsumers[bus]['kVar']['A']
    
              if bus in SolarPVs and 'A' in SolarPVs[bus]['phase']:
                injection_p -= pv_mult*SolarPVs[bus]['p']
                #print('SolarPVs A bus: ' + bus + ', value: ' +
                #      str(pv_mult*SolarPVs[bus]['p']), flush=True)
    
              if batname in Batteries and 'A' in Batteries[batname]['phase']:
                #print('Batteries A bus: ' + bus, flush=True)
                prob += lpSum(p_flow_A[idx] for idx in lines_in[bus_idx]['A'])-\
                        p_batt[Batteries[batname]['idx']] - injection_p == \
                        lpSum(p_flow_A[idx] for idx in lines_out[bus_idx]['A'])
                prob += lpSum(q_flow_A[idx] for idx in lines_in[bus_idx]['A'])-\
                        injection_q == lpSum(q_flow_A[idx] for idx in \
                        lines_out[bus_idx]['A'])
              else:
                prob += lpSum(p_flow_A[idx] for idx in lines_in[bus_idx]['A'])-\
                        injection_p == lpSum(p_flow_A[idx] for idx in \
                        lines_out[bus_idx]['A'])
                prob += lpSum(q_flow_A[idx] for idx in lines_in[bus_idx]['A'])-\
                        injection_q == lpSum(q_flow_A[idx] for idx in \
                        lines_out[bus_idx]['A'])
    
            if '2' in bus_info[bus]['phases']:
              injection_p, injection_q = 0, 0
              if bus in EnergyConsumers and 'B' in EnergyConsumers[bus]['kW']:
                injection_p = load_mult*EnergyConsumers[bus]['kW']['B']
                injection_q = load_mult*EnergyConsumers[bus]['kVar']['B']
    
              if bus in SolarPVs and 'B' in SolarPVs[bus]['phase']:
                injection_p -= pv_mult*SolarPVs[bus]['p']
                #print('SolarPVs B bus: ' + bus + ', value: ' +
                #      str(pv_mult*SolarPVs[bus]['p']), flush=True)

              if batname in Batteries and 'B' in Batteries[batname]['phase']:
                #print('Batteries B bus: ' + bus, flush=True)
                prob += lpSum(p_flow_B[idx] for idx in lines_in[bus_idx]['B'])-\
                        p_batt[Batteries[batname]['idx']] - injection_p == \
                        lpSum(p_flow_B[idx] for idx in lines_out[bus_idx]['B'])
                prob += lpSum(q_flow_B[idx] for idx in lines_in[bus_idx]['B'])-\
                        injection_q == lpSum(q_flow_B[idx] for idx in \
                        lines_out[bus_idx]['B'])
              else:
                prob += lpSum(p_flow_B[idx] for idx in lines_in[bus_idx]['B'])-\
                        injection_p == lpSum(p_flow_B[idx] for idx in \
                        lines_out[bus_idx]['B'])
                prob += lpSum(q_flow_B[idx] for idx in lines_in[bus_idx]['B'])-\
                        injection_q == lpSum(q_flow_B[idx] for idx in \
                        lines_out[bus_idx]['B'])
    
            if '3' in bus_info[bus]['phases']:
              injection_p, injection_q = 0, 0
              if bus in EnergyConsumers and 'C' in EnergyConsumers[bus]['kW']:
                injection_p = load_mult*EnergyConsumers[bus]['kW']['C']
                injection_q = load_mult*EnergyConsumers[bus]['kVar']['C']
    
              if bus in SolarPVs and 'C' in SolarPVs[bus]['phase']:
                injection_p -= pv_mult*SolarPVs[bus]['p']
                #print('SolarPVs C bus: ' + bus + ', value: ' +
                #      str(pv_mult*SolarPVs[bus]['p']), flush=True)
    
              if batname in Batteries and 'C' in Batteries[batname]['phase']:
                #print('Batteries C bus: ' + bus, flush=True)
                prob += lpSum(p_flow_C[idx] for idx in lines_in[bus_idx]['C'])-\
                        p_batt[Batteries[batname]['idx']] - injection_p == \
                        lpSum(p_flow_C[idx] for idx in lines_out[bus_idx]['C'])
                prob += lpSum(q_flow_C[idx] for idx in lines_in[bus_idx]['C'])-\
                        injection_q == lpSum(q_flow_C[idx] for idx in \
                        lines_out[bus_idx]['C'])
              else:
                prob += lpSum(p_flow_C[idx] for idx in lines_in[bus_idx]['C'])-\
                        injection_p == lpSum(p_flow_C[idx] for idx in \
                        lines_out[bus_idx]['C'])
                prob += lpSum(q_flow_C[idx] for idx in lines_in[bus_idx]['C'])-\
                        injection_q == lpSum(q_flow_C[idx] for idx in \
                        lines_out[bus_idx]['C'])

        for name in Batteries:
          Batteries[name]['state'] = 'idling'
          idx = Batteries[name]['idx']
          prob += soc[idx] == Batteries[name]['SoC'] + Batteries[name]['eff'] *\
                  p_batt_c[idx] * deltaT / Batteries[name]['ratedE'] + \
                  1 / Batteries[name]['eff'] * p_batt_d[idx] * \
                  deltaT / Batteries[name]['ratedE']
          prob += p_batt_c[idx] >= 0
          prob += p_batt_d[idx] <= 0
          prob += p_batt_c[idx] <= lambda_c[idx] * p_rated
          prob += p_batt_d[idx] >= - lambda_d[idx] * p_rated
          prob += p_batt[idx] == p_batt_c[idx] + p_batt_d[idx]
          prob += lambda_c[idx] + lambda_d[idx] <= 1


        # solve
        prob.solve(PULP_CBC_CMD(msg=0))
        prob.writeLP('Resilience.lp')
        print(', status:', LpStatus[prob.status], flush=True)
        #for idx in range(len(branch_info)):
        branch_flow = []
        for branch in branch_info:
          idx = branch_info[branch]['idx']
          branch_flow.append([branch, branch_info[branch]['from_bus'],
                      branch_info[branch]['to_bus'], p_flow_A[idx].varValue,
                      p_flow_B[idx].varValue, p_flow_C[idx].varValue,
                      q_flow_A[idx].varValue, q_flow_B[idx].varValue,
                      q_flow_C[idx].varValue])
    
        '''
        print(tabulate(branch_flow, headers=['Line Name', 'from', 'to',
                      'P_A', 'P_B', 'P_C', 'Q_A', 'Q_B', 'Q_C'], tablefmt='psql'))
    
        for idx in [118]:
          print('P Flow line ' + str(idx) + ', A:', p_flow_A[idx].varValue/1000,
                ', B:', p_flow_B[idx].varValue/1000,
                ', C:', p_flow_C[idx].varValue/1000, flush=True)
          print('Q Flow line ' + str(idx) + ', A:', q_flow_A[idx].varValue/1000,
                ', B:', q_flow_B[idx].varValue/1000,
                ', C:', q_flow_C[idx].varValue/1000, flush=True)
    
        print('Total Real Power ' + ', A:', feeder_power['p']['A']/1000,
              ', B:', feeder_power['p']['B']/1000,
              ', C:', feeder_power['p']['C']/1000, flush=True)
        print('Total Reactive Power ' + ', A:', feeder_power['q']['A']/1000,
              ', B:', feeder_power['q']['B']/1000,
              ', C:', feeder_power['q']['C']/1000, flush=True)
    
        bus_voltage = []
        v = []
        for bus in bus_info:
          idx = bus_info[bus]['idx']
          bus_voltage.append([bus, math.sqrt(v_A[idx].varValue),
                      math.sqrt(v_B[idx].varValue), math.sqrt(v_C[idx].varValue)])
          v.append(math.sqrt(v_A[idx].varValue) / 2401.77)
          v.append(math.sqrt(v_B[idx].varValue) / 2401.77)
          v.append(math.sqrt(v_C[idx].varValue) / 2401.77)
    
        print(tabulate(bus_voltage, headers=['Bus', 'V_A', 'V_B', 'V_C'],
                       tablefmt='psql'))
        '''
    
        p_batt_setpoint = []
        for name in Batteries:
          idx = Batteries[name]['idx']
          Batteries[name]['SoC'] = soc[idx].varValue
          p_batt_setpoint.append([name, p_batt[idx].varValue/1000.0,
                                  soc[idx].varValue])
    
        print(tabulate(p_batt_setpoint, headers=['Battery', 'P_batt',
                       'Target SoC'], tablefmt='psql'))
    
        regulator_taps = []
        for reg in Regulators:
          idx = Regulators[reg]['idx']
          for k in range(32):
            if reg_taps[(idx, k)].varValue >= 0.5:
              regulator_taps.append([reg, k-16, b_i[k]])
    
        print(tabulate(regulator_taps, headers=['Regulator', 'Tap', 'b_i'],
                       tablefmt='psql'), '\n')
    
    return


def _main():
  print('Starting app code...', flush=True)

  # for loading modules
  if (os.path.isdir('shared')):
    sys.path.append('.')
  elif (os.path.isdir('../shared')):
    sys.path.append('..')
  elif (os.path.isdir('app-deconfliction/competing-apps/shared')):
    sys.path.append('app-deconfliction/competing-apps')
  else:
    sys.path.append('/gridappsd/services/app-deconfliction/competing-apps')

  # sim_request = json.loads(opts.request.replace("\'", ""))
  #feeder_mrid = "_AAE94E4A-2465-6F5E-37B1-3E72183A4E44" # 9500
  #feeder_mrid = "_49AD8E07-3BF9-A4E2-CB8F-C3722F837B62" # 13
  #feeder_mrid = "_5B816B93-7A5F-B64C-8460-47C17D6E4B0F" # 13assets
  feeder_mrid = "_C1C3E687-6FFD-C753-582B-632A27E28507" # 123
  simulation_id = 0.0
  state = 'a'

  parser = argparse.ArgumentParser()
  parser.add_argument("type", help="Competing App Type")
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("state", nargs="?", default="Alert",
                      help="Alert or Emergency State")
  parser.add_argument("--outage", "--out", "-o", type=int, nargs=2)
  opts = parser.parse_args()

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(simulation_id)
  assert gapps.connected

  competing_app = CompetingApp(gapps, opts.type, feeder_mrid, simulation_id,
                               state)


if __name__ == "__main__":
  _main()

