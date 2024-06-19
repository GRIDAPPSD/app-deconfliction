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
import os
import argparse
import json
import importlib
import math
import pprint
import numpy as np
import csv
import queue

from time import sleep
from pulp import *

from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic

from datetime import datetime
from tabulate import tabulate

# suppress warnings about overriding optimization function from decarbonization
import warnings
warnings.simplefilter('ignore', UserWarning)

# find and add shared directory to path hopefully wherever it is from here
if (os.path.isdir('../shared')):
  sys.path.append('../shared')
elif (os.path.isdir('../competing-apps/shared')):
  sys.path.append('../competing-apps/shared')
elif (os.path.isdir('../../competing-apps/shared')):
  sys.path.append('../../competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')

from AppUtil import AppUtil
import MethodUtil

# 80 column ruler for continuation lines
#0000000011111111112222222222333333333344444444445555555555666666666677777777778
#2345678901234567890123456789012345678901234567890123456789012345678901234567890

class CompetingApp(GridAPPSD):

  def updateSoC(self, BatterySoC):
    for device, value in BatterySoC.items():
      self.Batteries[device]['SoC'] = value
      #print('Updated SoC for: ' + device + ' = ' + str(round(value, 4)),
      #      flush=True)


  def defineOptimizationDynamicProblem(self, timestamp, load_mult, pv_mult):
    # copy the base/static LpProblem that doesn't depend on time-series data
    # as a starting point to then add the time-series dependent part on
    self.dynamicProb = LpProblem.deepcopy(self.staticProb)

    # note that deepcopy assigns references for LpVariables so if any of
    # these are changed with the time-series data then I should google
    # python pulp deepcopy and look at the stackoverflow link for that

    # constraints
    for bus in self.bus_info:
      bus_idx = self.bus_info[bus]['idx']

      #if bus_idx not in self.lines_in:
      #  print('Source bus: ' + bus, flush=True)

      if bus_idx not in self.lines_out:
        self.lines_out[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in self.lines_in: # check for source bus
        if bus not in self.EnergyConsumers:
          self.dynamicProb += self.injection_p_load_A[bus_idx] == 0
          self.dynamicProb += self.injection_p_load_B[bus_idx] == 0
          self.dynamicProb += self.injection_p_load_C[bus_idx] == 0
          self.dynamicProb += self.injection_q_load_A[bus_idx] == 0
          self.dynamicProb += self.injection_q_load_B[bus_idx] == 0
          self.dynamicProb += self.injection_q_load_C[bus_idx] == 0

      if bus_idx in self.lines_in: # check for source bus
        if '1' in self.bus_info[bus]['phases']:
          #injection_p, injection_q = 0, 0
          injection_p_pv = 0
          if bus in self.EnergyConsumers and \
             'A' in self.EnergyConsumers[bus]['kW']:
            #injection_p = load_mult*self.EnergyConsumers[bus]['kW']['A']
            #injection_q = load_mult*self.EnergyConsumers[bus]['kVar']['A']
            self.dynamicProb += self.injection_p_load_A[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kW']['A']
            self.dynamicProb += self.injection_q_load_A[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kVar']['A']

          if bus in self.SolarPVs and 'A' in self.SolarPVs[bus]['phase']:
            #injection_p -= pv_mult*self.SolarPVs[bus]['p']
            injection_p_pv = -pv_mult*self.SolarPVs[bus]['p']
            #print('SolarPVs A bus: ' + bus + ', value: ' +
            #      str(pv_mult*self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'A' in self.Batteries_obj[bus]['phase']:
            #print('Batteries A bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicProb += lpSum(self.p_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - self.injection_p_load_A[bus_idx] - injection_p_pv == \
               lpSum(self.p_flow_A[idx] for idx in self.lines_out[bus_idx]['A'])

            self.dynamicProb += lpSum(self.q_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               self.injection_q_load_A[bus_idx] == lpSum(self.q_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A'])
          else:
            self.dynamicProb += lpSum(self.p_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               self.injection_p_load_A[bus_idx] - injection_p_pv == lpSum(self.p_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A'])

            self.dynamicProb += lpSum(self.q_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               self.injection_q_load_A[bus_idx] == lpSum(self.q_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A'])

        if '2' in self.bus_info[bus]['phases']:
          #injection_p, injection_q = 0, 0
          injection_p_pv = 0
          if bus in self.EnergyConsumers and \
             'B' in self.EnergyConsumers[bus]['kW']:
            #injection_p = load_mult*self.EnergyConsumers[bus]['kW']['B']
            #injection_q = load_mult*self.EnergyConsumers[bus]['kVar']['B']
            self.dynamicProb += self.injection_p_load_B[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kW']['B']
            self.dynamicProb += self.injection_q_load_B[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kVar']['B']

          if bus in self.SolarPVs and 'B' in self.SolarPVs[bus]['phase']:
            #injection_p -= pv_mult*self.SolarPVs[bus]['p']
            injection_p_pv = -pv_mult*self.SolarPVs[bus]['p']
            #print('SolarPVs B bus: ' + bus + ', value: ' +
            #      str(pv_mult*self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'B' in self.Batteries_obj[bus]['phase']:
            #print('Batteries B bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicProb += lpSum(self.p_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - self.injection_p_load_B[bus_idx] - injection_p_pv == \
               lpSum(self.p_flow_B[idx] for idx in self.lines_out[bus_idx]['B'])

            self.dynamicProb += lpSum(self.q_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               self.injection_q_load_B[bus_idx] == lpSum(self.q_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B'])
          else:
            self.dynamicProb += lpSum(self.p_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               self.injection_p_load_B[bus_idx] - injection_p_pv == lpSum(self.p_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B'])

            self.dynamicProb += lpSum(self.q_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               self.injection_q_load_B[bus_idx] == lpSum(self.q_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B'])

        if '3' in self.bus_info[bus]['phases']:
          #injection_p, injection_q = 0, 0
          injection_p_pv = 0
          if bus in self.EnergyConsumers and \
             'C' in self.EnergyConsumers[bus]['kW']:
            #injection_p = load_mult*self.EnergyConsumers[bus]['kW']['C']
            #injection_q = load_mult*self.EnergyConsumers[bus]['kVar']['C']
            self.dynamicProb += self.injection_p_load_C[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kW']['C']
            self.dynamicProb += self.injection_q_load_C[bus_idx] == self.alpha*load_mult*self.EnergyConsumers[bus]['kVar']['C']

          if bus in self.SolarPVs and 'C' in self.SolarPVs[bus]['phase']:
            #injection_p -= pv_mult*self.SolarPVs[bus]['p']
            injection_p_pv = -pv_mult*self.SolarPVs[bus]['p']
            #print('SolarPVs C bus: ' + bus + ', value: ' +
            #      str(pv_mult*self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'C' in self.Batteries_obj[bus]['phase']:
            #print('Batteries C bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicProb += lpSum(self.p_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - self.injection_p_load_C[bus_idx] - injection_p_pv == \
               lpSum(self.p_flow_C[idx] for idx in self.lines_out[bus_idx]['C'])

            self.dynamicProb += lpSum(self.q_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               self.injection_q_load_C[bus_idx] == lpSum(self.q_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C'])
          else:
            self.dynamicProb += lpSum(self.p_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               self.injection_p_load_C[bus_idx] - injection_p_pv == lpSum(self.p_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C'])

            self.dynamicProb += lpSum(self.q_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               self.injection_q_load_C[bus_idx] == lpSum(self.q_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C'])

    for mrid in self.Batteries:
      self.Batteries[mrid]['state'] = 'idling'
      idx = self.Batteries[mrid]['idx']
      self.dynamicProb += self.soc[idx] == self.Batteries[mrid]['SoC'] + \
              self.Batteries[mrid]['eff'] * self.p_batt_c[idx] * \
              self.deltaT / self.Batteries[mrid]['ratedE'] + \
              1 / self.Batteries[mrid]['eff'] * self.p_batt_d[idx] * \
              self.deltaT / self.Batteries[mrid]['ratedE']
      self.dynamicProb += self.p_batt_c[idx] >= 0
      self.dynamicProb += self.p_batt_d[idx] <= 0
      self.dynamicProb += self.p_batt_c[idx] <= \
              self.lambda_c[idx] * self.Batteries[mrid]['prated']
      self.dynamicProb += self.p_batt_d[idx] >= \
              -self.lambda_d[idx] * self.Batteries[mrid]['prated']
      self.dynamicProb += self.p_batt[idx] == \
              self.p_batt_c[idx] + self.p_batt_d[idx]
      self.dynamicProb += self.lambda_c[idx] + self.lambda_d[idx] <= 1


  def doOptimization(self, timestamp):
        data = self.dynamicProb.to_dict()
        opt_prob = {}
        opt_prob['utility_function'] = data['objective']
        opt_prob['constraints'] = data['constraints']
        opt_prob['variables'] = data['variables']
        opt_prob['parameters'] = data['parameters']
        opt_prob['sos1'] = data['sos1']
        opt_prob['sos2'] = data['sos2']

        # Dump optimization problem formulation to a file for Orestis'
        # methodology that combines them from multiple competing apps.
        # Eventually he'll get this from the message bus message, but
        # for now he reads from files
        json_opt = open('output/' + self.opt_type + '_' +
                                 str(timestamp) + '.json', 'w')
        json.dump(data, json_opt, indent=4)
        json_opt.close()

        # solve
        self.dynamicProb.solve(PULP_CBC_CMD(msg=0, gapRel=self.gapRel,
                               timeLimit=5))
        print('Optimization status:', LpStatus[self.dynamicProb.status],
              flush=True)

        objective = pulp.value(self.dynamicProb.objective)

        # Second stage for the decarbonization app
        if self.opt_type == 'decarbonization':
          bus_idx_batt = {'A': [], 'B': [], 'C': []}
          for mrid in self.Batteries:
            idx = self.Batteries[mrid]['idx']
            self.dynamicProb += self.p_batt[idx] == self.p_batt[idx].varValue
            bus = self.Batteries[mrid]['bus']
            if 'A' in self.Batteries[mrid]['phase']:
              bus_idx_batt['A'].append(self.bus_info[bus]['idx'])
            elif 'B' in self.Batteries[mrid]['phase']:
              bus_idx_batt['B'].append(self.bus_info[bus]['idx'])
            else:
              bus_idx_batt['C'].append(self.bus_info[bus]['idx'])

          self.dynamicProb += self.dynamicProb.objective-self.Psub_mod + \
                              lpSum(-self.v_A[i] for i in bus_idx_batt['A']) + \
                              lpSum(-self.v_B[i] for i in bus_idx_batt['B']) + \
                              lpSum(-self.v_C[i] for i in bus_idx_batt['C'])
          self.dynamicProb.solve(GLPK_CMD(msg=0, options=['--mipgap', '0.01']))
          print('Optimization Stage II:', LpStatus[self.dynamicProb.status],
                flush=True)

        # self.dynamicProb.writeLP('output/' + self.opt_type + '_' +
        #                          str(timestamp) + '.lp')

        '''
        branch_flow = []
        for branch in branch_info:
          idx = branch_info[branch]['idx']
          branch_flow.append([branch, branch_info[branch]['from_bus'],
                      branch_info[branch]['to_bus'], p_flow_A[idx].varValue,
                      p_flow_B[idx].varValue, p_flow_C[idx].varValue,
                      q_flow_A[idx].varValue, q_flow_B[idx].varValue,
                      q_flow_C[idx].varValue])

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

        regulator_taps = []
        set_points = {}
        for reg in self.Regulators:
          idx = self.Regulators[reg]['idx']
          for k in range(32):
            if self.reg_taps[(idx, k)].varValue >= 0.5:
              set_points[reg] = k-16
              regulator_taps.append([reg, k-16, self.b_i[k]])

        print(tabulate(regulator_taps, headers=['Regulator', 'Tap', 'b_i'],
                       tablefmt='psql'), '\n', flush=True)

        p_batt_setpoints = []
        for mrid in self.Batteries:
          idx = self.Batteries[mrid]['idx']
          self.Batteries[mrid]['SoC'] = self.soc[idx].varValue
          set_points[mrid] = self.p_batt[idx].varValue
          p_batt_setpoints.append([mrid, set_points[mrid]/1000,
                                   self.soc[idx].varValue])

        print(tabulate(p_batt_setpoints, headers=['Battery', 'P_batt (kW)',
                       'Target SoC'], tablefmt='psql'), flush=True)

        print('\nalpha: ' + str(self.alpha.varValue) + '\n', flush=True)

        out_message = {
          'app_name': self.opt_type+'-app',
          'timestamp': timestamp,
          'set_points': set_points
        }
          #'opt_prob': opt_prob,
          #'objective': objective
        #print('Sending message: ' + str(out_message), flush=True)
        self.gapps.send(self.publish_topic, out_message)


  def on_message(self, headers, message):
    #print('headers: ' + str(headers), flush=True)
    #print('message: ' + str(message), flush=True)
    self.messageQueue.put(message)


  def defineOptimizationVariables(self, len_branch_info, len_bus_info,
                                  len_Batteries, len_Regulators):
    flow_min, flow_max = -5e6, 5e6
    self.p_flow_A = LpVariable.dicts("p_flow_A",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    self.p_flow_B = LpVariable.dicts("p_flow_B",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    self.p_flow_C = LpVariable.dicts("p_flow_C",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')

    self.q_flow_A = LpVariable.dicts("q_flow_A",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    self.q_flow_B = LpVariable.dicts("q_flow_B",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')
    self.q_flow_C = LpVariable.dicts("q_flow_C",
                          (i for i in range(len_branch_info)),
                          lowBound=flow_min, upBound=flow_max, cat='Continuous')

    self.Psub = LpVariable("P_sub", lowBound=flow_min, upBound=flow_max, cat='Continuous')
    self.Psub_mod = LpVariable("P_sub_mod", lowBound=flow_min, upBound=flow_max, cat='Continuous')

    self.p_rated = 250e3
    self.p_batt = LpVariable.dicts("p_batt",
                        (i for i in range(len_Batteries)),
                        lowBound=-self.p_rated, upBound=self.p_rated,
                        cat='Continuous')
    self.p_batt_c = LpVariable.dicts("p_batt_c",
                        (i for i in range(len_Batteries)),
                        lowBound=-self.p_rated, upBound=self.p_rated,
                        cat='Continuous')
    self.p_batt_d = LpVariable.dicts("p_batt_d",
                        (i for i in range(len_Batteries)),
                        lowBound=-self.p_rated, upBound=self.p_rated,
                        cat='Continuous')
    self.soc = LpVariable.dicts("soc",
                        (i for i in range(len_Batteries)),
                        lowBound=0.2, upBound=0.9, cat='Continuous')
    self.lambda_c = LpVariable.dicts("lambda_c",
                        (i for i in range(len_Batteries)),
                        lowBound=0, upBound=1, cat='Binary')
    self.lambda_d = LpVariable.dicts("lambda_d",
                        (i for i in range(len_Batteries)),
                        lowBound=0, upBound=1, cat='Binary')

    v_min, v_max = (0.95 * 2401.77) ** 2, (1.05 * 2401.77) ** 2
    self.v_A = LpVariable.dicts("v_A", (i for i in range(len_bus_info)),
                          lowBound=v_min, upBound=v_max, cat='Continuous')
    self.v_B = LpVariable.dicts("v_B", (i for i in range(len_bus_info)),
                          lowBound=v_min, upBound=v_max, cat='Continuous')
    self.v_C = LpVariable.dicts("v_C", (i for i in range(len_bus_info)),
                          lowBound=v_min, upBound=v_max, cat='Continuous')

    self.reg_taps = LpVariable.dicts("reg_tap", [(i, tap) for i in
                          range(len_Regulators) for tap in range(32)],
                          lowBound=0, upBound=1, cat='Binary')

    self.alpha = LpVariable("alpha", lowBound=0, upBound=1, cat='Continuous')

    # SHIVA, any lower and upper bounds on these p and q load variables?
    self.injection_p_load_A = LpVariable.dicts("injection_p_load_A",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')
    self.injection_p_load_B = LpVariable.dicts("injection_p_load_B",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')
    self.injection_p_load_C = LpVariable.dicts("injection_p_load_C",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')

    self.injection_q_load_A = LpVariable.dicts("injection_q_load_A",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')
    self.injection_q_load_B = LpVariable.dicts("injection_q_load_B",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')
    self.injection_q_load_C = LpVariable.dicts("injection_q_load_C",
                                  (i for i in range(len_bus_info)),
                                  lowBound=0, cat='Continuous')


  def defineOptimizationStaticProblem(self, branch_info, RegIdx,
                                      len_Batteries, len_Regulators):
    # define base/static optimization problem that doesn't change with the
    # time-series multiplier values

    # objective
    if self.opt_type == 'decarbonization':
      self.staticProb = LpProblem("Min_Sub_Flow", LpMinimize)
      self.staticProb += self.Psub_mod
      self.staticProb += self.Psub_mod >= self.Psub
      self.staticProb += self.Psub_mod >= -self.Psub
      sub_flow_idx = self.EnergySource['flow_idx']
      self.staticProb += self.Psub == self.p_flow_A[sub_flow_idx] + \
                       self.p_flow_B[sub_flow_idx] + self.p_flow_C[sub_flow_idx]

    elif self.opt_type == 'resilience':
      self.staticProb = LpProblem("Max_Reserve", LpMinimize)
      # SHIVA magic scaling factor for SoC that causes the optmization to
      # come up with the correct results where -self.soc[i] doesn't.
      # Shiva will be investigating why this happens since we don't want
      # to be dependent on magic
      self.staticProb += lpSum(-100 * self.soc[i] for i in range(len_Batteries))

    elif self.opt_type == 'profit_cvr':
      self.staticProb = LpProblem("Min_Load_Demand", LpMinimize)
      self.staticProb += lpSum((self.v_A[i] + self.v_B[i] + self.v_C[i]) for i in range(len(self.bus_info)))

    elif self.opt_type == 'load_shedding':
      self.staticProb = LpProblem("Max_Load_Pickup", LpMinimize)
      self.staticProb += -self.alpha
      sub_flow_idx = self.EnergySource['flow_idx']
      self.staticProb += self.Psub == self.p_flow_A[sub_flow_idx] + \
                       self.p_flow_B[sub_flow_idx] + self.p_flow_C[sub_flow_idx]
      self.staticProb += self.Psub <= 500000 # USER DEFINED AVAILABILITY

    for branch in branch_info:
      if branch_info[branch]['type'] == 'regulator':
        M = 1e9
        if 'A' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.A']

          for k in range(32):
            self.staticProb += self.v_A[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[branch_info[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0
            self.staticProb += self.v_A[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[branch_info[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0

        if 'B' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.B']

          for k in range(32):
            self.staticProb += self.v_B[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[branch_info[branch]['from_bus_idx']]\
                    - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0
            self.staticProb += self.v_B[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[branch_info[branch]['from_bus_idx']]\
                    + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0

        if 'C' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.C']

          for k in range(32):
            self.staticProb += self.v_C[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[branch_info[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0
            self.staticProb += self.v_C[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[branch_info[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0

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
            print('*** Unrecognized two phases for branch: ' + branch +
                  ', phases: ' + phases, flush=True)

        elif zprim.size == 9:
          z_aa = zprim[0,0]
          z_bb = zprim[1,1]
          z_cc = zprim[2,2]
          z_ab = zprim[0,1]
          z_ac = zprim[0,2]
          z_bc = zprim[1,2]

        else:
          print('*** Unrecognized zprim size for branch: ' + branch +
                ', size: ' + str(zprim.size), flush=True)

        fr_bus_idx = branch_info[branch]['from_bus_idx']
        to_bus_idx = branch_info[branch]['to_bus_idx']
        idx = branch_info[branch]['idx']
        hfsqrt3 = math.sqrt(3.0)/2.0

        self.staticProb += self.v_A[to_bus_idx] == self.v_A[fr_bus_idx] - \
            2.0*(self.p_flow_A[idx]*z_aa.real + self.q_flow_A[idx]*z_aa.imag + \
            self.p_flow_B[idx]*(-0.5*z_ab.real + hfsqrt3*z_ab.imag) + \
            self.q_flow_B[idx]*(-0.5*z_ab.imag - hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_ac.real - hfsqrt3*z_ac.imag) + \
            self.q_flow_C[idx]*(-0.5*z_ac.imag + hfsqrt3*z_ac.real))

        self.staticProb += self.v_B[to_bus_idx] == self.v_B[fr_bus_idx] - \
            2.0*(self.p_flow_B[idx]*z_bb.real + self.q_flow_B[idx]*z_bb.imag + \
            self.p_flow_A[idx]*(-0.5*z_ab.real - hfsqrt3*z_ab.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ab.imag + hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_bc.real + hfsqrt3*z_bc.imag) + \
            self.q_flow_C[idx]*(-0.5*z_bc.imag - hfsqrt3*z_bc.real))

        self.staticProb += self.v_C[to_bus_idx] == self.v_C[fr_bus_idx] - \
            2.0*(self.p_flow_C[idx]*z_cc.real + self.q_flow_C[idx]*z_cc.imag + \
            self.p_flow_A[idx]*(-0.5*z_ac.real + hfsqrt3*z_ac.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ac.imag - hfsqrt3*z_ac.real) + \
            self.p_flow_B[idx]*(-0.5*z_bc.real - hfsqrt3*z_bc.imag) + \
            self.q_flow_B[idx]*(-0.5*z_bc.imag + hfsqrt3*z_bc.real))

    # fix source bus at 1.0
    sourcebus = self.EnergySource['bus']
    v_source = self.EnergySource['basev'] / math.sqrt(3)
    self.staticProb += self.v_A[self.bus_info[sourcebus]['idx']] == v_source ** 2
    self.staticProb += self.v_B[self.bus_info[sourcebus]['idx']] == v_source ** 2
    self.staticProb += self.v_C[self.bus_info[sourcebus]['idx']] == v_source ** 2

    for k in range(len_Regulators):
      self.staticProb += lpSum(self.reg_taps[(k, tap)] for tap in range(32))==1


  def __init__(self, gapps, opt_type, feeder_mrid, simulation_id, outage,state):
    self.gapps = gapps

    self.messageQueue = queue.Queue()

    # subscribe to simulation output messages
    # since messages are just going on a queue, subscribe right away to
    # keep from missing any sent during app initialization
    gapps.subscribe(service_output_topic('gridappsd-sim-sim',
                                         simulation_id), self)

    SPARQLManager = getattr(importlib.import_module('sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.outage = outage

    self.emergencyState = state.startswith('e') or state.startswith('E')

    #feeder_power = {'p': {'A': 0, 'B': 0, 'C': 0},
    #                'q': {'A': 0, 'B': 0, 'C': 0}}
    self.EnergyConsumers = {}
    bindings = sparql_mgr.energyconsumer_query()
    for obj in bindings:
      bus = obj['bus']['value'].upper()
      if bus not in self.EnergyConsumers:
        self.EnergyConsumers[bus] = {}
        self.EnergyConsumers[bus]['kW'] = {}
        self.EnergyConsumers[bus]['kVar'] = {}

      phases = obj['phases']['value']
      if phases == '':
        pval = float(obj['p']['value']) / 3.0
        qval = float(obj['q']['value']) / 3.0
        self.EnergyConsumers[bus]['kW']['A'] = pval
        self.EnergyConsumers[bus]['kW']['B'] = pval
        self.EnergyConsumers[bus]['kW']['C'] = pval
        self.EnergyConsumers[bus]['kVar']['A'] = qval
        self.EnergyConsumers[bus]['kVar']['B'] = qval
        self.EnergyConsumers[bus]['kVar']['C'] = qval
        #feeder_power['p']['A'] += pval
        #feeder_power['p']['B'] += pval
        #feeder_power['p']['C'] += pval
        #feeder_power['q']['A'] += qval
        #feeder_power['q']['B'] += qval
        #feeder_power['q']['C'] += qval
      else:
        pval = float(obj['p']['value'])
        qval = float(obj['q']['value'])
        self.EnergyConsumers[bus]['kW'][phases] = pval
        self.EnergyConsumers[bus]['kVar'][phases] = qval
        #feeder_power['p'][phases] += pval
        #feeder_power['q'][phases] += qval

    #print('EnergyConsumers[65]: ' + str(self.EnergyConsumers['65']),
    #      flush=True)
    #print('EnergyConsumers[47]: ' + str(self.EnergyConsumers['47']),
    #      flush=True)
    #print('EnergyConsumers[99]: ' + str(self.EnergyConsumers['99']),
    #      flush=True)

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

    #SynchronousMachines = AppUtil.getSynchronousMachines(sparql_mgr)

    self.Batteries = AppUtil.getBatteries(sparql_mgr)
    self.Batteries_obj = {}
    for mrid in self.Batteries:
      self.Batteries_obj[self.Batteries[mrid]['bus']] = {}
      self.Batteries_obj[self.Batteries[mrid]['bus']]['mrid'] = mrid
      self.Batteries_obj[self.Batteries[mrid]['bus']]['phase'] = self.Batteries[mrid]['phase']

    self.SolarPVs = AppUtil.getSolarPVs(sparql_mgr)

    self.EnergySource = AppUtil.getEnergySource(sparql_mgr)

    vnom = sparql_mgr.vnom_export()

    self.bus_info = {}
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

      self.bus_info[bus] = {}
      self.bus_info[bus]['idx'] = idx
      self.bus_info[bus]['phases'] = phases

      idx += 1

    #print('bus_info[65]: ' + str(self.bus_info['65']), flush=True)
    #print('bus_info[47]: ' + str(self.bus_info['47']), flush=True)
    #print('bus_info[150]: ' + str(self.bus_info['150']), flush=True)

    ysparse, nodelist = sparql_mgr.ybus_export()

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
    print('Count of ACLineSegments: ' + str(len(bindings)), flush=True)
    idx = 0
    for obj in bindings:
      name = obj['name']['value']
      bus1 = obj['bus1']['value'].upper()
      bus2 = obj['bus2']['value'].upper()
      phases = obj['phases']['value']
      if phases == '':
        phases = 'ABC'
      #print('ACLineSegment name: ' + name + ', bus1: ' + bus1 +
      #      ', bus2: ' + bus2 + ', phases: ' + phases, flush=True)

      branch_info[name] = {}
      branch_info[name]['idx'] = idx
      branch_info[name]['phases'] = phases
      branch_info[name]['type'] = 'line'
      branch_info[name]['from_bus'] = bus1
      branch_info[name]['from_bus_idx'] = self.bus_info[bus1]['idx']
      branch_info[name]['to_bus'] = bus2
      branch_info[name]['to_bus_idx'] = self.bus_info[bus2]['idx']
      #print(name + ': ' + str(branch_info[name]))
      #print(obj)
      idx += 1

    self.Regulators, RegIdx = AppUtil.getCombineRegulators(sparql_mgr)

    print('Regulators: ' + str(self.Regulators), flush=True)
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

        if 'RatioTapChanger.'+name in MethodUtil.NameToDevice and \
           MethodUtil.NameToDevice['RatioTapChanger.'+name] in self.Regulators:
          branch_info[name]['type'] = 'regulator'
        else:
          branch_info[name]['type'] = 'transformer'
        branch_info[name]['from_bus'] = bus
        branch_info[name]['from_bus_idx'] = self.bus_info[bus]['idx']
      else:
        branch_info[name]['to_bus'] = bus
        branch_info[name]['to_bus_idx'] = self.bus_info[bus]['idx']
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

        mrid = MethodUtil.NameToDevice['RatioTapChanger.'+name]
        pname = self.Regulators[mrid]['pname']
        if pname not in branch_info:
          branch_info[pname] = {}
          branch_info[pname]['idx'] = idx
          branch_info[pname]['phases'] = phase
          branch_info[pname]['type'] = 'regulator'
          branch_info[pname]['from_bus'] = bus
          branch_info[pname]['from_bus_idx'] = self.bus_info[bus]['idx']
          idx += 1
        elif bus != branch_info[pname]['from_bus']:
          if phase not in branch_info[pname]['phases']:
            branch_info[pname]['phases'] += phase
          branch_info[pname]['to_bus'] = bus
          branch_info[pname]['to_bus_idx'] = self.bus_info[bus]['idx']
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
        branch_info[name]['from_bus_idx'] = self.bus_info[bus1]['idx']
        branch_info[name]['to_bus'] = bus2
        branch_info[name]['to_bus_idx'] = self.bus_info[bus2]['idx']
        print(name + ': ' + str(branch_info[name]))
        #print(obj)
        idx += 1

    # setup two dictionaries for quick lookup of incident line and
    # outgoing lines for any bus index
    self.lines_in = {}
    self.lines_out = {}
    n_line_phase = {}
    for branch in branch_info:
      if branch_info[branch]['to_bus_idx'] not in self.lines_in:
        self.lines_in[branch_info[branch]['to_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}
      if branch_info[branch]['from_bus_idx'] not in self.lines_out:
        self.lines_out[branch_info[branch]['from_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}

      phases = branch_info[branch]['phases']
      for char in phases:
        self.lines_in[branch_info[branch]['to_bus_idx']][char].append(
                                                     branch_info[branch]['idx'])
        self.lines_out[branch_info[branch]['from_bus_idx']][char].append(
                                                     branch_info[branch]['idx'])
        if char not in n_line_phase:
          n_line_phase[char] = 0
        n_line_phase[char] += 1

      # Idenytify the line emerging out from the source bus.
      if branch_info[branch]['from_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = branch_info[branch]['idx']
      if branch_info[branch]['to_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = branch_info[branch]['idx']

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

    self.gapRel = 0.01
    self.interval = 1
    # uncomment the self.interval lines below to adjust the deltaT period
    # the optimization is based on per app and the frequency of messages
    if opt_type.startswith('r') or opt_type.startswith('R'):
      self.opt_type = 'resilience'
      #self.interval = 3
    elif opt_type.startswith('d') or opt_type.startswith('D'):
      self.opt_type = 'decarbonization'
      #self.interval = 4
    elif opt_type.startswith('p') or opt_type.startswith('P'):
      self.opt_type = 'profit_cvr'
      self.gapRel = 0.05
      #self.interval = 5
    elif opt_type.startswith('l') or opt_type.startswith('L'):
      self.opt_type = 'load_shedding'
      #self.interval = 6
    else:
      print('*** Exiting due to unrecognized optimization type: ' + opt_type,
            flush=True)
      exit()

    self.deltaT = 0.25 * self.interval

    self.b_i = np.arange(0.9, 1.1, 0.00625)

    self.defineOptimizationVariables(len(branch_info), len(self.bus_info),
                                     len(self.Batteries), len(self.Regulators))

    self.defineOptimizationStaticProblem(branch_info, RegIdx,
                                     len(self.Batteries), len(self.Regulators))

    # topic for sending out set_points messages
    self.publish_topic = service_output_topic('gridappsd-competing-app', '0')

    print('\nInitialized ' + opt_type +
          ' optimization competing app, waiting for messages...\n',
          flush=True)

    # counter for interval values > 1
    messageCounter = 0

    while True:
      if self.messageQueue.qsize() == 0:
        sleep(0.1)
        continue

      # discard messages other than most recent
      # comment this while loop out to never drain queue
      while self.messageQueue.qsize() > 1:
        print('Draining message queue, size: ' + str(self.messageQueue.qsize()),
              flush=True)
        self.messageQueue.get()
        messageCounter += 1

      message = self.messageQueue.get()
      messageCounter += 1

      # empty timestamp is end-of-data flag
      if message['timestamp'] == '':
        print('Time-series end-of-data!', flush=True)
        break

      if messageCounter % self.interval == 0:
        timestamp = int(message['timestamp'])
        loadshape = float(message['loadshape'])
        solar = float(message['solar'])
        price = float(message['price'])
        BatterySoC = message['BatterySoC']
        print('Time-series time: ' + str(timestamp) +
              ', loadshape: ' + str(loadshape) +
              ', solar: ' + str(solar) +
              ', price: ' + str(price) +
              ', BatterySoc: ' + str(BatterySoC), flush=True)

        self.updateSoC(BatterySoC)

        self.defineOptimizationDynamicProblem(timestamp, loadshape, solar)

        self.doOptimization(timestamp)


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

  parser = argparse.ArgumentParser()
  parser.add_argument("type", help="Competing App Type")
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("state", nargs="?", default="Alert",
                      help="Alert or Emergency State")
  parser.add_argument("--outage", "--out", "-o", type=int, nargs=2)
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(opts.simulation_id)
  assert gapps.connected

  competing_app = CompetingApp(gapps, opts.type, feeder_mrid,
                               opts.simulation_id, opts.outage, opts.state)


if __name__ == "__main__":
  _main()

