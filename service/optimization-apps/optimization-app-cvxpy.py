#!/usr/bin/env python3

# Copyright (c) 2024, Battelle Memorial Institute All rights reserved.
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
Created on August 14, 2023

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
import copy

from time import sleep
import cvxpy as cp

from gridappsd import GridAPPSD
from gridappsd import DifferenceBuilder
from gridappsd.topics import simulation_output_topic, simulation_log_topic, service_output_topic

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

  def defineOptimizationDynamicProblem(self, timestamp):
    # copy the base/static constraints that don't depend on time-series data
    # as a starting point to then add the time-series dependent part to that
    #self.dynamicConstraints = copy.deepcopy(self.staticConstraints)

    # GDB 8/25/23
    # For CVXPY since we have to redefine the whole problem with each new
    # timestamp, we can just share the same reference to the constraints
    # rather than copying it
    self.dynamicConstraints = self.staticConstraints

    v_min, v_max = (0.95 * 2401.77) ** 2, (1.05 * 2401.77) ** 2
    for bus in self.bus_info:
      bus_idx = self.bus_info[bus]['idx']
      self.dynamicConstraints.append(
        self.v_A[bus_idx] >= v_min)
      self.dynamicConstraints.append(
        self.v_A[bus_idx] <= v_max)
      self.dynamicConstraints.append(
        self.v_B[bus_idx] >= v_min)
      self.dynamicConstraints.append(
        self.v_B[bus_idx] <= v_max)
      self.dynamicConstraints.append(
        self.v_C[bus_idx] >= v_min)
      self.dynamicConstraints.append(
        self.v_C[bus_idx] <= v_max)

      #if bus_idx not in self.lines_in:
      #  print('Source bus: ' + bus, flush=True)

      if bus_idx not in self.lines_out:
        self.lines_out[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in self.lines_in: # check for source bus
        if '1' in self.bus_info[bus]['phases']:
          injection_p, injection_q = 0, 0
          if bus in self.EnergyConsumers and \
             'A' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['A']
            injection_q = self.EnergyConsumers[bus]['kVar']['A']

          if bus in self.SolarPVs and 'A' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs A bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'A' in self.Batteries_obj[bus]['phase']:
            #print('Batteries A bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicConstraints.append(sum(self.p_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - injection_p == \
               sum(self.p_flow_A[idx] for idx in self.lines_out[bus_idx]['A']))

            self.dynamicConstraints.append(sum(self.q_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               injection_q == sum(self.q_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A']))
          else:
            self.dynamicConstraints.append(sum(self.p_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               injection_p == sum(self.p_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A']))

            self.dynamicConstraints.append(sum(self.q_flow_A[idx] \
                 for idx in self.lines_in[bus_idx]['A']) - \
               injection_q == sum(self.q_flow_A[idx] \
                 for idx in self.lines_out[bus_idx]['A']))

        if '2' in self.bus_info[bus]['phases']:
          injection_p, injection_q = 0, 0
          if bus in self.EnergyConsumers and \
             'B' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['B']
            injection_q = self.EnergyConsumers[bus]['kVar']['B']

          if bus in self.SolarPVs and 'B' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs B bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'B' in self.Batteries_obj[bus]['phase']:
            #print('Batteries B bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicConstraints.append(sum(self.p_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - injection_p == \
               sum(self.p_flow_B[idx] for idx in self.lines_out[bus_idx]['B']))

            self.dynamicConstraints.append(sum(self.q_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               injection_q == sum(self.q_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B']))
          else:
            self.dynamicConstraints.append(sum(self.p_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               injection_p == sum(self.p_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B']))

            self.dynamicConstraints.append(sum(self.q_flow_B[idx] \
                 for idx in self.lines_in[bus_idx]['B']) - \
               injection_q == sum(self.q_flow_B[idx] \
                 for idx in self.lines_out[bus_idx]['B']))

        if '3' in self.bus_info[bus]['phases']:
          injection_p, injection_q = 0, 0
          if bus in self.EnergyConsumers and \
             'C' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['C']
            injection_q = self.EnergyConsumers[bus]['kVar']['C']

          if bus in self.SolarPVs and 'C' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs C bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if bus in self.Batteries_obj and \
             'C' in self.Batteries_obj[bus]['phase']:
            #print('Batteries C bus: ' + bus, flush=True)
            mrid = self.Batteries_obj[bus]['mrid']
            self.dynamicConstraints.append(sum(self.p_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               self.p_batt[self.Batteries[mrid]['idx']] - injection_p == \
               sum(self.p_flow_C[idx] for idx in self.lines_out[bus_idx]['C']))

            self.dynamicConstraints.append(sum(self.q_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               injection_q == sum(self.q_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C']))
          else:
            self.dynamicConstraints.append(sum(self.p_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               injection_p == sum(self.p_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C']))

            self.dynamicConstraints.append(sum(self.q_flow_C[idx] \
                 for idx in self.lines_in[bus_idx]['C']) - \
               injection_q == sum(self.q_flow_C[idx] \
                 for idx in self.lines_out[bus_idx]['C']))

    for mrid in self.Batteries:
      self.Batteries[mrid]['state'] = 'idling'
      idx = self.Batteries[mrid]['idx']
      self.dynamicConstraints.append(
              self.soc[idx] == self.Batteries[mrid]['SoC'] + \
              self.Batteries[mrid]['eff'] * self.p_batt_c[idx] * \
              self.deltaT / self.Batteries[mrid]['ratedE'] + \
              1 / self.Batteries[mrid]['eff'] * self.p_batt_d[idx] * \
              self.deltaT / self.Batteries[mrid]['ratedE'])

      self.dynamicConstraints.append(self.p_batt_c[idx] >= 0)

      self.dynamicConstraints.append(self.p_batt_d[idx] <= 0)

      self.dynamicConstraints.append(self.p_batt_c[idx] <= \
              self.lambda_c[idx] * self.Batteries[mrid]['prated'])

      self.dynamicConstraints.append(self.p_batt_d[idx] >= \
              -self.lambda_d[idx] * self.Batteries[mrid]['prated'])

      self.dynamicConstraints.append(self.p_batt[idx] == \
              self.p_batt_c[idx] + self.p_batt_d[idx])

      self.dynamicConstraints.append(self.lambda_c[idx] + self.lambda_d[idx] <= 1)

      # Battery SoC constraints added as Shiva couldn't identify PuLP's
      # equivalent of lb and ub
      self.dynamicConstraints.append(self.soc[idx] <= 0.9)
      self.dynamicConstraints.append(self.soc[idx] >= 0.2)


  def doOptimization(self, timestamp, coopFlag):
    # GDB Come back for this later to convert from PuLP to CVXPY
    #data = self.dynamicProb.to_dict()
    #opt_prob = {}
    #opt_prob['utility_function'] = data['objective']
    #opt_prob['constraints'] = data['constraints']
    #opt_prob['variables'] = data['variables']
    #opt_prob['parameters'] = data['parameters']
    #opt_prob['sos1'] = data['sos1']
    #opt_prob['sos2'] = data['sos2']

    # Dump optimization problem formulation to a file for Orestis'
    # methodology that combines them from multiple competing apps.
    # Eventually he'll get this from the message bus message, but
    # for now he reads from files
    #json_opt = open('log/' + self.opt_type + '_' +
    #                str(timestamp) + '.json', 'w')
    #json.dump(data, json_opt, indent=4)
    #json_opt.close()

    objective = None
    if self.opt_type == 'decarbonization':
      # objective
      # The latest version of cvxpy complains "unbounded" if not normalized
      objective = self.Psub_mod / 1000

    elif self.opt_type == 'resilience':
      # objective
      # SHIVA magic scaling factor for SoC that causes the optmization to
      # come up with the correct results where -self.soc[i] doesn't.
      # Shiva will be investigating why this happens since we don't want
      # to be dependent on magic
      objective = sum(-100 * self.soc[i] for i in range(len(self.Batteries)))

    elif self.opt_type == 'profit_cvr':
      # objective
      objective = sum((self.v_A[i] + self.v_B[i] + self.v_C[i]) for i in range(len(self.bus_info)))

    if coopFlag:
      # objective function for cooperartion from Tylor is added to the
      # base objective funtion regardless of whether it's decarbonization,
      # resilience, or profit_cvr
      # GDB 9/9/24: This creates a non-linear/non-convex objective function
      # that CVXPY free solvers balk at. The commercial Gurobi solver would
      # handle it according to Rabayet and Anita Bowers is the one to talk
      # to and tell her we are using it for research/publication purposes
      # to get a license without cost. But, we don't have time for that given
      # the end of FY24 deconfliction service deliverable so for now we won't
      # call doOptimization with coopFlag as True.
      objective += -1 * cp.sqrt(sum(cp.square(self.p_batt[i] - self.p_batt_proposed[i]) for i in range(len(self.Batteries))))/cp.sqrt(sum(cp.square(self.p_batt_greedy[i] - self.p_batt_proposed[i]) for i in range(len(self.Batteries))))

    # TODO: For some reason CVXPY fails to print the regulator taps unless
    #  substation regulator tap is fixed. For now fixing it to zero position
    self.dynamicConstraints.append(self.reg_taps[(0, 16)] == 1)
    problem = cp.Problem(cp.Minimize(objective), self.dynamicConstraints)

    # problem.solve(solver=cp.MOSEK, verbose=True)
    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, verbose=False)
    print('Optimization status:', problem.status, flush=True)

    objval = problem.value

    # Second stage for the decarbonization app
    if self.opt_type == 'decarbonization':
      bus_idx_batt = {'A': [], 'B': [], 'C': []}
      for mrid in self.Batteries:
        idx = self.Batteries[mrid]['idx']
        self.dynamicConstraints.append(self.p_batt[idx] == self.p_batt[idx].value)
        bus = self.Batteries[mrid]['bus']
        if 'A' in self.Batteries[mrid]['phase']:
          bus_idx_batt['A'].append(self.bus_info[bus]['idx'])
        elif 'B' in self.Batteries[mrid]['phase']:
          bus_idx_batt['B'].append(self.bus_info[bus]['idx'])
        else:
          bus_idx_batt['C'].append(self.bus_info[bus]['idx'])

      objective += -self.Psub_mod + \
                          sum(-self.v_A[i] for i in bus_idx_batt['A']) + \
                          sum(-self.v_B[i] for i in bus_idx_batt['B']) + \
                          sum(-self.v_C[i] for i in bus_idx_batt['C'])
      # problem = cp.Problem(cp.Minimize(objective), self.dynamicConstraints)
      # problem.solve(solver=cp.MOSEK)
      # print('Optimization State II status:', problem.status, flush=True)

    '''
    branch_flow = []
    for branch in self.branch_info:
      idx = self.branch_info[branch]['idx']
      branch_flow.append([branch, self.branch_info[branch]['from_bus'],
                  self.branch_info[branch]['to_bus'], self.p_flow_A[idx].value,
                  self.p_flow_B[idx].value, self.p_flow_C[idx].value,
                  self.q_flow_A[idx].value, self.q_flow_B[idx].value,
                  self.q_flow_C[idx].value])

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
    for bus in self.bus_info:
      idx = self.bus_info[bus]['idx']
      bus_voltage.append([bus, math.sqrt(self.v_A[idx].value),
                  math.sqrt(self.v_B[idx].value), math.sqrt(self.v_C[idx].value)])
      v.append(math.sqrt(self.v_A[idx].value) / 2401.77)
      v.append(math.sqrt(self.v_B[idx].value) / 2401.77)
      v.append(math.sqrt(self.v_C[idx].value) / 2401.77)

    print(tabulate(bus_voltage, headers=['Bus', 'V_A', 'V_B', 'V_C'],
                   tablefmt='psql'))
    '''

    regulator_taps = []
    for reg in self.Regulators:
      idx = self.Regulators[reg]['idx']
      for k in range(32):
        if self.reg_taps[(idx, k)].value:
          # new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                 k-16, None)
          regulator_taps.append([reg, k-16, self.b_i[k]])
          break # assume this will only happen once per regulator

    print(tabulate(regulator_taps, headers=['Regulator', 'Tap', 'b_i'],
                   tablefmt='psql'), '\n', flush=True)

    p_batt_setpoints = []
    for mrid in self.Batteries:
      idx = self.Batteries[mrid]['idx']
      self.Batteries[mrid]['SoC'] = self.soc[idx].value
      # new value before old value for DifferenceBuilder
      # note the optimized p_batt value is negated for the GridLAB-D
      # DifferenceBuilder message
      self.difference_builder.add_difference(mrid,
           'PowerElectronicsConnection.p', -self.p_batt[idx].value, None)
      p_batt_setpoints.append([mrid, self.p_batt[idx].value/1000,
                               self.soc[idx].value])

    print(tabulate(p_batt_setpoints, headers=['Battery', 'P_batt (kW)',
                   'Target SoC'], tablefmt='psql'), flush=True)

    # set p_batt_greedy/reg_greedy with every optimization based on measurements
    if not coopFlag:
      for mrid in self.Batteries:
        idx = self.Batteries[mrid]['idx']
        self.p_batt_greedy[idx] = self.p_batt[idx].value

      for reg in self.Regulators:
        idx = self.Regulators[reg]['idx']
        for k in range(32):
          if self.reg_taps[(idx, k)].value:
            self.reg_greedy[idx] = k-16
            break # assume this will only happen once per regulator

    dispatch_message = self.difference_builder.get_message()
    print('Sending Measurements DifferenceBuilder message!', flush=True)
    #print('Sending Measurements DifferenceBuilder message: ' +
    #      json.dumps(dispatch_message), flush=True)
    self.gapps.send(self.meas_publish_topic, json.dumps(dispatch_message))
    self.difference_builder.clear()


  def on_message(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      status = message['processStatus']
      if status=='COMPLETE' or status=='CLOSED':
        self.keepLoopingFlag = False
        print('Simulation ' + status + ' message received', flush=True)

    elif 'message' in message:
      self.messageQueue.put(message['message'])

    else:
      self.messageQueue.put(message)



  def defineOptimizationVariables(self, len_branch_info, len_bus_info,
                                  len_Batteries, len_Regulators):
    flow_min, flow_max = -5e6, 5e6
    self.p_flow_A = cp.Variable(len_branch_info, integer=False, name='p_flow_A')

    self.p_flow_B = cp.Variable(len_branch_info, integer=False, name='p_flow_B')

    self.p_flow_C = cp.Variable(len_branch_info, integer=False, name='p_flow_C')

    self.q_flow_A = cp.Variable(len_branch_info, integer=False, name='q_flow_A')

    self.q_flow_B = cp.Variable(len_branch_info, integer=False, name='q_flow_B')

    self.q_flow_C = cp.Variable(len_branch_info, integer=False, name='q_flow_C')

    self.Psub = cp.Variable(integer=False, name='P_sub')

    self.Psub_mod = cp.Variable(integer=False, name='P_sub_mod')

    self.p_batt = cp.Variable(len_Batteries, integer=False, name='p_batt')

    self.p_batt_c = cp.Variable(len_Batteries, integer=False, name='p_batt_c')

    self.p_batt_d = cp.Variable(len_Batteries, integer=False, name='p_batt_d')

    self.soc = cp.Variable(len_Batteries, integer=False, name='soc')

    self.lambda_c = cp.Variable(len_Batteries, boolean=True, name='lambda_c')

    self.lambda_d = cp.Variable(len_Batteries, boolean=True, name='lambda_d')

    self.v_A = cp.Variable(len_bus_info, integer=False, name='v_A')

    self.v_B = cp.Variable(len_bus_info, integer=False, name='v_B')

    self.v_C = cp.Variable(len_bus_info, integer=False, name='v_C')

    self.reg_taps = cp.Variable((len_Regulators, 32), boolean=True,
                                name='reg_taps')

    # cooperation variables
    # since these are held constant, I don't need to define them with
    # cp.Variable calls, but as fixed length vectors. It's still convenient
    # though to define them along with the other optimization variables.
    # GDB 9/9/24: I no longer attempt to perform cooperation optimizations,
    # but still need these vectors so I'll leave them as defined here.
    self.p_batt_proposed = [None] * len_Batteries
    self.p_batt_greedy = [None] * len_Batteries
    self.reg_proposed = [None] * len_Regulators
    self.reg_greedy = [None] * len_Regulators


  def defineOptimizationStaticProblem(self, branch_info, RegIdx):
    # define base/static optimization problem that doesn't change with the
    # time-series multiplier values

    self.staticConstraints = []

    # objective
    if self.opt_type == 'decarbonization':
      # constraints
      self.staticConstraints.append(self.Psub_mod >= self.Psub)

      self.staticConstraints.append(self.Psub_mod >= -self.Psub)

      flow_min, flow_max = -5e6, 5e6
      self.staticConstraints.append(self.Psub >= flow_min)
      self.staticConstraints.append(self.Psub <= flow_max)
      self.staticConstraints.append(self.Psub_mod >= flow_min)
      self.staticConstraints.append(self.Psub_mod <= flow_max)

      sub_flow_idx = self.EnergySource['flow_idx']
      self.staticConstraints.append(self.Psub == self.p_flow_A[sub_flow_idx] + \
                                                 self.p_flow_B[sub_flow_idx] + \
                                                 self.p_flow_C[sub_flow_idx])

    M = 1e9
    for branch in branch_info:
      if branch_info[branch]['type'] == 'regulator':
        if 'A' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.A']

          for k in range(32):
            self.staticConstraints.append(
                 self.v_A[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[branch_info[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0)

            self.staticConstraints.append(
                 self.v_A[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[branch_info[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0)

        if 'B' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.B']

          for k in range(32):
            self.staticConstraints.append(
                 self.v_B[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[branch_info[branch]['from_bus_idx']]\
                    - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0)

            self.staticConstraints.append(
                 self.v_B[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[branch_info[branch]['from_bus_idx']]\
                    + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0)

        if 'C' in branch_info[branch]['phases']:
          reg_idx = RegIdx[branch+'.C']

          for k in range(32):
            self.staticConstraints.append(
                 self.v_C[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[branch_info[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(reg_idx, k)]) <= 0)

            self.staticConstraints.append(
                 self.v_C[branch_info[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[branch_info[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(reg_idx, k)]) >= 0)

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

        self.staticConstraints.append(
            self.v_A[to_bus_idx] == self.v_A[fr_bus_idx] - \
            2.0*(self.p_flow_A[idx]*z_aa.real + self.q_flow_A[idx]*z_aa.imag + \
            self.p_flow_B[idx]*(-0.5*z_ab.real + hfsqrt3*z_ab.imag) + \
            self.q_flow_B[idx]*(-0.5*z_ab.imag - hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_ac.real - hfsqrt3*z_ac.imag) + \
            self.q_flow_C[idx]*(-0.5*z_ac.imag + hfsqrt3*z_ac.real)))

        self.staticConstraints.append(
            self.v_B[to_bus_idx] == self.v_B[fr_bus_idx] - \
            2.0*(self.p_flow_B[idx]*z_bb.real + self.q_flow_B[idx]*z_bb.imag + \
            self.p_flow_A[idx]*(-0.5*z_ab.real - hfsqrt3*z_ab.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ab.imag + hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_bc.real + hfsqrt3*z_bc.imag) + \
            self.q_flow_C[idx]*(-0.5*z_bc.imag - hfsqrt3*z_bc.real)))

        self.staticConstraints.append(
            self.v_C[to_bus_idx] == self.v_C[fr_bus_idx] - \
            2.0*(self.p_flow_C[idx]*z_cc.real + self.q_flow_C[idx]*z_cc.imag + \
            self.p_flow_A[idx]*(-0.5*z_ac.real + hfsqrt3*z_ac.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ac.imag - hfsqrt3*z_ac.real) + \
            self.p_flow_B[idx]*(-0.5*z_bc.real - hfsqrt3*z_bc.imag) + \
            self.q_flow_B[idx]*(-0.5*z_bc.imag + hfsqrt3*z_bc.real)))

    # fix source bus at 1.0
    sourcebus = self.EnergySource['bus']
    v_source = self.EnergySource['basev'] / math.sqrt(3)

    self.staticConstraints.append(self.v_A[self.bus_info[sourcebus]['idx']] == v_source ** 2)

    self.staticConstraints.append(self.v_B[self.bus_info[sourcebus]['idx']] == v_source ** 2)

    self.staticConstraints.append(self.v_C[self.bus_info[sourcebus]['idx']] == v_source ** 2)

    for k in range(len(self.Regulators)):
      self.staticConstraints.append(sum(self.reg_taps[(k, tap)] for tap in range(32)) == 1)


  def pol2cart(self, mag, angle_deg):
        # Convert degrees to radians. GridAPPS-D spits angle in degrees
        angle_rad =  math.radians(angle_deg)
        p = mag * np.cos(angle_rad)
        q = mag * np.sin(angle_rad)
        return p, q


  def updateEnergyConsumers(self, measurements):
    for bus in self.EnergyConsumers:
      for phase in self.EnergyConsumers[bus]['measid']:
        measid = self.EnergyConsumers[bus]['measid'][phase]
        if measid in measurements:
          p, q = self.pol2cart(measurements[measid]['magnitude'],
                               measurements[measid]['angle'])
          self.EnergyConsumers[bus]['kW'][phase] = p
          self.EnergyConsumers[bus]['kVar'][phase] = q


  def updateSolarPVs(self, measurements):
    for bus in self.SolarPVs:
      measid = self.SolarPVs[bus]['measid']
      if measid in measurements:
        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])
        self.SolarPVs[bus]['p'] = abs(p)


  def updateBatterySoC(self, measurements):
    for mrid in self.Batteries:
      measid = self.Batteries[mrid]['SoC_measid']
      if measid in measurements:
        self.Batteries[mrid]['SoC'] = measurements[measid]['value']/100.0
        print('Updated SoC for ' + self.Batteries[mrid]['name'] + ': ' + str(self.Batteries[mrid]['SoC']), flush=True)


  def __init__(self, gapps, opt_type, feeder_mrid, simulation_id, interval):

    self.gapps = gapps

    self.messageQueue = queue.Queue()

    # subscribe to simulation log and output messages
    # since messages are just going on a queue, subscribe right away to
    # keep from missing any sent during app initialization
    self.keepLoopingFlag = True
    out_id = gapps.subscribe(simulation_output_topic(simulation_id), self)
    log_id = gapps.subscribe(simulation_log_topic(simulation_id), self)
    coop_id = gapps.subscribe(service_output_topic('gridappsd-deconflictor-app',
                              simulation_id), self)

    SPARQLManager = getattr(importlib.import_module('sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.EnergyConsumers = AppUtil.getEnergyConsumers(sparql_mgr)
    #print('Starting EnergyConsumers: ' + json.dumps(self.EnergyConsumers, indent=2), flush=True)

    self.SolarPVs = AppUtil.getSolarPVs(sparql_mgr)
    #print('Starting SolarPVs: ' + json.dumps(self.SolarPVs, indent=2), flush=True)

    self.Batteries = AppUtil.getBatteries(sparql_mgr)
    print('Starting Batteries: ' + json.dumps(self.Batteries, indent=2), flush=True)

    self.Batteries_obj = {}
    for mrid in self.Batteries:
      self.Batteries_obj[self.Batteries[mrid]['bus']] = {}
      self.Batteries_obj[self.Batteries[mrid]['bus']]['mrid'] = mrid
      self.Batteries_obj[self.Batteries[mrid]['bus']]['phase'] = self.Batteries[mrid]['phase']

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

    if opt_type.startswith('r') or opt_type.startswith('R'):
      self.opt_type = 'resilience'
    elif opt_type.startswith('d') or opt_type.startswith('D'):
      self.opt_type = 'decarbonization'
    elif opt_type.startswith('p') or opt_type.startswith('P'):
      self.opt_type = 'profit_cvr'
    else:
      print('*** Exiting due to unrecognized optimization type: ' + opt_type,
            flush=True)
      exit()

    # deltaT is time between timesteps as fractional hours
    # optimization interval seconds is the number of simulation seconds
    # between triggering an optimization and must be a multiple of 3
    #optIntervalSec = 3 # optimize every GridLAB-D timestamp
    # 15 seconds is a good number for a real-time simulation
    optIntervalSec = 15
    # if attempting non-real-time, something like 600 is reasonable
    #optIntervalSec = 600
    if interval != None:
      optIntervalSec = int(interval)

    self.deltaT = optIntervalSec/3600.0

    self.b_i = np.arange(0.9, 1.1, 0.00625)

    self.defineOptimizationVariables(len(branch_info), len(self.bus_info),
                                     len(self.Batteries), len(self.Regulators))

    # GDB 8/25/23
    # Defining the part of the optimization problem that doesn't change with
    # each timestamp flies for PuLP, but not CVXPY. Based on how it sets up
    # the problem internally, it all needs to be redone each time.
    #self.defineOptimizationStaticProblem(branch_info, RegIdx)

    # topic for sending out set_points messages
    self.app_name = 'gridappsd-' + self.opt_type + '-app'
    self.meas_publish_topic = service_output_topic(self.app_name+':meas',
                                                   simulation_id)
    self.coop_publish_topic = service_output_topic(self.app_name+':coop',
                                                   simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

    print('\nInitialized ' + opt_type +
          ' CVXPY optimization competing app, waiting for messages...\n',
          flush=True)

    messageCounter = 0

    while self.keepLoopingFlag:
      if self.messageQueue.qsize() == 0:
        sleep(0.1)
        continue

      # discard messages other than most recent
      # comment this while loop out to never drain queue
      while self.messageQueue.qsize() > 1:
        print('Draining message queue, size: '+str(self.messageQueue.qsize()),
              flush=True)
        self.messageQueue.get()
        messageCounter += 1

      message = self.messageQueue.get()
      messageCounter += 1

      if 'measurements' in message: # this is a simulation measurements message
        # always update the EnergyConsumers, etc. data structures with new
        # measurements even if we aren't going to do an optimization so
        # they will be up to date with any cooperation messages received

        self.updateEnergyConsumers(message['measurements'])
        #print('Updated EnergyConsumers #' + str(messageCounter) + ': ' + json.dumps(self.EnergyConsumers, indent=2), flush=True)

        self.updateSolarPVs(message['measurements'])
        #print('Updated SolarPVs #' + str(messageCounter) + ': ' + json.dumps(self.SolarPVs, indent=2), flush=True)

        self.updateBatterySoC(message['measurements'])
        #print('Updated BatterySoC #' + str(messageCounter) + ': ' + json.dumps(self.Batteries, indent=2), flush=True)

        timestamp = int(message['timestamp'])

        # If doing real-time simulation must subtract 5 off timestamp to make it
        # evenly divisble by multiples of the 3 second GridLAB-D time interval
        if (timestamp-5) % optIntervalSec != 0:
        # If doing non-real-time simulation remove the 5 second offset because
        # GridLAB-D outputs at 60 second intervals
        #if timestamp % optIntervalSec != 0:
          print('Simulation timestamp (skipping optimization): '+str(timestamp),
                flush=True)
        else:
          print('Simulation timestamp for optimization: ' + str(timestamp),
                flush=True)

          # Need to define the full optimization problem each time anything
          # changes for CVXPY to be happy
          self.defineOptimizationStaticProblem(branch_info, RegIdx)

          self.defineOptimizationDynamicProblem(timestamp)

          self.doOptimization(timestamp, False)

      else: # this is a cooperation message from deconflictor
        # message consists of a target ResolutionVector that is a dictionary
        # with device mrid keys and target set-point values
        targetResolutionVector = message['targetResolutionVector']
        #for mrid in targetResolutionVector:
        #  print('DECONFLICTOR COOPERATE mrid ' + mrid + ' target set-point: ' + str(targetResolutionVector[mrid]), flush=True)

        for mrid in self.Batteries:
          if mrid in targetResolutionVector:
            idx = self.Batteries[mrid]['idx']
            self.p_batt_proposed[idx] = -targetResolutionVector[mrid][1]

        for reg in self.Regulators:
          if reg in targetResolutionVector:
            idx = self.Regulators[reg]['idx']
            self.reg_proposed[idx] = targetResolutionVector[reg][1]

        # Need to define the full optimization problem each time anything
        # changes for CVXPY to be happy
        # GDB 9/9/24: Can't do a new optimization for cooperation because
        # the objective function is non-linear/non-convex so we have an
        # alternative workflow implementation for supporting cooperation in
        # order to meet the FY24 deconfliction service deliverable
        '''
        self.defineOptimizationStaticProblem(branch_info, RegIdx)

        self.defineOptimizationDynamicProblem(timestamp)

        self.doOptimization(timestamp, True)
        '''

        print('DECONFLICTOR COOPERATE p_batt_greedy: ' + str(self.p_batt_greedy), flush=True)
        print('DECONFLICTOR COOPERATE p_batt_proposed: ' + str(self.p_batt_proposed), flush=True)

        # GDB 9/10/24: Here is the alternative support for cooperation via
        # ranking the differences between proposed and greedy setpoints:
        # first, create a list of differences
        len_Batteries = len(self.Batteries)
        p_batt_diff = [None] * len_Batteries
        for i in range(len_Batteries):
          p_batt_diff[i] = abs(self.p_batt_greedy[i] - self.p_batt_proposed[i])

        print('DECONFLICTOR COOPERATE p_batt_diff: ' + str(p_batt_diff), flush=True)

        # omit any setpoints where proposed == greeedy
        p_batt_sort = []
        for i in range(len_Batteries):
          if p_batt_diff[i] > 0:
            p_batt_sort.append(p_batt_diff[i])

        # sorts in place
        p_batt_sort.sort()

        coopCount = max(1, len(p_batt_sort)//2) # integer "floor" division

        # find the value associated with the last "cooperating" battery
        diffMax = p_batt_sort[coopCount-1]

        print('DECONFLICTOR COOPERATE batteries coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)

        # start with assuming no cooperation by copying p_batt_greedy
        p_batt_coop = self.p_batt_greedy.copy()

        for i in range(len_Batteries):
          # check if this is a "cooperating" battery
          if p_batt_diff[i] <= diffMax:
            # if so, set it to the proposed value
            p_batt_coop[i] = self.p_batt_proposed[i]

        print('DECONFLICTOR COOPERATE p_batt_coop: ' + str(p_batt_coop), flush=True)

        # now do the same for regulators
        print('DECONFLICTOR COOPERATE reg_greedy: ' + str(self.reg_greedy), flush=True)
        print('DECONFLICTOR COOPERATE reg_proposed: ' + str(self.reg_proposed), flush=True)

        len_Regulators = len(self.Regulators)
        reg_diff = [None] * len_Regulators
        for i in range(len_Regulators):
          reg_diff[i] = abs(self.reg_greedy[i] - self.reg_proposed[i])

        print('DECONFLICTOR COOPERATE reg_diff: ' + str(reg_diff), flush=True)

        # omit any setpoints where proposed == greeedy
        reg_sort = []
        for i in range(len_Regulators):
          if reg_diff[i] > 0:
            reg_sort.append(reg_diff[i])

        # sorts in place
        reg_sort.sort()

        # determine the number of regulators that will "cooperate"
        coopCount = max(1, len(reg_sort)//2) # integer "floor" division

        # find the value associated with the last "cooperating" regulator
        diffMax = reg_sort[coopCount-1]

        print('DECONFLICTOR COOPERATE regulators coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)

        # start with assuming no cooperation by copying p_batt_greedy
        reg_coop = self.reg_greedy.copy()

        for i in range(len_Regulators):
          # check if this is a "cooperating" regulator
          if reg_diff[i] <= diffMax:
            # if so, set it to the proposed value
            reg_coop[i] = self.reg_proposed[i]

        print('DECONFLICTOR COOPERATE reg_coop: ' + str(reg_coop), flush=True)

        # finally, send out the cooperation setpoints via DifferenceBuilder msg
        for reg in self.Regulators:
          idx = self.Regulators[reg]['idx']
          # new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                 reg_coop[idx], None)

        for mrid in self.Batteries:
          idx = self.Batteries[mrid]['idx']
          # new value before old value for DifferenceBuilder
          # note the p_batt value is negated for the GridLAB-D
          # DifferenceBuilder message
          self.difference_builder.add_difference(mrid,
               'PowerElectronicsConnection.p', -p_batt_coop[idx], None)

        dispatch_message = self.difference_builder.get_message()
        dispatch_message['cooperationPhase'] = \
                         message['cooperationPhase']
        print('Sending Cooperation DifferenceBuilder message!', flush=True)
        #print('Sending Cooperation DifferenceBuilder message: ' +
        #      json.dumps(dispatch_message), flush=True)
        self.gapps.send(self.coop_publish_topic, json.dumps(dispatch_message))
        self.difference_builder.clear()

    gapps.unsubscribe(out_id)
    gapps.unsubscribe(log_id)
    gapps.unsubscribe(coop_id)


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
  parser.add_argument("interval", nargs='?', help="Interval Between Optimizations")

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
                               opts.simulation_id, opts.interval)

  print('Goodbye!', flush=True)


if __name__ == "__main__":
  _main()

