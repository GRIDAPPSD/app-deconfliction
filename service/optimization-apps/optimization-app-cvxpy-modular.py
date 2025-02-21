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

  def fakeMain(self):
    self.includeBatteriesFlag = True
    self.includeRegulatorsFlag = True

    # does it make sense to exclude either of these two?
    self.includeEnergyConsumersFlag = True
    self.includeSolarPVsFlag = True

    self.includePFlowFlag = True
    self.includeQFlowFlag = True
    self.includeVoltagesFlag = True

    # TODO NOTE: Feedback from Monish
    # We will need something more generic and flexible for supporting different
    # objectives than the hardwired code I have for the three existing
    # objectives because we will have a bigger set of objectives that we
    # will build up. We also likely won't actually perform/solve the
    # optimization inside the function that defines the objective, but pull
    # that out.
    self.objectiveResilienceFlag = True
    self.objectiveCVRFlag = False
    self.objectiveDecarbonizationFlag = False

    # make sure only a single objective is specified
    if self.objectiveResilienceFlag:
      self.objectiveCVRFlag = False
      self.objectiveDecarbonizationFlag = False
    elif self.objectiveCVRFlag:
      self.objectiveResilienceFlag = False
      self.objectiveDecarbonizationFlag = False
    elif self.objectiveDecarbonizationFlag:
      self.objectiveResilienceFlag = False
      self.objectiveCVRFlag = False

    # enforce dependencies so the optimization is well-defined
    if self.objectiveResilienceFlag:
      self.includeBatteriesFlag = True

    if self.includeBatteriesFlag:
      self.includePFlowFlag = True

    if self.includeRegulatotorsFlag:
      self.includeVoltagesFlag = True

    if self.includeVoltagesFlag:
      self.includePFlowFlag = True
      self.includeQFlowFlag = True

    self.defineOptimizationVariables(includePFlowFlag, includeQFlowFlag,
               includeVoltagesFlag, includeBatteriesFlag, includeRegulatorsFlag)

    # set optimization constraints
    if self.includeBatteriesFlag:
      self.constraintsDERWithBatteries()

    if self.includeRegulatorsFlag:
      self.constraintsDERWithRegulators()

    if self.includePFlowFlag:
      self.constraintsNetworkWithPFlow(self.includeBatteriesFlag,
                                       self.includeEnergyConsumersFlag,
                                       self.inludeSolarPVsFlag)

    if self.includeQFlowFlag:
      self.constraintsNetworkWithQFlow(self.includeEnergyConsumersFlag)

    if self.includeVoltagesFlag:
      self.constraintsNetworkWithVoltages(self.includeRegulatorsFlag)

    if self.objectiveResilienceFlag:
      self.objectiveForResilience()

    if self.objectiveCVRFlag:
      self.objectiveForCVR()

    if self.objectiveDecarbonizationFlag:
      self.objectiveForDecarbonization()

    self.reportOptimization(self.includeRegulatorsFlag,
                            self.includeBatteriesFlag):


  def defineOptimizationVariables(self, includePFlowFlag, includeQFlowFlag,
            includeVoltagesFlag, includeBatteriesFlag, includeRegulatorsFlag):
    if includePFlowFlag:
      len_BranchInfo = len(self.BranchInfo)
      self.p_flow_A = cp.Variable(len_BranchInfo, integer=False,name='p_flow_A')
      self.p_flow_B = cp.Variable(len_BranchInfo, integer=False,name='p_flow_B')
      self.p_flow_C = cp.Variable(len_BranchInfo, integer=False,name='p_flow_C')

    if includeQFlowFlag:
      len_BranchInfo = len(self.BranchInfo)
      self.q_flow_A = cp.Variable(len_BranchInfo, integer=False,name='q_flow_A')
      self.q_flow_B = cp.Variable(len_BranchInfo, integer=False,name='q_flow_B')
      self.q_flow_C = cp.Variable(len_BranchInfo, integer=False,name='q_flow_C')

    if includeVoltagesFlag:
      len_BusInfo = len(self.BusInfo)
      self.v_A = cp.Variable(len_BusInfo, integer=False, name='v_A')
      self.v_B = cp.Variable(len_BusInfo, integer=False, name='v_B')
      self.v_C = cp.Variable(len_BusInfo, integer=False, name='v_C')

    if includeBatteriesFlag:
      len_BatteriesInfo = len(self.BatteriesInfo)
      self.p_batt = cp.Variable(len_BatteriesInfo, integer=False, name='p_batt')
      self.p_batt_c = cp.Variable(len_BatteriesInfo, integer=False, name='p_batt_c')
      self.p_batt_d = cp.Variable(len_BatteriesInfo, integer=False, name='p_batt_d')
      self.soc = cp.Variable(len_BatteriesInfo, integer=False, name='soc')
      self.lambda_c = cp.Variable(len_BatteriesInfo, boolean=True, name='lambda_c')
      self.lambda_d = cp.Variable(len_BatteriesInfo, boolean=True, name='lambda_d')

      # cooperation variables
      # since these are held constant, I don't need to define them with
      # cp.Variable calls, but as fixed length vectors. It's still convenient
      # though to define them along with the other optimization variables.
      self.p_batt_proposed = [None] * len_BatteriesInfo
      self.p_batt_greedy = [None] * len_BatteriesInfo

    if includeRegulatorsFlag:
    len_RegulatorsInfo = len(self.RegulatorsInfo)
      self.reg_taps = cp.Variable((len_RegulatorsInfo, 32), boolean=True,
                                  name='reg_taps')

      # cooperation variables
      # since these are held constant, I don't need to define them with
      # cp.Variable calls, but as fixed length vectors. It's still convenient
      # though to define them along with the other optimization variables.
      self.reg_proposed = [None] * len_RegulatorsInfo
      self.reg_greedy = [None] * len_RegulatorsInfo

    if objectiveDecarbonizationFlag:
      self.Psub = cp.Variable(integer=False, name='P_sub')
      self.Psub_mod = cp.Variable(integer=False, name='P_sub_mod')


  def constraintsDERWithBatteries(self):
    for mrid in self.BatteriesInfo:
      self.BatteriesInfo[mrid]['state'] = 'idling'
      idx = self.BatteriesIdx[mrid]
      self.Constraints.append(
              self.soc[idx] == self.BatteriesInfo[mrid]['SoC'] + \
              self.BatteriesInfo[mrid]['eff'] * self.p_batt_c[idx] * \
              self.deltaT / self.BatteriesInfo[mrid]['ratedE'] + \
              1 / self.BatteriesInfo[mrid]['eff'] * self.p_batt_d[idx] * \
              self.deltaT / self.BatteriesInfo[mrid]['ratedE'])

      self.Constraints.append(self.p_batt_c[idx] >= 0)
      self.Constraints.append(self.p_batt_c[idx] <= \
              self.lambda_c[idx] * self.BatteriesInfo[mrid]['prated'])

      self.Constraints.append(self.p_batt_d[idx] <= 0)
      self.Constraints.append(self.p_batt_d[idx] >= \
              -self.lambda_d[idx] * self.BatteriesInfo[mrid]['prated'])

      self.Constraints.append(self.p_batt[idx] == \
              self.p_batt_c[idx] + self.p_batt_d[idx])
      self.Constraints.append(self.lambda_c[idx] + self.lambda_d[idx] <= 1)

      # Battery SoC constraints added as Shiva couldn't identify CVXPY's
      # equivalent to PuLP's lb and ub
      self.Constraints.append(self.soc[idx] <= 0.9)
      self.Constraints.append(self.soc[idx] >= 0.2)


  def constraintsDERWithRegulators(self):
    for k in range(len(self.RegulatorsInfo)):
      self.Constraints.append(sum(self.reg_taps[(k, tap)] for tap in range(32)) == 1)

     # For some reason CVXPY fails to print the regulator taps unless
     # substation regulator tap is fixed--for now fixing it to zero position
     self.Constraints.append(self.reg_taps[(0, 16)] == 1)


  def constraintsNetworkWithPFlow(self, includeBatteriesFlag,
                               includeEnergyConsumersFlag, includeSolarPVsFlag):
    for bus in self.BusInfo:
      bus_idx = self.BusInfo[bus]['idx']
      if bus_idx not in self.LinesOut:
        self.LinesOut[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in self.LinesIn: # check for source bus
        if '1' in self.BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'A' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['A']

          if includeSolarPVsFlag and bus in self.SolarPVs and \
             'A' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs A bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in self.BatteriesObj and \
             'A' in self.BatteriesObj[bus]['phase']:
            #print('Batteries A bus: ' + bus, flush=True)
            mrid = self.BatteriesObj[bus]['mrid']
            self.Constraints.append(sum(self.p_flow_A[idx] \
                 for idx in self.LinesIn[bus_idx]['A']) - \
               self.p_batt[self.BatteriesIdx[mrid]] - injection_p == \
               sum(self.p_flow_A[idx] for idx in self.LinesOut[bus_idx]['A']))

          else:
            self.Constraints.append(sum(self.p_flow_A[idx] \
                 for idx in self.LinesIn[bus_idx]['A']) - injection_p == \
               sum(self.p_flow_A[idx] for idx in self.LinesOut[bus_idx]['A']))

        if '2' in self.BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'B' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['B']

          if includeSolarPVsFlag and bus in self.SolarPVs and \
             'B' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs B bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in self.BatteriesObj and \
             'B' in self.BatteriesObj[bus]['phase']:
            #print('Batteries B bus: ' + bus, flush=True)
            mrid = self.BatteriesObj[bus]['mrid']
            self.Constraints.append(sum(self.p_flow_B[idx] \
                 for idx in self.LinesIn[bus_idx]['B']) - \
               self.p_batt[self.BatteriesIdx[mrid]] - injection_p == \
               sum(self.p_flow_B[idx] for idx in self.LinesOut[bus_idx]['B']))

          else:
            self.Constraints.append(sum(self.p_flow_B[idx] \
                 for idx in self.LinesIn[bus_idx]['B']) - injection_p == \
               sum(self.p_flow_B[idx] for idx in self.LinesOut[bus_idx]['B']))

        if '3' in self.BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'C' in self.EnergyConsumers[bus]['kW']:
            injection_p = self.EnergyConsumers[bus]['kW']['C']

          if includeSolarPVsFlag and bus in self.SolarPVs and \
             'C' in self.SolarPVs[bus]['phase']:
            injection_p -= self.SolarPVs[bus]['p']
            #print('SolarPVs C bus: ' + bus + ', value: ' +
            #      str(self.SolarPVs[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in self.BatteriesObj and \
             'C' in self.BatteriesObj[bus]['phase']:
            #print('Batteries C bus: ' + bus, flush=True)
            mrid = self.BatteriesObj[bus]['mrid']
            self.Constraints.append(sum(self.p_flow_C[idx] \
                 for idx in self.LinesIn[bus_idx]['C']) - \
               self.p_batt[self.BatteriesIdx[mrid]] - injection_p == \
               sum(self.p_flow_C[idx] for idx in self.LinesOut[bus_idx]['C']))

          else:
            self.Constraints.append(sum(self.p_flow_C[idx] \
                 for idx in self.LinesIn[bus_idx]['C']) - injection_p == \
               sum(self.p_flow_C[idx] for idx in self.LinesOut[bus_idx]['C']))


  def constraintsNetworkWithQFlow(self, includeEnergyConsumersFlag):
    for bus in self.BusInfo:
      bus_idx = self.BusInfo[bus]['idx']
      if bus_idx not in self.LinesOut:
        self.LinesOut[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in self.LinesIn: # check for source bus
        if '1' in self.BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'A' in self.EnergyConsumers[bus]['kW']:
            injection_q = self.EnergyConsumers[bus]['kVar']['A']

          self.Constraints.append(sum(self.q_flow_A[idx] \
               for idx in self.LinesIn[bus_idx]['A']) - injection_q == \
             sum(self.q_flow_A[idx] for idx in self.LinesOut[bus_idx]['A']))

        if '2' in self.BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'B' in self.EnergyConsumers[bus]['kW']:
            injection_q = self.EnergyConsumers[bus]['kVar']['B']

          self.Constraints.append(sum(self.q_flow_B[idx] \
               for idx in self.LinesIn[bus_idx]['B']) - injection_q == \
             sum(self.q_flow_B[idx] for idx in self.LinesOut[bus_idx]['B']))

        if '3' in self.BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in self.EnergyConsumers and \
             'C' in self.EnergyConsumers[bus]['kW']:
            injection_q = self.EnergyConsumers[bus]['kVar']['C']

          self.Constraints.append(sum(self.q_flow_C[idx] \
               for idx in self.LinesIn[bus_idx]['C']) - injection_q == \
             sum(self.q_flow_C[idx] for idx in self.LinesOut[bus_idx]['C']))


  def constraintsNetworkWithVoltages(self, includeRegulatorsFlag):
    v_min, v_max = (0.95 * 2401.77) ** 2, (1.05 * 2401.77) ** 2
    for bus in self.BusInfo:
      bus_idx = self.BusInfo[bus]['idx']
      self.Constraints.append(self.v_A[bus_idx] >= v_min)
      self.Constraints.append(self.v_A[bus_idx] <= v_max)
      self.Constraints.append(self.v_B[bus_idx] >= v_min)
      self.Constraints.append(self.v_B[bus_idx] <= v_max)
      self.Constraints.append(self.v_C[bus_idx] >= v_min)
      self.Constraints.append(self.v_C[bus_idx] <= v_max)

    M = 1e9
    for branch in BranchInfo:
      # TODO NOTE: Feedback from Monish
      # We will need to define constraints in the case of it being a regulator
      # branch type, but with includeRegulatorsFlag==False where we have
      # no constraints at all currently. In this case we will need constraints
      # that have a constant value based on measurements in place of the
      # self.reg_taps optimization variable being used now.
      if BranchInfo[branch]['type']=='regulator' and includeRegulatorsFlag:
        if 'A' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.A']

          for k in range(32):
            self.Constraints.append(
                 self.v_A[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[BranchInfo[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 self.v_A[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_A[BranchInfo[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(idx, k)]) >= 0)

        if 'B' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.B']

          for k in range(32):
            self.Constraints.append(
                 self.v_B[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[BranchInfo[branch]['from_bus_idx']]\
                    - M * (1 - self.reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 self.v_B[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_B[BranchInfo[branch]['from_bus_idx']]\
                    + M * (1 - self.reg_taps[(idx, k)]) >= 0)

        if 'C' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.C']

          for k in range(32):
            self.Constraints.append(
                 self.v_C[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[BranchInfo[branch]['from_bus_idx']]\
                 - M * (1 - self.reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 self.v_C[BranchInfo[branch]['to_bus_idx']] - \
                 self.b_i[k]**2 * self.v_C[BranchInfo[branch]['from_bus_idx']]\
                 + M * (1 - self.reg_taps[(idx, k)]) >= 0)

      elif BranchInfo[branch]['type'] != 'regulator':
        zprim = BranchInfo[branch]['zprim']
        phases = BranchInfo[branch]['phases']
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

        fr_bus_idx = BranchInfo[branch]['from_bus_idx']
        to_bus_idx = BranchInfo[branch]['to_bus_idx']
        idx = BranchInfo[branch]['idx']
        hfsqrt3 = math.sqrt(3.0)/2.0

        self.Constraints.append(
            self.v_A[to_bus_idx] == self.v_A[fr_bus_idx] - \
            2.0*(self.p_flow_A[idx]*z_aa.real + self.q_flow_A[idx]*z_aa.imag + \
            self.p_flow_B[idx]*(-0.5*z_ab.real + hfsqrt3*z_ab.imag) + \
            self.q_flow_B[idx]*(-0.5*z_ab.imag - hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_ac.real - hfsqrt3*z_ac.imag) + \
            self.q_flow_C[idx]*(-0.5*z_ac.imag + hfsqrt3*z_ac.real)))

        self.Constraints.append(
            self.v_B[to_bus_idx] == self.v_B[fr_bus_idx] - \
            2.0*(self.p_flow_B[idx]*z_bb.real + self.q_flow_B[idx]*z_bb.imag + \
            self.p_flow_A[idx]*(-0.5*z_ab.real - hfsqrt3*z_ab.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ab.imag + hfsqrt3*z_ab.real) + \
            self.p_flow_C[idx]*(-0.5*z_bc.real + hfsqrt3*z_bc.imag) + \
            self.q_flow_C[idx]*(-0.5*z_bc.imag - hfsqrt3*z_bc.real)))

        self.Constraints.append(
            self.v_C[to_bus_idx] == self.v_C[fr_bus_idx] - \
            2.0*(self.p_flow_C[idx]*z_cc.real + self.q_flow_C[idx]*z_cc.imag + \
            self.p_flow_A[idx]*(-0.5*z_ac.real + hfsqrt3*z_ac.imag) + \
            self.q_flow_A[idx]*(-0.5*z_ac.imag - hfsqrt3*z_ac.real) + \
            self.p_flow_B[idx]*(-0.5*z_bc.real - hfsqrt3*z_bc.imag) + \
            self.q_flow_B[idx]*(-0.5*z_bc.imag + hfsqrt3*z_bc.real)))

    # fix source bus at 1.0
    sourcebus = self.EnergySource['bus']
    v_source = self.EnergySource['basev'] / math.sqrt(3)

    self.Constraints.append(self.v_A[self.BusInfo[sourcebus]['idx']] == v_source ** 2)
    self.Constraints.append(self.v_B[self.BusInfo[sourcebus]['idx']] == v_source ** 2)
    self.Constraints.append(self.v_C[self.BusInfo[sourcebus]['idx']] == v_source ** 2)


  def objectiveForResilience(self):
    # SHIVA magic scaling factor for SoC that causes the optmization to
    # come up with the correct results where -self.soc[i] doesn't.
    # Shiva will be investigating why this happens since we don't want
    # to be dependent on magic
    objective = sum(-100 * self.soc[i] for i in range(len(self.BatteriesInfo)))

    problem = cp.Problem(cp.Minimize(objective), self.Constraints)

    # problem.solve(solver=cp.MOSEK, verbose=True)
    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, verbose=False)
    print('Optimization status:', problem.status, flush=True)


  def objectiveForCVR(self):
    objective = sum((self.v_A[i] + self.v_B[i] + self.v_C[i]) for i in range(len(self.BusInfo)))

    problem = cp.Problem(cp.Minimize(objective), self.Constraints)

    # problem.solve(solver=cp.MOSEK, verbose=True)
    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, verbose=False)
    print('Optimization status:', problem.status, flush=True)


  def objectiveForDecarbonization(self):
    # constraints specific to decarbonization
    self.Constraints.append(self.Psub_mod >= self.Psub)

    self.Constraints.append(self.Psub_mod >= -self.Psub)

    flow_min, flow_max = -5e6, 5e6
    self.Constraints.append(self.Psub >= flow_min)
    self.Constraints.append(self.Psub <= flow_max)
    self.Constraints.append(self.Psub_mod >= flow_min)
    self.Constraints.append(self.Psub_mod <= flow_max)

    sub_flow_idx = self.EnergySource['flow_idx']
    self.Constraints.append(self.Psub == self.p_flow_A[sub_flow_idx] + \
                                         self.p_flow_B[sub_flow_idx] + \
                                         self.p_flow_C[sub_flow_idx])

    objective = self.Psub_mod / 1000

    problem = cp.Problem(cp.Minimize(objective), self.Constraints)

    # problem.solve(solver=cp.MOSEK, verbose=True)
    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, verbose=False)
    print('Optimization status:', problem.status, flush=True)

    # second stage only needed for decarbonization
    bus_idx_batt = {'A': [], 'B': [], 'C': []}
    for mrid in self.BatteriesInfo:
      idx = self.BatteriesIdx[mrid]
      self.Constraints.append(self.p_batt[idx] == self.p_batt[idx].value)
      bus = self.BatteriesInfo[mrid]['bus']
      if 'A' in self.BatteriesInfo[mrid]['phase']:
        bus_idx_batt['A'].append(self.BusInfo[bus]['idx'])
      elif 'B' in self.BatteriesInfo[mrid]['phase']:
        bus_idx_batt['B'].append(self.BusInfo[bus]['idx'])
      else:
        bus_idx_batt['C'].append(self.BusInfo[bus]['idx'])

    objective += -self.Psub_mod + \
                        sum(-self.v_A[i] for i in bus_idx_batt['A']) + \
                        sum(-self.v_B[i] for i in bus_idx_batt['B']) + \
                        sum(-self.v_C[i] for i in bus_idx_batt['C'])

    # TODO NOTE: Feedback from Monish
    # Need to figure out why with CVXPY we aren't able to solve for this
    # second stage optimization. Might need input from Shiva on this. For
    # the PuLP code it does do a second stage optimization with some different
    # settings so that's another clue.

    # problem = cp.Problem(cp.Minimize(objective), self.Constraints)
    # problem.solve(solver=cp.MOSEK)
    # print('Optimization Stage II status:', problem.status, flush=True)


  def reportOptimization(self, includeRegulatorsFlag, includeBatteriesFlag):
    if includeRegulatorsFlag:
      regulator_taps = []
      for reg in self.RegulatorsInfo:
        idx = self.RegulatorsInfo[reg]['idx']
        for k in range(32):
          if self.reg_taps[(idx, k)].value:
            # new value before old value for DifferenceBuilder
            self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                   k-16, None)
            regulator_taps.append([reg, k-16, self.b_i[k]])

            # set reg_greedy with every optimization based on measurements
            self.reg_greedy[idx] = k-16
            break # assume this will only happen once per regulator

      print(tabulate(regulator_taps, headers=['Regulator', 'Tap', 'b_i'],
                     tablefmt='psql'), '\n', flush=True)

    if includeBatteriesFlag:
      p_batt_setpoints = []
      for mrid in self.BatteriesInfo:
        idx = self.BatteriesIdx[mrid]
        self.BatteriesInfo[mrid]['SoC'] = self.soc[idx].value
        # new value before old value for DifferenceBuilder
        # note the optimized p_batt value is negated for the GridLAB-D
        # DifferenceBuilder message
        self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.p', -self.p_batt[idx].value, None)
        p_batt_setpoints.append([mrid, self.p_batt[idx].value/1000,
                                 self.soc[idx].value])

        # set p_batt_greedy with every optimization based on measurements
        self.p_batt_greedy[idx] = self.p_batt[idx].value

      print(tabulate(p_batt_setpoints, headers=['Battery', 'P_batt (kW)',
                     'Target SoC'], tablefmt='psql'), flush=True)

    if includeRegulatorsFlag or includeBatteriesFlag:
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
    for mrid in self.BatteriesInfo:
      measid = self.BatteriesInfo[mrid]['SoC_measid']
      if measid in measurements:
        self.BatteriesInfo[mrid]['SoC'] = measurements[measid]['value']/100.0
        print('Updated SoC for ' + self.BatteriesInfo[mrid]['name'] + ': ' + str(self.BatteriesInfo[mrid]['SoC']), flush=True)


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

    self.BatteriesInfo, self.BatteriesIdx = AppUtil.getBatteries(sparql_mgr)
    print('Starting BatteriesInfo: ' + json.dumps(self.BatteriesInfo, indent=2), flush=True)

    self.BatteriesObj = {}
    for mrid in self.BatteriesInfo:
      self.BatteriesObj[self.BatteriesInfo[mrid]['bus']] = {}
      self.BatteriesObj[self.BatteriesInfo[mrid]['bus']]['mrid'] = mrid
      self.BatteriesObj[self.BatteriesInfo[mrid]['bus']]['phase'] = self.BatteriesInfo[mrid]['phase']

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

    self.BusInfo = {}
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

      self.BusInfo[bus] = {}
      self.BusInfo[bus]['idx'] = idx
      self.BusInfo[bus]['phases'] = phases

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

    BranchInfo = {}

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

      BranchInfo[name] = {}
      BranchInfo[name]['idx'] = idx
      BranchInfo[name]['phases'] = phases
      BranchInfo[name]['type'] = 'line'
      BranchInfo[name]['from_bus'] = bus1
      BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus1]['idx']
      BranchInfo[name]['to_bus'] = bus2
      BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus2]['idx']
      #print(name + ': ' + str(BranchInfo[name]))
      #print(obj)
      idx += 1

    self.RegulatorsInfo, RegulatorsIdx = AppUtil.getCombineRegulators(sparql_mgr)

    print('RegulatorsInfo: ' + str(self.RegulatorsInfo), flush=True)
    print('RegulatorsIdx: ' + str(RegulatorsIdx), flush=True)

    bindings = sparql_mgr.power_transformer_connectivity_query()
    print('\nCount of PowerTransformers: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['xfmr_name']['value']
      bus = obj['bus']['value'].upper()
      print('PowerTransformer name: ' + name + ', bus: ' + bus, flush=True)
      #print(obj)

      if name not in BranchInfo:
        BranchInfo[name] = {}
        BranchInfo[name]['idx'] = idx
        BranchInfo[name]['phases'] = 'ABC'

        if 'RatioTapChanger.'+name in MethodUtil.NameToDevice and \
           MethodUtil.NameToDevice['RatioTapChanger.'+name] in self.RegulatorsInfo:
          BranchInfo[name]['type'] = 'regulator'
        else:
          BranchInfo[name]['type'] = 'transformer'
        BranchInfo[name]['from_bus'] = bus
        BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus]['idx']
      else:
        BranchInfo[name]['to_bus'] = bus
        BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus]['idx']
        print(name + ': ' + str(BranchInfo[name]))
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
        pname = self.RegulatorsInfo[mrid]['pname']
        if pname not in BranchInfo:
          BranchInfo[pname] = {}
          BranchInfo[pname]['idx'] = idx
          BranchInfo[pname]['phases'] = phase
          BranchInfo[pname]['type'] = 'regulator'
          BranchInfo[pname]['from_bus'] = bus
          BranchInfo[pname]['from_bus_idx'] = self.BusInfo[bus]['idx']
          idx += 1
        elif bus != BranchInfo[pname]['from_bus']:
          if phase not in BranchInfo[pname]['phases']:
            BranchInfo[pname]['phases'] += phase
          BranchInfo[pname]['to_bus'] = bus
          BranchInfo[pname]['to_bus_idx'] = self.BusInfo[bus]['idx']
          print(pname + ': ' + str(BranchInfo[pname]))

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
        BranchInfo[name] = {}
        BranchInfo[name]['idx'] = idx
        BranchInfo[name]['phases'] = phases
        BranchInfo[name]['type'] = 'line'
        BranchInfo[name]['from_bus'] = bus1
        BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus1]['idx']
        BranchInfo[name]['to_bus'] = bus2
        BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus2]['idx']
        print(name + ': ' + str(BranchInfo[name]))
        #print(obj)
        idx += 1

    # setup two dictionaries for quick lookup of incident line and
    # outgoing lines for any bus index
    self.LinesIn = {}
    self.LinesOut = {}
    n_line_phase = {}
    for branch in BranchInfo:
      if BranchInfo[branch]['to_bus_idx'] not in self.LinesIn:
        self.LinesIn[BranchInfo[branch]['to_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}
      if BranchInfo[branch]['from_bus_idx'] not in self.LinesOut:
        self.LinesOut[BranchInfo[branch]['from_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}

      phases = BranchInfo[branch]['phases']
      for char in phases:
        self.LinesIn[BranchInfo[branch]['to_bus_idx']][char].append(
                                                     BranchInfo[branch]['idx'])
        self.LinesOut[BranchInfo[branch]['from_bus_idx']][char].append(
                                                     BranchInfo[branch]['idx'])
        if char not in n_line_phase:
          n_line_phase[char] = 0
        n_line_phase[char] += 1

      # Identify the line emerging out from the source bus
      if BranchInfo[branch]['from_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = BranchInfo[branch]['idx']
      if BranchInfo[branch]['to_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = BranchInfo[branch]['idx']

      if BranchInfo[branch]['type'] == 'line':
        fr_bus = BranchInfo[branch]['from_bus']
        to_bus = BranchInfo[branch]['to_bus']
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

        BranchInfo[branch]['zprim'] = -1 * \
                            np.linalg.inv(ybus[np.ix_(fr_nodes, to_nodes)])

        #print('added line BranchInfo for: ' + branch + ', zprim: ' +
        #      str(BranchInfo[branch]['zprim']), flush=True)
      else:
        BranchInfo[branch]['zprim'] = np.zeros((3, 3), dtype=complex)
        #print('added non-line BranchInfo for: ' + branch + ', zprim: empty',
        #      flush=True)

    print('\nBranchInfo phase count: ' + str(n_line_phase), flush=True)

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

    self.defineOptimizationVariables(len(BranchInfo), len(self.BusInfo),
                                     len(self.BatteriesInfo), len(self.RegulatorsInfo))

    # GDB 8/25/23
    # Defining the part of the optimization problem that doesn't change with
    # each timestamp flies for PuLP, but not CVXPY. Based on how it sets up
    # the problem internally, it all needs to be redone each time.
    #self.defineOptimizationStaticProblem(BranchInfo, RegulatorsIdx)

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
        #print('Updated BatterySoC #' + str(messageCounter) + ': ' + json.dumps(self.BatteriesInfo, indent=2), flush=True)

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

          self.Constraints = []

          self.defineOptimizationStaticProblem(BranchInfo, RegulatorsIdx)

          self.defineOptimizationDynamicProblem(timestamp)

          self.doOptimization(timestamp, False)

      else: # this is a cooperation message from deconflictor
        # message consists of a target ResolutionVector that is a dictionary
        # with device mrid keys and target set-point values
        targetResolutionVector = message['targetResolutionVector']
        #for mrid in targetResolutionVector:
        #  print('DECONFLICTOR COOPERATE mrid ' + mrid + ' target set-point: ' + str(targetResolutionVector[mrid]), flush=True)

        for mrid in self.BatteriesInfo:
          if mrid in targetResolutionVector:
            idx = self.BatteriesIdx[mrid]
            self.p_batt_proposed[idx] = -targetResolutionVector[mrid][1]

        for reg in self.RegulatorsInfo:
          if reg in targetResolutionVector:
            idx = self.RegulatorsInfo[reg]['idx']
            self.reg_proposed[idx] = targetResolutionVector[reg][1]

        # Need to define the full optimization problem each time anything
        # changes for CVXPY to be happy
        # GDB 9/9/24: Can't do a new optimization for cooperation because
        # the objective function is non-linear/non-convex so we have an
        # alternative workflow implementation for supporting cooperation in
        # order to meet the FY24 deconfliction service deliverable
        '''
        self.defineOptimizationStaticProblem(BranchInfo, RegulatorsIdx)

        self.defineOptimizationDynamicProblem(timestamp)

        self.doOptimization(timestamp, True)
        '''

        print('DECONFLICTOR COOPERATE p_batt_greedy: ' + str(self.p_batt_greedy), flush=True)
        print('DECONFLICTOR COOPERATE p_batt_proposed: ' + str(self.p_batt_proposed), flush=True)

        # GDB 9/10/24: Here is the alternative support for cooperation via
        # ranking the differences between proposed and greedy setpoints:
        # first, create a list of differences
        len_BatteriesInfo = len(self.BatteriesInfo)
        p_batt_diff = [None] * len_BatteriesInfo
        for i in range(len_BatteriesInfo):
          p_batt_diff[i] = abs(self.p_batt_greedy[i] - self.p_batt_proposed[i])

        print('DECONFLICTOR COOPERATE p_batt_diff: ' + str(p_batt_diff), flush=True)

        # omit any setpoints where proposed == greeedy
        p_batt_sort = []
        for i in range(len_BatteriesInfo):
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

        for i in range(len_BatteriesInfo):
          # check if this is a "cooperating" battery
          if p_batt_diff[i] <= diffMax:
            # if so, set it to the proposed value
            p_batt_coop[i] = self.p_batt_proposed[i]

        print('DECONFLICTOR COOPERATE p_batt_coop: ' + str(p_batt_coop), flush=True)

        # now do the same for regulators
        print('DECONFLICTOR COOPERATE reg_greedy: ' + str(self.reg_greedy), flush=True)
        print('DECONFLICTOR COOPERATE reg_proposed: ' + str(self.reg_proposed), flush=True)

        len_RegulatorsInfo = len(self.RegulatorsInfo)
        reg_diff = [None] * len_RegulatorsInfo
        for i in range(len_RegulatorsInfo):
          reg_diff[i] = abs(self.reg_greedy[i] - self.reg_proposed[i])

        print('DECONFLICTOR COOPERATE reg_diff: ' + str(reg_diff), flush=True)

        # omit any setpoints where proposed == greeedy
        reg_sort = []
        for i in range(len_RegulatorsInfo):
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

        for i in range(len_RegulatorsInfo):
          # check if this is a "cooperating" regulator
          if reg_diff[i] <= diffMax:
            # if so, set it to the proposed value
            reg_coop[i] = self.reg_proposed[i]

        print('DECONFLICTOR COOPERATE reg_coop: ' + str(reg_coop), flush=True)

        # finally, send out the cooperation setpoints via DifferenceBuilder msg
        for reg in self.RegulatorsInfo:
          idx = self.RegulatorsInfo[reg]['idx']
          # new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                 reg_coop[idx], None)

        for mrid in self.BatteriesInfo:
          idx = self.BatteriesIdx[mrid]
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

