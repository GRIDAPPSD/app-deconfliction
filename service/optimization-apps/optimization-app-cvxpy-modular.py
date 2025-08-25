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
#import cylp
import cvxpy as cp
import pandas as pd

from gridappsd import GridAPPSD
from gridappsd import DifferenceBuilder
from gridappsd.topics import simulation_input_topic, simulation_output_topic
from gridappsd.topics import simulation_log_topic
from gridappsd.topics import service_input_topic, service_output_topic

from datetime import datetime
from tabulate import tabulate

# suppress warnings about overriding objective function from max_local
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

  def optPrelimScalability(self, line):
    tokens = line.split(',')

    self.app_name = tokens[0].strip() + '-app'
    print('\nScalability app_name: ' + self.app_name, flush=True)

    objWeights = tokens[1].strip()
    # objWeights is of form: [<obj1> <obj2> <obj3> ...]
    if objWeights.startswith('['):
      objWeights = objWeights[1:]
    if objWeights.endswith(']'):
      objWeights = objWeights[:-1]
    weights = objWeights.split()

    # this is the original implementation to use the existing resilience,
    # CVR, and max_local objectives for scalability testing
    '''
    self.objectiveResilienceFlag = False
    self.objectiveCVRFlag = False
    self.objectiveMaxLocalFlag = False

    if float(weights[0]) > 0.0:
      self.objectiveResilienceFlag = True
      self.opt_type = 'resilience'
    elif float(weights[1]) > 0.0:
      self.objectiveCVRFlag = True
      self.opt_type = 'cvr'
    elif float(weights[2]) > 0.0:
      self.objectiveMaxLocalFlag = True
      self.opt_type = 'max_local'

    # default to resilience if nothing is set
    if not (self.objectiveResilienceFlag or self.objectiveCVRFlag or \
            self.objectiveMaxLocalFlag):
      self.objectiveResilienceFlag = True
      self.opt_type = 'resilience'

    print('Scalability objective: ' + self.opt_type, flush=True)
    '''

    # this is the new way
    self.objectiveWeights = []
    for wt in weights:
      if float(wt) > 0.0:
        self.objectiveWeights.append(float(wt))
      else:
        self.objectiveWeights.append(None)

    print('Scalability objective weights: ' + str(self.objectiveWeights),
          flush=True)

    self.includeEnergyConsumersFlag = bool(int(tokens[2]))
    self.includeSolarPVsPFlag = bool(int(tokens[3]))
    self.includeSolarPVsQFlag = bool(int(tokens[4]))
    self.includeBatteriesFlag = bool(int(tokens[5]))
    self.includeRegulatorsFlag = bool(int(tokens[6]))
    self.includePFlowFlag = bool(int(tokens[7]))
    self.includeQFlowFlag = bool(int(tokens[8]))
    self.includeVoltagesFlag = bool(int(tokens[9]))

    print('Scalability include EnergyConsumers: ' +
          str(self.includeEnergyConsumersFlag), flush=True)
    print('Scalability include SolarPVs Active: ' +
          str(self.includeSolarPVsPFlag), flush=True)
    print('Scalability include SolarPVs Reactive: ' +
          str(self.includeSolarPVsQFlag), flush=True)
    print('Scalability include Batteries: ' +
          str(self.includeBatteriesFlag), flush=True)
    print('Scalability include Regulators: ' +
          str(self.includeRegulatorsFlag), flush=True)
    print('Scalability include PFlow: ' +
          str(self.includePFlowFlag), flush=True)
    print('Scalability include QFlow: ' +
          str(self.includeQFlowFlag), flush=True)
    print('Scalability include Voltages: ' +
          str(self.includeVoltagesFlag), flush=True)


  def optPrelimClassic(self):
    self.includeBatteriesFlag = True
    self.includeRegulatorsFlag = True

    # does it make sense to exclude any of these?
    self.includeEnergyConsumersFlag = True
    self.includeSolarPVsPFlag = True
    self.includeSolarPVsQFlag = True

    self.includePFlowFlag = True
    self.includeQFlowFlag = True
    self.includeVoltagesFlag = True

    self.objectiveResilienceFlag = False
    self.objectiveCVRFlag = False
    self.objectiveMaxLocalFlag = False
    if self.opt_type == 'resilience':
      self.objectiveResilienceFlag = True
    elif self.opt_type == 'cvr':
      self.objectiveCVRFlag = True
    elif self.opt_type == 'max_local':
      self.objectiveMaxLocalFlag = True

    self.app_name = self.opt_type + '-app'

    # make sure only a single objective is specified
    if self.objectiveResilienceFlag:
      self.objectiveCVRFlag = False
      self.objectiveMaxLocalFlag = False
    elif self.objectiveCVRFlag:
      self.objectiveResilienceFlag = False
      self.objectiveMaxLocalFlag = False
    elif self.objectiveMaxLocalFlag:
      self.objectiveResilienceFlag = False
      self.objectiveCVRFlag = False

    # enforce dependencies so the optimization is well-defined
    if self.objectiveResilienceFlag:
      self.includeBatteriesFlag = True

    if self.objectiveCVRFlag:
      self.includeVoltagesFlag = True

    if self.objectiveMaxLocalFlag:
      self.includeBatteriesFlag = True
      self.includeVoltagesFlag = True

    if self.includeBatteriesFlag:
      self.includePFlowFlag = True

    if self.includeRegulatorsFlag:
      self.includeVoltagesFlag = True

    if self.includeVoltagesFlag:
      self.includePFlowFlag = True
      self.includeQFlowFlag = True


  def optPerform(self):
    self.Constraints = []

    if self.includeBatteriesFlag:
      self.optConstraintsDERWithBatteries(self.BatteriesInfo, self.deltaT,
                                          self.soc, self.p_batt,
                                          self.p_batt_c, self.p_batt_d,
                                          self.lambda_c, self.lambda_d)
    else:
      ## Adding some battery limits to ensure feasiblity when battery is not being used
      self.optConstraintsLimitsBatteries(self.BatteriesInfo, self.soc, self.p_batt)

    if self.includeRegulatorsFlag:
      self.optConstraintsDERWithRegulators(self.RegulatorsInfo, self.reg_taps)

    if self.includeSolarPVsPFlag:
      self.optConstraintsDERWithSolarPVs(self.SolarPVsInfo,
                                         self.includeSolarPVsQFlag,
                                         self.p_pv_A, self.p_pv_B, self.p_pv_C,
                                         self.q_pv_A, self.q_pv_B, self.q_pv_C)

    if self.includePFlowFlag:
      self.optConstraintsNetworkWithPFlow(self.includeBatteriesFlag,
                     self.includeEnergyConsumersFlag, self.includeSolarPVsPFlag,
                     self.BusInfo, self.LinesIn, self.LinesOut,
                     self.EnergyConsumers, self.SolarPVsInfo,
                     self.BatteriesBus, self.BatteriesInfo, self.p_batt,
                     self.p_flow_A, self.p_flow_B, self.p_flow_C,
                     self.p_pv_A, self.p_pv_B, self.p_pv_C)

    if self.includeQFlowFlag:
      self.optConstraintsNetworkWithQFlow(self.includeEnergyConsumersFlag,
                     self.includeSolarPVsPFlag, self.BusInfo, self.LinesIn,
                     self.LinesOut, self.EnergyConsumers, self.SolarPVsInfo,
                     self.q_flow_A, self.q_flow_B, self.q_flow_C,
                     self.q_pv_A, self.q_pv_B, self.q_pv_C)


    if self.includeVoltagesFlag:
      # depending on whether solving for regulator tap positions is part of the
      # optimization, either pass in CVXPY self.reg_taps variable to specify
      # voltage constraints or pass in the self.meas_reg_taps dictionary that
      # holds the current tap positions set from simulation measurements
      if self.includeRegulatorsFlag:
        self.optConstraintsNetworkWithVoltages(self.BusInfo, self.BranchInfo,
           self.RegulatorsIdx, self.EnergySource, self.b_i, self.reg_taps,
           self.p_flow_A, self.p_flow_B, self.p_flow_C,
           self.q_flow_A, self.q_flow_B, self.q_flow_C,
           self.v_A, self.v_B, self.v_C)
      else:
        # print('current regulator taps: {}'.format(self.meas_reg_taps))
        self.optConstraintsNetworkWithVoltages(self.BusInfo, self.BranchInfo,
           self.RegulatorsIdx, self.EnergySource, self.b_i, self.meas_reg_taps,
           self.p_flow_A, self.p_flow_B, self.p_flow_C,
           self.q_flow_A, self.q_flow_B, self.q_flow_C,
           self.v_A, self.v_B, self.v_C)

    validFlag = True

    if self.opt_type == 'scalability':
      objective = 0
      numWeights = len(self.objectiveWeights)

      if numWeights>0 and self.objectiveWeights[0]!=None:
        objective += self.objectiveWeights[0] * self.optObjective1(self.BusInfo, self.SolarPVsInfo,
                                                self.v_A, self.v_B, self.v_C,
                                                self.p_pv_A, self.p_pv_B, self.p_pv_C)

      if numWeights>1 and self.objectiveWeights[1]!=None:
        objective += self.objectiveWeights[1] * self.optObjective2(
                                    self.EnergySource, self.Psub, self.Psub_mod,
                                    self.Qsub, self.Qsub_mod,
                                    self.p_flow_A, self.p_flow_B, self.p_flow_C,
                                    self.q_flow_A, self.q_flow_B, self.q_flow_C)

      if numWeights>2 and self.objectiveWeights[2]!=None:
        objective += self.objectiveWeights[2] * self.optObjective3( self.SolarPVsInfo, self.BatteriesInfo,
                                                                    self.p_pv_A, self.p_pv_B, self.p_pv_C, self.p_batt)

      if numWeights>3 and self.objectiveWeights[3]!=None:


        objective += self.objectiveWeights[3] * self.optObjective4( self.EnergySource, self.Psub,
                                                                    self.Psub_mod, self.p_flow_A,
                                                                    self.p_flow_B, self.p_flow_C)

      if numWeights>4 and self.objectiveWeights[4]!=None:
        objective += self.objectiveWeights[4] * self.optObjective5(
                                                   self.BatteriesInfo, self.soc)

    else:
      if self.objectiveResilienceFlag:
        objective = self.optObjectiveForResilience(self.BatteriesInfo, self.soc)

      if self.objectiveCVRFlag:
        objective = self.optObjectiveForCVR(self.BusInfo,
                                            self.v_A, self.v_B, self.v_C)

      if self.objectiveMaxLocalFlag:
        # note max_local is a two stage optimization and the first stage
        # is run within the objectiveForMaxLocal function
        validFlag, objective = self.optObjectiveForMaxLocal(self.BusInfo,
                                    self.BatteriesInfo, self.EnergySource,
                                    self.Psub, self.Psub_mod,
                                    self.p_flow_A, self.p_flow_B, self.p_flow_C,
                                    self.p_batt, self.v_A, self.v_B, self.v_C)

    if validFlag and self.optDo(objective):
      self.optDispatch(self.includeRegulatorsFlag, self.includeBatteriesFlag,
                       self.includeSolarPVsPFlag, self.includeVoltagesFlag)


  def optDefineVariables(self, includePFlowFlag, includeQFlowFlag,
                         includeVoltagesFlag, includeBatteriesFlag,
                         includeRegulatorsFlag, includeSolarPVsPFlag):

    # if includePFlowFlag:
    len_BranchInfo = len(self.BranchInfo)
    self.p_flow_A = cp.Variable(len_BranchInfo, integer=False,name='p_flow_A')
    self.p_flow_B = cp.Variable(len_BranchInfo, integer=False,name='p_flow_B')
    self.p_flow_C = cp.Variable(len_BranchInfo, integer=False,name='p_flow_C')

    # if includeQFlowFlag:
    len_BranchInfo = len(self.BranchInfo)
    self.q_flow_A = cp.Variable(len_BranchInfo, integer=False,name='q_flow_A')
    self.q_flow_B = cp.Variable(len_BranchInfo, integer=False,name='q_flow_B')
    self.q_flow_C = cp.Variable(len_BranchInfo, integer=False,name='q_flow_C')

    # if includeVoltagesFlag:
    len_BusInfo = len(self.BusInfo)
    self.v_A = cp.Variable(len_BusInfo, integer=False, name='v_A')
    self.v_B = cp.Variable(len_BusInfo, integer=False, name='v_B')
    self.v_C = cp.Variable(len_BusInfo, integer=False, name='v_C')

    # if includeBatteriesFlag:
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
    else:
      # if not including regulators in optimization problem then we need a
      # dictionary to track the current tap position from measurements
      self.meas_reg_taps = {}

    # if includeSolarPVsPFlag:
    len_SolarPVsInfo = len(self.SolarPVsInfo)
    self.p_pv_A = cp.Variable(len_SolarPVsInfo, integer=False,name='p_pv_A')
    self.p_pv_B = cp.Variable(len_SolarPVsInfo, integer=False,name='p_pv_B')
    self.p_pv_C = cp.Variable(len_SolarPVsInfo, integer=False,name='p_pv_C')
    self.q_pv_A = cp.Variable(len_SolarPVsInfo, integer=False,name='q_pv_A')
    self.q_pv_B = cp.Variable(len_SolarPVsInfo, integer=False,name='q_pv_B')
    self.q_pv_C = cp.Variable(len_SolarPVsInfo, integer=False,name='q_pv_C')

      # cooperation variables
      # since these are held constant, I don't need to define them with
      # cp.Variable calls, but as fixed length vectors. It's still convenient
      # though to define them along with the other optimization variables.
    self.pq_pv_proposed = [None] * len_SolarPVsInfo
    self.pq_pv_greedy = [None] * len_SolarPVsInfo

    self.Psub = cp.Variable(integer=False, name='P_sub')
    self.Psub_mod = cp.Variable(integer=False, name='P_sub_mod')
    self.Qsub = cp.Variable(integer=False, name='Q_sub')
    self.Qsub_mod = cp.Variable(integer=False, name='Q_sub_mod')


  def optConstraintsDERWithBatteries(self, BatteriesInfo, deltaT, soc, p_batt,
                                     p_batt_c, p_batt_d, lambda_c, lambda_d):
    for mrid in BatteriesInfo:
      BatteriesInfo[mrid]['state'] = 'idling'
      idx = BatteriesInfo[mrid]['idx']
      self.Constraints.append(
              soc[idx] == BatteriesInfo[mrid]['SoC'] + \
              BatteriesInfo[mrid]['eff'] * p_batt_c[idx] * \
              deltaT / BatteriesInfo[mrid]['ratedE'] + \
              1 / BatteriesInfo[mrid]['eff'] * p_batt_d[idx] * \
              deltaT / BatteriesInfo[mrid]['ratedE'])

      self.Constraints.append(p_batt_c[idx] >= 0)
      self.Constraints.append(p_batt_c[idx] <= \
              lambda_c[idx] * BatteriesInfo[mrid]['prated'])

      self.Constraints.append(p_batt_d[idx] <= 0)
      self.Constraints.append(p_batt_d[idx] >= \
              -lambda_d[idx] * BatteriesInfo[mrid]['prated'])

      self.Constraints.append(p_batt[idx] == p_batt_c[idx] + p_batt_d[idx])
      self.Constraints.append(lambda_c[idx] + lambda_d[idx] <= 1)

      # Battery SoC constraints added as Shiva couldn't identify CVXPY's
      # equivalent to PuLP's lb and ub
      # OPTDBG: tweak SoC limits so the optimization isn't infeasible
      self.Constraints.append(soc[idx] >= 0.2)
      self.Constraints.append(soc[idx] <= 0.9)
      #self.Constraints.append(soc[idx] >= 0.0)
      #self.Constraints.append(soc[idx] <= 1.0)

  def optConstraintsLimitsBatteries(self, BatteriesInfo, soc, p_batt):
    print('Setting Some arbitrary limits for batteries')
    for mrid in BatteriesInfo:
      BatteriesInfo[mrid]['state'] = 'idling'
      idx = BatteriesInfo[mrid]['idx']
      self.Constraints.append(p_batt[idx] == 0)
      # Battery SoC constraints added as Shiva couldn't identify CVXPY's
      # equivalent to PuLP's lb and ub
      self.Constraints.append(soc[idx] >= 0.2)
      self.Constraints.append(soc[idx] <= 0.9)


  def optConstraintsDERWithRegulators(self, RegulatorsInfo, reg_taps):
    for k in range(len(RegulatorsInfo)):
      self.Constraints.append(sum(reg_taps[(k, tap)] for tap in range(32)) == 1)

    # For some reason CVXPY fails to print the regulator taps unless
    # substation regulator tap is fixed--for now fixing it to zero position
    self.Constraints.append(reg_taps[(0, 16)] == 1)


  def optConstraintsDERWithSolarPVs(self, SolarPVsInfo, includeSolarPVsQFlag,
                                    p_pv_A, p_pv_B, p_pv_C,
                                    q_pv_A, q_pv_B, q_pv_C):
    for bus in SolarPVsInfo:
      idx = SolarPVsInfo[bus]['idx']

      numphases = len(SolarPVsInfo[bus]['phase'])
      if 'N' in SolarPVsInfo[bus]['phase']:
        numphases -= 1

      ratedS = SolarPVsInfo[bus]['ratedS']/numphases
      ratedP = SolarPVsInfo[bus]['p']/numphases

      coeff = math.sqrt(2) - 1 ### Coefficient for Octagon Constraints

      if 'A' in SolarPVsInfo[bus]['phase']:
        self.Constraints.append(p_pv_A[idx] <= ratedP)
        self.Constraints.append(p_pv_A[idx] >= 0)

        if includeSolarPVsQFlag:
          self.Constraints.append(q_pv_A[idx] <= ratedS)
          self.Constraints.append(q_pv_A[idx] >= -ratedS)
          self.Constraints.append(coeff*p_pv_A[idx] + q_pv_A[idx] <= ratedS)
          self.Constraints.append(coeff*p_pv_A[idx] - q_pv_A[idx] <= ratedS)
          self.Constraints.append(p_pv_A[idx] + coeff*q_pv_A[idx] <= ratedS)
          self.Constraints.append(p_pv_A[idx] - coeff*q_pv_A[idx] <= ratedS)
        else:
          self.Constraints.append(q_pv_A[idx] == 0)


      if 'B' in SolarPVsInfo[bus]['phase']:
        self.Constraints.append(p_pv_B[idx] <= ratedP)
        self.Constraints.append(p_pv_B[idx] >= 0)

        if includeSolarPVsQFlag:
          self.Constraints.append(q_pv_B[idx] <=  ratedS)
          self.Constraints.append(q_pv_B[idx] >= -ratedS)
          self.Constraints.append(coeff*p_pv_B[idx] + q_pv_B[idx] <= ratedS)
          self.Constraints.append(coeff*p_pv_B[idx] - q_pv_B[idx] <= ratedS)
          self.Constraints.append(p_pv_B[idx] + coeff*q_pv_B[idx] <= ratedS)
          self.Constraints.append(p_pv_B[idx] - coeff*q_pv_B[idx] <= ratedS)
        else:
          self.Constraints.append(q_pv_B[idx] == 0)

      if 'C' in SolarPVsInfo[bus]['phase']:
        self.Constraints.append(p_pv_C[idx] <= ratedP)
        self.Constraints.append(p_pv_C[idx] >= 0)

        if includeSolarPVsQFlag:
          self.Constraints.append(q_pv_C[idx] <=  ratedS)
          self.Constraints.append(q_pv_C[idx] >= -ratedS)
          self.Constraints.append(coeff*p_pv_C[idx] + q_pv_C[idx] <= ratedS)
          self.Constraints.append(coeff*p_pv_C[idx] - q_pv_C[idx] <= ratedS)
          self.Constraints.append(p_pv_C[idx] + coeff*q_pv_C[idx] <= ratedS)
          self.Constraints.append(p_pv_C[idx] - coeff*q_pv_C[idx] <= ratedS)
        else:
          self.Constraints.append(q_pv_C[idx] == 0)

  def optConstraintsNetworkWithPFlow(self, includeBatteriesFlag,
         includeEnergyConsumersFlag, includeSolarPVsPFlag,
         BusInfo, LinesIn, LinesOut, EnergyConsumers, SolarPVsInfo,
         BatteriesBus, BatteriesInfo, p_batt, p_flow_A, p_flow_B, p_flow_C,
         p_pv_A, p_pv_B, p_pv_C):
    for bus in BusInfo:
      bus_idx = BusInfo[bus]['idx']
      if bus_idx not in LinesOut:
        LinesOut[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in LinesIn: # check for source bus
        if '1' in BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'A' in EnergyConsumers[bus]['kW']:
            injection_p = EnergyConsumers[bus]['kW']['A']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'A' in SolarPVsInfo[bus]['phase']:
            #injection_p -= SolarPVsInfo[bus]['p']
            idx = SolarPVsInfo[bus]['idx']
            injection_p -= p_pv_A[idx]
            #print('SolarPVsInfo A bus: ' + bus + ', value: ' +
            #      str(SolarPVsInfo[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in BatteriesBus and \
             'A' in BatteriesBus[bus]['phase']:
            #print('Batteries A bus: ' + bus, flush=True)
            mrid = BatteriesBus[bus]['mrid']
            self.Constraints.append(sum(p_flow_A[idx] \
                 for idx in LinesIn[bus_idx]['A']) - \
               p_batt[BatteriesInfo[mrid]['idx']] - injection_p == \
               sum(p_flow_A[idx] for idx in LinesOut[bus_idx]['A']))

          else:
            self.Constraints.append(sum(p_flow_A[idx] \
                 for idx in LinesIn[bus_idx]['A']) - injection_p == \
               sum(p_flow_A[idx] for idx in LinesOut[bus_idx]['A']))

        if '2' in BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'B' in EnergyConsumers[bus]['kW']:
            injection_p = EnergyConsumers[bus]['kW']['B']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'B' in SolarPVsInfo[bus]['phase']:
            idx = SolarPVsInfo[bus]['idx']
            injection_p -= p_pv_B[idx]
            #print('SolarPVsInfo B bus: ' + bus + ', value: ' +
            #      str(SolarPVsInfo[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in BatteriesBus and \
             'B' in BatteriesBus[bus]['phase']:
            #print('Batteries B bus: ' + bus, flush=True)
            mrid = BatteriesBus[bus]['mrid']
            self.Constraints.append(sum(p_flow_B[idx] \
                 for idx in LinesIn[bus_idx]['B']) - \
               p_batt[BatteriesInfo[mrid]['idx']] - injection_p == \
               sum(p_flow_B[idx] for idx in LinesOut[bus_idx]['B']))

          else:
            self.Constraints.append(sum(p_flow_B[idx] \
                 for idx in LinesIn[bus_idx]['B']) - injection_p == \
               sum(p_flow_B[idx] for idx in LinesOut[bus_idx]['B']))

        if '3' in BusInfo[bus]['phases']:
          injection_p = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'C' in EnergyConsumers[bus]['kW']:
            injection_p = EnergyConsumers[bus]['kW']['C']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'C' in SolarPVsInfo[bus]['phase']:
            idx = SolarPVsInfo[bus]['idx']
            injection_p -= p_pv_C[idx]
            #print('SolarPVsInfo C bus: ' + bus + ', value: ' +
            #      str(SolarPVsInfo[bus]['p']), flush=True)

          if includeBatteriesFlag and bus in BatteriesBus and \
             'C' in BatteriesBus[bus]['phase']:
            #print('Batteries C bus: ' + bus, flush=True)
            mrid = BatteriesBus[bus]['mrid']
            self.Constraints.append(sum(p_flow_C[idx] \
                 for idx in LinesIn[bus_idx]['C']) - \
               p_batt[BatteriesInfo[mrid]['idx']] - injection_p == \
               sum(p_flow_C[idx] for idx in LinesOut[bus_idx]['C']))

          else:
            self.Constraints.append(sum(p_flow_C[idx] \
                 for idx in LinesIn[bus_idx]['C']) - injection_p == \
               sum(p_flow_C[idx] for idx in LinesOut[bus_idx]['C']))


  def optConstraintsNetworkWithQFlow(self, includeEnergyConsumersFlag,
                                     includeSolarPVsPFlag, BusInfo,
                                     LinesIn, LinesOut,
                                     EnergyConsumers, SolarPVsInfo,
                                     q_flow_A, q_flow_B, q_flow_C,
                                     q_pv_A, q_pv_B, q_pv_C):
    for bus in BusInfo:
      bus_idx = BusInfo[bus]['idx']
      if bus_idx not in LinesOut:
        LinesOut[bus_idx] = {'A': [], 'B': [], 'C': []}

      if bus_idx in LinesIn: # check for source bus
        if '1' in BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'A' in EnergyConsumers[bus]['kW']:
            injection_q = EnergyConsumers[bus]['kVar']['A']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'A' in SolarPVsInfo[bus]['phase']:
            idx = SolarPVsInfo[bus]['idx']
            injection_q -= q_pv_A[idx]

          self.Constraints.append(sum(q_flow_A[idx] \
               for idx in LinesIn[bus_idx]['A']) - injection_q == \
             sum(q_flow_A[idx] for idx in LinesOut[bus_idx]['A']))

        if '2' in BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'B' in EnergyConsumers[bus]['kW']:
            injection_q = EnergyConsumers[bus]['kVar']['B']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'B' in SolarPVsInfo[bus]['phase']:
            idx = SolarPVsInfo[bus]['idx']
            injection_q -= q_pv_B[idx]

          self.Constraints.append(sum(q_flow_B[idx] \
               for idx in LinesIn[bus_idx]['B']) - injection_q == \
             sum(q_flow_B[idx] for idx in LinesOut[bus_idx]['B']))

        if '3' in BusInfo[bus]['phases']:
          injection_q = 0
          if includeEnergyConsumersFlag and bus in EnergyConsumers and \
             'C' in EnergyConsumers[bus]['kW']:
            injection_q = EnergyConsumers[bus]['kVar']['C']

          if includeSolarPVsPFlag and bus in SolarPVsInfo and \
             'C' in SolarPVsInfo[bus]['phase']:
            idx = SolarPVsInfo[bus]['idx']
            injection_q -= q_pv_C[idx]


          self.Constraints.append(sum(q_flow_C[idx] \
               for idx in LinesIn[bus_idx]['C']) - injection_q == \
             sum(q_flow_C[idx] for idx in LinesOut[bus_idx]['C']))


  def optConstraintsNetworkWithVoltages(self, BusInfo, BranchInfo,
                               RegulatorsIdx, EnergySource, b_i, reg_taps,
                               p_flow_A, p_flow_B, p_flow_C,
                               q_flow_A, q_flow_B, q_flow_C, v_A, v_B, v_C):
    v_min, v_max = (0.9 * 2401.77) ** 2, (1.1 * 2401.77) ** 2
    for bus in BusInfo:
      bus_idx = BusInfo[bus]['idx']
      self.Constraints.append(v_A[bus_idx] >= v_min)
      self.Constraints.append(v_A[bus_idx] <= v_max)
      self.Constraints.append(v_B[bus_idx] >= v_min)
      self.Constraints.append(v_B[bus_idx] <= v_max)
      self.Constraints.append(v_C[bus_idx] >= v_min)
      self.Constraints.append(v_C[bus_idx] <= v_max)

    M = 1e9
    for branch in BranchInfo:
      # TODO NOTE: Feedback from Monish
      # We will need to define constraints in the case of it being a regulator
      # branch type, but with includeRegulatorsFlag==False where we have
      # no constraints at all currently. In this case we will need constraints
      # that have a constant value based on measurements in place of the
      # reg_taps optimization variable being used now.
      if BranchInfo[branch]['type']=='regulator':
        if 'A' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.A']

          for k in range(32):
            self.Constraints.append(
                 v_A[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_A[BranchInfo[branch]['from_bus_idx']]\
                 - M * (1 - reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 v_A[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_A[BranchInfo[branch]['from_bus_idx']]\
                 + M * (1 - reg_taps[(idx, k)]) >= 0)

        if 'B' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.B']

          for k in range(32):
            self.Constraints.append(
                 v_B[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_B[BranchInfo[branch]['from_bus_idx']]\
                    - M * (1 - reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 v_B[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_B[BranchInfo[branch]['from_bus_idx']]\
                    + M * (1 - reg_taps[(idx, k)]) >= 0)

        if 'C' in BranchInfo[branch]['phases']:
          idx = RegulatorsIdx[branch+'.C']

          for k in range(32):
            self.Constraints.append(
                 v_C[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_C[BranchInfo[branch]['from_bus_idx']]\
                 - M * (1 - reg_taps[(idx, k)]) <= 0)

            self.Constraints.append(
                 v_C[BranchInfo[branch]['to_bus_idx']] - \
                 b_i[k]**2 * v_C[BranchInfo[branch]['from_bus_idx']]\
                 + M * (1 - reg_taps[(idx, k)]) >= 0)

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
            v_A[to_bus_idx] == v_A[fr_bus_idx] - \
            2.0*(p_flow_A[idx]*z_aa.real + q_flow_A[idx]*z_aa.imag + \
            p_flow_B[idx]*(-0.5*z_ab.real + hfsqrt3*z_ab.imag) + \
            q_flow_B[idx]*(-0.5*z_ab.imag - hfsqrt3*z_ab.real) + \
            p_flow_C[idx]*(-0.5*z_ac.real - hfsqrt3*z_ac.imag) + \
            q_flow_C[idx]*(-0.5*z_ac.imag + hfsqrt3*z_ac.real)))

        self.Constraints.append(
            v_B[to_bus_idx] == v_B[fr_bus_idx] - \
            2.0*(p_flow_B[idx]*z_bb.real + q_flow_B[idx]*z_bb.imag + \
            p_flow_A[idx]*(-0.5*z_ab.real - hfsqrt3*z_ab.imag) + \
            q_flow_A[idx]*(-0.5*z_ab.imag + hfsqrt3*z_ab.real) + \
            p_flow_C[idx]*(-0.5*z_bc.real + hfsqrt3*z_bc.imag) + \
            q_flow_C[idx]*(-0.5*z_bc.imag - hfsqrt3*z_bc.real)))

        self.Constraints.append(
            v_C[to_bus_idx] == v_C[fr_bus_idx] - \
            2.0*(p_flow_C[idx]*z_cc.real + q_flow_C[idx]*z_cc.imag + \
            p_flow_A[idx]*(-0.5*z_ac.real + hfsqrt3*z_ac.imag) + \
            q_flow_A[idx]*(-0.5*z_ac.imag - hfsqrt3*z_ac.real) + \
            p_flow_B[idx]*(-0.5*z_bc.real - hfsqrt3*z_bc.imag) + \
            q_flow_B[idx]*(-0.5*z_bc.imag + hfsqrt3*z_bc.real)))

    # fix source bus at 1.0
    sourcebus = EnergySource['bus']
    v_source = EnergySource['basev'] / math.sqrt(3)

    self.Constraints.append(v_A[BusInfo[sourcebus]['idx']] == v_source ** 2)
    self.Constraints.append(v_B[BusInfo[sourcebus]['idx']] == v_source ** 2)
    self.Constraints.append(v_C[BusInfo[sourcebus]['idx']] == v_source ** 2)


  def optObjectiveForResilience(self, BatteriesInfo, soc):
    # SHIVA magic scaling factor for SoC that causes the optmization to
    # come up with the correct results where -soc[i] doesn't.
    # Shiva will be investigating why this happens since we don't want
    # to be dependent on magic
    objective = sum(-100 * soc[i] for i in range(len(BatteriesInfo)))
    return objective


  def optObjectiveForCVR(self, BusInfo, v_A, v_B, v_C):
    objective = sum((v_A[i] + v_B[i] + v_C[i]) for i in range(len(BusInfo)))
    return objective


  def optObjectiveForMaxLocal(self, BusInfo, BatteriesInfo, EnergySource,
                              Psub, Psub_mod, p_flow_A, p_flow_B, p_flow_C,
                              p_batt, v_A, v_B, v_C):
    # constraints specific to max_local
    self.Constraints.append(Psub_mod >= Psub)
    self.Constraints.append(Psub_mod >= -Psub)

    flow_min, flow_max = -5e6, 5e6
    self.Constraints.append(Psub >= flow_min)
    self.Constraints.append(Psub <= flow_max)
    self.Constraints.append(Psub_mod >= flow_min)
    self.Constraints.append(Psub_mod <= flow_max)

    sub_flow_idx = EnergySource['flow_idx']
    self.Constraints.append(Psub == p_flow_A[sub_flow_idx] + \
                                    p_flow_B[sub_flow_idx] + \
                                    p_flow_C[sub_flow_idx])

    # originally Psub_mod was scaled by 1000 to solve an "unbounded" error
    # with some version of CVXPY, but now I'm seeing it run fine without that
    # scaling and I don't like the mismatch with Psub_mod on the second stage
    # optmization so I'm going to go back to no scaling. The PuLP version
    # never had scaling.
    #objective = Psub_mod / 1000
    objective = Psub_mod

    if self.optDo(objective):
      # second stage for max_local
      bus_idx_batt = {'A': [], 'B': [], 'C': []}
      for mrid in BatteriesInfo:
        idx = BatteriesInfo[mrid]['idx']
        self.Constraints.append(p_batt[idx] == p_batt[idx].value)
        bus = BatteriesInfo[mrid]['bus']
        if 'A' in BatteriesInfo[mrid]['phase']:
          bus_idx_batt['A'].append(BusInfo[bus]['idx'])
        elif 'B' in BatteriesInfo[mrid]['phase']:
          bus_idx_batt['B'].append(BusInfo[bus]['idx'])
        else:
          bus_idx_batt['C'].append(BusInfo[bus]['idx'])

      objective += -Psub_mod + \
                          sum(-v_A[i] for i in bus_idx_batt['A']) + \
                          sum(-v_B[i] for i in bus_idx_batt['B']) + \
                          sum(-v_C[i] for i in bus_idx_batt['C'])
      return (True, objective)

    else:
      return (False, objective)


  def optObjective1(self, BusInfo, SolarPVsInfo, v_A, v_B, v_C, p_pv_A, p_pv_B, p_pv_C):
    print('Adding Objective 1 for CVR at time {}'.format(ts_time))
    objective = sum((v_A[i] + v_B[i] + v_C[i]) for i in range(len(BusInfo))) / ((2401.77 ** 2) * (123*3))
    #### Adding additional term to minimize active power curtailment
    objective_pv = 0
    for bus in SolarPVsInfo:
      idx = SolarPVsInfo[bus]['idx']
      if 'A' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_A[idx]
      if 'B' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_B[idx]
      if 'C' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_C[idx]

    objective -= (objective_pv)/1000000
    return objective


  def optObjective2(self, EnergySource, Psub, Psub_mod, Qsub, Qsub_mod,
                    p_flow_A, p_flow_B, p_flow_C, q_flow_A, q_flow_B, q_flow_C):
    print('Adding Objective 2 for PF at time {}'.format(ts_time))
    self.Constraints.append(Psub_mod >= Psub)
    self.Constraints.append(Psub_mod >= -Psub)

    self.Constraints.append(Qsub_mod >= Qsub)
    self.Constraints.append(Qsub_mod >= -Qsub)

    flow_min, flow_max = -5e6, 5e6
    self.Constraints.append(Psub >= flow_min)
    self.Constraints.append(Psub <= flow_max)
    self.Constraints.append(Psub_mod >= flow_min)
    self.Constraints.append(Psub_mod <= flow_max)

    sub_flow_idx = EnergySource['flow_idx']
    self.Constraints.append(Psub == p_flow_A[sub_flow_idx] + \
                                    p_flow_B[sub_flow_idx] + \
                                    p_flow_C[sub_flow_idx])
    self.Constraints.append(Qsub == q_flow_A[sub_flow_idx] + \
                                    q_flow_B[sub_flow_idx] + \
                                    q_flow_C[sub_flow_idx])

    ####### simplified implementation of power factor #######
    objective = (Qsub_mod - Psub_mod) / 2000000
    return objective


  def optObjective3(self, SolarPVsInfo, BatteriesInfo, p_pv_A, p_pv_B, p_pv_C, p_batt):
    cost = pd.read_csv('lmp_data.csv')
    cost['time'] = pd.to_datetime(cost['time'])
    ts_target = pd.to_datetime(str(ts_time))
    idx_cost = abs(cost['time'] - ts_target).idxmin()
    cost = cost['price'].values/1000
    average_cost = np.mean(cost)
    cost_now = cost[idx_cost]

    print('Adding Objective 3 for Arbitrage at time {} with current and average price {}, {}'.format(ts_target, cost_now, average_cost))
    objective_batt = sum((cost_now-average_cost) * p_batt[i] for i in range(len(BatteriesInfo)))/1000

    objective_pv = 0
    for bus in SolarPVsInfo:
      idx = SolarPVsInfo[bus]['idx']
      if 'A' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_A[idx]
      if 'A' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_B[idx]
      if 'B' in SolarPVsInfo[bus]['phase']:
        objective_pv += p_pv_C[idx]

    objective_pv = -1 * cost_now * objective_pv /1000
    objective = (objective_batt + objective_pv) * self.deltaT

    return objective


  def optObjective4(self, EnergySource, Psub, Psub_mod, p_flow_A, p_flow_B, p_flow_C):

    print('Adding Objective 4 for Peak Load at time {}'.format(ts_time))
    target_peak = 2e6
    self.Constraints.append(Psub_mod >= Psub - target_peak)
    self.Constraints.append(Psub_mod >= -Psub + target_peak)

    flow_min, flow_max = -5e6, 5e6
    self.Constraints.append(Psub >= flow_min)
    self.Constraints.append(Psub <= flow_max)
    self.Constraints.append(Psub_mod >= flow_min)
    self.Constraints.append(Psub_mod <= flow_max)

    sub_flow_idx = EnergySource['flow_idx']
    self.Constraints.append(Psub == p_flow_A[sub_flow_idx] + \
                                    p_flow_B[sub_flow_idx] + \
                                    p_flow_C[sub_flow_idx])

    # originally Psub_mod was scaled by 1000 to solve an "unbounded" error
    # with some version of CVXPY, but now I'm seeing it run fine without that
    # scaling and I don't like the mismatch with Psub_mod on the second stage
    # optmization so I'm going to go back to no scaling. The PuLP version
    # never had scaling.
    #objective = Psub_mod / 1000
    objective = Psub_mod /1000000

    return objective


  def optObjective5(self, BatteriesInfo, soc):
    print('Adding Objective 5 for Resilience at time {}'.format(ts_time))
    objective = sum(-100 * soc[i] for i in range(len(BatteriesInfo))) / (100)
    return objective


  def optDo(self, objective):
    problem = cp.Problem(cp.Minimize(objective), self.Constraints)

    startTime = datetime.now()
    #problem.solve(solver=cp.MOSEK, verbose=True) # commercial solver
    #problem.solve(solver=cp.CBC, verbose=False)
    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, verbose=False)
    print('Optimization status:', problem.status, flush=True)
    print('Optimization Value:', problem.value, flush=True)
    now = datetime.now()
    optTime = (now - startTime).total_seconds()
    optInterval= (now - self.lastTime).total_seconds()
    self.lastTime = now
    print('Optimization time: ' + str(optTime), flush=True)
    print('Optimization time interval: ' + str(optInterval), flush=True)
    print('OPTDBG: Optimization status: ' + problem.status, flush=True)

    return (problem.status  == 'optimal')


  def optDispatch(self, includeRegulatorsFlag, includeBatteriesFlag,
                  includeSolarPVsPFlag, includeVoltagesFlag):

    if includeVoltagesFlag:
      # volt_sum = sum((self.v_A[i].value + self.v_B[i].value + self.v_C[i].value) for i in range(len(self.BusInfo))) / (2401.77 ** 2)
      volt_sum = sum((self.v_A[i].value + self.v_B[i].value + self.v_C[i].value) for i in range(len(self.BusInfo))) / ((2401.77 ** 2) * (123 * 3))
      print("Optimized sum of Voltages: {}".format(volt_sum))

    if includeRegulatorsFlag:
      regulator_taps = []
      for reg in self.RegulatorsInfo:
        idx = self.RegulatorsInfo[reg]['idx']
        name = self.RegulatorsInfo[reg]['name']
        for k in range(32):
          if self.reg_taps[(idx, k)].value:
            # new value before old value for DifferenceBuilder
            self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                   k-16, None)
            regulator_taps.append([name, k-16, self.b_i[k]])

            # set reg_greedy with every optimization based on measurements
            self.reg_greedy[idx] = k-16
            break # assume this will only happen once per regulator

      print(tabulate(regulator_taps, headers=['Regulator', 'Tap', 'b_i'],
                     tablefmt='psql'), '\n', flush=True)

    if includeBatteriesFlag:
      p_batt_setpoints = []
      for mrid in self.BatteriesInfo:
        idx = self.BatteriesInfo[mrid]['idx']
        name = self.BatteriesInfo[mrid]['name']
        self.BatteriesInfo[mrid]['SoC'] = self.soc[idx].value
        # new value before old value for DifferenceBuilder
        # note the optimized p_batt value is negated for the GridLAB-D
        # DifferenceBuilder message
        self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.p', -self.p_batt[idx].value, None)
        p_batt_setpoints.append([name, self.p_batt[idx].value/1000,
                                 self.soc[idx].value])

        # set p_batt_greedy with every optimization based on measurements
        self.p_batt_greedy[idx] = self.p_batt[idx].value

      print(tabulate(p_batt_setpoints, headers=['Battery', 'P_batt (kW)',
                     'Target SoC'], tablefmt='psql'), flush=True)

    if includeSolarPVsPFlag:
      pq_pv_setpoints = []
      for bus in self.SolarPVsInfo:
        idx = self.SolarPVsInfo[bus]['idx']
        mrid = self.SolarPVsInfo[bus]['mrid']
        name = self.SolarPVsInfo[bus]['name']

        total_p = self.p_pv_A[idx].value + self.p_pv_B[idx].value + \
                  self.p_pv_C[idx].value
        self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.p', -total_p, None)
        total_q = self.q_pv_A[idx].value + self.q_pv_B[idx].value + \
                  self.q_pv_C[idx].value
        self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.q', -total_q, None)

        pq_pv_setpoints.append([name, bus, total_p/1000, total_q/1000])

        # set pq_pv_greedy with every optimization based on measurements
        self.pq_pv_greedy[idx] = complex(total_p, total_q)

      print(tabulate(pq_pv_setpoints, headers=['SolarPV', 'bus', 'Total p (kW)',
                     'Total q (kW)'], tablefmt='psql'), flush=True)

    '''
    if self.includePFlowFlag:
      if self.objectiveMaxLocalFlag:
        print('')
        print('Psub: ' + str(self.Psub.value), flush=True)
        print('Psub_mod: ' + str(self.Psub_mod.value), flush=True)
      print('')
      for i in range(len(self.BranchInfo)):
        print('p_flow[' + str(i) + '] A: ' + str(self.p_flow_A[i].value) + ', B: ' + str(self.p_flow_B[i].value) + ', C: ' + str(self.p_flow_C[i].value), flush=True)
      print('')
    '''

    if includeRegulatorsFlag or includeBatteriesFlag or includeSolarPVsPFlag:
      dispatch_message = self.difference_builder.get_message()
      dispatch_message['app_name'] = self.app_name
      print('Sending Measurements DifferenceBuilder message!', flush=True)
      #print('Sending Measurements DifferenceBuilder message: ' +
      #      json.dumps(dispatch_message), flush=True)

      # these can go either to the simulation or the deconfliction pipeline
      # based on the deconflictionAsServiceFlag value
      self.gapps.send(self.sim_publish_topic, json.dumps(dispatch_message))

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
    for bus in self.SolarPVsInfo:
      measid = self.SolarPVsInfo[bus]['measid']
      if measid in measurements:
        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])
        self.SolarPVsInfo[bus]['p'] = abs(p)

  def updateBatterySoC(self, measurements):
    for mrid in self.BatteriesInfo:
      measid = self.BatteriesInfo[mrid]['SoC_measid']
      if measid in measurements:
        self.BatteriesInfo[mrid]['SoC'] = measurements[measid]['value']/100.0
        print('Updated SoC for ' + self.BatteriesInfo[mrid]['name'] + ': ' + str(self.BatteriesInfo[mrid]['SoC']), flush=True)
        print('OPTDBG: updateBatterySoC mrid: ' + mrid + ', SoC: ' + str(self.BatteriesInfo[mrid]['SoC']), flush=True)


  def updateRegulatorTaps(self, measurements):
    for mrid in self.RegulatorsInfo:
      measid = self.RegulatorsInfo[mrid]['measid']
      if measid in measurements:
        # find the index associated with the regulator
        idx = self.RegulatorsInfo[mrid]['idx']
        # zero out all the 32 positions and then update to the one set
        for k in range(32):
          self.meas_reg_taps[(idx, k)] = 0

        pos = int(measurements[measid]['value'])
        # measurement tap position is -16 to +15 so need to offset by 16 for
        # the proper position index for the optimization problem
        self.meas_reg_taps[(idx, pos+16)] = 1
        print('Updated Tap for ' + self.RegulatorsInfo[mrid]['name'] + ': ' + str(pos), flush=True)


  def __init__(self, gapps, opt_type, feeder_mrid, simulation_id, interval):

    self.realtimeFlag = True
    #self.realtimeFlag = False
    self.realtimeFlag = False

    self.gapps = gapps

    self.messageQueue = queue.Queue()

    # subscribe to simulation log and output messages
    # since messages are just going on a queue, subscribe right away to
    # keep from missing any sent during app initialization
    self.keepLoopingFlag = True
    out_id = gapps.subscribe(simulation_output_topic(simulation_id), self)
    log_id = gapps.subscribe(simulation_log_topic(simulation_id), self)
    coop_id = gapps.subscribe(service_output_topic('deconfliction.cooperation',
                              simulation_id), self)

    SPARQLManager = getattr(importlib.import_module('sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.EnergyConsumers = AppUtil.getEnergyConsumers(sparql_mgr)
    #print('Starting EnergyConsumers: ' + json.dumps(self.EnergyConsumers, indent=2), flush=True)

    self.SolarPVsInfo, self.SolarPVs = AppUtil.getSolarPVs(sparql_mgr)
    #print('Starting SolarPVsInfo: ' + json.dumps(self.SolarPVsInfo, indent=2), flush=True)

    self.BatteriesInfo, self.BatteriesBus = AppUtil.getBatteries(sparql_mgr)
    # print('Starting BatteriesInfo: ' + json.dumps(self.BatteriesInfo, indent=2), flush=True)

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

    self.BranchInfo = {}

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

      self.BranchInfo[name] = {}
      self.BranchInfo[name]['idx'] = idx
      self.BranchInfo[name]['phases'] = phases
      self.BranchInfo[name]['type'] = 'line'
      self.BranchInfo[name]['from_bus'] = bus1
      self.BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus1]['idx']
      self.BranchInfo[name]['to_bus'] = bus2
      self.BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus2]['idx']
      #print(name + ': ' + str(self.BranchInfo[name]))
      #print(obj)
      idx += 1

    self.RegulatorsInfo, self.RegulatorsIdx = AppUtil.getCombineRegulators(sparql_mgr)

    # Need a way to map from a measid to the mrid for regulators in order
    # to process tap position changes in new measurements
    RegsForMeasID = AppUtil.getRegulators(sparql_mgr)
    for mrid in self.RegulatorsInfo:
      if mrid in RegsForMeasID:
        self.RegulatorsInfo[mrid]['measid'] = RegsForMeasID[mrid]['measid']

    print('RegulatorsInfo: ' + str(self.RegulatorsInfo), flush=True)
    print('RegulatorsIdx: ' + str(self.RegulatorsIdx), flush=True)

    bindings = sparql_mgr.power_transformer_connectivity_query()
    print('\nCount of PowerTransformers: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['xfmr_name']['value']
      bus = obj['bus']['value'].upper()
      print('PowerTransformer name: ' + name + ', bus: ' + bus, flush=True)
      #print(obj)

      if name not in self.BranchInfo:
        self.BranchInfo[name] = {}
        self.BranchInfo[name]['idx'] = idx
        self.BranchInfo[name]['phases'] = 'ABC'

        if 'RatioTapChanger.'+name in MethodUtil.NameToDevice and \
           MethodUtil.NameToDevice['RatioTapChanger.'+name] in self.RegulatorsInfo:
          self.BranchInfo[name]['type'] = 'regulator'
        else:
          self.BranchInfo[name]['type'] = 'transformer'
        self.BranchInfo[name]['from_bus'] = bus
        self.BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus]['idx']
      else:
        self.BranchInfo[name]['to_bus'] = bus
        self.BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus]['idx']
        print(name + ': ' + str(self.BranchInfo[name]))
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
        if pname not in self.BranchInfo:
          self.BranchInfo[pname] = {}
          self.BranchInfo[pname]['idx'] = idx
          self.BranchInfo[pname]['phases'] = phase
          self.BranchInfo[pname]['type'] = 'regulator'
          self.BranchInfo[pname]['from_bus'] = bus
          self.BranchInfo[pname]['from_bus_idx'] = self.BusInfo[bus]['idx']
          idx += 1
        elif bus != self.BranchInfo[pname]['from_bus']:
          if phase not in self.BranchInfo[pname]['phases']:
            self.BranchInfo[pname]['phases'] += phase
          self.BranchInfo[pname]['to_bus'] = bus
          self.BranchInfo[pname]['to_bus_idx'] = self.BusInfo[bus]['idx']
          print(pname + ': ' + str(self.BranchInfo[pname]))

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
        self.BranchInfo[name] = {}
        self.BranchInfo[name]['idx'] = idx
        self.BranchInfo[name]['phases'] = phases
        self.BranchInfo[name]['type'] = 'line'
        self.BranchInfo[name]['from_bus'] = bus1
        self.BranchInfo[name]['from_bus_idx'] = self.BusInfo[bus1]['idx']
        self.BranchInfo[name]['to_bus'] = bus2
        self.BranchInfo[name]['to_bus_idx'] = self.BusInfo[bus2]['idx']
        print(name + ': ' + str(self.BranchInfo[name]))
        #print(obj)
        idx += 1

    # setup two dictionaries for quick lookup of incident line and
    # outgoing lines for any bus index
    self.LinesIn = {}
    self.LinesOut = {}
    n_line_phase = {}
    for branch in self.BranchInfo:
      if self.BranchInfo[branch]['to_bus_idx'] not in self.LinesIn:
        self.LinesIn[self.BranchInfo[branch]['to_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}
      if self.BranchInfo[branch]['from_bus_idx'] not in self.LinesOut:
        self.LinesOut[self.BranchInfo[branch]['from_bus_idx']] = \
                                         {'A': [], 'B': [], 'C': []}

      phases = self.BranchInfo[branch]['phases']
      for char in phases:
        self.LinesIn[self.BranchInfo[branch]['to_bus_idx']][char].append(
                                                 self.BranchInfo[branch]['idx'])
        self.LinesOut[self.BranchInfo[branch]['from_bus_idx']][char].append(
                                                 self.BranchInfo[branch]['idx'])
        if char not in n_line_phase:
          n_line_phase[char] = 0
        n_line_phase[char] += 1

      # Identify the line emerging out from the source bus
      if self.BranchInfo[branch]['from_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = self.BranchInfo[branch]['idx']
      if self.BranchInfo[branch]['to_bus'] == self.EnergySource['bus']:
        self.EnergySource['flow_idx'] = self.BranchInfo[branch]['idx']

      if self.BranchInfo[branch]['type'] == 'line':
        fr_bus = self.BranchInfo[branch]['from_bus']
        to_bus = self.BranchInfo[branch]['to_bus']
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

        self.BranchInfo[branch]['zprim'] = -1 * \
                            np.linalg.inv(ybus[np.ix_(fr_nodes, to_nodes)])

        #print('added line BranchInfo for: ' + branch + ', zprim: ' +
        #      str(self.BranchInfo[branch]['zprim']), flush=True)
      else:
        self.BranchInfo[branch]['zprim'] = np.zeros((3, 3), dtype=complex)
        #print('added non-line BranchInfo for: ' + branch + ', zprim: empty',
        #      flush=True)

    print('\nBranchInfo phase count: ' + str(n_line_phase), flush=True)

    if opt_type.startswith('r') or opt_type.startswith('R'):
      self.opt_type = 'resilience'
    elif opt_type.startswith('m') or opt_type.startswith('M'):
      self.opt_type = 'max_local'
    elif opt_type.startswith('c') or opt_type.startswith('C'):
      self.opt_type = 'cvr'
    elif opt_type.startswith('s') or opt_type.startswith('S'):
      self.opt_type = 'scalability'
    else:
      print('*** Exiting due to unrecognized optimization type: ' + opt_type,
            flush=True)
      exit()

    # deltaT is time between timesteps as fractional hours
    # optimization interval seconds is the number of simulation seconds
    # between triggering an optimization and must be a multiple of 3
    # for a real-time simulation
    if self.realtimeFlag:
      #optIntervalSec = 3 # optimize every GridLAB-D timestamp
      # 15 seconds is a good number for a real-time simulation
      optIntervalSec = 15
    else:
      # if attempting non-real-time, something like 600 is reasonable
      # OPTDBG: add fudge factor to optIntervalSec for deltaT
      optIntervalSec = 600
      #optIntervalSec = 600 + 300

    if self.opt_type!='scalability' and interval!=None:
      optIntervalSec = int(interval)

    self.deltaT = optIntervalSec/3600.0

    self.b_i = np.arange(0.9, 1.1, 0.00625)

    if self.opt_type == 'scalability':
      # the interval value is actually the app_setup.csv line
      self.optPrelimScalability(interval)
    else:
      self.optPrelimClassic()

    self.optDefineVariables(self.includePFlowFlag, self.includeQFlowFlag,
                          self.includeVoltagesFlag, self.includeBatteriesFlag,
                          self.includeRegulatorsFlag, self.includeSolarPVsPFlag)

    # topic for sending out cooperation responses
    self.coop_publish_topic = service_input_topic('deconfliction.cooperation',
                                                  simulation_id)

    # determine whether to send directly to simulation or the deconfliction
    # pipeline
    #deconflictionAsServiceFlag = False
    # GDB 8/25/25: Set as service just to send to simulation for debugging
    # outside of running deconfliction pipeline
    deconflictionAsServiceFlag = True
    if deconflictionAsServiceFlag:
      self.sim_publish_topic = simulation_input_topic(simulation_id)
    else:
      self.sim_publish_topic = service_input_topic('deconfliction.measurements',
                                                   simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

    print('\nInitialized modularized ' + opt_type +
          ' CVXPY optimization competing app, waiting for messages...\n',
          flush=True)

    messageCounter = 0
    currentCoopPhase = None
    self.lastTime = datetime.now()

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

        if self.includeEnergyConsumersFlag:
          self.updateEnergyConsumers(message['measurements'])
          #print('Updated EnergyConsumers #' + str(messageCounter) + ': ' + json.dumps(self.EnergyConsumers, indent=2), flush=True)

        if self.includeSolarPVsPFlag:
          self.updateSolarPVs(message['measurements'])
          #print('Updated SolarPVsInfo #' + str(messageCounter) + ': ' + json.dumps(self.SolarPVsInfo, indent=2), flush=True)

        if self.includeBatteriesFlag:
          self.updateBatterySoC(message['measurements'])
          #print('Updated BatterySoC #' + str(messageCounter) + ': ' + json.dumps(self.BatteriesInfo, indent=2), flush=True)

        # tap positions only need to be tracked when not solving for the
        # positions as part of the optimization problem
        if not self.includeRegulatorsFlag:
          self.updateRegulatorTaps(message['measurements'])

        global ts_time
        ts_unix = int(message['timestamp'])
        ts_time = datetime.utcfromtimestamp(ts_unix).time()
        print('OPTDBG: timestamp: ' + str(ts_unix) + ', wall time: ' + str(ts_time), flush=True)

        # If doing real-time simulation must subtract 5 off timestamp to make it
        # evenly divisble by multiples of the 3 second GridLAB-D time interval
        skipFlag = False
        if self.realtimeFlag:
          skipFlag = (ts_unix-5) % optIntervalSec != 0
        else:
          # If doing non-real-time simulation remove the 5 second offset because
          # GridLAB-D outputs at 60 second intervals
          skipFlag = ts_unix % optIntervalSec != 0

        if skipFlag:
          print('Simulation timestamp (skipping optimization): '+str(ts_time), flush=True)
        else:
          print('Simulation timestamp for optimization: ' + str(ts_time), flush=True)
          self.optPerform()

      elif self.includeBatteriesFlag or self.includeRegulatorsFlag:
        # this is a cooperation message from deconflictor, but it only
        # makes sense to do anything if there are batteries and/or regulators
        # as part of the optimization where cooperation is being attempted

        # message consists of a target ResolutionVector that is a dictionary
        # with device mrid keys and target set-point values
        targetResolutionVector = message['targetResolutionVector']

        # except for SolarPVs the set-point values are tuples and they are
        # easier to work with as complex numbers so do that translation now
        for mrid, value in targetResolutionVector.items():
          # I create tuples for the complex SolarPV setpoints for serialization,
          # but JSON serializes those as lists so the reverse deserialization
          # needs to check for lists rather than tuples
          if isinstance(value[1], list):
            targetResolutionVector[mrid] = (value[0],
                                            complex(value[1][0], value[1][1]))

        #for mrid in targetResolutionVector:
        #  print('DECONFLICTOR COOPERATE mrid ' + mrid + ' target set-point: ' + str(targetResolutionVector[mrid]), flush=True)

        # coopCounter allows diminishing cooperation with each succeeding
        # solicitation within a phase
        coopPhase = message['coop_phase']
        if coopPhase == currentCoopPhase:
          # comment out incrementing coopCounter to not diminish cooperation
          coopCounter += 1
        else:
          currentCoopPhase = coopPhase
          coopCounter = 0

        if self.includeBatteriesFlag:
          for mrid in self.BatteriesInfo:
            if mrid in targetResolutionVector:
              idx = self.BatteriesInfo[mrid]['idx']
              self.p_batt_proposed[idx] = -targetResolutionVector[mrid][1]

        if self.includeRegulatorsFlag:
          for reg in self.RegulatorsInfo:
            if reg in targetResolutionVector:
              idx = self.RegulatorsInfo[reg]['idx']
              self.reg_proposed[idx] = targetResolutionVector[reg][1]

        if self.includeSolarPVsPFlag:
          for mrid in self.SolarPVs:
            if mrid in targetResolutionVector:
              idx = self.SolarPVs[mrid]['idx']
              self.pq_pv_proposed[idx] = -targetResolutionVector[mrid][1]

        # Need to define the full optimization problem each time anything
        # changes for CVXPY to be happy
        # GDB 9/9/24: Can't do a new optimization for cooperation because
        # the objective function is non-linear/non-convex so we have an
        # alternative workflow implementation for supporting cooperation in
        # order to meet the FY24 deconfliction service deliverable
        '''
        self.optPerform()
        '''

        print('DECONFLICTOR COOPERATE p_batt_greedy: ' + str(self.p_batt_greedy), flush=True)
        print('DECONFLICTOR COOPERATE p_batt_proposed: ' + str(self.p_batt_proposed), flush=True)

        # GDB 9/10/24: Here is the alternative support for cooperation via
        # ranking the differences between proposed and greedy setpoints:
        if self.includeBatteriesFlag:
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

          # GDB 3/25/25: Handle the case of only proposed == greedy
          diffMax = 0
          if len(p_batt_sort) > 0:
            coopCount = max(1, -(len(p_batt_sort)//-2)) # integer "ceiling" division

            # find the value associated with the last "cooperating" battery
            diffMax = p_batt_sort[coopCount-1]

            print('DECONFLICTOR COOPERATE batteries coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
          else:
            print('DECONFLICTOR COOPERATE batteries coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

          p_batt_denom = [] # just for diagnostic logging
          for i in range(len_BatteriesInfo):
            # check if this is a "cooperating" battery
            if p_batt_diff[i]>0 and p_batt_diff[i]<=diffMax:
              # full cooperation by setting the greedy value to proposed value
              #self.p_batt_greedy[i] = self.p_batt_proposed[i]
              # adjust cooperation level based on difference
              # find which entry this p_batt_diff is within p_batt_sort to
              # determine how much to cooperate. This is tricky code in that
              # a loop iterator varible is referenced after the loop.
              for ic in range(len(p_batt_sort)):
                if p_batt_diff[i] == p_batt_sort[ic]:
                  break
              fcoop = float(ic/2.0) + 1.0 # more cooperation
              #fcoop = float(ic/1.5) + 1.0 # in-between cooperation
              #fcoop = float(ic/1.0) + 1.0 # less cooperation

              ratio = (self.p_batt_proposed[i] - self.p_batt_greedy[i])/ \
                      float(fcoop + coopCounter)
              self.p_batt_greedy[i] += ratio
              p_batt_denom.append((fcoop, coopCounter))
            else:
              p_batt_denom.append(None)

          print('DECONFLICTOR COOPERATE p_batt_coop: ' + str(self.p_batt_greedy), flush=True)
          print('DECONFLICTOR COOPERATE p_batt_denom: ' + str(p_batt_denom), flush=True)

          for mrid in self.BatteriesInfo:
            idx = self.BatteriesInfo[mrid]['idx']
            # new value before old value for DifferenceBuilder
            # note the p_batt value is negated for the GridLAB-D
            # DifferenceBuilder message
            self.difference_builder.add_difference(mrid,
                 'PowerElectronicsConnection.p', -self.p_batt_greedy[idx], None)

        if self.includeSolarPVsPFlag:
          print('DECONFLICTOR COOPERATE pq_pv_greedy: ' + str(self.pq_pv_greedy), flush=True)
          print('DECONFLICTOR COOPERATE pq_pv_proposed: ' + str(self.pq_pv_proposed), flush=True)
          len_SolarPVsInfo = len(self.SolarPVsInfo)
          pq_pv_diff = [None] * len_SolarPVsInfo
          for i in range(len_SolarPVsInfo):
            # note this is the same difference code for SolarPVs as the others
            # even though the greedy and proposed vectors are complex
            pq_pv_diff[i] = abs(self.pq_pv_greedy[i] - self.pq_pv_proposed[i])

          print('DECONFLICTOR COOPERATE pq_pv_diff: ' + str(pq_pv_diff), flush=True)

          # omit any setpoints where proposed == greeedy
          pq_pv_sort = []
          for i in range(len_SolarPVsInfo):
            if pq_pv_diff[i] > 0:
              pq_pv_sort.append(pq_pv_diff[i])

          # sorts in place
          pq_pv_sort.sort()

          # handle the case of only proposed == greedy
          diffMax = 0
          if len(pq_pv_sort) > 0:
            coopCount = max(1, -(len(pq_pv_sort)//-2)) # integer "ceiling" division

            # find the value associated with the last "cooperating" battery
            diffMax = pq_pv_sort[coopCount-1]

            print('DECONFLICTOR COOPERATE solarPVs coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
          else:
            print('DECONFLICTOR COOPERATE solarPVs coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

          pq_pv_denom = [] # just for diagnostic logging
          for i in range(len_SolarPVsInfo):
            # check if this is a "cooperating" solarPV
            if pq_pv_diff[i]>0 and pq_pv_diff[i]<=diffMax:
              # full cooperation by setting the greedy value to proposed value
              #self.pq_pv_greedy[i] = self.pq_pv_proposed[i]
              # adjust cooperation level based on difference
              # find which entry this p_batt_diff is within p_batt_sort to
              # determine how much to cooperate. This is tricky code in that
              # a loop iterator varible is referenced after the loop.
              for ic in range(len(pq_pv_sort)):
                if pq_pv_diff[i] == pq_pv_sort[ic]:
                  break
              fcoop = float(ic/2.0) + 1.0 # more cooperation
              #fcoop = float(ic/1.5) + 1.0 # in-between cooperation
              #fcoop = float(ic/1.0) + 1.0 # less cooperation

              # again, these are complex numbers, but division by a scalar
              # is done to each of them giving a complex result that is then
              # added to the original complex number. This is equivalent to
              # breaking up the work into the real and imag components.
              ratio = (self.pq_pv_proposed[i] - self.pq_pv_greedy[i])/ \
                      float(fcoop + coopCounter)
              self.pq_pv_greedy[i] += ratio
              pq_pv_denom.append((fcoop, coopCounter))
            else:
              pq_pv_denom.append(None)

          print('DECONFLICTOR COOPERATE pq_pv_coop: ' + str(self.pq_pv_greedy), flush=True)
          print('DECONFLICTOR COOPERATE pq_pv_denom: ' + str(pq_pv_denom), flush=True)

          for mrid in self.SolarPVs:
            idx = self.SolarPVs[mrid]['idx']
            # new value before old value for DifferenceBuilder
            # note the p and q values are negated for the GridLAB-D
            # DifferenceBuilder message
            self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.p', -self.pq_pv_greedy[idx].real, None)
            self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.q', -self.pq_pv_greedy[idx].imag, None)

        if self.includeRegulatorsFlag:
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

          # GDB 3/25/25: Handle the case of only proposed == greedy
          diffMax = 0
          if len(reg_sort) > 0:
            # determine the number of regulators that will "cooperate"
            coopCount = max(1, -(len(reg_sort)//-2)) # integer "ceiling" division

            # find the value associated with the last "cooperating" regulator
            diffMax = reg_sort[coopCount-1]

            print('DECONFLICTOR COOPERATE regulators coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
          else:
            print('DECONFLICTOR COOPERATE regulators coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

          reg_denom = [] # just for diagnostic logging
          for i in range(len_RegulatorsInfo):
            # check if this is a "cooperating" regulator
            if reg_diff[i]>0 and reg_diff[i]<=diffMax:
              # full cooperation by setting the greedy value to proposed value
              #self.reg_greedy[i] = self.reg_proposed[i]
              # adjust cooperation level based on difference
              # find which entry this p_batt_diff is within p_batt_sort to
              # determine how much to cooperate. This is tricky code in that
              # a loop iterator varible is referenced after the loop.
              for ic in range(len(reg_sort)):
                if reg_diff[i] == reg_sort[ic]:
                  break
              fcoop = float(ic/2.0) + 1.0 # more cooperation
              #fcoop = float(ic/1.5) + 1.0 # in-between cooperation
              #fcoop = float(ic/1.0) + 1.0 # less cooperation

              ratio = int((self.reg_proposed[i] - self.reg_greedy[i])/ \
                          (fcoop + coopCounter))
              self.reg_greedy[i] += ratio
              reg_denom.append((fcoop, coopCounter))
            else:
              reg_denom.append(None)

          print('DECONFLICTOR COOPERATE reg_coop: ' + str(self.reg_greedy), flush=True)
          print('DECONFLICTOR COOPERATE reg_denom: ' + str(reg_denom), flush=True)

          for reg in self.RegulatorsInfo:
            idx = self.RegulatorsInfo[reg]['idx']
            # new value before old value for DifferenceBuilder
            self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                   self.reg_greedy[idx], None)

        # finally, send out the cooperation setpoints via DifferenceBuilder msg
        dispatch_message = self.difference_builder.get_message()
        dispatch_message['app_name'] = self.app_name
        dispatch_message['coop_phase'] = coopPhase
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

