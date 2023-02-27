
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

from gridappsd import GridAPPSD

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime


class CompetingApp(GridAPPSD):

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


  def resiliency(self, EnergyConsumers, SynchronousMachines, Batteries,
                 SolarPVs, time, load_mult, pv_mult, price, deltaT,
                 emergencyState):

    P_load = 0.0
    for name in EnergyConsumers:
      P_load += load_mult*EnergyConsumers[name]['kW']

    P_ren = 0.0
    for name in SolarPVs:
      P_ren += pv_mult*SolarPVs[name]['kW']

    if emergencyState:
      print('\nRESILIENCY APP OUTPUT: Emergency state\n--------------------------------------', flush=True)
    else:
      print('\nRESILIENCY APP OUTPUT: Alert state\n----------------------------------', flush=True)

    print('time: ' + str(time), flush=True)
    print('Total EnergyConsumers P_load: ' + str(round(P_load,4)), flush=True)
    print('Total SolarPVs P_ren: ' + str(round(P_ren,4)), flush=True)

    P_sub = 2.0*P_load

    if time > 56 and time < 68:
    # if time > 72 and time < 84:
      P_sub = 0.0
      emergencyState = True

    if not emergencyState: # alert state
      P_batt_total = 0.0
      for name in Batteries:
        Batteries[name]['state'] = 'idling'
        if Batteries[name]['SoC'] < 0.9:
          P_batt_c = (0.9 - Batteries[name]['SoC'])*Batteries[name]['ratedE'] / (Batteries[name]['eff_c'] * deltaT)
          Batteries[name]['P_batt_c'] = P_batt_c = min(P_batt_c, Batteries[name]['ratedkW'])
          P_batt_total += P_batt_c
        else:
          Batteries[name]['P_batt_c'] = P_batt_c = 0.0

      if P_batt_total > 0.0:
        if P_ren > P_load:
          if P_ren - P_load >= P_batt_total:
            # print('Charging from renewables', flush=True)
            # YES, Charge ESS
            self.AppUtil.charge_batteries(Batteries, deltaT)

          else:
            # NO, Check P_sub
            if P_ren + P_sub > P_load:
              # print('P_ren<P_load Charging from renewable + substation', flush=True)
              self.AppUtil.charge_batteries(Batteries, deltaT)

        else:
          # Check P_sub
          if P_ren + P_sub > P_load:
            # print('P_ren+P_sub>P_load Charging from renewable + substation', flush=True)
            self.AppUtil.charge_batteries(Batteries, deltaT)

    else: # emergency state
      # Shiva HACK
      P_sub = 0.0
      if P_ren > P_load:
        P_batt_total = 0.0
        for name in Batteries:
          Batteries[name]['state'] = 'idling'
          if Batteries[name]['SoC'] < 0.9:
            P_batt_c = (0.9 - Batteries[name]['SoC']) * Batteries[name]['ratedE'] / (Batteries[name]['eff_c'] * deltaT)
            Batteries[name]['P_batt_c'] = P_batt_c = min(P_batt_c, Batteries[name]['ratedkW'])
            P_batt_total += P_batt_c
          else:
            Batteries[name]['P_batt_c'] = P_batt_c = 0.0

        P_surplus = P_ren - P_load
        print('P_surplus: ' + str(P_surplus) + ', P_batt_total: ' + str(P_batt_total), flush=True)

        # print('Charging from renewables', flush=True)
        if P_surplus < P_batt_total:
          for name in Batteries:
            if 'P_batt_c' in Batteries[name]:
              Batteries[name]['P_batt_c'] *= P_surplus / P_batt_total

        if P_batt_total > 0.0:
          self.AppUtil.charge_batteries(Batteries, deltaT)
      else:
        self.AppUtil.dispatch_DGSs(Batteries, SynchronousMachines, deltaT, P_load, P_ren, P_sub)
    for name in Batteries:
      if Batteries[name]['state'] == 'charging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_c: ' + str(round(Batteries[name]['P_batt_c'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      elif Batteries[name]['state'] == 'discharging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_d: ' + str(round(Batteries[name]['P_batt_d'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      else:
        print('Battery name: ' + name + ', P_batt_c = P_batt_d = 0.0, updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)

    return


  def decarbonization(self, EnergyConsumers, SynchronousMachines, Batteries,
                      SolarPVs, time, load_mult, pv_mult, profit, deltaT,
                      emergencyState):

    P_load = 0.0
    for name in EnergyConsumers:
      P_load += load_mult*EnergyConsumers[name]['kW']

    P_ren = 0.0
    for name in SolarPVs:
      P_ren += pv_mult*SolarPVs[name]['kW']

    print('\nDECARBONIZATION APP OUTPUT\n--------------------------', flush=True)

    print('time: ' + str(time), flush=True)
    print('Total EnergyConsumers P_load: ' + str(round(P_load,4)), flush=True)
    print('Total SolarPVs P_ren: ' + str(round(P_ren,4)), flush=True)


    P_sub = 2.0*P_load

    if time > 56 and time < 68:
    # if time > 72 and time < 84:
      P_sub = 0.0

    P_batt_total = 0.0
    for name in Batteries:
      Batteries[name]['state'] = 'idling'
      if Batteries[name]['SoC'] < 0.9:
        P_batt_c = (0.9 - Batteries[name]['SoC'])*Batteries[name]['ratedE'] / (Batteries[name]['eff_c'] * deltaT)
        Batteries[name]['P_batt_c'] = P_batt_c = min(P_batt_c, Batteries[name]['ratedkW'])
        P_batt_total += P_batt_c

    if P_ren > P_load:
      P_surplus = P_ren - P_load
      print('P_surplus: ' + str(round(P_surplus,4)) + ', P_batt_total: ' + str(round(P_batt_total,4)), flush=True)

      # print('Charging from renewables', flush=True)
      if P_surplus < P_batt_total:
        for name in Batteries:
          if 'P_batt_c' in Batteries[name]:
            Batteries[name]['P_batt_c'] *= P_surplus/P_batt_total

      if P_batt_total > 0.0:
        self.AppUtil.charge_batteries(Batteries, deltaT)

    else:
      self.AppUtil.dispatch_DGSs(Batteries, SynchronousMachines, deltaT, P_load, P_ren, P_sub)

    for name in Batteries:
      if Batteries[name]['state'] == 'charging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_c: ' + str(round(Batteries[name]['P_batt_c'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      elif Batteries[name]['state'] == 'discharging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_d: ' + str(round(Batteries[name]['P_batt_d'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      else:
        print('Battery name: ' + name + ', P_batt_c = P_batt_d = 0.0, updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)

    return


  def profit(self, EnergyConsumers, SynchronousMachines, Batteries,
             SolarPVs, time, load_mult, pv_mult, price, deltaT, emergencyState):

    P_load = 0.0
    for name in EnergyConsumers:
      P_load += load_mult*EnergyConsumers[name]['kW']

    P_ren = 0.0
    for name in SolarPVs:
      P_ren += pv_mult*SolarPVs[name]['kW']

    print('\nPROFIT APP OUTPUT\n-----------------', flush=True)

    print('time: ' + str(time), flush=True)
    print('Total EnergyConsumers P_load: ' + str(round(P_load,4)), flush=True)
    print('Total SolarPVs P_ren: ' + str(round(P_ren,4)), flush=True)


    P_sub = 2.0*P_load

    if time > 56 and time < 68:
    # if time > 72 and time < 84:
      P_sub = 0.0

    P_batt_total = 0.0
    for name in Batteries:
      Batteries[name]['state'] = 'idling'
      if Batteries[name]['SoC'] < 0.9:
        P_batt_c = (0.9 - Batteries[name]['SoC'])*Batteries[name]['ratedE'] / (Batteries[name]['eff_c'] * deltaT)
        Batteries[name]['P_batt_c'] = P_batt_c = min(P_batt_c, Batteries[name]['ratedkW'])
        P_batt_total += P_batt_c

    if P_ren > P_load:
      P_surplus = P_ren - P_load
      print('P_surplus: ' + str(round(P_surplus,4)) + ', P_batt_total: ' + str(round(P_batt_total,4)), flush=True)

      # print('Charging from renewables', flush=True)
      if P_surplus < P_batt_total:
        for name in Batteries:
          if 'P_batt_c' in Batteries[name]:
            Batteries[name]['P_batt_c'] *= P_surplus/P_batt_total

      if P_batt_total > 0.0:
        self.AppUtil.charge_batteries(Batteries, deltaT)

    else:
      self.AppUtil.dispatch_DGSs(Batteries, SynchronousMachines, deltaT, P_load, P_ren, P_sub, price)

    for name in Batteries:
      if Batteries[name]['state'] == 'charging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_c: ' + str(round(Batteries[name]['P_batt_c'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      elif Batteries[name]['state'] == 'discharging':
        print('Battery name: ' + name + ', ratedkW: ' + str(round(Batteries[name]['ratedkW'],4)) + ', P_batt_d: ' + str(round(Batteries[name]['P_batt_d'],4)) + ', updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      else:
        print('Battery name: ' + name + ', P_batt_c = P_batt_d = 0.0, updated SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)

    return


  def __init__(self, gapps, feeder_mrid, simulation_id, app, state):
    self.AppUtil = getattr(importlib.import_module('shared.apputil'), 'AppUtil')

    SPARQLManager = getattr(importlib.import_module('shared.sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    # Competing Apps Start

    emergencyState = state.startswith('e') or state.startswith('E')

    EnergyConsumers = {}
    objs = sparql_mgr.obj_dict_export('EnergyConsumer')
    print('Count of EnergyConsumers Dict: ' + str(len(objs)), flush=True)
    for item in objs:
      name = item['IdentifiedObject.name']
      EnergyConsumers[name] = {}
      # Shiva HACK to force battery charging...
      #EnergyConsumers[name]['kW'] = 0.01*float(item['EnergyConsumer.p'])/1000.0
      EnergyConsumers[name]['kW'] = float(item['EnergyConsumer.p'])/1000.0
      EnergyConsumers[name]['kVar'] = float(item['EnergyConsumer.q'])/1000.0
      #print('EnergyConsumer name: ' + name + ', kW: ' + str(EnergyConsumers[name]['kW']) + ', kVar: ' + str(EnergyConsumers[name]['kVar']), flush=True)

    #objs = sparql_mgr.obj_meas_export('EnergyConsumer')
    #print('Count of EnergyConsumers Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('EnergyConsumer: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    #print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('PowerElectronicsConnection: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_dict_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Dict: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_meas_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item), flush=True)

    SynchronousMachines = {}
    objs = sparql_mgr.obj_dict_export('SynchronousMachine')
    print('Count of SynchronousMachines Dict: ' + str(len(objs)), flush=True)
    for item in objs:
      name = item['IdentifiedObject.name']
      SynchronousMachines[name] = {}
      SynchronousMachines[name]['kW'] = float(item['SynchronousMachine.p'])/1000.0
      SynchronousMachines[name]['kVar'] = float(item['SynchronousMachine.q'])/1000.0
      SynchronousMachines[name]['ratedS'] = float(item['SynchronousMachine.ratedS'])/1000.0
      print('SynchronousMachine name: ' + name + ', kW: ' + str(round(SynchronousMachines[name]['kW'],4)) + ', kVar: ' + str(round(SynchronousMachines[name]['kVar'],4)), flush=True)

    objs = sparql_mgr.obj_meas_export('SynchronousMachine')
    #print('Count of SynchronousMachines Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('SynchronousMachine: ' + str(item), flush=True)

    Batteries = {}
    bindings = sparql_mgr.battery_query()
    print('Count of Batteries: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['name']['value']
      #bus = obj['bus']['value'].upper()
      Batteries[name] = {}
      Batteries[name]['ratedkW'] = float(obj['ratedS']['value'])/1000.0
      Batteries[name]['ratedE'] = float(obj['ratedE']['value'])/1000.0
      # Shiva HACK
      Batteries[name]['SoC'] = 0.5
      #Batteries[name]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
      # eff_c and eff_d don't come from the query, but they are used throughout
      # and this is a convenient point to assign them with query results
      Batteries[name]['eff_c'] = 0.975 * 0.86
      Batteries[name]['eff_d'] = 0.975 * 0.86
      print('Battery name: ' + name + ', ratedE: ' + str(round(Batteries[name]['ratedE'],4)) + ', SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)

    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    print('Count of SolarPV: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['name']['value']
      #bus = obj['bus']['value'].upper()
      #ratedS = float(obj['ratedS']['value'])
      #ratedU = float(obj['ratedU']['value'])
      SolarPVs[name] = {}
      SolarPVs[name]['kW'] = float(obj['p']['value'])/1000.0
      SolarPVs[name]['kVar'] = float(obj['q']['value'])/1000.0
      #print('SolarPV name: ' + name + ', kW: ' + str(SolarPVs[name]['kW']) + ', kVar: ' + str(SolarPVs[name]['kVar']), flush=True)

    # make sure output directory exists since that's where results go
    if not os.path.isdir('output'):
      os.makedirs('output')

    # Deconfliction methods
    competing_func = None
    competing_name = None
    if app.startswith('r') or app.startswith('R'):
      # 1. Resilience Exclusivity
      competing_func = self.resiliency
      competing_name = 'resilience'
    elif app.startswith('d') or app.startswith('D'):
      # 2. Decarbonization Exclusivity
      competing_func = self.decarbonization
      competing_name = 'decarbonization'
    elif app.startswith('p') or app.startswith('P'):
      # 3. Profit Exclusivity
      competing_func = self.profit
      competing_name = 'profit'

    with open('time-series-data.csv', 'r') as f:
      reader = csv.reader(f)
      next(reader) # skip header

      deltaT = 0.25 # timestamp interval in fractional hours, 0.25 = 15 min.

      if competing_func != None:
        # for plotting
        t_plot = []
        soc_plot = {}
        p_batt_plot = {}
        for name in Batteries:
          soc_plot[name] = []
          p_batt_plot[name] = []

        solution = {}
        for row in reader:
          time = int(row[0])
          loadshape = float(row[1])
          solar = float(row[2])
          price = float(row[3])
          #print('time series time: ' + str(time) + ', loadshape: ' + str(loadshape) + ', solar: ' + str(solar) + ', price: ' + str(price), flush=True)

          t_plot.append(self.AppUtil.to_datetime(time)) # for plotting
          solution[time] = {}

          competing_func(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, time, loadshape, solar, price, deltaT, emergencyState)

          for name in Batteries:
            solution[time][name] = {}
            if Batteries[name]['state'] == 'charging':
              solution[time][name]['P_batt'] = Batteries[name]['P_batt_c']
            elif Batteries[name]['state'] == 'discharging':
              solution[time][name]['P_batt'] = -Batteries[name]['P_batt_d']
            else:
              solution[time][name]['P_batt'] = 0.0

            solution[time][name]['SoC'] = Batteries[name]['SoC']
            p_batt_plot[name].append(solution[time][name]['P_batt']) # for plotting
            soc_plot[name].append(Batteries[name]['SoC']) # for plotting

        json_fp = open('output/' + competing_name + '_solution.json', 'w')
        json.dump(solution, json_fp, indent=2)
        json_fp.close()

        self.AppUtil.make_plots(competing_name.capitalize() + ' Exclusivity', competing_name, Batteries, t_plot, p_batt_plot, soc_plot)

      elif app.startswith('c') or app.startswith('C'):
        # 4. Compromise (between resilience and decarbonization)
        # for plotting
        t_plot = []
        soc_plot = {}
        p_batt_plot = {}
        for name in Batteries:
          # Restore original state of batteries
          Batteries[name]['initial_SoC'] = Batteries[name]['SoC']
          soc_plot[name] = []
          p_batt_plot[name] = []

        solution = {}
        for row in reader:
          time = int(row[0])
          loadshape = float(row[1])
          solar = float(row[2])
          price = float(row[3])
          #print('time series time: ' + str(time) + ', loadshape: ' + str(loadshape) + ', solar: ' + str(solar) + ', price: ' + str(price), flush=True)

          t_plot.append(self.AppUtil.to_datetime(time)) # for plotting
          resilience_solution = {}
          decarbonization_solution = {}
          solution[time] = {}
          self.resiliency(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, time, loadshape, solar, price, deltaT, emergencyState)
          for name in Batteries:
            resilience_solution[name] = {}
            if Batteries[name]['state'] == 'charging':
              resilience_solution[name]['P_batt'] = Batteries[name]['P_batt_c']
            elif Batteries[name]['state'] == 'discharging':
              resilience_solution[name]['P_batt'] = -Batteries[name]['P_batt_d']
            else:
              resilience_solution[name]['P_batt'] = 0.0

            resilience_solution[name]['SoC'] = Batteries[name]['SoC']
            Batteries[name]['SoC'] = Batteries[name]['initial_SoC']

          self.decarbonization(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, time, loadshape, solar, price, deltaT, emergencyState)
          for name in Batteries:
            decarbonization_solution[name] = {}
            if Batteries[name]['state'] == 'charging':
              decarbonization_solution[name]['P_batt'] = Batteries[name]['P_batt_c']
            elif Batteries[name]['state'] == 'discharging':
              decarbonization_solution[name]['P_batt'] = -Batteries[name]['P_batt_d']
            else:
              decarbonization_solution[name]['P_batt'] = 0.0

            decarbonization_solution[name]['SoC'] = Batteries[name]['SoC']
            Batteries[name]['SoC'] = Batteries[name]['initial_SoC']

          for name in Batteries:
            solution[time][name] = {}
            solution[time][name]['P_batt'] = (resilience_solution[name]['P_batt'] + decarbonization_solution[name]['P_batt']) / 2
            if solution[time][name]['P_batt'] > 0:
              Batteries[name]['state'] = 'charging'
              Batteries[name]['P_batt_c'] = solution[time][name]['P_batt']
              Batteries[name]['P_batt_d'] = 0.0
            elif solution[time][name]['P_batt'] < 0:
              Batteries[name]['state'] = 'discharging'
              Batteries[name]['P_batt_d'] = -solution[time][name]['P_batt']
              Batteries[name]['P_batt_c'] = 0.0
            else:
              Batteries[name]['P_batt_c'] = Batteries[name]['P_batt_d'] = 0.0

          self.AppUtil.charge_batteries(Batteries, deltaT)
          self.AppUtil.discharge_batteries(Batteries, deltaT)
          for name in Batteries:
            solution[time][name]['SoC'] = Batteries[name]['initial_SoC'] = Batteries[name]['SoC']
            p_batt_plot[name].append(solution[time][name]['P_batt'])  # plotting
            soc_plot[name].append(solution[time][name]['SoC'])  # plotting

        json_fp = open('output/compromise_solution.json', 'w')
        json.dump(solution, json_fp, indent=2)
        json_fp.close()

        self.AppUtil.make_plots('Compromise', 'compromise', Batteries, t_plot, p_batt_plot, soc_plot)

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

  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("app", nargs="?", default="Resiliency", help="Resiliency or Decarbonization or Profit")
  parser.add_argument("state", nargs="?", default="Alert", help="Alert or Emergency State")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]
  simulation_id = opts.simulation_id
  app = opts.app
  state = opts.state

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(simulation_id)
  assert gapps.connected

  competing_app = CompetingApp(gapps, feeder_mrid, simulation_id, app, state)


if __name__ == "__main__":
  _main()

