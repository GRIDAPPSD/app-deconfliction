
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
Created on March 6, 2023

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
from gridappsd.topics import service_output_topic

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime

from time import sleep

class CompetingApp(GridAPPSD):

  def decarbonization(self, EnergyConsumers, SynchronousMachines, Batteries,
                      SolarPVs, deltaT, time, load_mult, pv_mult, profit):

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


  def on_message(self, headers, in_message):
    #print('headers: ' + str(headers), flush=True)
    #print('message: ' + str(in_message), flush=True)

    # empty timestamp is end-of-data flag
    if in_message['timestamp'] == '':
      print('time series end-of-data!', flush=True)
      self.exit_flag = True
      return

    # if we get here we must have data to process
    time = int(in_message['timestamp'])
    loadshape = float(in_message['loadshape'])
    solar = float(in_message['solar'])
    price = float(in_message['price'])
    #print('time series time: ' + str(time) + ', loadshape: ' + str(loadshape) + ', solar: ' + str(solar) + ', price: ' + str(price), flush=True)

    self.t_plot.append(self.AppUtil.to_datetime(time)) # plotting

    self.decarbonization(self.EnergyConsumers, self.SynchronousMachines,
                         self.Batteries, self.SolarPVs, self.deltaT,
                         time, loadshape, solar, price)

    self.solution[time] = {}
    self.AppUtil.batt_to_solution(self.Batteries, self.solution[time])

    for name in self.Batteries:
      self.p_batt_plot[name].append(self.solution[time][name]['P_batt'])
      self.soc_plot[name].append(self.Batteries[name]['SoC'])

    solution = self.solution[time]
    set_points = {}
    for name in solution:
      set_points[name] = solution[name]['P_batt']

    out_message = {
      'app_name': 'decarbonization-app',
      'timestamp': in_message['timestamp'],
      'set_points': set_points
    }
    print('\nSending message: ' + str(out_message), flush=True)
    self.gapps.send(self.publish_topic, out_message)

    return


  def __init__(self, gapps, feeder_mrid, simulation_id):
    self.gapps = gapps

    self.AppUtil = getattr(importlib.import_module('shared.apputil'), 'AppUtil')

    SPARQLManager = getattr(importlib.import_module('shared.sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.EnergyConsumers = self.AppUtil.getEnergyConsumers(sparql_mgr)

    self.SynchronousMachines = self.AppUtil.getSynchronousMachines(sparql_mgr)

    self.Batteries = self.AppUtil.getBatteries(sparql_mgr)

    self.SolarPVs = self.AppUtil.getSolarPVs(sparql_mgr)

    self.deltaT = 0.25 # timestamp interval in fractional hours, 0.25 = 15 min.

    # for plotting
    self.t_plot = []
    self.soc_plot = {}
    self.p_batt_plot = {}
    for name in self.Batteries:
      self.soc_plot[name] = []
      self.p_batt_plot[name] = []

    self.solution = {}

    # topic for sending out set_points messages
    self.publish_topic = service_output_topic('gridappsd-competing-app', '0')

    # subscribe to simulation output messages
    gapps.subscribe(service_output_topic('gridappsd-pseudo-sim', simulation_id), self)

    print('Initialized decarbonization app and now waiting for messages...', flush=True)

    self.exit_flag = False

    while not self.exit_flag:
      time.sleep(0.1)

    # make sure output directory exists since that's where results go
    if not os.path.isdir('output'):
      os.makedirs('output')

    json_fp = open('output/decarbonization_solution.json', 'w')
    json.dump(self.solution, json_fp, indent=2)
    json_fp.close()

    self.AppUtil.make_plots('Decarbonization Exclusivity', 'decarbonization', self.Batteries, self.t_plot, self.p_batt_plot, self.soc_plot)

    return


def _main():
  print('Starting decarbonization app code...', flush=True)

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
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]
  simulation_id = opts.simulation_id

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(simulation_id)
  assert gapps.connected

  competing_app = CompetingApp(gapps, feeder_mrid, simulation_id)


if __name__ == "__main__":
  _main()

