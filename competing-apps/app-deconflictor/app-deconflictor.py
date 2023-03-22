
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
Created on March 8, 2023

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
import copy

from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime

from time import sleep

class AppDeconflictor(GridAPPSD):

  def on_message(self, headers, message):
    #print('headers: ' + str(headers), flush=True)
    print('message: ' + str(message), flush=True)

    app_name = message['app_name']
    timestamp = int(message['timestamp'])

    # Step 1: Update conflict matrix with newly provided set-points
    self.ConflictTimestamps[app_name] = timestamp

    # delete any existing matches for app_name so there are no stragglers
    # from past timestamps
    for device in self.ConflictSetpoints:
      if app_name in self.ConflictSetpoints[device]:
        self.ConflictSetpoints[device].pop(app_name)

    # now add the new set-points for app_name
    set_points = message['set_points']
    for device, value in set_points.items():
      #print('device: ' + device + ', value: ' + str(value), flush=True)
      if device not in self.ConflictSetpoints:
        self.ConflictSetpoints[device] = {}

      self.ConflictSetpoints[device][app_name] = value

    print('ConflictTimestamps: ' + str(self.ConflictTimestamps), flush=True)
    print('ConflictSetpoints: ' + str(self.ConflictSetpoints), flush=True)

    # Step 2: Determine if there is a new conflict
    conflictFlag = False
    for device in set_points:
      for app in self.ConflictSetpoints[device]:
        if app!=app_name and \
           (set_points[device]!=self.ConflictSetpoints[device][app] or \
            timestamp!=self.ConflictTimestamps[app]):
          conflictFlag = True
          break # breaking out of nested loops courtesy of Stack Overflow
        else:
          continue # only executed if the inner loop did not break
      break # only executed if the inner loop did break
    #conflictFlag = True # just override it to always call deconflict method
    print('Solution conflictFlag: ' + str(conflictFlag), flush=True)

    # Step 3: If there is a conflict, then the call the deconflict method for
    #         the given methodology to produce a solution
    if conflictFlag:
      newSolutionSetpoints,newSolutionTimestamps= self.decon_method.deconflict()

    # Step 4: If there is no conflict, then the new solution is simply the
    #         last solution with the new set-points added in
    else:
      newSolutionSetpoints = copy.deepcopy(self.SolutionSetpoints)
      newSolutionTimestamps = copy.deepcopy(self.SolutionTimestamps)
      for device, value in set_points.items():
        newSolutionSetpoints[device] = value
        newSolutionTimestamps[device] = timestamp

    print('Solution deconflicted set-points: ' + str(newSolutionSetpoints), flush=True)
    print('Solution deconflicted timestamps: ' + str(newSolutionTimestamps), flush=True)

    # Step 5: Iterate over solution and send set-points to devices that have
    #         different or new values
    for device, value in newSolutionSetpoints.items():
      if device not in self.SolutionSetpoints or \
         (newSolutionTimestamps[device]==timestamp and \
          (self.SolutionTimestamps[device]!=timestamp or \
           self.SolutionSetpoints[device]!=value)):

        print('==> Solution sending value to device: ' + device + ', value: ' +
              str(value), flush=True)

        # update battery SoC
        if value > 0: # charging
          self.Batteries[device]['SoC'] += self.Batteries[device]['eff_c']*value* \
                               self.deltaT/self.Batteries[device]['ratedE']
        elif value < 0: # discharging
          self.Batteries[device]['SoC'] += 1/self.Batteries[device]['eff_d']*value* \
                                 self.deltaT/self.Batteries[device]['ratedE']

    # it's also possible a device from the last solution does not appear in
    # the new solution.  In this case it's a "don't care" for the new solution
    # and the device is left at the previous value so nothing is sent
    if len(self.SolutionSetpoints) > len(newSolutionSetpoints):
      for device in self.SolutionSetpoints:
        if device not in newSolutionSetpoints:
          print('==> Solution device deleted: ' + device + ', no value sent',
                flush=True)

    # Step 6: Update the current solution to the new solution to be ready
    #         for the next set-points message
    self.SolutionSetpoints.clear()
    self.SolutionTimestamps.clear()
    self.SolutionSetpoints = newSolutionSetpoints
    self.SolutionTimestamps = newSolutionTimestamps

    # Report battery SoC values
    for name in self.Batteries:
      print('Battery name: ' + name + ', SoC: ' +
            str(self.Batteries[name]['SoC']), flush=True)

    print(flush=True)

    return

    # just sample code for sending out a message
    solution = self.solution[time]
    set_points = {}
    for name in solution:
      set_points[name] = solution[name]['P_batt']

    out_message = {
      'app_name': 'resilience-app',
      'timestamp': in_message['timestamp'],
      'set_points': set_points
    }
    print('\nSending message: ' + str(out_message), flush=True)
    self.gapps.send(self.publish_topic, out_message)

    return


  def __init__(self, gapps, feeder_mrid, simulation_id, method):
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

    self.ConflictSetpoints = {}
    self.ConflictTimestamps = {}

    self.SolutionSetpoints = {}
    self.SolutionTimestamps = {}

    # Step 0: Import deconfliction methodology class for this invocation of
    #         the Deconflictor based on method command line argument and
    #         create an instance of the class
    DeconflictionMethod = getattr(importlib.import_module(method),
                                  'DeconflictionMethod')
    self.decon_method = DeconflictionMethod(self.ConflictSetpoints,
                                            self.ConflictTimestamps)

    # subscribe to simulation output messages
    gapps.subscribe(service_output_topic('gridappsd-competing-app', simulation_id), self)

    print('Initialized app deconflictor and now waiting for set-point messages...\n', flush=True)

    self.gapps = gapps
    self.exit_flag = False

    while not self.exit_flag:
      time.sleep(0.1)

    # make sure output directory exists since that's where results go
    if not os.path.isdir('output'):
      os.makedirs('output')

    json_fp = open('output/deconflictor_solution.json', 'w')
    json.dump(self.solution, json_fp, indent=2)
    json_fp.close()

    self.AppUtil.make_plots('Deconflictor Solution', 'deconflictor', self.Batteries, self.t_plot, self.p_batt_plot, self.soc_plot)

    return


def _main():
  print('Starting app deconflictor code...', flush=True)

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
  parser.add_argument("method", help="Deconfliction Methodology")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-app-deconflictor'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(opts.simulation_id)
  assert gapps.connected

  deconflictor = AppDeconflictor(gapps, feeder_mrid,
                                 opts.simulation_id, opts.method)


if __name__ == "__main__":
  _main()

