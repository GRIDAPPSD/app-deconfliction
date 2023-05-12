
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

class DeconflictionPipeline(GridAPPSD):

  def SetpointProcessor(self, app_name, timestamp, set_points):
    # Update conflict matrix with newly provided set-points
    self.ConflictMatrix['timestamps'][app_name] = timestamp

    # delete any existing matches for app_name so there are no stragglers
    # from past timestamps
    for device in self.ConflictMatrix['setpoints']:
      if app_name in self.ConflictMatrix['setpoints'][device]:
        self.ConflictMatrix['setpoints'][device].pop(app_name)

    # now add the new set-points for app_name
    for device, value in set_points.items():
      #print('device: ' + device + ', value: ' + str(value), flush=True)
      if device not in self.ConflictMatrix['setpoints']:
        self.ConflictMatrix['setpoints'][device] = {}

      self.ConflictMatrix['setpoints'][device][app_name] = value

    print('ConflictMatrix: ' + str(self.ConflictMatrix), flush=True)


  def ConflictIdentification(self, app_name, timestamp, set_points):
    # Determine if there is a new conflict
    conflictFlag = False
    for device in set_points:
      for app in self.ConflictMatrix['setpoints'][device]:
        if app!=app_name and \
           (set_points[device]!=self.ConflictMatrix['setpoints'][device][app] \
            or timestamp!=self.ConflictMatrix['timestamps'][app]):
          conflictFlag = True
          break # breaking out of nested loops courtesy of Stack Overflow
        else:
          continue # only executed if the inner loop did not break
      break # only executed if the inner loop did break

    #conflictFlag = True # just override it to always call deconflict method
    print('conflictFlag: ' + str(conflictFlag), flush=True)

    return conflictFlag


  def DeconflictionToResolution(self, timestamp, set_points, conflictFlag):
    # If there is a conflict, then the call the deconflict method for the given
    # methodology to produce a resolution
    if conflictFlag:
      newResolutionVector = self.decon_method.deconflict()

    # If there is no conflict, then the new resolution is simply the last
    # resolution with the new set-points added in
    else:
      newResolutionVector = copy.deepcopy(self.ResolutionVector)
      for device, value in set_points.items():
        newResolutionVector['setpoints'][device] = value
        newResolutionVector['timestamps'][device] = timestamp

    print('ResolutionVector: ' + str(newResolutionVector), flush=True)

    return newResolutionVector


  def DeviceDispatcher(self, timestamp, newResolutionVector):
    # Iterate over resolution and send set-points to devices that have
    # different or new values
    revised_socs = {}
    for device, value in newResolutionVector['setpoints'].items():
      if device.startswith('BatteryUnit:'):
        if device not in self.ResolutionVector['setpoints'] or \
           (newResolutionVector['timestamps'][device]==timestamp and \
            (self.ResolutionVector['timestamps'][device]!=timestamp or \
             self.ResolutionVector['setpoints'][device]!=value)):

          # determine if a resolution for this device for this timestamp has
          # already been sent
          if device in self.ResolutionVector['timestamps'] and \
             self.ResolutionVector['timestamps'][device]==timestamp:
            # rollback the previous contribution to SoC as the new one overrides
            backval = self.ResolutionVector['setpoints'][device]
            if backval > 0:
              self.Batteries[device]['SoC'] -= self.AppUtil.charge_SoC(backval,
                                            device, self.Batteries, self.deltaT)
            elif backval < 0:
              self.Batteries[device]['SoC'] -= self.AppUtil.discharge_SoC(
                                   backval, device, self.Batteries, self.deltaT)
            print('*** Revised projected SoC for device: ' + device + ', SoC: ' + str(self.Batteries[device]['SoC']), flush=True)

          # update battery SoC
          if value > 0: # charging
            self.Batteries[device]['SoC'] += self.AppUtil.charge_SoC(value,
                                            device, self.Batteries, self.deltaT)
          elif value < 0: # discharging
            self.Batteries[device]['SoC'] += self.AppUtil.discharge_SoC(value,
                                            device, self.Batteries, self.deltaT)

          # for message back to competing apps
          revised_socs[device] = self.Batteries[device]['SoC']

          print('==> Dispatching value to device: ' + device + ', value: ' +
                str(value) + ' (projected SoC: ' +
                str(revised_socs[device]) + ')', flush=True)

      # not a battery so only dispatch value if it's changed or if it's
      # never been dispatched before
      elif device not in self.ResolutionVector['setpoints'] or \
           self.ResolutionVector['setpoints'][device]!=value:
        print('==> Dispatching value to device: ' + device + ', value: ' +
              str(value), flush=True)

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector['setpoints']) > \
       len(newResolutionVector['setpoints']):
      for device in self.ResolutionVector['setpoints']:
        if device not in newResolutionVector['setpoints']:
          print('==> Device deleted from resolution: ' + device, flush=True)

    return revised_socs


  def AppFeedback(self, timestamp, revised_socs):
    # If running from a GridLAB-D simulation where Deconflictor Pipeline
    # updates devices in simulation, this feedback would come to apps through
    # simulation measurement messages and there would be no need to explicitly
    # publish updates
    if len(revised_socs) > 0:
      socs_message = {
        'timestamp': timestamp,
        'SoC': revised_socs
      }
      print('Sending revised-socs message: ' + str(socs_message), flush=True)
      self.gapps.send(self.publish_topic, socs_message)

    print(flush=True) # tidy up output with blank line


  def on_message(self, headers, message):
    print('Received set-points message: ' + str(message), flush=True)

    app_name = message['app_name']
    timestamp = int(message['timestamp'])
    set_points = message['set_points']

    # Step 1: Setpoint Processor
    self.SetpointProcessor(app_name, timestamp, set_points)

    # Step 2: Feasibility Maintainer -- not implemented for prototype

    # Step 3: Deconflictor
    # Step 3.1: Conflict Identification
    conflictFlag = self.ConflictIdentification(app_name, timestamp, set_points)

    # Steps 3.2 and 3.3: Deconfliction Solution and Resolution
    newResolutionVector = self.DeconflictionToResolution(timestamp, set_points,
                                                         conflictFlag)

    # Step 4: Setpoint Validator -- not implemented for prototype

    # Step 5: Device Dispatcher
    revised_socs = self.DeviceDispatcher(timestamp, newResolutionVector)

    # Feedback loop with competing apps through revised SoC values so they
    # can make new set-point requests based on actual changes
    self.AppFeedback(timestamp, revised_socs)

    # Update the current resolution to the new resolution to be ready for the
    # next set-points message
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # For plotting
    datetime = self.AppUtil.to_datetime(timestamp)
    if datetime not in self.t_plot:
      self.t_plot.append(datetime)
      for name in self.ResolutionVector['setpoints']:
        if name.startswith('BatteryUnit:'):
          self.p_batt_plot[name].append(
                                 self.ResolutionVector['setpoints'][name])
          self.soc_plot[name].append(self.Batteries[name]['SoC'])
    else:
      # replacing last list item is equivalent to rollback
      for name in self.ResolutionVector['setpoints']:
        if name.startswith('BatteryUnit:'):
          self.p_batt_plot[name][-1] = self.ResolutionVector['setpoints'][name]
          self.soc_plot[name][-1] = self.Batteries[name]['SoC']

    return


  def __init__(self, gapps, method, feeder_mrid, simulation_id):
    self.AppUtil = getattr(importlib.import_module('shared.apputil'), 'AppUtil')

    SPARQLManager = getattr(importlib.import_module('shared.sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = self.AppUtil.getBatteries(sparql_mgr)

    # SHIVA HACK for 123 model testing
    if feeder_mrid == '_C1C3E687-6FFD-C753-582B-632A27E28507':
      self.Batteries['BatteryUnit:65'] = {'idx': 0, 'prated': 250000,
            'phase': 'A', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.35}
      self.Batteries['BatteryUnit:65']['eff_c'] = \
                                   self.Batteries['BatteryUnit:65']['eff_d'] = \
                                   self.Batteries['BatteryUnit:65']['eff']
      self.Batteries['BatteryUnit:52'] = {'idx': 1, 'prated': 250000,
            'phase': 'B', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.275}
      self.Batteries['BatteryUnit:52']['eff_c'] = \
                                   self.Batteries['BatteryUnit:52']['eff_d'] = \
                                   self.Batteries['BatteryUnit:52']['eff']
      self.Batteries['BatteryUnit:76'] = {'idx': 2, 'prated': 250000,
            'phase': 'C', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.465}
      self.Batteries['BatteryUnit:76']['eff_c'] = \
                                   self.Batteries['BatteryUnit:76']['eff_d'] = \
                                   self.Batteries['BatteryUnit:76']['eff']

    self.deltaT = 0.25 # timestamp interval in fractional hours, 0.25 = 15 min

    # for plotting
    self.t_plot = []
    self.soc_plot = {}
    self.p_batt_plot = {}
    for name in self.Batteries:
      self.soc_plot[name] = []
      self.p_batt_plot[name] = []

    self.ConflictMatrix = {}
    self.ConflictMatrix['setpoints'] = {}
    self.ConflictMatrix['timestamps'] = {}

    self.ResolutionVector = {}
    self.ResolutionVector['setpoints'] = {}
    self.ResolutionVector['timestamps'] = {}

    # Step 0: Import deconfliction methodology class for this invocation of
    #         the Deconflictor based on method command line argument and
    #         create an instance of the class
    if method.endswith('.py'):
      method = method[:-3] # allow full python file name

    DeconflictionMethod = getattr(importlib.import_module(method),
                                  'DeconflictionMethod')
    self.decon_method = DeconflictionMethod(self.ConflictMatrix)

    self.publish_topic = service_output_topic(
                                 'gridappsd-deconfliction-pipeline', '0')

    # subscribe to simulation output messages
    gapps.subscribe(service_output_topic('gridappsd-competing-app',
                                         simulation_id), self)

    print('Initialized deconfliction pipeline, waiting for set-points messages...\n', flush=True)

    self.gapps = gapps

    try:
      while True:
        time.sleep(0.1)

    except KeyboardInterrupt:
      # make sure output directory exists since that's where results go
      if not os.path.isdir('output'):
        os.makedirs('output')

      self.AppUtil.make_plots('Deconfliction Resolution', 'deconfliction',
                   self.Batteries, self.t_plot, self.p_batt_plot, self.soc_plot)

    return


def _main():
  print('Starting deconfliction pipeline code...', flush=True)

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
  parser.add_argument("method", help="Deconfliction Methodology")
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-deconfliction-pipeline'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD(opts.simulation_id)
  assert gapps.connected

  DeconflictionPipeline(gapps, opts.method, feeder_mrid, opts.simulation_id)


if __name__ == "__main__":
  _main()

