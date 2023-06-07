
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
      if device.startswith('BatteryUnit.'):
        print('~~> setpoints from app: ' + app_name +
              ', timestamp: ' + str(timestamp) +
              ', device: ' + device + ', value: ' + str(value), flush=True)

      if device not in self.ConflictMatrix['setpoints']:
        self.ConflictMatrix['setpoints'][device] = {}

      self.ConflictMatrix['setpoints'][device][app_name] = value

    print('ConflictMatrix: ' + str(self.ConflictMatrix), flush=True)
    # for Alex
    #pprint.pprint(self.ConflictMatrix)


  def ConflictIdentification(self, app_name, timestamp, set_points):
    # Determine if there is a new conflict
    self.ConflictOnlyMatrix['setpoints'].clear()
    self.ConflictOnlyMatrix['timestamps'].clear()
    for device in set_points:
      for app1 in self.ConflictMatrix['setpoints'][device]:
        if app1!=app_name and \
           (set_points[device]!=self.ConflictMatrix['setpoints'][device][app1] \
            or timestamp!=self.ConflictMatrix['timestamps'][app1]):

          if device not in self.ConflictOnlyMatrix['setpoints']:
            self.ConflictOnlyMatrix['setpoints'][device] = {}

          self.ConflictOnlyMatrix['setpoints'][device][app_name] = \
                                   set_points[device]
          self.ConflictOnlyMatrix['timestamps'][app_name] = timestamp

          # once a conflict is found, add all apps for this device into
          # ConflictOnlyMatrix
          for app2 in self.ConflictMatrix['setpoints'][device]:
            if app2 != app_name:
              self.ConflictOnlyMatrix['setpoints'][device][app2] = \
                                  self.ConflictMatrix['setpoints'][device][app2]
              self.ConflictOnlyMatrix['timestamps'][app2] = \
                                  self.ConflictMatrix['timestamps'][app2]

          # skip to next device since all apps for this one are in matrix
          break

    print('ConflictOnlyMatrix: ' + str(self.ConflictOnlyMatrix), flush=True)


  def DeconflictionToResolution(self, timestamp, set_points):
    # If there is a conflict, then the call the deconflict method for the given
    # methodology to produce a resolution
    if len(self.ConflictOnlyMatrix['setpoints']) > 0:
      fullResolutionFlag, newResolutionVector = self.decon_method.deconflict()

      # GDB 6/2/23 The logic that updates the conflict matrix below is flawed
      # in that updates effectively indicate that apps are happy with the
      # resolution set-points when really they may not be, which will impact
      # subsequent resolutions in ways that may not be valid
      '''
      # Update conflict matrix with resolution vector setpoint values so the
      # deconfliction methodologies don't need to resolve the same conflicts
      # each time in addition to new conflicts from the latest set-points
      for device, value in newResolutionVector['setpoints'].items():
        for app in self.ConflictMatrix['setpoints'][device]:
          self.ConflictMatrix['setpoints'][device][app] = value
          self.ConflictMatrix['timestamps'][app] = \
                              max(newResolutionVector['timestamps'][device],
                                  self.ConflictMatrix['timestamps'][app])
      '''

      if fullResolutionFlag:
        print('ResolutionVector (from full): ' + str(newResolutionVector),
              flush=True)
        return newResolutionVector

    # to create a full resolution vector from a partial one, start with
    # a copy of the previous resolution
    fullResolutionVector = copy.deepcopy(self.ResolutionVector)

    # next copy the new set-points over top of the previous resolution
    # which handles any devices where there was no conflict and if there
    # is conflict, we are counting on newResolutionVector to override those
    for device, value in set_points.items():
      # uncomment the following line to only include batteries in resolution
      # for testing
      #if device.startswith('BatteryUnit.'):
        fullResolutionVector['setpoints'][device] = value
        fullResolutionVector['timestamps'][device] = timestamp

    # finally, if there were conflicts, then overlay the resolution to those
    if len(self.ConflictOnlyMatrix['setpoints']) > 0:
      for device in newResolutionVector['setpoints']:
        fullResolutionVector['setpoints'][device] = \
                                       newResolutionVector['setpoints'][device]
        fullResolutionVector['timestamps'][device] = \
                                       newResolutionVector['timestamps'][device]
      print('ResolutionVector (from partial): ' + str(fullResolutionVector),
            flush=True)
    else:
      print('ResolutionVector (no conflict): ' + str(fullResolutionVector),
            flush=True)

    return fullResolutionVector


  def updateSoC(self, name, P_batt, timestamp, revised_socs):
    if 'refTimestamp' not in self.Batteries[name]:
      self.Batteries[name]['refTimestamp'] = timestamp
      self.Batteries[name]['runTimestamp'] = timestamp
      self.Batteries[name]['refP_batt'] = P_batt
      self.Batteries[name]['runP_batt'] = P_batt
      self.Batteries[name]['refSoC'] = self.Batteries[name]['SoC']
      self.Batteries[name]['runSoC'] = self.Batteries[name]['SoC']

    ##print('\n~YYY updateSoC for device: ' + name + ', timestamp: ' + str(timestamp) + ', P_batt: ' + str(P_batt) + ', SoC: ' + str(self.Batteries[name]['SoC']), flush=True)
    ##print('~YYY updateSoC for device: ' + name + ', refTimestamp: ' + str(self.Batteries[name]['refTimestamp']) + ', runTimestamp: ' + str(self.Batteries[name]['runTimestamp']), flush=True)
    ##print('~YYY updateSoC for device: ' + name + ', refP_batt: ' + str(self.Batteries[name]['refP_batt']) + ', runP_batt: ' + str(self.Batteries[name]['runP_batt']), flush=True)
    ##print('~YYY updateSoC for device: ' + name + ', refSoC: ' + str(self.Batteries[name]['refSoC']) + ', runSoC: ' + str(self.Batteries[name]['runSoC']), flush=True)

    if timestamp > self.Batteries[name]['runTimestamp']:
      ##print('~YYY updateSoC for device: ' + name + ', updating refSoC and refTimestamp to run values', flush=True)
      self.Batteries[name]['refTimestamp'] = \
                                       self.Batteries[name]['runTimestamp']
      self.Batteries[name]['refP_batt'] = self.Batteries[name]['runP_batt']
      self.Batteries[name]['refSoC'] = self.Batteries[name]['runSoC']

    ##print('~YYY updateSoC for device: ' + name + ', timestamp: ' + str(timestamp) + ', refTimestamp: ' + str(self.Batteries[name]['refTimestamp']) + ', runTimestamp: ' + str(self.Batteries[name]['runTimestamp']), flush=True)
    actContrib = 0.0
    if timestamp > self.Batteries[name]['refTimestamp']:
      ##print('~YYY actual contrib based on P_batt: ' + str(self.Batteries[name]['refP_batt']) + ', current timestamp: ' + str(timestamp) + ', reference timestamp: ' + str(self.Batteries[name]['refTimestamp']), flush=True)
      actContrib = self.AppUtil.contrib_SoC(self.Batteries[name]['refP_batt'],
                                 timestamp-self.Batteries[name]['refTimestamp'],
                                 self.Batteries[name], self.deltaT)
    elif timestamp < self.Batteries[name]['refTimestamp']:
      # consider going back in time the same as equal timestamps other than
      # reporting it
      print('*** WARNING: time has gone backwards with set-points message for device: ' + name + ', timestamp: ' + str(timestamp) + ', last timestamp: ' + str(self.Batteries[name]['refTimestamp']), flush=True)

    self.Batteries[name]['runSoC'] = self.Batteries[name]['refSoC'] + actContrib

    projContrib = self.AppUtil.contrib_SoC(P_batt, 1, self.Batteries[name],
                                           self.deltaT)
    projSoC = self.Batteries[name]['runSoC'] + projContrib

    print('~SOC updateSoC magic for device: ' + name + ', reference SoC: ' + str(self.Batteries[name]['refSoC']) + ', actual SoC contrib: ' + str(actContrib) + ', projected SoC contrib: ' + str(projContrib) + ', projected SoC: ' + str(projSoC), end='')

    if projSoC != self.Batteries[name]['SoC']:
      revised_socs[name] = self.Batteries[name]['SoC'] = projSoC
      print(' (revised)', flush=True)
    else:
      print(' (not revised)', flush=True)

    self.Batteries[name]['runTimestamp'] = max(timestamp,
                                           self.Batteries[name]['runTimestamp'])
    self.Batteries[name]['runP_batt'] = P_batt


  timestampOld = {}
  P_battOld = {}
  SoCOld = {}
  SoCOlder = {}

  def oldUpdateSoC(self, name, P_batt, timestamp, revised_socs):
    if name not in self.timestampOld:
      self.timestampOld[name] = timestamp
      self.P_battOld[name] = P_batt
      self.SoCOlder[name] = self.SoCOld[name] = self.Batteries[name]['OldSoC']

    ##print('\n~OLD updateSoC for device: ' + name + ', timestamp: ' + str(timestamp) + ', P_batt: ' + str(P_batt) + ', SoC: ' + str(self.Batteries[name]['OldSoC']), flush=True)
    ##print('~OLD updateSoC for device: ' + name + ', timestampOld: ' + str(self.timestampOld[name]), flush=True)
    ##print('~OLD updateSoC for device: ' + name + ', P_battOld: ' + str(self.P_battOld[name]), flush=True)
    ##print('~OLD updateSoC for device: ' + name + ', SoCOld: ' + str(self.SoCOld[name]) + ', SoCOlder: ' + str(self.SoCOlder[name]), flush=True)

    start = self.SoCOlder[name]

    actual = 0.0
    if timestamp > self.timestampOld[name]:
      ##print('~OLD actual contrib based on P_batt: ' + str(self.P_battOld[name]) + ', current timestamp: ' + str(timestamp) + ', old timestamp: ' + str(self.timestampOld[name]), flush=True)
      actual = self.AppUtil.contrib_SoC(self.P_battOld[name],
           timestamp-self.timestampOld[name], self.Batteries[name], self.deltaT)
    elif timestamp < self.timestampOld[name]:
      # consider going back in time the same as equal timestamps other than
      # reporting it
      print('*** WARNING: time has gone backwards with set-points message for device: ' + name + ', timestamp: ' + str(timestamp) + ', last timestamp: ' + str(self.timestampOld[name]), flush=True)

    ##print('~OLD projected contrib based on P_batt: ' + str(P_batt) + ', over 1 timestamp', flush=True)
    projected = self.AppUtil.contrib_SoC(P_batt, 1, self.Batteries[name],
                                         self.deltaT)

    newSoC = start + actual + projected

    #print('~OLD SOC oldUpdateSoC magic for device: ' + name + ', start SoC: ' + str(start) + ', actual SoC contrib: ' + str(actual) + ', projected SoC contrib: ' + str(projected) + ', new SoC: ' + str(newSoC), end='')

    if newSoC != self.Batteries[name]['OldSoC']:
      revised_socs[name] = self.Batteries[name]['OldSoC'] = newSoC
      #print(' (revised)', flush=True)
    else:
      pass
      #print(' (not revised)', flush=True)

    if timestamp > self.timestampOld[name]:
      self.SoCOlder[name] = self.SoCOld[name]
      ##print('~OLD updateSoC SoCOlder updated to: ' + str(self.SoCOlder[name]), flush=True)

    self.timestampOld[name] = timestamp
    self.P_battOld[name] = P_batt
    self.SoCOld[name] = self.Batteries[name]['OldSoC']


  def rollbackUpdateSoC(self, name, P_batt, timestamp, revised_socs):
    # determine if a resolution for this device for this timestamp has
    # already been sent
    if name in self.ResolutionVector['timestamps'] and \
       self.ResolutionVector['timestamps'][name]==timestamp:

      # rollback the previous contribution to SoC as the new one overrides
      backval = self.ResolutionVector['setpoints'][name]
      #print('~ROLLBACK SOC need to rollback SoC for device: ' + name + ', P_batt: ' + str(backval) + ', pre-rollback SoC: ' + str(self.Batteries[name]['RollbackSoC']), flush=True)
      if backval > 0:
        self.Batteries[name]['RollbackSoC'] -= self.AppUtil.charge_SoC(backval,
                                      name, self.Batteries, self.deltaT)
        revised_socs[name] = self.Batteries[name]['RollbackSoC']
      elif backval < 0:
        self.Batteries[name]['RollbackSoC'] -= self.AppUtil.discharge_SoC(
                             backval, name, self.Batteries, self.deltaT)
        revised_socs[name] = self.Batteries[name]['RollbackSoC']
      #print('~ROLLBACK SOC done with rollback for device: ' + name + ', P_batt: ' + str(backval) + ', post-rollback SoC: ' + str(self.Batteries[name]['RollbackSoC']), flush=True)

    # update battery SoC
    #print('~ROLLBACK SOC need to compute new SoC for device: ' + name + ', P_batt: ' + str(P_batt) + ', pre-compute SoC: ' + str(self.Batteries[name]['RollbackSoC']), flush=True)
    if P_batt > 0: # charging
      self.Batteries[name]['RollbackSoC'] += self.AppUtil.charge_SoC(P_batt,
                                      name, self.Batteries, self.deltaT)
      revised_socs[name] = self.Batteries[name]['RollbackSoC']
    elif P_batt < 0: # discharging
      self.Batteries[name]['RollbackSoC'] += self.AppUtil.discharge_SoC(P_batt,
                                      name, self.Batteries, self.deltaT)
      revised_socs[name] = self.Batteries[name]['RollbackSoC']
    #print('~ROLLBACK SOC done computing new SoC for device: ' + name + ', P_batt: ' + str(P_batt) + ', post-compute SoC: ' + str(self.Batteries[name]['RollbackSoC']), flush=True)


  def DeviceDispatcher(self, timestamp, newResolutionVector):
    # Iterate over resolution and send set-points to devices that have
    # different or new values
    revised_socs = {}
    old_revised_socs = {}
    rollback_revised_socs = {}
    for device, value in newResolutionVector['setpoints'].items():
      if device.startswith('BatteryUnit.'):
        # batteries dispatch values even if they are the same as the last time
        # as long as the value is associated with the current timestamp
        if device not in self.ResolutionVector['setpoints'] or \
           (newResolutionVector['timestamps'][device]==timestamp and \
            (self.ResolutionVector['timestamps'][device]!=timestamp or \
             self.ResolutionVector['setpoints'][device]!=value)):

          self.updateSoC(device, value, timestamp, revised_socs)

          self.oldUpdateSoC(device, value, timestamp, old_revised_socs)

          self.rollbackUpdateSoC(device, value, timestamp, rollback_revised_socs)

          print('*** ' + device + ' SoC: ' + str(self.Batteries[device]['SoC']) + ', SoC Old: ' + str(self.Batteries[device]['OldSoC']) + ', SoC Rollback: ' + str(self.Batteries[device]['RollbackSoC']), flush=True)
          if abs(self.Batteries[device]['SoC'] - self.Batteries[device]['RollbackSoC'])>1e-6:
            print('!!!!!!!!!!!!!!!!!!!!!!!!! ROLLBACK DIFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!', flush=True)
          if abs(self.Batteries[device]['SoC'] - self.Batteries[device]['OldSoC'])>1e-6:
            print('!!!!!!!!!!!!!!!!!!!!!!!!! NEW VS. OLD DIFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!', flush=True)
            exit()

          print('~~> Dispatching to device: ' + device + ', timestamp: ' +
                str(timestamp) + ', value: ' + str(value) +
                ' (projected SoC: ' + str(self.Batteries[device]['SoC']) + ')',
                flush=True)

      # not a battery so only dispatch value if it's changed or if it's
      # never been dispatched before
      elif device not in self.ResolutionVector['setpoints'] or \
           self.ResolutionVector['setpoints'][device]!=value:
        print('==> Dispatching to device: ' + device + ', timestamp: ' +
              str(timestamp) + ', value: ' + str(value), flush=True)

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector['setpoints']) > \
       len(newResolutionVector['setpoints']):
      for device in self.ResolutionVector['setpoints']:
        if device not in newResolutionVector['setpoints']:
          print('==> Device deleted from resolution: ' + device, flush=True)

    return revised_socs


  def AppFeedback(self, app_name, timestamp, revised_socs):
    # If running from a GridLAB-D simulation where Deconflictor Pipeline
    # updates devices in simulation, this feedback would come to apps through
    # simulation measurement messages and there would be no need to explicitly
    # publish updates
    if len(revised_socs) > 0:
      socs_message = {
        'timestamp': timestamp,
        'SoC': revised_socs
      }
      print('~~> Sending revised-socs message: ' + str(socs_message) +
            ' (driven by set-points from ' + app_name + ')', flush=True)
      self.gapps.send(self.publish_topic, socs_message)

    print('~', flush=True) # tidy output with "blank" line


  def on_message(self, headers, message):
    print('Received set-points message: ' + str(message), flush=True)

    app_name = message['app_name']
    timestamp = message['timestamp']
    set_points = message['set_points']

    # Step 1: Setpoint Processor
    self.SetpointProcessor(app_name, timestamp, set_points)

    # Step 2: Feasibility Maintainer -- not implemented for prototype

    # Step 3: Deconflictor
    # Step 3.1: Conflict Identification
    self.ConflictIdentification(app_name, timestamp, set_points)

    # Steps 3.2 and 3.3: Deconfliction Solution and Resolution
    newResolutionVector = self.DeconflictionToResolution(timestamp, set_points)

    # Step 4: Setpoint Validator -- not implemented for prototype

    # Step 5: Device Dispatcher
    revised_socs = self.DeviceDispatcher(timestamp, newResolutionVector)

    # Feedback loop with competing apps through revised SoC values so they
    # can make new set-point requests based on actual changes
    self.AppFeedback(app_name, timestamp, revised_socs)

    # Update the current resolution to the new resolution to be ready for the
    # next set-points message
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # For plotting
    datetime = self.AppUtil.to_datetime(timestamp)
    if datetime not in self.t_plot:
      self.t_plot.append(datetime)
      for name in self.ResolutionVector['setpoints']:
        if name.startswith('BatteryUnit.'):
          self.p_batt_plot[name].append(
                                 self.ResolutionVector['setpoints'][name])
          self.soc_plot[name].append(self.Batteries[name]['SoC'])
    else:
      # replacing last list item is equivalent to rollback
      for name in self.ResolutionVector['setpoints']:
        if name.startswith('BatteryUnit.'):
          self.p_batt_plot[name][-1] = self.ResolutionVector['setpoints'][name]
          self.soc_plot[name][-1] = self.Batteries[name]['SoC']

    return


  def __init__(self, gapps, method, feeder_mrid, simulation_id):
    self.AppUtil = getattr(importlib.import_module('shared.apputil'), 'AppUtil')

    SPARQLManager = getattr(importlib.import_module('shared.sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = self.AppUtil.getBatteries(sparql_mgr)

    '''
    # SHIVA HACK for 123 model testing
    if feeder_mrid == '_C1C3E687-6FFD-C753-582B-632A27E28507':
      self.Batteries['BatteryUnit.65'] = {'idx': 0, 'prated': 250000,
            'phase': 'A', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.35}
      self.Batteries['BatteryUnit.65']['eff_c'] = \
                                   self.Batteries['BatteryUnit.65']['eff_d'] = \
                                   self.Batteries['BatteryUnit.65']['eff']
      self.Batteries['BatteryUnit.52'] = {'idx': 1, 'prated': 250000,
            'phase': 'B', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.275}
      self.Batteries['BatteryUnit.52']['eff_c'] = \
                                   self.Batteries['BatteryUnit.52']['eff_d'] = \
                                   self.Batteries['BatteryUnit.52']['eff']
      self.Batteries['BatteryUnit.76'] = {'idx': 2, 'prated': 250000,
            'phase': 'C', 'eff': 0.975 * 0.86, 'ratedE': 500000, 'SoC': 0.465}
      self.Batteries['BatteryUnit.76']['eff_c'] = \
                                   self.Batteries['BatteryUnit.76']['eff_d'] = \
                                   self.Batteries['BatteryUnit.76']['eff']
    '''

    # to support the old way of updating SoC for testing
    for name in self.Batteries:
      self.Batteries[name]['RollbackSoC'] = self.Batteries[name]['OldSoC'] = self.Batteries[name]['SoC']

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

    self.ConflictOnlyMatrix = {}
    self.ConflictOnlyMatrix['setpoints'] = {}
    self.ConflictOnlyMatrix['timestamps'] = {}

    self.ResolutionVector = {}
    self.ResolutionVector['setpoints'] = {}
    self.ResolutionVector['timestamps'] = {}

    # Step 0: Import deconfliction methodology class for this invocation of
    #         the Deconflictor based on method command line argument and
    #         create an instance of the class
    if method.endswith('.py'):
      method = method[:-3] # allow full python file name

    dirname = os.path.dirname(method)
    if dirname != '':
      sys.path.append(dirname)

    basename = os.path.basename(method)
    print('Importing DeconflictionMethod class from module: ' + basename,
           flush=True)

    DeconflictionMethod = getattr(importlib.import_module(basename),
                                  'DeconflictionMethod')
    self.decon_method = DeconflictionMethod(self.ConflictMatrix,
                                            self.ConflictOnlyMatrix, False)

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

