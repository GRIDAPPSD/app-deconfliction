
# Copyright (c) 2024, Battelle Memorial Institute All rights reserved.
# Battelle Memorial Institute (hereinafter Battelle) hereby grants permission
# to any person or entity lawfully obtaining a copy of this software and
# associated documentation files (hereinafter the Software) to redistribute and
# use the Software in source and binary forms, with or without modification.
# Such person or entity may use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and may permit others to do
# so, subject to the following conditions:
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimers.
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# Other than as used herein, neither the name Battelle Memorial Institute or
# Battelle may be used in any form whatsoever without the express written
# consent of Battelle.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL BATTELLE OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# General disclaimer for use with OSS licenses
#
# This material was prepared as an account of work sponsored by an agency of
# the United States Government. Neither the United States Government nor the
# United States Department of Energy, nor Battelle, nor any of their employees,
# nor any jurisdiction or organization that has cooperated in the development
# of these materials, makes any warranty, express or implied, or assumes any
# legal liability or responsibility for the accuracy, completeness, or
# usefulness or any information, apparatus, product, software, or process
# disclosed, or represents that its use would not infringe privately owned
# rights.
#
# Reference herein to any specific commercial product, process, or service by
# trade name, trademark, manufacturer, or otherwise does not necessarily
# constitute or imply its endorsement, recommendation, or favoring by the
# United States Government or any agency thereof, or Battelle Memorial
# Institute. The views and opinions of authors expressed herein do not
# necessarily state or reflect those of the United States Government or any
# agency thereof.
#
# PACIFIC NORTHWEST NATIONAL LABORATORY operated by BATTELLE for the
# UNITED STATES DEPARTMENT OF ENERGY under Contract DE-AC05-76RL01830
# ------------------------------------------------------------------------------
"""
Created on March 8, 2023

@author: Gary Black
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
import copy
import queue

from time import sleep


from gridappsd import GridAPPSD
from gridappsd import DifferenceBuilder
from gridappsd.topics import simulation_output_topic, simulation_log_topic, simulation_input_topic, service_output_topic

from datetime import datetime

# for loading shared modules
if (os.path.isdir('shared')):
  sys.path.append('./shared')
elif (os.path.isdir('../shared')):
  sys.path.append('../shared')
elif (os.path.isdir('app-deconfliction/competing-apps/shared')):
  sys.path.append('app-deconfliction/competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')

from AppUtil import AppUtil
import MethodUtil

class DeconflictionPipeline(GridAPPSD):

  def SetpointProcessor(self, app_name, timestamp, set_points):
    # Update ConflictMatrix with newly provided set-points

    # delete any existing matches for app_name so there are no stragglers
    # from past timestamps
    for device in self.ConflictMatrix:
      if app_name in self.ConflictMatrix[device]:
        self.ConflictMatrix[device].pop(app_name)

    # now add the new set-points for app_name
    for point in set_points:
      device = point['object']
      #attribute = point['attribute']
      value = point['value']

      if device not in self.ConflictMatrix:
        self.ConflictMatrix[device] = {}

      self.ConflictMatrix[device][app_name] = (timestamp, value)

    print('ConflictMatrix: ' + str(self.ConflictMatrix), flush=True)

    if self.testDeviceName:
      devid = MethodUtil.NameToDevice[self.testDeviceName]
      if devid in set_points:
        print('~TEST: set-points message with ' + self.testDeviceName +
              ' set-point: ' + str(set_points[devid]) +
              ', app: ' + app_name +
              ', timestamp: ' + str(timestamp), flush=True)
        print('~TEST: ConflictMatrix for ' + self.testDeviceName + ': ' +
              str(self.ConflictMatrix[devid]), flush=True)
      else:
        print('~TEST: set-points message does not contain ' +
              self.testDeviceName, flush=True)


  def ConflictMetric(self, timestamp):
    centroid = {}
    apps = {}
    n_devices = len(self.ConflictMatrix)

    # GDB 5/21/24: Don't crash with an empty ConflictMatrix
    if n_devices == 0:
      print('Conflict Metric: Undefined (no conflicts), timestamp: ' +
            str(timestamp), flush=True)
      return

    for device in self.ConflictMatrix:
      n_apps_device = len(self.ConflictMatrix[device])

      device_setpoints = []

      for app in self.ConflictMatrix[device]:
        gamma_d_a = self.ConflictMatrix[device][app][1]
        if app not in apps:
          apps[app] = {}
        name = MethodUtil.DeviceToName[device]
        if name.startswith('BatteryUnit.'):
          # Normalize setpoints using max charge and discharge possible
          sigma_d_a = (gamma_d_a + self.Batteries[device]['prated']) / \
                      (2 * self.Batteries[device]['prated'])
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

        elif name.startswith('RatioTapChanger.'):
          # Normalize setpoints using highStep and lowStep
          sigma_d_a = (gamma_d_a + abs(self.Regulators[device]['highStep'])) / \
                      (self.Regulators[device]['highStep'] + \
                       abs(self.Regulators[device]['lowStep']))
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

      # Find centroid
      # GDB 6/26/24: Also don't crash with empty ConflictMatrix for a device
      if n_apps_device > 0:
        centroid[device] = sum(device_setpoints) / n_apps_device

    # Distance vector:
    # Distance between setpoints requested by each app to the centroid vector
    dist_centroid = []
    n_apps = len(apps)
    for app in apps:
      sum_dist = 0
      for device in centroid:
        if device in apps[app]:
          sum_dist += (centroid[device] - apps[app][device]) ** 2

      dist_centroid.append(math.sqrt(sum_dist))

    # Compute conflict metric: average distance
    conflict_metric = sum(dist_centroid) / n_apps
    # Ensuring 0 <= conflict_metric <= 1
    conflict_metric = conflict_metric * 2 / math.sqrt(n_devices)
    print('Conflict Metric: ' + str(conflict_metric) + ', timestamp: ' +
          str(timestamp), flush=True)


  def ConflictIdentification(self, app_name, timestamp, set_points):
    # Determine if there is a new conflict
    for point in set_points:
      device = point['object']

      for app1 in self.ConflictMatrix[device]:
        if app1!=app_name and \
           (set_points[device]!=self.ConflictMatrix[device][app1][1] \
            or timestamp!=self.ConflictMatrix[device][app1][0]):
          # once a conflict is found, return immediately
          return True

    return False


  def OptimizationDeconflict(self, app_name, timestamp):
    ResolutionVector = {}

    for device in self.ConflictMatrix:
      optTimestamp = 0
      optNumerator = 0.0
      optDenominator = 0.0

      for app in self.ConflictMatrix[device]:
        optTimestamp = max(optTimestamp, self.ConflictMatrix[device][app][0])

        if app in self.OptDevWeights and device in self.OptDevWeights[app]:
          optNumerator += self.ConflictMatrix[device][app][1] * \
                          self.OptDevWeights[app][device]
          optDenominator += self.OptDevWeights[app][device]

        elif app in self.OptAppWeights:
          optNumerator += self.ConflictMatrix[device][app][1] * \
                          self.OptAppWeights[app]
          optDenominator += self.OptAppWeights[app]

        else:
          optNumerator += self.ConflictMatrix[device][app][1]
          optDenominator += 1.0

      if optDenominator > 0.0:
        name = MethodUtil.DeviceToName[device]
        if name.startswith('RatioTapChanger.'):
          ResolutionVector[device] = (optTimestamp,
                                      round(optNumerator/optDenominator))
        else:
          ResolutionVector[device] = (optTimestamp, optNumerator/optDenominator)

    return ResolutionVector


  def DeconflictionToResolution(self, app_name, timestamp, set_points,
                                conflictFlag):
    # If there is a conflict, then perform combined/staged deconfliction to
    # produce a resolution
    if conflictFlag:
      # TODO: This is where the Rules & Heuristics stage deconfliction will go

      # TODO: This is where the Cooperation stage deconfliction will go

      # Optimization stage deconfliction
      newResolutionVector = self.OptimizationDeconflict(app_name, timestamp)
      print('ResolutionVector (conflict): ' + str(newResolutionVector),
            flush=True)

      if self.testDeviceName:
        devid = MethodUtil.NameToDevice[self.testDeviceName]
        if devid in newResolutionVector:
          print('~TEST: ResolutionVector (conflict) for ' +
                self.testDeviceName + ' setpoint: ' +
                str(newResolutionVector[devid][1]) + ', timestamp: ' +
                str(newResolutionVector[devid][0]), flush=True)
        else:
          print('~TEST: ResolutionVector (conflict) does not contain ' +
                self.testDeviceName, flush=True)

    else: # no conflict
      # start with a copy of the previous resolution
      newResolutionVector = copy.deepcopy(self.ResolutionVector)

      # next copy the new set-points over top of the previous resolution
      for point in set_points:
        newResolutionVector[point['object']] = (timestamp, point['value'])

      print('ResolutionVector (no conflict): ' + str(newResolutionVector),
            flush=True)

      if self.testDeviceName:
        devid = MethodUtil.NameToDevice[self.testDeviceName]
        if devid in newResolutionVector:
          print('~TEST: ResolutionVector (no conflict) for ' +
                self.testDeviceName + ' setpoint: ' +
                str(newResolutionVector[devid][1]) +
                ', timestamp: ' +
                str(newResolutionVector[devid][0]),
                flush=True)
        else:
          print('~TEST: ResolutionVector (no conflict) does not contain ' +
                self.testDeviceName, flush=True)

    return newResolutionVector


  def DeviceDispatcher(self, timestamp, newResolutionVector):
    # Iterate over resolution and send set-points to devices that have
    # different values
    diffCount = 0

    for devid, value in newResolutionVector.items():
      name = MethodUtil.DeviceToName[devid]
      if name.startswith('BatteryUnit.'):
        if value[1] != self.Batteries[devid]['P_batt_inv']:
          #new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(devid,
                                       'PowerElectronicsConnection.p', value[1],
                                       self.Batteries[devid]['P_batt_inv'])
          diffCount += 1

          print('~~> Dispatching to battery device: ' + devid + ', name: ' +
                name + ', timestamp: ' + str(timestamp) + ', new value: ' +
                str(value[1]) + ', old value: ' +
                str(self.Batteries[devid]['P_batt_inv']), flush=True)

          if self.testDeviceName and name==self.testDeviceName:
            print('~TEST: Dispatching to battery device: ' + devid +
                  ', name: ' + name + ', timestamp: ' + str(timestamp) +
                  ', new value: ' + str(value[1]) + ', old value: ' +
                  str(self.Batteries[devid]['P_batt_inv']), flush=True)

        else:
          print('~~> NO DISPATCH needed to battery device: ' + devid +
                ', name: ' + name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]), flush=True)

      elif name.startswith('RatioTapChanger.'):
        # Dispatch regulator tap positions whenever they are different from the
        # current tap position
        if value[1] != self.Regulators[devid]['step']:
          # new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(devid,
                   'TapChanger.step', value[1], self.Regulators[devid]['step'])
          diffCount += 1

          print('==> Dispatching to regulator device: ' + devid + ', name: ' +
                name + ', timestamp: ' + str(timestamp) + ', new value: ' +
                str(value[1]) + ', old value: ' +
                str(self.Regulators[devid]['step']), flush=True)

          if self.testDeviceName and name==self.testDeviceName:
              print('~TEST: Dispatching to regulator device: ' + devid +
                    ', name: ' + name + ', timestamp: ' + str(timetstamp) +
                    ', new value: ' + str(value[1]) + ', old value: ' +
                    str(self.Regulators[devid]['step']), flush=True)

        else:
          print('~~> NO DISPATCH needed to regulator device: ' + devid +
                ', name: ' + name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]), flush=True)

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector) > len(newResolutionVector):
      for devid in self.ResolutionVector:
        if devid not in newResolutionVector:
          print('==> Device deleted from resolution: ' + devid +
                ', name: ' + MethodUtil.DeviceToName[devid], flush=True)

          if self.testDeviceName and \
             MethodUtil.DeviceToName[devid]==self.testDeviceName:
            print('~TEST: Device deleted from resolution: ' + devid +
                  ', name: ' + MethodUtil.DeviceToName[devid], flush=True)

    if diffCount > 0:
      dispatch_message = self.difference_builder.get_message()
      print('~~> Sending device dispatch DifferenceBuilder message: ' +
            json.dumps(dispatch_message), flush=True)
      self.gapps.send(self.publish_topic, json.dumps(dispatch_message))
      self.difference_builder.clear()


  def on_sim_message(self, header, message):
    #print('Received simulation message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      status = message['processStatus']
      if status=='COMPLETE' or status=='CLOSED':
        self.keepLoopingFlag = False
        print('Simulation ' + status + ' message received', flush=True)

    else:
      self.messageQueue.put((None, message['message']))


  def pol2cart(self, mag, angle_deg):
        # Convert degrees to radians. GridAPPS-D spits angle in degrees
        angle_rad =  math.radians(angle_deg)
        p = mag * np.cos(angle_rad)
        q = mag * np.sin(angle_rad)
        return p, q


  def processSimulationMessage(self, message):
    # update SoC values in MethodUtil for same reason
    measurements = message['measurements']
    for devid in self.Batteries:
      measid = self.Batteries[devid]['SoC_measid']
      if measid in measurements:
        self.Batteries[devid]['SoC'] = measurements[measid]['value']/100.0
        MethodUtil.BatterySoC[devid] = self.Batteries[devid]['SoC']
        print('Timestamp ' + str(message['timestamp']) + ' updated SoC for ' + self.Batteries[devid]['name'] + ': ' + str(self.Batteries[devid]['SoC']), flush=True)

      measid = self.Batteries[devid]['P_batt_measid']
      if measid in measurements:
        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])
        # negate the p value from the simulation so it is directly comparable
        # to the value that must be given to GridLAB-D in a DifferenceBuilder
        # message
        self.Batteries[devid]['P_batt_inv'] = -p
        MethodUtil.BatteryP_batt_inv[devid] =self.Batteries[devid]['P_batt_inv']
        print('Timestamp ' + str(message['timestamp']) + ' updated P_batt_inv for ' + self.Batteries[devid]['name'] + ': ' + str(self.Batteries[devid]['P_batt_inv']), flush=True)

    for devid in self.Regulators:
      measid = self.Regulators[devid]['measid']
      if measid in measurements:
        self.Regulators[devid]['step'] = measurements[measid]['value']
        MethodUtil.RegulatorPos[devid] = self.Regulators[devid]['step']
        print('Timestamp ' + str(message['timestamp']) + ' updated tap position for ' + self.Regulators[devid]['name'] + ': ' + str(self.Regulators[devid]['step']), flush=True)

    if self.testDeviceName:
      devid = MethodUtil.NametoDevice[self.testDeviceName]
      if devid in self.Batteries:
        print('~TEST simulation updated SoC for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', SoC: ' + str(self.Batteries[devid]['SoC']), flush=True)
        print('~TEST simulation updated P_batt_inv for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', P_batt_inv: ' + str(self.Batteries[devid]['P_batt_inv']), flush=True)
      elif devid in self.Regulators:
        print('~TEST simulation updated tap position for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', pos: ' + str(self.Regulators[devid]['step']), flush=True)


  def on_setpoints_message(self, header, message):
    print('### Received set-points message: ' + str(message), flush=True)
    print('### Received set-points header: ' + str(header), flush=True)
    self.messageQueue.put((header, message['input']['message']))


  def get_app_name(self, header):
    app_name = header['destination']

    if app_name.startswith('/topic/goss.gridappsd.simulation.'):
      endind = app_name[33:].find('.')
      app_name = app_name[33:33+endind]

    return app_name


  def processSetpointsMessage(self, message, header):
    # for SHIVA conflict metric testing
    #realTime = round(time.time(), 4)

    app_name = self.get_app_name(header)

    timestamp = message['timestamp']

    # set_points used to be a simple device, value dictionary. Now it is the
    # forward_differences part of the DifferenceBuilder message with keys of
    # object, attribute, and value.
    set_points = message['forward_differences']

    # for checking order of messages received
    print('~ORDER: timestamp: ' + str(timestamp) + ', app: ' + app_name,
          flush=True)

    # Step 1: Setpoint Processor
    self.SetpointProcessor(app_name, timestamp, set_points)

    self.ConflictMetric(timestamp)

    # Step 2: Feasibility Maintainer -- not implemented for prototype

    # Step 3: Deconflictor
    # Step 3.1: Conflict Identification
    conflictFlag = self.ConflictIdentification(app_name, timestamp, set_points)

    # Steps 3.2 and 3.3: Deconfliction Solution and Resolution
    newResolutionVector = self.DeconflictionToResolution(app_name, timestamp,
                                                       set_points, conflictFlag)

    # Step 4: Setpoint Validator -- not implemented for prototype

    # Step 5: Device Dispatcher
    self.DeviceDispatcher(timestamp, newResolutionVector)

    print('~', flush=True) # tidy output with "blank" line

    # Update the current resolution to the new resolution to be ready for the
    # next set-points message
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # for SHIVA conflict metric testing
    #self.TimeConflictMatrix[realTime] = copy.deepcopy(self.ConflictMatrix)
    #self.TimeResolutionVector[realTime] = copy.deepcopy(self.ResolutionVector)


  def __init__(self, gapps, feeder_mrid, simulation_id, weights_base):
    self.gapps = gapps

    self.messageQueue = queue.Queue()

    # must enumerate all possible apps since I need separate topics for each
    # to distinguish them via message header
    competing_apps = ['gridappsd-resilience-app',
                      'gridappsd-decarbonization-app',
                      'gridappsd-profit_cvr-app']
    set_id = {}
    for app in competing_apps:
      # subscribe to competing app set-points messages
      set_id[app] = gapps.subscribe(service_output_topic(app,
                                    simulation_id), self.on_setpoints_message)

    # subscribe to simulation log and output messages
    self.keepLoopingFlag = True
    out_id = gapps.subscribe(simulation_output_topic(simulation_id),
                             self.on_sim_message)
    log_id = gapps.subscribe(simulation_log_topic(simulation_id),
                             self.on_sim_message)

    # simulation topic for sending DifferenceBuilder messages
    self.publish_topic = simulation_input_topic(simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

    # test/debug settings
    # set this to the name of the device for detailed testing, e.g.,
    # 'BatteryUnit.battery1', or None to omit test output
    self.testDeviceName = None
    #self.testDeviceName = 'BatteryUnit.battery1'

    SPARQLManager = getattr(importlib.import_module('sparql'),
                            'SPARQLManager')
    MethodUtil.sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = AppUtil.getBatteries(MethodUtil.sparql_mgr)
    print('Starting Batteries: ' + str(self.Batteries), flush=True)

    self.Regulators = AppUtil.getRegulators(MethodUtil.sparql_mgr)
    print('Starting Regulators: ' + str(self.Regulators), flush=True)

    self.deltaT = 0.25 # timestamp interval in fractional hours, 0.25 = 15 min

    self.ConflictMatrix = {}
    self.ResolutionVector = {}

    # for SHIVA conflict metric testing
    #self.TimeConflictMatrix = {}
    #self.TimeResolutionVector = {}

    # Optimization weighting factors dictionaries
    self.OptAppWeights = {}
    self.OptDevWeights = {}

    if weights_base != None:
      # Load weighting factors from json files
      appname =  weights_base + "-app.json"
      appflag = False
      try:
        with open(appname) as f:
          data = f.read()
          self.OptAppWeights = json.loads(data)
          print('\nApplying optimization deconfliction stage application weighting factors in ' + appname + ': ' + str(self.OptAppWeights))
      except:
        appflag = True

      devname =  weights_base + "-dev.json"
      devflag = False
      try:
        with open(devname) as f:
          data = f.read()
          self.OptDevWeights = json.loads(data)
          print('\nApplying optimization deconfliction stage device weighting factors in ' + devname + ': ' + str(self.OptDevWeights))
      except:
        devflag = True

      if appflag and devflag:
        print('\n*** WARNING: Could not find or load either optimization weighting factors files ' + appname + ' or ' + devname)

    else:
      print('\nNo optimization deconfliction stage weighting factors applied')

    print('\nInitialized deconfliction pipeline, waiting for messages...\n',
          flush=True)

    while self.keepLoopingFlag:
      if self.messageQueue.qsize() == 0:
        sleep(0.1)
        continue

      # GDB 5/20/24: Queue draining for the pipeline can't be done the same
      # way as the competing apps because the queue isn't all just simulation
      # messages, but also setpoints messages. Only drain messages if they are
      # simulation messages.
      # This will need to be revisited if the pipeline starts falling behind
      # as more sophisticated deconfliction is implemented and new simulation
      # messages arriving every 3 seconds. In that case it seems as if older
      # setpoints messages could be discarded, but it's not straightforward
      # as it would only make sense to discard when there is a newer setpoints
      # message from the same competing app.
      while self.messageQueue.qsize() > 0:
        header, message = self.messageQueue.get()
        if header != None:
          break

      if header == None:
        self.processSimulationMessage(message)

      else:
        self.processSetpointsMessage(message, header)

    for app in competing_apps:
      gapps.unsubscribe(set_id[app])
    gapps.unsubscribe(out_id)
    gapps.unsubscribe(log_id)

    # for SHIVA conflict metric
    #json_file = open('log/ConflictMatrix_' + basename + '.json', 'w')
    #json.dump(self.TimeConflictMatrix, json_file, indent=4)
    #json_file.close()
    #json_file = open('log/ResolutionVector_' + basename + '.json', 'w')
    #json.dump(self.TimeResolutionVector, json_file, indent=4)
    #json_file.close()


def _main():
  print('Starting deconfliction pipeline code...', flush=True)

  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("--weights", nargs='?', help="Optimization Weights Base Filename")
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

  DeconflictionPipeline(gapps, feeder_mrid, opts.simulation_id, opts.weights)

  print('Goodbye!', flush=True)


if __name__ == "__main__":
  _main()

