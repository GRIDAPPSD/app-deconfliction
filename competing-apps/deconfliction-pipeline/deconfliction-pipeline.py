
# Copyright (c) 2023, Battelle Memorial Institute All rights reserved.
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
import time

from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic

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
    for device, value in set_points.items():
      #print('device: ' + device + ', value: ' + str(value), flush=True)
      if device.startswith('BatteryUnit.'):
        print('~~> setpoints from app: ' + app_name +
              ', timestamp: ' + str(timestamp) +
              ', device: ' + device + ', value: ' + str(value), flush=True)

      if device not in self.ConflictMatrix:
        self.ConflictMatrix[device] = {}

      self.ConflictMatrix[device][app_name] = (timestamp, value)

    print('ConflictMatrix: ' + str(self.ConflictMatrix), flush=True)

    if self.testDevice:
      if  self.testDevice in set_points:
        print('~TEST: set-points message with ' + self.testDevice +
              ' set-point: ' + str(set_points[self.testDevice]) +
              ', app: ' + app_name +
              ', timestamp: ' + str(timestamp), flush=True)
        print('~TEST: ConflictMatrix for ' + self.testDevice + ': ' +
             str(self.ConflictMatrix[self.testDevice]), flush=True)
      else:
        print('~TEST: set-points message does not contain ' + self.testDevice,
              flush=True)



  def ConflictMetric(self, timestamp):
    centroid = {}
    apps = {}
    n_devices = len(self.ConflictMatrix)
    for device in self.ConflictMatrix:
      n_apps_device = len(self.ConflictMatrix[device])
      device_setpoints = []

      for app in self.ConflictMatrix[device]:
        gamma_d_a = self.ConflictMatrix[device][app][1]
        if app not in apps:
          apps[app] = {}
        if device.startswith('BatteryUnit.'):
          # Normalize setpoints using max charge and discharge possible
          sigma_d_a = (gamma_d_a + self.Batteries[device]['prated']) / \
                      (2 * self.Batteries[device]['prated'])
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)
        elif device.startswith('RatioTapChanger.'):
          # Normalize setpoints using highStep and lowStep
          sigma_d_a = (gamma_d_a + abs(self.Regulators[device]['highStep'])) / \
                      (self.Regulators[device]['highStep'] + \
                       abs(self.Regulators[device]['lowStep']))
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

      # Find centroid
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
    for device in set_points:
      for app1 in self.ConflictMatrix[device]:
        if app1!=app_name and \
           (set_points[device]!=self.ConflictMatrix[device][app1][1] \
            or timestamp!=self.ConflictMatrix[device][app1][0]):
          # once a conflict is found, return immediately
          return True

    return False


  def DeconflictionToResolution(self, app_name, timestamp, set_points,
                                conflictFlag):
    # If there is a conflict, then the call the deconflict method for the given
    # methodology to produce a resolution
    if conflictFlag:
      newResolutionVector = self.decon_method.deconflict(app_name, timestamp)
      print('ResolutionVector (conflict): ' + str(newResolutionVector),
            flush=True)

      # GDB 6/2/23 The logic that updates ConflictMatrix below is flawed
      # in that updates effectively indicate that apps are happy with the
      # resolution set-points when really they may not be, which will impact
      # subsequent resolutions in ways that are likely not valid
      '''
      # Update ConflictMatrix with resolution vector setpoint values so the
      # deconfliction methodologies don't need to resolve the same conflicts
      # each time in addition to new conflicts from the latest set-points
      for device, value in newResolutionVector.items():
        for app in self.ConflictMatrix[device]:
          self.ConflictMatrix[device][app] = \
                  (max(value[0], self.ConflictMatrix[device][app][0]), value[1])
      '''

      if self.testDevice:
        if self.testDevice in newResolutionVector:
          print('~TEST: ResolutionVector (conflict) for ' +
                self.testDevice + ' setpoint: ' +
                str(newResolutionVector[self.testDevice][1]) +
                ', timestamp: ' +
                str(newResolutionVector[self.testDevice][0]),
                flush=True)
        else:
          print('~TEST: ResolutionVector (conflict) does not contain ' +
                self.testDevice, flush=True)

    else: # no conflict
      # start with a copy of the previous resolution
      newResolutionVector = copy.deepcopy(self.ResolutionVector)

      # next copy the new set-points over top of the previous resolution
      for device, value in set_points.items():
        # uncomment the following line to only include batteries in resolution
        # for testing
        #if device.startswith('BatteryUnit.'):
          newResolutionVector[device] = (timestamp, value)

      print('ResolutionVector (no conflict): ' + str(newResolutionVector),
            flush=True)

      if self.testDevice:
        if self.testDevice in newResolutionVector:
          print('~TEST: ResolutionVector (no conflict) for ' +
                self.testDevice + ' setpoint: ' +
                str(newResolutionVector[self.testDevice][1]) +
                ', timestamp: ' +
                str(newResolutionVector[self.testDevice][0]),
                flush=True)
        else:
          print('~TEST: ResolutionVector (no conflict) does not contain ' +
                self.testDevice, flush=True)


    if self.testDeconMethodFlag:
      if conflictFlag:
        testNewResolutionVector = self.decon_method_test.deconflict(timestamp)

      else:
        testNewResolutionVector = copy.deepcopy(self.ResolutionVector)

        for device, value in set_points.items():
          # uncomment the following line to only include batteries in resolution
          # for testing
          #if device.startswith('BatteryUnit.'):
            testNewResolutionVector[device] = (timestamp, value)

      testDiffFlag = False
      for device in newResolutionVector:
        if device not in testNewResolutionVector:
          testDiffFlag = True
          break

        if device.startswith('BatteryUnit.'):
          if abs(newResolutionVector[device][1] - \
                 testNewResolutionVector[device][1]) > 1e-6:
            testDiffFlag = True
            break
        else:
          if newResolutionVector[device][1] != \
             testNewResolutionVector[device][1]:
            testDiffFlag = True
            break

        if newResolutionVector[device][0] != \
           testNewResolutionVector[device][0]:
          testDiffFlag = True
          break

      if testDiffFlag:
        print('!!!!!!!!!!!!!!!!!!!!!!!!! DIFF TEST ResolutionVector: ' +
              str(testNewResolutionVector), flush=True)
        exit()

    return newResolutionVector


  ''' SOC MOVE
  def updateSoC(self, name, P_batt, timestamp, revised_socs):
    if 'refTimestamp' not in self.Batteries[name]:
      self.Batteries[name]['refTimestamp'] = timestamp
      self.Batteries[name]['runTimestamp'] = timestamp
      self.Batteries[name]['refP_batt'] = P_batt
      self.Batteries[name]['runP_batt'] = P_batt
      self.Batteries[name]['refSoC'] = self.Batteries[name]['SoC']
      self.Batteries[name]['runSoC'] = self.Batteries[name]['SoC']

    if self.testDevice and name==self.testDevice:
      print('~TEST updateSoC for device: ' + name + ', timestamp: ' +
            str(timestamp) + ', P_batt: ' + str(P_batt) +
            ', SoC: ' + str(self.Batteries[name]['SoC']), flush=True)
      print('~TEST updateSoC for device: ' + name + ', refTimestamp: ' +
            str(self.Batteries[name]['refTimestamp']) + ', runTimestamp: ' +
            str(self.Batteries[name]['runTimestamp']), flush=True)
      print('~TEST updateSoC for device: ' + name + ', refP_batt: ' +
            str(self.Batteries[name]['refP_batt']) + ', runP_batt: ' +
            str(self.Batteries[name]['runP_batt']), flush=True)
      print('~TEST updateSoC for device: ' + name + ', refSoC: ' +
            str(self.Batteries[name]['refSoC']) + ', runSoC: ' +
            str(self.Batteries[name]['runSoC']), flush=True)

    if timestamp > self.Batteries[name]['runTimestamp']:
      if self.testDevice and name==self.testDevice:
        print('~TEST updateSoC for device: ' + name +
              ', updating refSoC and refTimestamp to run values', flush=True)

      self.Batteries[name]['refTimestamp'] = \
                                       self.Batteries[name]['runTimestamp']
      self.Batteries[name]['refP_batt'] = self.Batteries[name]['runP_batt']
      self.Batteries[name]['refSoC'] = self.Batteries[name]['runSoC']

    actContrib = 0.0
    if timestamp > self.Batteries[name]['refTimestamp']:
      if self.testDevice and name==self.testDevice:
        print('~TEST updateSoC actual contrib based on P_batt: ' +
              str(self.Batteries[name]['refP_batt']) +
              ', current timestamp: ' + str(timestamp) +
              ', reference timestamp: ' +
              str(self.Batteries[name]['refTimestamp']), flush=True)

      actContrib = AppUtil.contrib_SoC(self.Batteries[name]['refP_batt'],
                                 timestamp-self.Batteries[name]['refTimestamp'],
                                 self.Batteries[name], self.deltaT)

    elif timestamp < self.Batteries[name]['refTimestamp']:
      # consider going back in time the same as equal timestamps other than
      # reporting it
      print('*** WARNING: time has gone backwards with set-points message for device: ' +
            name + ', timestamp: ' + str(timestamp) + ', last timestamp: ' +
            str(self.Batteries[name]['refTimestamp']), flush=True)

      if self.testDevice and name==self.testDevice:
        print('~TEST WARNING: time has gone backwards with set-points message for device: ' +
              name + ', timestamp: ' + str(timestamp) + ', last timestamp: ' +
              str(self.Batteries[name]['refTimestamp']), flush=True)

    self.Batteries[name]['runSoC'] = self.Batteries[name]['refSoC'] + actContrib

    projContrib = AppUtil.contrib_SoC(P_batt, 1, self.Batteries[name],
                                      self.deltaT)
    projSoC = self.Batteries[name]['runSoC'] + projContrib

    print('~SOC updateSoC magic for device: ' + name + ', reference SoC: ' +
          str(self.Batteries[name]['refSoC']) + ', actual SoC contrib: ' +
          str(actContrib) + ', projected SoC contrib: ' + str(projContrib),
          end='')

    constrainedSoC = projSoC
    if projSoC > 0.9:
      constrainedSoC = 0.9
    elif projSoC < 0.2:
      constrainedSoC = 0.2

    if constrainedSoC != projSoC:
      print(', projected constrained SoC: ' + str(constrainedSoC), end='')
    else:
      print(', projected SoC: ' + str(projSoC), end='')

    if constrainedSoC != self.Batteries[name]['SoC']:
      revised_socs[name] = self.Batteries[name]['SoC'] = \
                           MethodUtil.BatterySoC[name] = constrainedSoC
      print(' (revised)', flush=True)
    else:
      print(' (not revised)', flush=True)

    if self.testDevice and name==self.testDevice:
      if constrainedSoC != projSoC:
        print('~TEST updateSoC magic for device: ' + name + ', reference SoC: '+
              str(self.Batteries[name]['refSoC']) + ', actual SoC contrib: ' +
              str(actContrib) + ', projected SoC contrib: ' + str(projContrib) +
              ', projected constrained SoC: ' + str(constrainedSoC), flush=True)
      else:
        print('~TEST updateSoC magic for device: ' + name + ', reference SoC: '+
              str(self.Batteries[name]['refSoC']) + ', actual SoC contrib: ' +
              str(actContrib) + ', projected SoC contrib: ' + str(projContrib) +
              ', projected SoC: ' + str(projSoC), flush=True)

    self.Batteries[name]['runTimestamp'] = max(timestamp,
                                           self.Batteries[name]['runTimestamp'])
    self.Batteries[name]['runP_batt'] = P_batt
  '''


  def DeviceDispatcher(self, timestamp, newResolutionVector):
    # Iterate over resolution and send set-points to devices that have
    # different or new values
    DevicesToDispatch ={}
    ''' SOC MOVE
    revised_socs = {}
    '''
    for device, value in newResolutionVector.items():
      if device.startswith('BatteryUnit.'):
        # batteries dispatch values even if they are the same as the last time
        # as long as the value is associated with the current timestamp

        # GDB 6/22/23: First is the original version and then the new version
        # that doesn't dispatch P_batt of 0 beyond the initial change to 0
        #if device not in self.ResolutionVector or \
        #   (newResolutionVector[device][0]==timestamp and \
        #    (self.ResolutionVector[device][0]!=timestamp or \
        #     self.ResolutionVector[device][1]!=value[1])):
        if device not in self.ResolutionVector or \
           (newResolutionVector[device][0]==timestamp and \
            (self.ResolutionVector[device][1]!=value[1] or \
             (self.ResolutionVector[device][0]!=timestamp and \
              self.ResolutionVector[device][1]!=0.0))):
          DevicesToDispatch[device] = value[1]

          print('~~> Dispatching to device: ' + device + ', timestamp: ' +
                str(timestamp) + ', value: ' + str(value[1]), flush=True)

          ''' SOC MOVE
          self.updateSoC(device, value[1], timestamp, revised_socs)
          print('~~> Dispatching to device: ' + device + ', timestamp: ' +
                str(timestamp) + ', value: ' + str(value[1]) +
                ' (projected SoC: ' + str(self.Batteries[device]['SoC']) + ')',
                flush=True)
          '''

          if self.testDevice and device==self.testDevice:
            print('~TEST: Dispatching to device: ' + device + ', timestamp: ' +
                  str(timestamp) + ', value: ' + str(value[1]), flush=True)
            ''' SOC MOVE
            print('~TEST: Dispatching to device: ' + device + ', timestamp: ' +
                  str(timestamp) + ', value: ' + str(value[1]) +
                  ' (projected SoC: ' + str(self.Batteries[device]['SoC']) +')'
                  , flush=True)
            '''

      # not a battery so only dispatch value if it's changed or if it's
      # never been dispatched before
      elif device not in self.ResolutionVector or \
           self.ResolutionVector[device][1]!=value[1]:
        DevicesToDispatch[device] = value[1]

        print('==> Dispatching to device: ' + device + ', timestamp: ' +
              str(timestamp) + ', value: ' + str(value[1]), flush=True)

        if self.testDevice and device==self.testDevice:
            print('~TEST: Dispatching to device: ' + device + ', timestamp: ' +
                  str(timetstamp) + ', value: ' + str(value[1]), flush=True)

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector) > len(newResolutionVector):
      for device in self.ResolutionVector:
        if device not in newResolutionVector:
          print('==> Device deleted from resolution: ' + device, flush=True)

          if self.testDevice and device==self.testDevice:
            print('~TEST: Device deleted from resolution: ' + device,flush=True)

    # GDB 6/23/23 I was only dispatching when len(DevicesToDispatch)>0, but
    # we added a mode where sim-sim would count the number of disptach messages
    # to determine when to send out the next timestamp data rather than on a
    # fixed interval so now I always send a message and it is a noop for
    # sim-sim other than counting messages when there are no devices
    dispatch_message = {
      'timestamp': timestamp,
      'dispatch': DevicesToDispatch
    }
    print('~~> Sending device dispatch message: ' + str(dispatch_message),
          flush=True)
    self.gapps.send(self.publish_topic, dispatch_message)

    ''' SOC MOVE
    return revised_socs
    '''


  ''' SOC MOVE
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
      self.gapps.send(self.publish_socs_topic, socs_message)

      if self.testDevice and self.testDevice in revised_socs:
        print('~TEST: revised-socs message with device ' + self.testDevice +
              ' SoC: ' + str(revised_socs[self.testDevice]) + ', timestamp: ' +
              str(timestamp) + ', from app: ' + app_name, flush=True)
  '''


  def on_sim_message(self, headers, message):
    print('Received sim message: ' + str(message), flush=True)

    # trigger an exit based on timestamp
    if message['timestamp'] == '':
      self.exitFlag = True

    # update device set-point values in MethodUtil for the benefit of
    # DeconflictionMethod classes
    MethodUtil.DeviceSetpoints.clear()
    DeviceSetpoints = message['DeviceSetpoints']
    for device, value in DeviceSetpoints.items():
      MethodUtil.DeviceSetpoints[device] = value

    if self.testDevice and self.testDevice in DeviceSetpoints:
      print('~TEST simulation updated set-point for device: ' + self.testDevice+
            ', timestamp: ' + str(message['timestamp']) +
            ', set-point: ' + str(DeviceSetpoints[self.testDevice]), flush=True)

    # update SoC values in MethodUtil for same reason
    BatterySoC = message['BatterySoC']
    for device, value in BatterySoC.items():
      MethodUtil.BatterySoC[device] = value

    if self.testDevice and self.testDevice in BatterySoC:
      print('~TEST simulation updated SoC for device: ' + self.testDevice +
            ', timestamp: ' + str(message['timestamp']) +
            ', SoC: ' + str(BatterySoC[self.testDevice]), flush=True)


  def on_setpoints_message(self, headers, message):
    print('Received set-points message: ' + str(message), flush=True)

    # for SHIVA conflict metric testing
    #realTime = round(time.time(), 4)

    app_name = message['app_name']
    timestamp = message['timestamp']
    set_points = message['set_points']

    # GDB 8/14/23 Comment this out while we get cvxpy optimization working
    #MethodUtil.OptimizationProblem = message['opt_prob']
    #MethodUtil.Objective = message['objective']

    # for checking order of messages received
    print('~ORDER: timestamp: ' + str(timestamp) + ', app: ' + app_name,
          flush=True)

    # Step 1: Setpoint Processor
    self.SetpointProcessor(app_name, timestamp, set_points)

    # GDB 11/2/24 SHIVA NOVEMBER special request
    # count messages so deconfliction is only done when all competing apps
    # have sent their set-point messages for a timestamp
    #if timestamp not in self.message_count:
    #  self.message_count[timestamp] = 1
    #else:
    #  self.message_count[timestamp] += 1

    #if self.message_count[timestamp] < 2: # hardwired for 2 apps currently
    #  return

    #del self.message_count[timestamp]
    #if len(self.message_count) > 0:
    #  print('*** PANIC: set-point message counter in bad state: ' + str(self.message_counter), flush=True)
    #  exit()
    # end SHIVA NOVEMBER special request

    self.ConflictMetric(timestamp)

    # for Alex
    #print('!!! ALEX ConflictMatrix START !!!', flush=True)
    #pprint.pprint(self.ConflictMatrix)
    #print('!!! ALEX ConflictMatrix FINISH !!!', flush=True)

    # Step 2: Feasibility Maintainer -- not implemented for prototype

    # Step 3: Deconflictor
    # Step 3.1: Conflict Identification
    conflictFlag = self.ConflictIdentification(app_name, timestamp, set_points)

    # Steps 3.2 and 3.3: Deconfliction Solution and Resolution
    newResolutionVector = self.DeconflictionToResolution(app_name, timestamp,
                                                       set_points, conflictFlag)

    # Step 4: Setpoint Validator -- not implemented for prototype

    # Step 5: Device Dispatcher
    ''' SOC MOVE
    revised_socs = self.DeviceDispatcher(timestamp, newResolutionVector)
    '''
    self.DeviceDispatcher(timestamp, newResolutionVector)

    ''' SOC MOVE
    # Feedback loop with competing apps through revised SoC values so they
    # can make new set-point requests based on actual changes
    self.AppFeedback(app_name, timestamp, revised_socs)
    '''

    print('~', flush=True) # tidy output with "blank" line

    # Update the current resolution to the new resolution to be ready for the
    # next set-points message
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # for SHIVA conflict metric testing
    #self.TimeConflictMatrix[realTime] = copy.deepcopy(self.ConflictMatrix)
    #self.TimeResolutionVector[realTime] = copy.deepcopy(self.ResolutionVector)

    # for ALEX
    #print('!!! ALEX ResolutionVector START !!!', flush=True)
    #pprint.pprint(self.ResolutionVector)
    #print('!!! ALEX ResolutionVector FINISH !!!', flush=True)


  def __init__(self, gapps, feeder_mrid, simulation_id, method, method_test):
    self.gapps = gapps

    # subscribe to competing app set-points messages
    gapps.subscribe(service_output_topic('gridappsd-competing-app',
                                      simulation_id), self.on_setpoints_message)

    # subscribe to simulation output messages to get updated SoC values
    gapps.subscribe(service_output_topic('gridappsd-sim-sim', simulation_id),
                                         self.on_sim_message)

    # test/debug settings
    self.testDeconMethodFlag = method_test != None
    # set this to the name of the device for detailed testing, e.g.,
    # 'BatteryUnit.battery1', or None to omit test output
    self.testDevice = None
    #self.testDevice = 'BatteryUnit.battery1'

    # GDB 11/2/24 SHIVA NOVEMBER special request
    #self.message_count = {}
    # End SHIVA NOVEMBER special request

    SPARQLManager = getattr(importlib.import_module('sparql'),
                            'SPARQLManager')
    MethodUtil.sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = AppUtil.getBatteries(MethodUtil.sparql_mgr)

    self.Regulators = AppUtil.getRegulators(MethodUtil.sparql_mgr)

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

    # initialize BatterySoC dictionary for deconfliction method usage
    for name in self.Batteries:
      MethodUtil.BatterySoC[name] = self.Batteries[name]['SoC']

    self.deltaT = 0.25 # timestamp interval in fractional hours, 0.25 = 15 min

    self.ConflictMatrix = {}
    self.ResolutionVector = {}

    # for SHIVA conflict metric testing
    #self.TimeConflictMatrix = {}
    #self.TimeResolutionVector = {}

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
    self.decon_method = DeconflictionMethod(self.ConflictMatrix)

    if self.testDeconMethodFlag:
      if method_test.endswith('.py'):
        method_test = method_test[:-3] # allow full python file name

      dirname_test = os.path.dirname(method_test)
      if dirname_test!='' and dirname_test!=dirname:
        sys.path.append(dirname_test)

      basename_test = os.path.basename(method_test)
      print('Importing DeconflictionMethod test class from module: ' +
            basename_test, flush=True)

      DeconflictionMethodTest = getattr(importlib.import_module(basename_test),
                                        'DeconflictionMethod')
      self.decon_method_test = DeconflictionMethod(self.ConflictMatrix)

    ''' SOC MOVE
    self.publish_socs_topic = service_output_topic(
                                   'gridappsd-deconfliction-pipeline-socs', '0')
    '''

    self.publish_topic = service_output_topic(
                                        'gridappsd-deconfliction-pipeline', '0')

    print('Initialized deconfliction pipeline, waiting for messages...\n',
          flush=True)

    self.exitFlag = False

    while not self.exitFlag:
      time.sleep(0.1)

    # for SHIVA conflict metric
    #json_file = open('output/ConflictMatrix_' + basename + '.json', 'w')
    #json.dump(self.TimeConflictMatrix, json_file, indent=4)
    #json_file.close()
    #json_file = open('output/ResolutionVector_' + basename + '.json', 'w')
    #json.dump(self.TimeResolutionVector, json_file, indent=4)
    #json_file.close()


def _main():
  print('Starting deconfliction pipeline code...', flush=True)

  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("method", help="Deconfliction Methodology")
  parser.add_argument("method_test", nargs='?',
                      help="Test Deconfliction Methodology")
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

  DeconflictionPipeline(gapps, feeder_mrid, opts.simulation_id,
                        opts.method, opts.method_test)


if __name__ == "__main__":
  _main()

