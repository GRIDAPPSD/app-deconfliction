
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
Created: March 8, 2023
FY23 Prototype Last Updated: August 14, 2023
FY24 Service Last Updated: September 26, 2024

@author: Gary Black
"""""

import sys
import os
import argparse
import json
#import pprint
import math
import copy
from time import sleep
from datetime import datetime

# GDB 8/28/25: Magic that puts message handling into its own process
# to make sure it is keeping up with simulation measurements
from multiprocessing import Process, Queue


from gridappsd import GridAPPSD
from gridappsd import DifferenceBuilder
from gridappsd.topics import simulation_input_topic, simulation_output_topic
from gridappsd.topics import simulation_log_topic
from gridappsd.topics import service_input_topic, service_output_topic

# magic so all print statements flush without having to add flush=True
#import functools
#print = functools.partial(print, flush=True)

# TODO: this flag needs to be changed when running as a containerized service
# vs. an application started from wrapper scripts like optimization apps
deconflictionAsServiceFlag = False
#deconflictionAsServiceFlag = True
logDir = 'log/'
if deconflictionAsServiceFlag:
  logDir = '/tmp/'

# redirect stderr to a file
sys.stderr = open(logDir + 'deconfliction-pipeline-stderr.log', 'w')


# went with prlog for name so it's the exact length as print since I had to
# replace all the existing print calls and that would be a mess otherwise
def prlog(msg):
  try:
    print(msg, flush=True)
    with open(logDir + 'deconfliction-pipeline.log', 'a') as flog:
      flog.write(msg + '\n')
  except:
    pass


def msglog(msg):
  try:
    with open(logDir + 'deconfliction-pipeline-messages.log', 'a') as flog:
      flog.write(str(datetime.now()) + ': ' + msg + '\n')
  except:
    pass


prlog('Starting deconfliction pipeline...')

# for loading the shared modules just below
if (os.path.isdir('shared')):
  sys.path.append('./shared')
elif (os.path.isdir('../shared')):
  sys.path.append('../shared')
elif (os.path.isdir('app-deconfliction/service/shared')):
  sys.path.append('app-deconfliction/service/shared')
else:
  # GDB 7/3/25: In the docker container the repo has to be named with the
  # full service id
  #sys.path.append('/gridappsd/services/app-deconfliction/service/shared')
  sys.path.append('/gridappsd/services/gridappsd-app-deconfliction-service/service/shared')

from AppUtil import AppUtil
import MethodUtil
from sparql import SPARQLManager


class DeconflictionPipeline(GridAPPSD):

  # start of message listener process methods

  def messageListenerProcess(self, simulation_id):
    # authenticate with GridAPPS-D Platform
    gapps = GridAPPSD(simulation_id)
    assert gapps.connected

    # subscribe to simulation log and output messages
    out_id = gapps.subscribe(simulation_output_topic(simulation_id),
                             self.OnSimOutputMessage)

    if self.simLogSubscribedFlag:
      log_id = gapps.subscribe(simulation_log_topic(simulation_id),
                               self.OnSimLogMessage)

    if deconflictionAsServiceFlag:
      meas_id = gapps.subscribe(simulation_input_topic(simulation_id),
                                self.OnMeasSetpointsMessage)
    else:
      meas_id =gapps.subscribe(service_input_topic('deconfliction.measurements',
                               simulation_id), self.OnMeasSetpointsMessage)

    coop_id = gapps.subscribe(service_input_topic('deconfliction.cooperation',
                              simulation_id), self.OnCoopSetpointsMessage)

    # GDB 9/24/25: Keep track of the most recent simulation measurement
    # timestamp for indexing the application Difference Builder messages that
    # don't have simulation-based timestamps
    self.simTimestamp = None

    self.keepLoopingFlag = True

    #prlog('messageListenerProcess--start listening for simulation messages...')

    while self.keepLoopingFlag:
      # GDB 9/2/25: Warning: increasing the sleep duration above 0.1 such as
      # 0.5 can lead to bad things. With two processes sleeping on both ends
      # (apps and deconfliction pipeline) that's 4 sleep statements that are
      # part of processing messages leading to a potential 2 second total
      # delay (with 0.5 sleeps), which is horrible for cooperation messages.
      #sleep(0.1)
      sleep(0.05)

    gapps.unsubscribe(out_id)
    if self.simLogSubscribedFlag:
      gapps.unsubscribe(log_id)
    gapps.unsubscribe(meas_id)
    gapps.unsubscribe(coop_id)


  def OnSimOutputMessage(self, header, message):
    #prlog('OnSimOutputMessage--received message: ' + str(message))
    if not self.keepLoopingFlag:
      return

    self.simTimestamp = int(message['message']['timestamp'])

    # GDB 10/3/25: Fix for not being able to subscribe to log messages, but
    # still needing to recognize end of simulation
    if not self.simLogSubscribedFlag:
      if self.simTimestampStart == None:
        self.simTimestampStart = self.simTimestamp

      if (self.simTimestamp - self.simTimestampStart) >= self.simDuration:
        self.keepLoopingFlag = False
        message['processStatus'] = 'COMPLETE'
        self.messageQueue.put((None, None, self.simTimestamp, message))

    if self.realtimeFlag:
      self.messageQueue.put((None, None, self.simTimestamp, message['message']))
    else:
      # only add every 5th measurement message to the queue to
      # allow sufficient time for cooperation
      if self.simTimestamp % 300 == 0:
        self.messageQueue.put((None, None, self.simTimestamp,
                               message['message']))


  def OnSimLogMessage(self, header, message):
    #prlog('OnSimLogMessage--received message: ' + str(message))
    if not self.keepLoopingFlag:
      return

    status = message['processStatus']
    if status=='COMPLETE' or status=='CLOSED':
      self.keepLoopingFlag = False
      self.messageQueue.put((None, None, self.simTimestamp, message))


  def OnMeasSetpointsMessage(self, header, message):
    if self.printAllMessagesFlag:
      prlog('OnMeasSetpointsMessage--received message: ' + str(message))
      prlog('OnMeasSetpointsMessage--received header: ' + str(header))

    if self.logMessagesFlag:
      time_sent = datetime.strptime(message['time_sent'],'%Y-%m-%d %H:%M:%S.%f')
      diff_sec = (datetime.now() - time_sent).total_seconds()
      msglog('received new measurement setpoints|app:' + message['app_name'] +
             '|delay:' + str(diff_sec))

    # GDB 10/2/25: delays over a second seem to be common for these messages
    # so for now don't check and just put those on the queue
    '''
    if diff_sec < 5.0:
      self.messageQueue.put((message['app_name'], None, self.simTimestamp,
                             message['input']['message']))
    else:
      # initiate abort sequence
      self.keepLoopingFlag = False
      message['processStatus'] = 'ABORT'
      message['delaySeconds'] = str(diff_sec)
      self.messageQueue.put((None, None, self.simTimestamp, message))
    '''
    self.messageQueue.put((message['app_name'], None, self.simTimestamp,
                           message['input']['message']))


  def OnCoopSetpointsMessage(self, header, message):
    if self.printAllMessagesFlag:
      prlog('OnCoopSetpointsMessage--received message: ' + str(message))
      prlog('OnCoopSetpointsMessage--received header: ' + str(header))

    time_sent = datetime.strptime(message['time_sent'], '%Y-%m-%d %H:%M:%S.%f')
    diff_sec = (datetime.now() - time_sent).total_seconds()

    if self.logMessagesFlag:
      msglog('received cooperation response|app:' + message['app_name'] +
              '|msgid:' + str(message['coop_msgid']) + '|series:' +
              str(message['coop_series']) + '|delay:' + str(diff_sec))

    if diff_sec < 5.0:
      self.messageQueue.put((message['app_name'], message['coop_series'],
                             self.simTimestamp, message['input']['message']))
    else:
      # initiate abort sequence
      self.keepLoopingFlag = False
      message['processStatus'] = 'ABORT'
      message['delaySeconds'] = str(diff_sec)
      self.messageQueue.put((None, None, self.simTimestamp, message))

  # end of message listener process methods


  def SetpointProcessor(self, app_name, timestamp, set_points, meas_msg_flag,
                        printAllConflictsResolutionsFlag=False):
    # Update ConflictMatrix with newly provided set-points

    # meas_msg_flag could be checked within the device loop to make the code
    # more compact, but better performance by making it a top-level check
    if meas_msg_flag:
      for device in self.ConflictMatrix:
        # delete any existing matches for app_name so there are no stragglers
        # from past timestamps
        if app_name in self.ConflictMatrix[device]:
          self.ConflictMatrix[device].pop(app_name)

      if self.pltFlag:
        self.pltFile.write(app_name)
        self.pltFile.write(',')
        diff = (datetime.now() - self.pltTZero).total_seconds()
        self.pltFile.write(str(diff))
        self.pltFile.write(',')
        self.pltFile.write(str(timestamp))

    else:
      MinSetpoints = {}
      MaxSetpoints = {}
      MinSetpointsReal = {}
      MaxSetpointsReal = {}
      MinSetpointsImag = {}
      MaxSetpointsImag = {}

      # find the min/max setpoints for each device to make sure new
      # setpoints fall in that range
      for device in self.ConflictMatrix:
        for app in self.ConflictMatrix[device]:
          value = self.ConflictMatrix[device][app][1]
          if not isinstance(value, complex):
            if device not in MinSetpoints:
              MinSetpoints[device] = value
              MaxSetpoints[device] = value
            else:
              MinSetpoints[device] = min(MinSetpoints[device], value)
              MaxSetpoints[device] = max(MaxSetpoints[device], value)

          else:
            # SolarPVs have complex values
            value = self.ConflictMatrix[device][app][1].real
            if device not in MinSetpointsReal:
              MinSetpointsReal[device] = value
              MaxSetpointsReal[device] = value
            else:
              MinSetpointsReal[device] = min(MinSetpointsReal[device], value)
              MaxSetpointsReal[device] = max(MaxSetpointsReal[device], value)

            value = self.ConflictMatrix[device][app][1].imag
            if device not in MinSetpointsImag:
              MinSetpointsImag[device] = value
              MaxSetpointsImag[device] = value
            else:
              MinSetpointsImag[device] = min(MinSetpointsImag[device], value)
              MaxSetpointsImag[device] = max(MaxSetpointsImag[device], value)

        # delete any existing matches for app_name so there are no stragglers
        # from past timestamps
        if app_name in self.ConflictMatrix[device]:
          self.ConflictMatrix[device].pop(app_name)

    # deviceSetPoints is an intermediate structure with only a single entry
    # per device where set_points will have multiple entries for SolarPVs that
    # will be translated to complex values here
    deviceSetPoints = {}
    for point in set_points:
      device = point['object']
      #attribute = point['attribute']
      value = point['value']

      if device not in deviceSetPoints:
        deviceSetPoints[device] = value
      elif not isinstance(deviceSetPoints[device], complex):
        # this assumes the p value for the SolarPV was processed first
        # so this will add the q value
        deviceSetPoints[device] = complex(deviceSetPoints[device], value)

    # now add the new setpoints for app_name into ConflictMatrix
    for device, value in deviceSetPoints.items():
      if device not in self.ConflictMatrix:
        self.ConflictMatrix[device] = {}

      # for cooperation messages, make sure value falls in the min/max range
      if not meas_msg_flag:
        if device in MinSetpoints:
          if value < MinSetpoints[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] +
                  '--cooperation setpoint below min to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MinSetpoints[device]))
            value = MinSetpoints[device]
          elif value > MaxSetpoints[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] +
                  '--cooperation setpoint above max to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MaxSetpoints[device]))
            value = MaxSetpoints[device]

        elif device in MinSetpointsReal:
          if value.real < MinSetpointsReal[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] + '--cooperation setpoint ' +
                  '(real) below min to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MinSetpointsReal[device]))
            value = complex(MinSetpointsReal[device], value.imag)
          elif value.real > MaxSetpointsReal[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] + '--cooperation setpoint ' +
                  '(real) above max to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MaxSetpointsReal[device]))
            value = complex(MaxSetpointsReal[device], value.imag)

          if value.imag < MinSetpointsImag[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] + '--cooperation setpoint ' +
                  '(imag) below min to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MinSetpointsImag[device]))
            value = complex(value.real, MinSetpointsImag[device])
          elif value.imag > MaxSetpointsImag[device]:
            prlog('SetpointProcessor--app: ' + app_name + ', device: ' +
                  MethodUtil.DeviceToName[device] + '--cooperation setpoint ' +
                  '(imag) above max to prevent backtracking: ' +
                  str(value) + ', reset to: ' + str(MaxSetpointsImag[device]))
            value = complex(value.real, MaxSetpointsImag[device])

      self.ConflictMatrix[device][app_name] = (timestamp, value)

      if meas_msg_flag and self.pltFlag:
        self.pltFile.write(',')
        self.pltFile.write(MethodUtil.DeviceToName[device])
        self.pltFile.write(',')
        self.pltFile.write(str(value))

    if meas_msg_flag and self.pltFlag:
      self.pltFile.write('\n')
      self.pltFile.flush()

      # Monish wants ConflictMatrix changes written separately for plotting
      self.logConflictMatrix()

    if printAllConflictsResolutionsFlag:
      prlog('SetpointProcessor--ConflictMatrix: ' +str(self.ConflictMatrix))

    if self.testDeviceName:
      device = MethodUtil.NameToDevice[self.testDeviceName]
      if device in set_points:
        prlog('~TEST set-points message with ' + self.testDeviceName +
              ' set-point: ' + str(set_points[device]) +
              ', app: ' + app_name + ', timestamp: ' + str(timestamp))
        prlog('~TEST ConflictMatrix for ' + self.testDeviceName + ': ' +
              str(self.ConflictMatrix[device]))
      else:
        prlog('~TEST set-points message does not contain ' +
              self.testDeviceName)


  def ConflictMetricComputation(self, timestamp, printAllMetricsFlag=False):
    # GDB 5/21/24: Don't crash with an empty ConflictMatrix
    if len(self.ConflictMatrix) == 0:
      prlog('ConflictMetricComputation--conflict metric undefined ' +
            '(no conflicts), timestamp: ' + str(timestamp))
      return 0.0

    centroid = {}
    apps = {}
    n_devices = 0

    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]

      # CMDBG code to bypass SolarPVs in computation
      #if name.startswith('PhotovoltaicUnit.'):
      #  continue

      n_devices += 1
      device_setpoints = []
      for app in self.ConflictMatrix[device]:
        gamma_d_a = self.ConflictMatrix[device][app][1]
        if app not in apps:
          apps[app] = {}

        if printAllMetricsFlag:
          prlog('ConflictMetricComputation--device: ' + name + ', app: ' +
                app + ', setpoint: ' + str(gamma_d_a))

        if name.startswith('BatteryUnit.'):
          # Normalize setpoints using max charge and discharge possible
          sigma_d_a = (gamma_d_a + self.BatteriesInfo[device]['prated']) / \
                      (2 * self.BatteriesInfo[device]['prated'])
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

        elif name.startswith('RatioTapChanger.'):
          # Normalize setpoints using highStep and lowStep
          sigma_d_a = (gamma_d_a + abs(self.Regulators[device]['highStep'])) / \
                      (self.Regulators[device]['highStep'] + \
                       abs(self.Regulators[device]['lowStep']))
          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

        elif name.startswith('PhotovoltaicUnit.'):
          # Normalize setpoints by dividing by rated power. A complex number
          # will result and be assigned to apps and device_setpoints, but this
          # will be dealt with later to produce a scalar

          # GDB 8/31/25: original commented out calculation produced centroid
          # values greater than 1 so abs() is applied to remedy this
          # CMDGB code to try to force sigma_d_a to be <= 1 with abs()
          #sigma_d_a = gamma_d_a / self.SolarPVs[device]['ratedS']
          #sigma_d_a = abs(gamma_d_a) / self.SolarPVs[device]['ratedS']
          ### MM 09/12/25: Updating scaling to account for kW and kVAR
          # spans of PVs
          sigma_d_a_real = gamma_d_a.real / self.SolarPVs[device]['ratedS']
          sigma_d_a_imag = gamma_d_a.imag / (2*self.SolarPVs[device]['ratedS'])
          sigma_d_a = complex(sigma_d_a_real, sigma_d_a_imag)

          apps[app][device] = sigma_d_a
          device_setpoints.append(sigma_d_a)

      # Find centroid
      # GDB 6/26/24: Also don't crash with empty ConflictMatrix for a device
      n_apps_device = len(self.ConflictMatrix[device])
      if n_apps_device > 0:
        centroid[device] = sum(device_setpoints) / n_apps_device

        # CMDBG code--must use abs() if centroid is complex value
        if abs(centroid[device]) > 1.0:
          prlog('CMDBG: device: ' + name + ', sigma_d_a: ' +
                str(device_setpoints) + ', centroid: ' + str(centroid[device]) +
                ', abs(centroid): ' + str(abs(centroid[device])) +
                ', timestamp: ' + str(timestamp))

    # Distance vector:
    # Distance between setpoints requested by each app to the centroid vector
    dist_centroid = []
    for app in apps:
      sum_dist = 0
      for device in centroid:
        if device in apps[app]:
          # note this works on either complex (SolarPVs) or scalar values.
          # The abs() function wouldn't be needed for scalars, but doesn't
          # hurt either since it's just squaring the value so it just makes
          # the code more compact not to check if it is operating on scalar
          # or complex values. For complex values the abs() takes the magnitude
          # of the difference and gets us back into the scalar world.
          sum_dist += abs(centroid[device] - apps[app][device]) ** 2

      dist_centroid.append(math.sqrt(sum_dist))

    # Compute conflict metric: average distance
    n_apps = len(apps)
    conflict_metric = sum(dist_centroid) / n_apps
    # Ensuring 0 <= conflict_metric <= 1
    conflict_metric = conflict_metric * 2 / math.sqrt(n_devices)
    # CMDBG code to catch out of range conflict metric
    if conflict_metric > 1.0:
      prlog('CMDBG: ConflictMetricComputation--conflict metric: ' +
            str(conflict_metric) + ', timestamp: ' + str(timestamp))
    else:
      prlog('ConflictMetricComputation--conflict metric: ' +
            str(conflict_metric) + ', timestamp: ' + str(timestamp))

    return conflict_metric


  def CooperationWeightsUpdate(self, timestamp, ConflictMatrix,
                               TargetResolutionVector):
    sigma_d_t = {}
    app_list = []

    # find the sigma values for the target resolution vector first because
    # they are needed while iterating over each of the apps later
    for device in TargetResolutionVector:
      gamma_d_t = TargetResolutionVector[device][1]

      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        # Normalize setpoints using max charge and discharge possible
        sigma_d_t[device] = (gamma_d_t + self.BatteriesInfo[device]['prated'])/\
                            (2 * self.BatteriesInfo[device]['prated'])

      elif name.startswith('RatioTapChanger.'):
        # Normalize setpoints using highStep and lowStep
        sigma_d_t[device] = (gamma_d_t +
                             abs(self.Regulators[device]['highStep'])) / \
                            (self.Regulators[device]['highStep'] + \
                             abs(self.Regulators[device]['lowStep']))

      elif name.startswith('PhotovoltaicUnit.'):
        # Normalize setpoints using rated power
        #sigma_d_t[device] = gamma_d_t / self.SolarPVs[device]['ratedS']
        ### MM 09/12/25: Updating scaling to account for kW and kVAR
        # spans of PVs
        sigma_d_t_real = gamma_d_t.real / self.SolarPVs[device]['ratedS']
        sigma_d_t_imag = gamma_d_t.imag / (2*self.SolarPVs[device]['ratedS'])
        sigma_d_t[device] = complex(sigma_d_t_real, sigma_d_t_imag)

      # while we are iterating over devices, build up a list of apps we
      # need to compute weights for since that's buried down a level within
      # the conflict matrix
      for app in ConflictMatrix[device]:
        if app not in app_list:
          app_list.append(app)

    minWeight = 1.0

    # now with a list of apps we can loop over that at the top level since
    # we ultimately need a weight for every app (that includes all devices)
    for app in app_list:
      centroid = {}
      sigma_d_a = {}

      for device in TargetResolutionVector:
        # need to make sure the app has an entry for the device
        if app in ConflictMatrix[device]:
          gamma_d_a = ConflictMatrix[device][app][1]

          name = MethodUtil.DeviceToName[device]
          if name.startswith('BatteryUnit.'):
            # Normalize setpoints using max charge and discharge possible
            sigma = (gamma_d_a + self.BatteriesInfo[device]['prated']) / \
                    (2 * self.BatteriesInfo[device]['prated'])
            sigma_d_a[device] = sigma
            centroid[device] = (sigma + sigma_d_t[device]) / 2

          elif name.startswith('RatioTapChanger.'):
            # Normalize setpoints using highStep and lowStep
            sigma = (gamma_d_a + abs(self.Regulators[device]['highStep'])) / \
                    (self.Regulators[device]['highStep'] + \
                     abs(self.Regulators[device]['lowStep']))
            sigma_d_a[device] = sigma
            centroid[device] = (sigma + sigma_d_t[device]) / 2

          elif name.startswith('PhotovoltaicUnit.'):
            # Normalize setpoints using rated power
            #sigma = gamma_d_a / self.SolarPVs[device]['ratedS']
            ### MM 09/12/25: Updating scaling to account for kW and kVAR
            # spans of PVs
            sigma_real = gamma_d_a.real / self.SolarPVs[device]['ratedS']
            sigma_imag = gamma_d_a.imag / (2*self.SolarPVs[device]['ratedS'])
            sigma = complex(sigma_real, sigma_imag)

            sigma_d_a[device] = sigma
            centroid[device] = (sigma + sigma_d_t[device]) / 2

      # Distance vector:
      sum_dist_a = 0
      sum_dist_t = 0
      for device in centroid:
        # applying abs() is the magic to convert complex numbers to scalars
        # so it handles solarPV devices with complex setpoints as well as
        # batteries and regulators with scalar setpoints as taking the abs()
        # of those differences before squaring yields the same results
        sum_dist_a += abs(centroid[device] - sigma_d_a[device]) ** 2
        sum_dist_t += abs(centroid[device] - sigma_d_t[device]) ** 2

      dist_centroid_a = math.sqrt(sum_dist_a)
      dist_centroid_t = math.sqrt(sum_dist_t)

      # Compute per app conflict metric: average distance
      conflict_metric = (dist_centroid_a + dist_centroid_t) / 2
      # Ensuring 0 <= conflict_metric <= 1
      conflict_metric = conflict_metric * 2 / math.sqrt(len(sigma_d_a))

      # The lower the conflict metric, the higher the app incentive weight value
      self.OptAppWeights[app] = 1.0 - conflict_metric
      #prlog('CooperationWeightsUpdate--timestamp: ' + str(timestamp) +
      #   ', app: ' + app + ', initial weight: ' + str(self.OptAppWeights[app]))

      # compute lowest weight over all apps to make adjustments below
      minWeight = min(minWeight, self.OptAppWeights[app])

    # Adjust weights to give even more incentive for better cooperating apps
    # If we didn't "boost" the incentive then apps would not be provided any
    # real benefit because they have already compromised with their preferred
    # setpoints and they'd only be given compensation for that compromise
    # not extra to make it "worth their while"
    weightLoss = 0.75 * minWeight # boost the incentive
    for app in app_list:
      self.OptAppWeights[app] -= weightLoss
      prlog('CooperationWeightsUpdate--timestamp: ' + str(timestamp) +
            ', app: ' + app + ', weight: ' + str(self.OptAppWeights[app]))


  def CooperationWeightsClear(self, timestamp, ConflictMatrix):
    app_list = []

    for device in ConflictMatrix:
      for app in ConflictMatrix[device]:
        # build up a list of apps we need to clear weights for since that's
        # buried down a level within the conflict matrix
        if app not in app_list:
          app_list.append(app)

    # now with a list of apps we can loop over that to clear weights
    for app in app_list:
      # remove weight by calling pop because that works even if the app isn't
      # in the dictionary
      self.OptAppWeights.pop(app, None)
      prlog('CooperationWeightsClear--timestamp: ' + str(timestamp) +
            ', app: ' + app)


  def ConflictIdentification(self):
    for device in self.ConflictMatrix:
      setpoint = None
      for app in self.ConflictMatrix[device]:
        # this is a weird/clever way of determining if any of the setpoints
        # are different by comparing each one with the previous one, which
        # isn't the intuitive way to do it, but it is fast and compact
        if setpoint!=None and setpoint!=self.ConflictMatrix[device][app][1]:
          return True
        setpoint = self.ConflictMatrix[device][app][1]

    return False


  def FeasibilityMaintainerForBatteries(self, printAllFeasibilityFlag=False):
    # find the maximum P_batt charge and discharge values per battery to
    # prevent overcharging or undercharging
    for device in self.BatteriesInfo:
      chargeSoCMax = max(0.0, (0.9 - self.BatteriesInfo[device]['SoC']))
      self.BatteriesInfo[device]['P_batt_charge_max'] = \
                         (chargeSoCMax*self.BatteriesInfo[device]['ratedE']) /\
                         (self.BatteriesInfo[device]['eff_c']*self.deltaT)
      if printAllFeasibilityFlag:
        prlog('FeasibilityMaintainerForBatteries--device: ' +
              MethodUtil.DeviceToName[device] +
              ', max charge SoC contribution: ' + str(chargeSoCMax) +
              ', max charge P_batt: ' +
              str(self.BatteriesInfo[device]['P_batt_charge_max']))

      dischargeSoCMax = min(0.0, (0.2 - self.BatteriesInfo[device]['SoC']))
      self.BatteriesInfo[device]['P_batt_discharge_max'] = \
                       (dischargeSoCMax*self.BatteriesInfo[device]['ratedE']) /\
                       (1/self.BatteriesInfo[device]['eff_d']*self.deltaT)
      if printAllFeasibilityFlag:
        prlog('FeasibilityMaintainerForBatteries--device: ' +
              MethodUtil.DeviceToName[device] +
              ', max discharge SoC contribution: ' + str(dischargeSoCMax) +
              ', max discharge P_batt: ' +
              str(self.BatteriesInfo[device]['P_batt_discharge_max']))

    # iterate over all battery setpoints in ConflictMatrix to make sure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        for app in self.ConflictMatrix[device]:
          # check vs. battery rated power
          if round(self.ConflictMatrix[device][app][1]) > \
             round(self.BatteriesInfo[device]['prated']):
            prlog('FeasibilityMaintainerForBatteries--device: ' + name +
                  ', app: ' + app + ', P_batt setpoint exceeds battery rated ' +
                  'power: ' + str(self.ConflictMatrix[device][app][1]) +
                  ', reset to rated power: ' +
                  str(self.BatteriesInfo[device]['prated']))
            self.ConflictMatrix[device][app] = \
                                         (self.ConflictMatrix[device][app][0],
                                          self.BatteriesInfo[device]['prated'])

          elif -round(self.ConflictMatrix[device][app][1]) > \
                round(self.BatteriesInfo[device]['prated']):
            prlog('FeasibilityMaintainerForBatteries--device: ' + name +
                  ', app: ' + app + ', P_batt setpoint exceeds battery rated ' +
                  'power: ' + str(self.ConflictMatrix[device][app][1]) +
                  ', reset to rated power: ' +
                  str(-self.BatteriesInfo[device]['prated']))
            self.ConflictMatrix[device][app] = \
                                         (self.ConflictMatrix[device][app][0],
                                          -self.BatteriesInfo[device]['prated'])

          # check vs. battery SoC limits
          if -round(self.ConflictMatrix[device][app][1]) > \
              round(self.BatteriesInfo[device]['P_batt_charge_max']):
            prlog('FeasibilityMaintainerForBatteries--device: ' + name +
                  ', app: ' + app + ', P_batt setpoint would exceeed 0.9 SoC ' +
                  'limit: ' + str(-self.ConflictMatrix[device][app][1]) +
                  ', reset to max allowed charge P_batt: ' +
                  str(self.BatteriesInfo[device]['P_batt_charge_max']))
            self.ConflictMatrix[device][app] = \
                              (self.ConflictMatrix[device][app][0],
                               -self.BatteriesInfo[device]['P_batt_charge_max'])

          elif -round(self.ConflictMatrix[device][app][1]) < \
                round(self.BatteriesInfo[device]['P_batt_discharge_max']):
            prlog('FeasibilityMaintainerForBatteries--device: ' + name +
                  ', app: ' + app + ', P_batt setpoint would fall below 0.2 ' +
                  'SoC limit: ' + str(-self.ConflictMatrix[device][app][1]) +
                  ', reset to max allowed discharge P_batt: ' +
                  str(self.BatteriesInfo[device]['P_batt_discharge_max']))
            self.ConflictMatrix[device][app]= \
                           (self.ConflictMatrix[device][app][0],
                            -self.BatteriesInfo[device]['P_batt_discharge_max'])


  def FeasibilityMaintainerForRegulators(self, printAllFeasibilityFlag=False):
    # iterate over all regulator tap setpoints in ConflictMatrix to make sure
    # they fall within the feasible +16/-16 range
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        for app in self.ConflictMatrix[device]:
          if self.ConflictMatrix[device][app][1] > 16:
            prlog('FeasibilityMaintainerForRegulators--device: ' + name +
                  ', app: ' + app + '--tap pos setpoint above max feasible ' +
                  'pos: ' + str(self.ConflictMatrix[device][app][1]) +
                  ', reset to max feasible pos: 16')
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0], 16)

          elif self.ConflictMatrix[device][app][1] < -16:
            prlog('FeasibilityMaintainerForRegulators--device: ' + name +
                  ', app: ' + app + '--tap pos setpoint below min feasible ' +
                  'pos: ' + str(self.ConflictMatrix[device][app][1]) +
                  ', reset to min feasible pos: -16')
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0], -16)


  def SetpointValidatorForBatteries(self, newResolutionVector,
                                    printAllValidatorFlag=False):
    # find the maximum P_batt charge and discharge values per battery to
    # prevent overcharging or undercharging
    for device in self.BatteriesInfo:
      chargeSoCMax = max(0.0, (0.9 - self.BatteriesInfo[device]['SoC']))
      self.BatteriesInfo[device]['P_batt_charge_max'] = \
                         (chargeSoCMax*self.BatteriesInfo[device]['ratedE']) / \
                         (self.BatteriesInfo[device]['eff_c']*self.deltaT)
      if printAllValidatorFlag:
        prlog('SetpointValidatorForBatteries--device: ' +
              MethodUtil.DeviceToName[device] +
              ', max charge SoC contribution: ' + str(chargeSoCMax) +
              ', max charge P_batt: ' +
              str(self.BatteriesInfo[device]['P_batt_charge_max']))

      dischargeSoCMax = min(0.0, (0.2 - self.BatteriesInfo[device]['SoC']))
      self.BatteriesInfo[device]['P_batt_discharge_max'] = \
                      (dischargeSoCMax*self.BatteriesInfo[device]['ratedE']) / \
                      (1/self.BatteriesInfo[device]['eff_d']*self.deltaT)
      if printAllValidatorFlag:
        prlog('SetpointValidatorForBatteries--device: ' +
              MethodUtil.DeviceToName[device] +
              ', max discharge SoC contribution: ' + str(dischargeSoCMax) +
              ', max discharge P_batt: ' +
              str(self.BatteriesInfo[device]['P_batt_discharge_max']))

    # iterate over all battery setpoints in ResolutionVector to make sure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        # check vs. battery rated power
        if round(newResolutionVector[device][1]) > \
           round(self.BatteriesInfo[device]['prated']):
          prlog('SetpointValidatorForBatteries--device: ' + name +
                ', P_batt setpoint exceeds battery rated power: ' +
                str(newResolutionVector[device][1]) +
                ', reset to rated power: ' +
                str(self.BatteriesInfo[device]['prated']))
          newResolutionVector[device] = (newResolutionVector[device][0],
                                         self.BatteriesInfo[device]['prated'])

        elif -round(newResolutionVector[device][1]) > \
              round(self.BatteriesInfo[device]['prated']):
          prlog('SetpointValidatorForBatteries--device: ' + name +
                ', P_batt setpoint exceeds battery rated power: ' +
                str(newResolutionVector[device][1]) +
                ', reset to rated power: ' +
                str(-self.BatteriesInfo[device]['prated']))
          newResolutionVector[device] = (newResolutionVector[device][0],
                                         -self.BatteriesInfo[device]['prated'])

        # check vs. battery SoC limits
        if -round(newResolutionVector[device][1]) > \
            round(self.BatteriesInfo[device]['P_batt_charge_max']):
          prlog('SetpointValidatorForBatteries--device: ' + name +
                ', P_batt setpoint would exceed 0.9 SoC limit: ' +
                str(-newResolutionVector[device][1]) +
                ', reset to max allowed charge P_batt: ' +
                str(self.BatteriesInfo[device]['P_batt_charge_max']))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              -self.BatteriesInfo[device]['P_batt_charge_max'])

        elif -round(newResolutionVector[device][1]) < \
              round(self.BatteriesInfo[device]['P_batt_discharge_max']):
          prlog('SetpointValidatorForBatteries--device: ' + name +
                ', P_batt setpoint would fall below 0.2 SoC limit: ' +
                str(-newResolutionVector[device][1]) +
                ', reset to max allowed discharge P_batt: ' +
                str(self.BatteriesInfo[device]['P_batt_discharge_max']))
          newResolutionVector[device] = \
                           (newResolutionVector[device][0],
                            -self.BatteriesInfo[device]['P_batt_discharge_max'])

        # bail if rules aren't being applied at all
        if self.noValidatorRulesFlag or \
           not (self.rulesStageFirstFlag or self.rulesStageLastFlag):
          continue

        # enforce the change of charge/discharge state rule if the rules
        # weren't applied last
        if not self.rulesStageLastFlag and \
           self.BatteriesInfo[device]['switch_P_batt_inv'] != None:
          prev_P_batt_inv = self.BatteriesInfo[device]['switch_P_batt_inv']
          if (prev_P_batt_inv>0 and newResolutionVector[device][1]<0) or \
             (prev_P_batt_inv<0 and newResolutionVector[device][1]>0):
            prlog('SetpointValidatorForBatteries--device: ' + name +
                  ', P_batt setpoint attempt to change charge/discharge ' +
                  'state: ' + str(newResolutionVector[device][1]) +
                  ', reset to zero')
            # force the setpoint request back to zero to avoid a change in
            # charge/discharge state
            newResolutionVector[device] = \
                                (newResolutionVector[device][0], 0.0)


  def SetpointValidatorForRegulators(self, newResolutionVector,
                                     printAllValidatorFlag=False):
    # iterate over all regulator tap setpoints in ResolutionVector to make sure
    # they fall within the feasible +16/-16 range
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        if newResolutionVector[device][1] > 16:
          prlog('SetpointValidatorForRegulators--device: ' + name +
                '--tap pos setpoint above max feasible pos: ' +
                str(newResolutionVector[device][1]) +
                ', reset to max feasible pos: 16')
          newResolutionVector[device] = (newResolutionVector[device][0], 16)

        elif newResolutionVector[device][1] < -16:
          prlog('SetpointValidatorForRegulators--device: ' + name +
                '--tap pos setpoint below min feasible pos: ' +
                str(newResolutionVector[device][1]) +
                ', reset to min feasible pos: -16')
          newResolutionVector[device] = (newResolutionVector[device][0], -16)

        # bail if rules aren't being applied at all
        if self.noValidatorRulesFlag or \
           not (self.rulesStageFirstFlag or self.rulesStageLastFlag):
          continue

        # enforce the tap budget rule if the rules weren't applied last
        if not self.rulesStageLastFlag:
          if newResolutionVector[device][1] > \
             self.Regulators[device]['maxStep']:
            prlog('SetpointValidatorForRegulators--device: ' + name +
                  '--tap pos setpoint above max rules pos: ' +
                  str(newResolutionVector[device][1]) +
                  ', reset to max rules pos: ' +
                  str(self.Regulators[device]['maxStep']))
            newResolutionVector[device] = (newResolutionVector[device][0],
                                           self.Regulators[device]['maxStep'])

          elif newResolutionVector[device][1] < \
             self.Regulators[device]['minStep']:
            prlog('SetpointValidatorForRegulators--device: ' + name +
                  '--tap pos setpoint below min rules pos: ' +
                  str(newResolutionVector[device][1]) +
                  ', reset to min rules pos: ' +
                  str(self.Regulators[device]['minStep']))
            newResolutionVector[device] = (newResolutionVector[device][0],
                                           self.Regulators[device]['minStep'])


  def RulesForBatteriesConflict(self, printAllRulesFlag=False):
    for device in self.BatteriesInfo:
      histList = self.BatteryHistory[device]
      if printAllRulesFlag:
        prlog('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[device] + ', BatteryHistory: ' +
              str(histList))

      # iterate backwards through histList counting switches
      rollingSwitchCount = 0
      rollingStartTime = self.BatteriesInfo[device]['timestamp'] - \
                         self.rulesBattTimeInterval
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingSwitchCount += 1

      if printAllRulesFlag:
        prlog('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[device] +
              ', rolling charge/discharge switches: ' + str(rollingSwitchCount)+
              ', vs. allowed: ' + str(self.rulesBattSwitchesAllowed))

      if rollingSwitchCount >= self.rulesBattSwitchesAllowed:
        # save the final P_batt_inv in the history list since we need to make
        # sure not to allow the opposite direction in any setpoint requests
        self.BatteriesInfo[device]['switch_P_batt_inv'] = \
                              self.BatteryHistory[device][-1][1]
      else:
        self.BatteriesInfo[device]['switch_P_batt_inv'] = None

    # iterate over all battery setpoints in ConflictMatrix to make sure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        for app in self.ConflictMatrix[device]:
          # check for switching between charge/discharge if over limit
          if self.BatteriesInfo[device]['switch_P_batt_inv'] != None:
            prev_P_batt_inv = self.BatteriesInfo[device]['switch_P_batt_inv']
            if (prev_P_batt_inv>0 and self.ConflictMatrix[device][app][1]<0) or\
               (prev_P_batt_inv<0 and self.ConflictMatrix[device][app][1]>0):
              prlog('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                     app + ', P_batt setpoint attempted to change ' +
                     'charge/discharge state: ' +
                     str(self.ConflictMatrix[device][app][1]) +
                     ', P_batt setpoint reset to zero')
              # this is pretty harsh to force the setpoint request back to zero
              # to avoid a possible change in charge/discharge state, but no
              # other choice when the rule is applied before other stages
              self.ConflictMatrix[device][app]= \
                                  (self.ConflictMatrix[device][app][0], 0.0)


  def RulesForBatteriesResolution(self, newResolutionVector,
                                  printAllRulesFlag=False):
    for device in self.BatteriesInfo:
      histList = self.BatteryHistory[device]
      if printAllRulesFlag:
        prlog('RulesForBatteriesResolution--device: ' +
              MethodUtil.DeviceToName[device] + ', BatteryHistory: ' +
              str(histList))

      # iterate backwards through histList counting switches
      rollingSwitchCount = 0
      rollingStartTime = self.BatteriesInfo[device]['timestamp'] - \
                         self.rulesBattTimeInterval
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingSwitchCount += 1

      if printAllRulesFlag:
        prlog('RulesForBatteriesResolution--device: ' +
              MethodUtil.DeviceToName[device] +
              ', rolling charge/discharge switches: ' + str(rollingSwitchCount)+
              ', vs. allowed: ' + str(self.rulesBattSwitchesAllowed))

      if rollingSwitchCount >= self.rulesBattSwitchesAllowed:
        # save the final P_batt_inv in the history list since we need to make
        # sure not to allow the opposite direction in any setpoint requests
        self.BatteriesInfo[device]['switch_P_batt_inv'] = \
                              self.BatteryHistory[device][-1][1]
      else:
        self.BatteriesInfo[device]['switch_P_batt_inv'] = None

    # iterate over all battery setpoints in newResolutionVector to insure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        # check for switching between charge/discharge if over limit
        if self.BatteriesInfo[device]['switch_P_batt_inv'] != None:
          prev_P_batt_inv = self.BatteriesInfo[device]['switch_P_batt_inv']
          if (prev_P_batt_inv>0 and newResolutionVector[device][1]<0) or \
             (prev_P_batt_inv<0 and newResolutionVector[device][1]>0):
            prlog('RulesForBatteriesResolution--device: ' + name +
              ', P_batt setpoint attempted to change charge/discharge state: ' +
              str(newResolutionVector[device][1]) +
              ', P_batt setpoint reset to zero')
            # force the setpoint request back to zero to avoid a change in
            # charge/discharge state
            newResolutionVector[device] = \
                                (newResolutionVector[device][0], 0.0)


  def logConflictReg(self, msg):
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        for app in self.ConflictMatrix[device]:
          prlog('REG DEBUG ConflictMatrix ' + msg + ', app: ' + app +
                ', device: ' + name +
                ', setpoint: ' + str(self.ConflictMatrix[device][app][1]))


  def logConflictPV(self, msg):
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('PhotovoltaicUnit.'):
        for app in self.ConflictMatrix[device]:
          prlog('PV DEBUG ConflictMatrix ' + msg + ', app: ' + app +
                ', device: ' + name + ', setpoint p: ' +
                str(self.ConflictMatrix[device][app][1].real) +
                ', q: ' + str(self.ConflictMatrix[device][app][1].imag))


  def logConflictTest(self, msg):
    if self.testDeviceName != None:
      self.refCount += 1
      for device in self.ConflictMatrix:
        name = MethodUtil.DeviceToName[device]
        if name == self.testDeviceName:
          for app in self.ConflictMatrix[device]:
            prlog('~TEST DEBUG ConflictMatrix ' + msg + ', app: ' + app +
                  ', device: ' + name + ', ref: ' + str(self.refCount) +
                  ', setpoint: ' + str(self.ConflictMatrix[device][app][1]))


  def logConflictMatrix(self):
    noComplexCMat = copy.deepcopy(self.ConflictMatrix)

    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('PhotovoltaicUnit.'):
        for app in self.ConflictMatrix[device]:
          cmplx = self.ConflictMatrix[device][app][1]
          noComplexCMat[device][app] = (self.ConflictMatrix[device][app][0],
                                        (cmplx.real, cmplx.imag))

    print(json.dumps(noComplexCMat), file=self.cmatFile)
    self.cmatFile.flush()


  def logResolutionPV(self, msg, resolutionVector):
    for device in resolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('PhotovoltaicUnit.'):
        prlog('PV DEBUG ResolutionVector ' + msg +
              ', device: ' + name + ', setpoint p: ' +
              str(resolutionVector[device][1].real) +
              ', q: ' + str(resolutionVector[device][1].imag))


  def logResolutionTest(self, msg, resolutionVector):
    if self.testDeviceName != None:
      self.refCount += 1
      for device in resolutionVector:
        name = MethodUtil.DeviceToName[device]
        if name == self.testDeviceName:
          prlog('~TEST DEBUG ResolutionVector ' + msg +
                ', device: ' + name + ', ref: ' + str(self.refCount) +
                ', setpoint: ' + str(resolutionVector[device][1]))


  def RulesForRegulatorsBudget(self, device, rollingTimeInterval,
                               rollingStepsAllowed, printAllRulesFlag):

    name = MethodUtil.DeviceToName[device]
    histList = self.RegulatorHistory[device]
    if printAllRulesFlag:
      prlog('RulesForRegulatorsBudget--interval: ' +
            str(rollingTimeInterval) + ', device: ' + name +
            ', RegulatorHistory: ' + str(histList))
    if name == self.testDeviceName:
      prlog('~TEST DEBUG RulesForRegulatorsBudget--interval: ' +
            str(rollingTimeInterval) + ', device: ' + name +
            ', RegulatorHistory: ' + str(histList))

    # iterate backwards through histList counting steps changed
    rollingStepCount = 0
    rollingStartTime = self.Regulators[device]['timestamp'] - \
                       rollingTimeInterval
    if name == self.testDeviceName:
      prlog('~TEST DEBUG RulesForRegulatorsBudget--interval: ' +
            str(rollingTimeInterval) + ', device: ' + name +
            ', currentTime: ' + str(self.Regulators[device]['timestamp']),
            ', rollingStartTime: ' + str(rollingStartTime))
    for it in range(len(histList)-1, 0, -1):
      if histList[it][0] < rollingStartTime:
        if name == self.testDeviceName:
          prlog('~TEST DEBUG RulesForRegulatorsBudget--interval: ' +
                str(rollingTimeInterval) + ', device: ' + name +
                ', BREAK historyTime: ' + str(histList[it-1][0]))
        break
      rollingStepCount += abs(histList[it][1] - histList[it-1][1])

    tapBudget = max(0, rollingStepsAllowed - rollingStepCount)

    if printAllRulesFlag:
      prlog('RulesForRegulatorsBudget--interval: ' +
            str(rollingTimeInterval) + ', device: ' + name +
            ', rolling steps: ' + str(rollingStepCount) +
            ', vs. allowed: ' + str(rollingStepsAllowed) +
            ', tap budget: ' + str(tapBudget))
    if name == self.testDeviceName:
      prlog('~TEST DEBUG RulesForRegulatorsBudget--interval: ' +
            str(rollingTimeInterval) + ', device: ' + name +
            ', rolling steps: ' + str(rollingStepCount) +
            ', vs. allowed: ' + str(rollingStepsAllowed) +
            ', tap budget: ' + str(tapBudget))

    return tapBudget


  def RulesForRegulatorsConflict(self, printAllRulesFlag=False):
    # iterate over all regulator tap setpoints in ConflictMatrix to make sure
    # they fall within the acceptable tap budget range of the current position
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        outerTapBudget = self.RulesForRegulatorsBudget(device,
                                                self.rulesRegOuterTimeInterval,
                                                self.rulesRegOuterStepsAllowed,
                                                printAllRulesFlag)

        innerTapBudget = self.RulesForRegulatorsBudget(device,
                                                self.rulesRegInnerTimeInterval,
                                                self.rulesRegInnerStepsAllowed,
                                                printAllRulesFlag)

        tapBudget = min(outerTapBudget, innerTapBudget)

        if printAllRulesFlag:
          prlog('RulesForRegulatorsConflict--device: ' + name +
                ', overall tap budget: ' + str(tapBudget))
        if name == self.testDeviceName:
          prlog('~TEST DEBUG RulesForRegulatorsConflict--device: ' +
                name + ', overall tap budget: ' + str(tapBudget))

        # constrain by the overall tap budget and physical device limits
        self.Regulators[device]['minStep'] = max(
                               self.Regulators[device]['step'] - tapBudget, -16)
        self.Regulators[device]['maxStep'] = min(
                               self.Regulators[device]['step'] + tapBudget, 16)

        if printAllRulesFlag:
          prlog('RulesForRegulatorsConflict--device: ' + name +
                ', current pos: ' + str(self.Regulators[device]['step']) +
                ', min pos: ' + str(self.Regulators[device]['minStep']) +
                ', max pos: ' + str(self.Regulators[device]['maxStep']))
        if name == self.testDeviceName:
          prlog('~TEST DEBUG RulesForRegulatorsConflict--device: ' + name +
                ', current pos: ' + str(self.Regulators[device]['step']) +
                ', min pos: ' + str(self.Regulators[device]['minStep']) +
                ', max pos: ' + str(self.Regulators[device]['maxStep']))

        for app in self.ConflictMatrix[device]:
          if self.ConflictMatrix[device][app][1] > \
             self.Regulators[device]['maxStep']:
            prlog('RulesForRegulatorsConflict--device: ' + name +
                  ', app: ' + app + '--tap pos setpoint: ' +
                  str(self.ConflictMatrix[device][app][1]) +
                  ', above max allowable asset health pos, reset to: ' +
                  str(self.Regulators[device]['maxStep']))
            if name == self.testDeviceName:
              prlog('~TEST DEBUG RulesForRegulatorsConflict--device: ' + name +
                    ', app: ' + app + '--tap pos setpoint: ' +
                    str(self.ConflictMatrix[device][app][1]) +
                    ', above max allowable asset health pos, reset to: ' +
                    str(self.Regulators[device]['maxStep']))
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0],
                                self.Regulators[device]['maxStep'])

          elif self.ConflictMatrix[device][app][1] < \
               self.Regulators[device]['minStep']:
            prlog('RulesForRegulatorsConflict--device: ' + name +
                  ', app: ' + app + '--tap pos setpoint: ' +
                  str(self.ConflictMatrix[device][app][1]) +
                  ', below min allowable asset health pos, reset to: ' +
                  str(self.Regulators[device]['minStep']))
            if name == self.testDeviceName:
              prlog('~TEST DEBUG RulesForRegulatorsConflict--device: ' + name +
                    ', app: ' + app + '--tap pos setpoint: ' +
                    str(self.ConflictMatrix[device][app][1]) +
                    ', below min allowable asset health pos, reset to: ' +
                    str(self.Regulators[device]['minStep']))
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0],
                                self.Regulators[device]['minStep'])

          else:
            if name == self.testDeviceName:
              prlog('~TEST DEBUG RulesForRegulatorsConflict--device: ' + name +
                    ', app: ' + app + ', tap pos setpoint: ' +
                    str(self.ConflictMatrix[device][app][1]) +
                    ', in allowed range of min: ' +
                    str(self.Regulators[device]['minStep']) + ', max: ' +
                    str(self.Regulators[device]['maxStep']))


  def RulesForRegulatorsResolution(self,newResolutionVector,
                                   printAllRulesFlag=False):
    # iterate over all regulator tap setpoints in newResolutionVector to insure
    # they fall within the acceptable tap budget range of the current position
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        outerTapBudget = self.RulesForRegulatorsBudget(device,
                                                self.rulesRegOuterTimeInterval,
                                                self.rulesRegOuterStepsAllowed,
                                                printAllRulesFlag)

        innerTapBudget = self.RulesForRegulatorsBudget(device,
                                                self.rulesRegInnerTimeInterval,
                                                self.rulesRegInnerStepsAllowed,
                                                printAllRulesFlag)

        tapBudget = min(outerTapBudget, innerTapBudget)

        if printAllRulesFlag:
          prlog('RulesForRegulatorsResolution--device: ' + name +
                ', overall tap budget: ' + str(tapBudget))
        if name == self.testDeviceName:
          prlog('~TEST DEBUG RulesForRegulatorsResolution--device: ' + name +
                ', overall tap budget: ' + str(tapBudget))

        # constrain by the overall tap budget and physical device limits
        self.Regulators[device]['minStep'] = max(
                               self.Regulators[device]['step'] - tapBudget, -16)
        self.Regulators[device]['maxStep'] = min(
                               self.Regulators[device]['step'] + tapBudget, 16)

        if printAllRulesFlag:
          prlog('RulesForRegulatorsResolution--device: ' + name +
                ', current pos: ' + str(self.Regulators[device]['step']) +
                ', min pos: ' + str(self.Regulators[device]['minStep']) +
                ', max pos: ' + str(self.Regulators[device]['maxStep']))
        if name == self.testDeviceName:
          prlog('~TEST DEBUG RulesForRegulatorsResolution--device: ' + name +
                ', current pos: ' + str(self.Regulators[device]['step']) +
                ', min pos: ' + str(self.Regulators[device]['minStep']) +
                ', max pos: ' + str(self.Regulators[device]['maxStep']))

        if newResolutionVector[device][1] > self.Regulators[device]['maxStep']:
          prlog('RulesForRegulatorsResolution--device: ' + name +
                ', pos setpoint: ' +
                str(newResolutionVector[device][1]) +
                ', above max allowable asset health pos, reset to: ' +
                str(self.Regulators[device]['maxStep']))
          if name == self.testDeviceName:
            prlog('~TEST DEBUG RulesForRegulatorsResolution--device: ' + name +
                  ', pos setpoint: ' +
                  str(newResolutionVector[device][1]) +
                  ', above max allowable asset health pos, reset to: ' +
                  str(self.Regulators[device]['maxStep']))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Regulators[device]['maxStep'])

        elif newResolutionVector[device][1] <self.Regulators[device]['minStep']:
          prlog('RulesForRegulatorsResolution--device: ' + name +
                ', pos setpoint: ' +
                str(newResolutionVector[device][1]) +
                ', below min allowable asset health pos, reset to: ' +
                str(self.Regulators[device]['minStep']))
          if name == self.testDeviceName:
            prlog('~TEST DEBUG RulesForRegulatorsResolution--device: ' + name +
                  ', pos setpoint: ' +
                  str(newResolutionVector[device][1]) +
                  ', below min allowable asset health pos, reset to: ' +
                  str(self.Regulators[device]['minStep']))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Regulators[device]['minStep'])

        else:
          if name == self.testDeviceName:
            prlog('~TEST DEBUG RulesForRegulatorsResolution--device: ' + name +
                  ', pos setpoint: ' +
                  str(newResolutionVector[device][1]) +
                  ', in allowed range of min: ' +
                  str(self.Regulators[device]['minStep']) + ', max: ' +
                  str(self.Regulators[device]['maxStep']))


  def Optimization(self, timestamp, ConflictMatrix):
    ResolutionVector = {}

    # This should work whether the conflict matrix setpoint values are
    # scalars as with batteries and regulators or complex numbers as with
    # solarPVs. Storing those solarPV p,q values as complex numbers pays
    # off here.
    for device in ConflictMatrix:
      optTimestamp = 0
      optNumerator = 0.0
      optDenominator = 0.0

      for app in ConflictMatrix[device]:
        optTimestamp = max(optTimestamp, ConflictMatrix[device][app][0])

        if app in self.OptDevWeights and device in self.OptDevWeights[app]:
          optNumerator += ConflictMatrix[device][app][1] * \
                          self.OptDevWeights[app][device]
          optDenominator += self.OptDevWeights[app][device]

        elif app in self.OptAppWeights:
          optNumerator += ConflictMatrix[device][app][1] * \
                          self.OptAppWeights[app]
          optDenominator += self.OptAppWeights[app]

        else:
          optNumerator += ConflictMatrix[device][app][1]
          optDenominator += 1.0

      if optDenominator > 0.0:
        name = MethodUtil.DeviceToName[device]
        if name.startswith('RatioTapChanger.'):
          # note round() function yields an int
          ResolutionVector[device] = (optTimestamp,
                                      round(optNumerator/optDenominator))
        else:
          ResolutionVector[device] = (optTimestamp, optNumerator/optDenominator)

    return ResolutionVector


  def CoopOptimization(self, timestamp, ConflictMatrix):
    TargetResolutionVector = {}
    CoopProposed = {}

    # GDB 9/10/25: This version to call for generating a target resolution
    # vector for cooperation checks to see if there is conflict for a device
    # or not. If not, then that device isn't included in the target because
    # there is no need/desire for solicitating cooperation when there is
    # already agreement.

    # This should work whether the conflict matrix setpoint values are
    # scalars as with batteries and regulators or complex numbers as with
    # solarPVs. Storing those SolarPV p,q values as complex numbers pays
    # off here.
    for device in ConflictMatrix:
      conflictFlag = False
      setpoint = None
      for app in ConflictMatrix[device]:
        # this is a weird/clever way of determining if any of the setpoints
        # are different by comparing each one with the previous one, which
        # isn't the intuitive way to do it, but it is fast and compact
        if setpoint!=None and setpoint!=ConflictMatrix[device][app][1]:
          conflictFlag = True
          break
        setpoint = self.ConflictMatrix[device][app][1]

      optTimestamp = 0
      optNumerator = 0.0
      optDenominator = 0.0

      for app in ConflictMatrix[device]:
        optTimestamp = max(optTimestamp, ConflictMatrix[device][app][0])

        if app in self.OptDevWeights and device in self.OptDevWeights[app]:
          optNumerator += ConflictMatrix[device][app][1] * \
                          self.OptDevWeights[app][device]
          optDenominator += self.OptDevWeights[app][device]

        elif app in self.OptAppWeights:
          optNumerator += ConflictMatrix[device][app][1] * \
                          self.OptAppWeights[app]
          optDenominator += self.OptAppWeights[app]

        else:
          optNumerator += ConflictMatrix[device][app][1]
          optDenominator += 1.0

      if optDenominator > 0.0:
        name = MethodUtil.DeviceToName[device]
        if name.startswith('RatioTapChanger.'):
          # note round() function yields an int
          TargetResolutionVector[device] = (optTimestamp,
                                            round(optNumerator/optDenominator))
        else:
          TargetResolutionVector[device] = (optTimestamp,
                                            optNumerator/optDenominator)

      if conflictFlag and device in TargetResolutionVector:
        value = TargetResolutionVector[device]
        # can't serialize complex numbers for SolarPV setpoints so need
        # to translate all of those to tuples
        if isinstance(value[1], complex):
          CoopProposed[device] = (value[0], (value[1].real, value[1].imag))
        else:
          CoopProposed[device] = value

    return TargetResolutionVector, CoopProposed


  def DeviceDispatcher(self, timestamp, newResolutionVector,
                       printAllDispatchesFlag=False):
    # Iterate over resolution and send set-points to devices that have
    # different values
    diffCount = 0

    for device, value in newResolutionVector.items():
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        if value[1] != self.BatteriesInfo[device]['P_batt_inv']:
          #new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(device,
                                       'PowerElectronicsConnection.p', value[1],
                                       self.BatteriesInfo[device]['P_batt_inv'])
          diffCount += 1

          switchStr = ''
          if value[1]>0 and self.BatteriesInfo[device]['P_batt_inv']<0:
            switchStr = ' (SWITCH from discharging to charging)'
          elif value[1]<0 and self.BatteriesInfo[device]['P_batt_inv']>0:
            switchStr = ' (SWITCH from charging to discharging)'

          if printAllDispatchesFlag:
            prlog('DeviceDispatcher--battery device: ' + name +
                  ', timestamp: ' + str(timestamp) + ', new value: ' +
                  str(value[1]) + ', old value: ' +
                  str(self.BatteriesInfo[device]['P_batt_inv']) + switchStr)

          if self.testDeviceName and name==self.testDeviceName:
            prlog('~TEST Dispatching to battery id: ' + device +
                  ', device: ' + name + ', timestamp: ' + str(timestamp) +
                  ', new value: ' + str(value[1]) + ', old value: ' +
                  str(self.BatteriesInfo[device]['P_batt_inv']) + switchStr)

          if self.instantSetpointUpdateFlag:
            self.BatteriesInfo[device]['P_batt_inv'] = value[1]
            MethodUtil.BatteryP_batt_inv[device] = value[1]

        elif name == self.testDeviceName:
          prlog('~TEST DEBUG DeviceDispatcher--DISPATCH NOT needed, battery' +
                ' device: ' + name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

        elif printAllDispatchesFlag:
          prlog('DeviceDispatcher--DISPATCH NOT needed, battery device: ' +
                name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

      elif name.startswith('PhotovoltaicUnit.'):
        # for SolarPV devices the value is complex so if either of those
        # components has changed for a device, both get dispatched
        if value[1] != self.SolarPVs[device]['PQ_pv_inv']:
          #new value before old value for DifferenceBuilder
          # GDB 9/18/25: hardwired p to 0 for some debugging
          #self.difference_builder.add_difference(device,
          #                        'PowerElectronicsConnection.p', 0.0, 0.0)
          self.difference_builder.add_difference(device,
                                  'PowerElectronicsConnection.p', value[1].real,
                                  self.SolarPVs[device]['PQ_pv_inv'].real)
          self.difference_builder.add_difference(device,
                                  'PowerElectronicsConnection.q', value[1].imag,
                                  self.SolarPVs[device]['PQ_pv_inv'].imag)

          diffCount += 1

          if printAllDispatchesFlag:
            prlog('DeviceDispatcher--solarPV device: ' + name +
                  ', timestamp: ' + str(timestamp) + ', new value: ' +
                  str(value[1]) + ', old value: ' +
                  str(self.SolarPVs[device]['PQ_pv_inv']))

          if self.testDeviceName and name==self.testDeviceName:
            prlog('~TEST Dispatching to solarPV id: ' + device +
                  ', device: ' + name + ', timestamp: ' + str(timestamp) +
                  ', new value: ' + str(value[1]) + ', old value: ' +
                  str(self.SolarPVs[device]['PQ_pv_inv']))

          if self.instantSetpointUpdateFlag:
            self.SolarPVs[device]['PQ_pv_inv'] = value[1]
            MethodUtil.SolarPVs_inv[device] = value[1]

        elif name == self.testDeviceName:
          prlog('~TEST DEBUG DeviceDispatcher--DISPATCH NOT needed, solarPV' +
                ' device: ' + name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

        elif printAllDispatchesFlag:
          prlog('DeviceDispatcher--DISPATCH NOT needed, solarPV device: ' +
                name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

      elif name.startswith('RatioTapChanger.'):
        # Dispatch regulator tap positions whenever they are different from the
        # current tap position
        if value[1] != self.Regulators[device]['step']:
          # new value before old value for DifferenceBuilder
          # TODO INVESTIGATE REG4 ISSUE WITH PHASES BEING TIED TOGETHER
          # Uncomment the "if" block to keep from dispatching 4a and 4c
          # setpoint changes to see how it behaves without coupling. Also
          # comment out the same logic that's inside the "if" block
          '''
          if name!='RatioTapChanger.reg4a' and name!='RatioTapChanger.reg4c':
            self.difference_builder.add_difference(device,
                   'TapChanger.step', value[1], self.Regulators[device]['step'])
            diffCount += 1
          '''
          self.difference_builder.add_difference(device,
                   'TapChanger.step', value[1], self.Regulators[device]['step'])
          diffCount += 1

          if printAllDispatchesFlag:
            prlog('DeviceDispatcher--regulator device: ' + name +
                  ', timestamp: ' + str(timestamp) + ', new value: ' +
                  str(value[1]) + ', old value: ' +
                  str(self.Regulators[device]['step']))

          if self.testDeviceName and name==self.testDeviceName:
              prlog('~TEST Dispatching to regulator id: ' + device +
                    ', device: ' + name + ', timestamp: ' + str(timestamp) +
                    ', new value: ' + str(value[1]) + ', old value: ' +
                    str(self.Regulators[device]['step']))

          if self.instantSetpointUpdateFlag:
            self.Regulators[device]['step'] = value[1]
            MethodUtil.RegulatorPos[device] = value[1]

        elif name == self.testDeviceName:
          prlog('~TEST DEBUG DeviceDispatcher--DISPATCH NOT needed, regulator' +
                ' device: ' + name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

        elif printAllDispatchesFlag:
          prlog('DeviceDispatcher--DISPATCH NOT needed, regulator device: ' +
                name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector) > len(newResolutionVector):
      for device in self.ResolutionVector:
        if device not in newResolutionVector:
          if printAllDispatchesFlag:
            prlog('DeviceDispatcher--deleted from resolution, device: ' +
                  MethodUtil.DeviceToName[device])

          if self.testDeviceName and \
             MethodUtil.DeviceToName[device]==self.testDeviceName:
            prlog('~TEST deleted from resolution, id: ' + device +
                  ', device: ' + MethodUtil.DeviceToName[device])

    if diffCount > 0:
      dispatch_message = self.difference_builder.get_message()

      if printAllDispatchesFlag:
        prlog('DeviceDispatcher--sending device dispatch ' +
              'DifferenceBuilder message: ' + json.dumps(dispatch_message))
      else:
        prlog('DeviceDispatcher--sending device dispatch ' +
              'DifferenceBuilder message!')

      self.gapps.send(self.publish_topic, json.dumps(dispatch_message))
      self.difference_builder.clear()

      self.simMessageCounter = 0

    return diffCount


  def pol2cart(self, mag, angle_deg):
        # Convert degrees to radians. GridAPPS-D spits angle in degrees
        angle_rad =  math.radians(angle_deg)
        p = mag * math.cos(angle_rad)
        q = mag * math.sin(angle_rad)
        return p, q


  def ProcessSimulationMessage(self, message, timestamp,
                               printAllMessagesFlag=False):
    if self.pltFlag:
      self.pltFile.write('SIMULATION,')
      diff = (datetime.now() - self.pltTZero).total_seconds()
      self.pltFile.write(str(diff))
      self.pltFile.write(',')
      self.pltFile.write(str(timestamp))

    if not printAllMessagesFlag:
      ts_time = datetime.utcfromtimestamp(timestamp).time()
      prlog('ProcessSimulationMessage--timestamp: ' + str(timestamp) +
            ', wall time: ' + str(ts_time))

    self.simMessageCounter += 1

    measurements = message['measurements']
    for device in self.BatteriesInfo:
      measid = self.BatteriesInfo[device]['SoC_measid']
      if measid in measurements:
        self.BatteriesInfo[device]['SoC'] = measurements[measid]['value']/100.0
        MethodUtil.BatterySoC[device] = self.BatteriesInfo[device]['SoC']
        # comment this out and output it below with P_batt_inv to save space
        #if printAllMessagesFlag:
        #  prlog('ProcessSimulationMessage--timestamp: ' +
        #        str(timestamp) + ', device: ' +
        #        self.BatteriesInfo[device]['name'] +
        #        ', SoC: ' + str(self.BatteriesInfo[device]['SoC']))

      measid = self.BatteriesInfo[device]['P_batt_measid']
      if measid in measurements:
        # always update timestamp because it's needed for running history rule
        self.BatteriesInfo[device]['timestamp'] = timestamp

        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])
        # negate the p value from the simulation so it is directly comparable
        # to the value that must be given to GridLAB-D in a DifferenceBuilder
        # message
        meas_P_batt_inv = -p

        if 'P_batt_inv' in self.BatteriesInfo[device] and \
            meas_P_batt_inv!=self.BatteriesInfo[device]['P_batt_inv']:
          if printAllMessagesFlag:
            prlog('ProcessSimulationMessage--BatteryHistory candidate,' +
                  'device: ' + self.BatteriesInfo[device]['name'] +
                  ', old: ' + str(self.BatteriesInfo[device]['P_batt_inv']) +
                  ', new: ' + str(meas_P_batt_inv))
          # check if this is a change from charging to discharging or vice versa
          if (meas_P_batt_inv>0 and \
              self.BatteriesInfo[device]['P_batt_inv']<0) or \
             (meas_P_batt_inv<0 and self.BatteriesInfo[device]['P_batt_inv']>0):
            # append the timestamp, P_batt_inv to the running history
            self.BatteryHistory[device].append((timestamp, meas_P_batt_inv))
            prlog('ProcessSimulationMessage--BatteryHistory match, device: ' +
                  self.BatteriesInfo[device]['name'] +
                  ', history: ' + str(self.BatteryHistory[device]))

        # I think for BatteryHistory there is no need to get a starting point
        # like there is for regulators since we are just tracking changes from
        # charge to discharge and vice versa and not all changes
        #elif len(self.BatteryHistory[device]) == 0:
        #  self.BatteryHistory[device].append((timestamp, meas_P_batt_inv))
        #  prlog('ProcessSimulationMessage--BatteryHistory initialize, device: '
        #        + self.BatteriesInfo[device]['name'] +
        #        ', history: ' + str(self.BatteryHistory[device]))

        self.BatteriesInfo[device]['P_batt_inv'] = meas_P_batt_inv
        MethodUtil.BatteryP_batt_inv[device] = meas_P_batt_inv
        if printAllMessagesFlag:
          prlog('ProcessSimulationMessage--timestamp: ' +
                str(timestamp) + ', device: ' +
                self.BatteriesInfo[device]['name'] + ', P_batt_inv: ' +
                str(self.BatteriesInfo[device]['P_batt_inv']) + ', SoC: ' +
                str(self.BatteriesInfo[device]['SoC']))

        if self.pltFlag:
          self.pltFile.write(',')
          self.pltFile.write(self.BatteriesInfo[device]['name'])
          self.pltFile.write(',')
          self.pltFile.write(str(self.BatteriesInfo[device]['P_batt_inv']))
          self.pltFile.write(',')
          self.pltFile.write(str(self.BatteriesInfo[device]['SoC']))

    for device in self.Regulators:
      measid = self.Regulators[device]['measid']
      if measid in measurements:
        # always update timestamp because it's needed for running history rule
        self.Regulators[device]['timestamp'] = timestamp

        # only update the rest if there is a value change
        if measurements[measid]['value'] != self.Regulators[device]['step']:
          self.Regulators[device]['step'] = measurements[measid]['value']
          MethodUtil.RegulatorPos[device] = self.Regulators[device]['step']
          if printAllMessagesFlag:
            prlog('ProcessSimulationMessage--timestamp: ' +
                  str(timestamp) + ', device: ' +
                  self.Regulators[device]['name'] + ', tap position: ' +
                  str(self.Regulators[device]['step']))

          # append the timestamp, step to the running history
          self.RegulatorHistory[device].append((timestamp,
                                               self.Regulators[device]['step']))
          if self.Regulators[device]['name'] == self.testDeviceName:
            self.refCount += 1
            prlog('~TEST DEBUG CHANGE ProcessSimulationMessage--device: ' + 
                  self.Regulators[device]['name'] + ', timestamp: ' +
                  str(timestamp) + ', tap position: ' +
                  str(self.Regulators[device]['step']) +
                  ', ref: ' + str(self.refCount))

        elif len(self.RegulatorHistory[device]) == 0:
          # need to get a starting history data point at the current timestamp
          self.RegulatorHistory[device].append((timestamp,
                                               self.Regulators[device]['step']))

        if self.pltFlag:
          self.pltFile.write(',')
          self.pltFile.write(self.Regulators[device]['name'])
          self.pltFile.write(',')
          self.pltFile.write(str(self.Regulators[device]['step']))

    for bus in self.SolarPVsInfo:
      measid = self.SolarPVsInfo[bus]['measid']
      if measid in measurements:
        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])

        device = self.SolarPVsInfo[bus]['mrid']

        # negate the p and q values from the simulation so it is directly
        # comparable to the value that must be given to GridLAB-D in a
        # DifferenceBuilder message
        meas_PQ_pv_inv = complex(-p, -q)

        # only update if there is a value change
        if meas_PQ_pv_inv != self.SolarPVs[device]['PQ_pv_inv']:
          self.SolarPVs[device]['PQ_pv_inv'] = meas_PQ_pv_inv
          MethodUtil.SolarPVs_inv[device] = meas_PQ_pv_inv
          if printAllMessagesFlag:
            prlog('ProcessSimulationMessage--timestamp: ' +
                  str(timestamp) + ', device: ' +
                  self.SolarPVsInfo[bus]['name'] + ', PQ_pv_inv: ' +
                  str(self.SolarPVs[device]['PQ_pv_inv']))

        if self.pltFlag:
          self.pltFile.write(',')
          self.pltFile.write(self.SolarPVsInfo[bus]['name'])
          self.pltFile.write(',')
          self.pltFile.write(str(self.SolarPVs[device]['PQ_pv_inv']))

    if self.pltFlag:
      self.pltFile.write('\n')
      self.pltFile.flush()

    if self.testDeviceName:
      device = MethodUtil.NameToDevice[self.testDeviceName]
      if device in self.BatteriesInfo:
        prlog('~TEST simulation updated SoC for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(timestamp)+
              ', SoC: ' + str(self.BatteriesInfo[device]['SoC']))
        prlog('~TEST simulation updated P_batt_inv for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(timestamp)+
              ', P_batt_inv: ' + str(self.BatteriesInfo[device]['P_batt_inv']))
      elif device in self.Regulators:
        prlog('~TEST simulation updated tap position for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(timestamp) +
              ', pos: ' + str(self.Regulators[device]['step']))
      elif device in self.SolarPVs:
        prlog('~TEST simulation updated PQ_pv_inv for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(timestamp)+
              ', PQ_pv_inv: ' + str(self.SolarPVs[device]['PQ_pv_inv']))


  def PlotDispatch(self, reason, newResolutionVector):
    if self.pltFlag:
      timerRunning = (datetime.now() - self.pltTZero).total_seconds()
      self.pltFile.write('device_dispatch,reason:' + reason + ',runningTime:' + str(timerRunning) + ',rulesTime:' + str(self.timerRules) + ',coopTime:' + str(self.timerCoop) + ',optTime:' + str(self.timerOpt))

      for device, value in newResolutionVector.items():
        name = MethodUtil.DeviceToName[device]
        if name.startswith('BatteryUnit.'):
          if value[1] != self.BatteriesInfo[device]['P_batt_inv']:
            self.pltFile.write(',' + name + ':' + str(value[1]))
        elif name.startswith('PhotovoltaicUnit.'):
          if value[1] != self.SolarPVs[device]['PQ_pv_inv']:
            self.pltFile.write(',' + name + ':' + str(value[1]))
        elif name.startswith('RatioTapChanger.'):
          if value[1] != self.Regulators[device]['step']:
            self.pltFile.write(',' + name + ':' + str(value[1]))

      self.pltFile.write('\n')


  def ProcessSetpointsMessage(self, message, timestamp, app_name, meas_msg_flag,
                              coop_series, printAllConflictsResolutionsFlag):
    if meas_msg_flag:
      prlog('>>>\n>>> ProcessSetpointsMessage--MEAS message timestamp: ' +
            str(timestamp) + ', app: ' + app_name)
    else:
      prlog('>>>\n>>> ProcessSetpointsMessage--COOP message timestamp: ' +
            str(timestamp) + ', app: ' + app_name + ', series: '+
            str(coop_series))

    if not meas_msg_flag and coop_series!=self.coopCurrentSeries:
      # discard any cooperation messages when not currently cooperating or
      # when from a previous cooperation series
      prlog('>>> ProcessSetpointsMessage--discard of nonmatching ' +
            'COOP message, series: ' + str(coop_series) + ', current series: ' +
            str(self.coopCurrentSeries))
      prlog('ProcessSetpointsMessage--finished processing, timestamp: ' +
            str(timestamp) + ', app: ' + app_name)
      return False

    if meas_msg_flag and self.coopCurrentFlag:
      if self.coopTimestamp == timestamp:
        prlog('>>> ProcessSetpointsMessage--special case skipping device ' +
              'dispatch for MEAS message with running COOPERATION initiated ' +
              'for same timestamp: ' + str(timestamp))

      else:
        # checking for coopTimestamp!=timestamp fixes a special case where we've
        # already ended the last series of cooperation but then more meas
        # messages arrive and we don't want to immediately do further dispatches
        prlog('>>> ProcessSetpointsMessage--conclude running COOPERATION ' +
              'series with new MEAS message received, coopTimestamp: ' +
              str(self.coopTimestamp))

        self.coopTimestamp = 0
        self.coopCurrentFlag = False

        # GDB 9/10/25: If we don't want to do the device dispatch if cooperation
        # was interrupted by a new measurements based setpoints request, comment
        # out code starting here until the comment below with the same date tag.
        # The danger in doing this if the schedule for optimizations or the
        # time they take in relation to the interval between optimizations is
        # such that cooperation thresholds aren't being met, then there won't
        # be any device dispatches.

        # we were cooperating when a measurement message arrived so need to
        # conclude that cooperation before processing the new message

        # replace running ConflictMatrix with the minimum conflict version and
        # we'll roll with that from this point on
        self.ConflictMatrix = copy.deepcopy(self.MinConflictMatrix)

        # set the final conflict metric value to the minimum achieved to
        # correspond to the minimum ConflictMatrix
        self.conflictMetric = min(self.conflictMetric, self.minConflictMetric)

        # Published IEEE Access Foundational Paper Reference:
        #   Step 3.2--Deconfliction Solution
        #   Step 3.3--Resolution
        # OPTIMIZATION stage deconfliction
        prlog('ProcessSetpointsMessage--applying OPTIMIZATION stage ' +
             'deconfliction to minimum conflict matrix for running COOPERATION')

        # update incentive weights using minimum conflict matrix before final
        # optimization stage and device dispatch
        self.CooperationWeightsUpdate(timestamp, self.ConflictMatrix,
                                      self.TargetResolutionVector)

        self.logConflictTest('running COOPERATION before OPTIMIZATION')

        # start optimization triggered by new setpoints interrupting cooperation

        # timers for scalability testing
        coopFinish = datetime.now()
        self.timerCoop = (coopFinish - self.timerCoopStart).total_seconds()

        newResolutionVector = self.Optimization(timestamp, self.ConflictMatrix)
        self.logResolutionTest('running COOPERATION after OPTIMIZATION',
                               newResolutionVector)
        # timers for scalability testing
        self.timerOpt = (datetime.now() - coopFinish).total_seconds()

        # Published IEEE Access Foundational Paper Reference:
        #   Step 3.2--Deconfliction Solution
        # RULES & HEURISTICS stage deconfliction done last
        if self.rulesStageLastFlag:
          prlog('ProcessSetpointsMessage--applying final RULES & HEURISTICS ' +
                'stage deconfliction for running COOPERATION')
          self.RulesForBatteriesResolution(newResolutionVector,
                                           self.printAllRulesFlag)
          self.logConflictTest('running COOPERATION before last rules stage')
          self.RulesForRegulatorsResolution(newResolutionVector,
                                            self.printAllRulesFlag)
          self.logConflictTest('running COOPERATION after last rules stage')

        # Published IEEE Access Foundational Paper Reference:
        #   Step 4--Setpoint Validator
        self.SetpointValidatorForBatteries(newResolutionVector,
                                           self.printAllValidatorFlag)
        self.SetpointValidatorForRegulators(newResolutionVector,
                                            self.printAllValidatorFlag)

        if self.logMessagesFlag:
          msglog('INTERRUPT cooperation with new meas setpoints|responses:' +
                 str(self.coopResponseCounter) + '|series:' +
                 str(self.coopCurrentSeries))

        # Output conflict metric data to plot_data.csv for a concluded
        # cooperation where it hasn't reached thresholds
        if self.pltFlag:
          self.pltFile.write('conflict_metric,')
          diff = (datetime.now() - self.pltTZero).total_seconds()
          self.pltFile.write(str(diff))
          self.pltFile.write(',')
          self.pltFile.write(str(timestamp))
          self.pltFile.write(',')
          self.pltFile.write(str(self.startConflictMetric))
          self.pltFile.write(',')
          if self.rulesStageFirstFlag:
            self.pltFile.write(str(self.rulesFirstConflictMetric))
            self.pltFile.write(',')
          self.pltFile.write(str(self.conflictMetric))
          self.pltFile.write(',')
          self.pltFile.write('Delta:N/A')
          self.pltFile.write(',Responses:' + str(self.coopResponseCounter))
          self.pltFile.write(',AppCounts:' + str(self.AppCoopCount))
          self.pltFile.write(',Series:')
          self.pltFile.write(str(self.coopCurrentSeries))
          self.pltFile.write(',Reason:New_Optimization_Setpoints')
          self.pltFile.write('\n')

        # Published IEEE Access Foundational Paper Reference:
        #   Step 5--Device Dispatcher
        # start dispatch triggered by new setpoints interrupting cooperation
        # logging for scalability testing
        self.PlotDispatch('CoopInterrupted', newResolutionVector)
        dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                              self.printAllDispatchesFlag)
        prlog('>>> ProcessSetpointsMessage--invoked device dispatch for ' +
              'running COOPERATION, # devices dispatched: ' +str(dispatchCount))

        # update the current resolution to the new resolution to be ready for
        # the next dispatch
        self.ResolutionVector.clear()
        self.ResolutionVector = newResolutionVector

        # GDB 9/10/25: End of code to comment out for no devices dispatches
        # when a cooperation series is interrupted by a new measurements based
        # setpoint message

        # reset running minimums for conflict metric and matrix
        self.minConflictMetric = 1.0
        # reset running counts for cooperation messages
        self.AppCoopCount.clear()

    # if this is a cooperation response and we are in a cooperation series
    # we need to increment the counters
    elif self.coopCurrentFlag:
      self.coopResponseCounter += 1

      if app_name in self.AppCoopCount:
        self.AppCoopCount[app_name] += 1
      else:
        self.AppCoopCount[app_name] = 1

    # set_points are the forward_differences part of the DifferenceBuilder
    # message with keys of object, attribute, and value
    set_points = message['forward_differences']

    # Published IEEE Access Foundational Paper Reference:
    #   Step 1--Setpoint Processor
    prlog('ProcessSetpointsMessage--invoking setpoint processor')
    self.SetpointProcessor(app_name, timestamp, set_points, meas_msg_flag,
                           printAllConflictsResolutionsFlag)

    self.logConflictTest('after SetpointProcessor with new setpoints')

    return True


  def DeconflictSetpoints(self, timestamp, meas_msg_flag,
                          printAllConflictsResolutionsFlag):
    if meas_msg_flag:
      self.startConflictMetric = self.ConflictMetricComputation(timestamp)

      # no need to invoke Feasibility Maintainer or Rules stage for cooperation
      # messages because the SetpointProcessor insures there is no backtracking
      # of setpoints from what was already processed by the Feasibility
      # Maintainer and Rules stage

      # Published IEEE Access Foundational Paper Reference:
      #   Step 2--Feasibility Maintainer
      self.FeasibilityMaintainerForBatteries(self.printAllFeasibilityFlag)
      self.FeasibilityMaintainerForRegulators(self.printAllFeasibilityFlag)

      # Published IEEE Access Foundational Paper Reference:
      #   Step 3.2--Deconfliction Solution
      # RULES & HEURISTICS stage deconfliction done first
      if self.rulesStageFirstFlag:
        prlog('DeconflictSetpoints--applying initial RULES & HEURISTICS ' +
              'stage deconfliction')
        # start rules
        # timers for scalability testing
        rulesStart = datetime.now()
        self.RulesForBatteriesConflict(self.printAllRulesFlag)

        self.logConflictTest('start deconfliction before first rules stage')
        self.RulesForRegulatorsConflict(self.printAllRulesFlag)
        # finish rules
        # timers for scalability testing
        self.timerRules = (datetime.now() - rulesStart).total_seconds()
        self.logConflictTest('start deconfliction after first rules stage')

        self.rulesFirstConflictMetric =self.ConflictMetricComputation(timestamp)

        # save the conflict metric/matrix after applying rules since sometimes
        # cooperation can't do any better than this so we need to go with this
        self.minConflictMetric = self.rulesFirstConflictMetric
        self.MinConflictMatrix = copy.deepcopy(self.ConflictMatrix)

    # Published IEEE Access Foundational Paper Reference:
    #   Step 3--Deconflictor
    # Published IEEE Access Foundational Paper Reference:
    #   Step 3.1--Conflict Identification

    prlog('DeconflictSetpoints--invoking conflict identification')
    conflictFlag = self.ConflictIdentification()

    if not conflictFlag:
      prlog('>>> DeconflictSetpoints--conflict NOT found in conflict ' +
            'matrix')
      # zero conflict metric since by definition there is none
      self.conflictMetric = 0.0

      # start with a copy of the previous resolution
      newResolutionVector = copy.deepcopy(self.ResolutionVector)

      # copy the new set-points over top of the previous resolution
      for device in self.ConflictMatrix:
        if len(self.ConflictMatrix[device]) > 0:
          maxtime = 0
          for app in self.ConflictMatrix[device]:
            maxtime = max(maxtime, self.ConflictMatrix[device][app][0])
            # since there is no conflict all setpoints are the same so grab any
            setpoint = self.ConflictMatrix[device][app][1]
          newResolutionVector[device] = (maxtime, setpoint)

      # Published IEEE Access Foundational Paper Reference:
      #   Step 3.2--Deconfliction Solution
      # RULES & HEURISTICS stage deconfliction done last
      # if there is no conflict and rules were just applied, there is no
      # need to apply them again since the ConflictMatrix has not changed
      if self.rulesStageLastFlag and not self.rulesStageFirstFlag:
        prlog('DeconflictSetpoints--applying final RULES & HEURISTICS ' +
              'stage deconfliction')
        self.RulesForBatteriesResolution(newResolutionVector,
                                         self.printAllRulesFlag)
        self.logResolutionTest('no conflict before last rules stage',
                               newResolutionVector)
        self.RulesForRegulatorsResolution(newResolutionVector,
                                          self.printAllRulesFlag)
        self.logResolutionTest('no conflict after last rules stage',
                               newResolutionVector)

      if printAllConflictsResolutionsFlag:
        prlog('DeconflictSetpoints--ResolutionVector (no conflict): ' +
              str(newResolutionVector))

      if self.testDeviceName:
        device = MethodUtil.NameToDevice[self.testDeviceName]
        if device in newResolutionVector:
          prlog('~TEST ResolutionVector (no conflict) for ' +
                self.testDeviceName + ' setpoint: ' +
                str(newResolutionVector[device][1]) +
                ', timestamp: ' +
                str(newResolutionVector[device][0]))
        else:
          prlog('~TEST ResolutionVector (no conflict) does not contain ' +
                self.testDeviceName)

      # Published IEEE Access Foundational Paper Reference:
      #   Step 4--Setpoint Validator
      self.SetpointValidatorForBatteries(newResolutionVector,
                                         self.printAllValidatorFlag)
      self.SetpointValidatorForRegulators(newResolutionVector,
                                          self.printAllValidatorFlag)

      # Published IEEE Access Foundational Paper Reference:
      #   Step 5--Device Dispatcher
      dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                            self.printAllDispatchesFlag)
      prlog('>>> DeconflictSetpoints--invoked device dispatch, # ' +
            'devices dispatched: ' +str(dispatchCount))

      # update the current resolution to the new resolution to be ready for the
      # next dispatch
      self.ResolutionVector.clear()
      self.ResolutionVector = newResolutionVector

      # zero the cooperation timestamp to indicate no active cooperation
      self.coopTimestamp = 0
      self.coopCurrentFlag = False
      # reset running minimums for conflict metric and matrix
      self.minConflictMetric = 1.0
      # reset running counts for cooperation messages
      self.AppCoopCount.clear()
      prlog('DeconflictSetpoints--finished processing, timestamp: ' +
            str(timestamp))
      return

    # conflict identified logic
    prlog('DeconflictSetpoints--conflict YES found in conflict matrix')
    if meas_msg_flag:
      # start with a "target" resolution vector using the optimization code
      # that computes a centroid/target per device
      # GDB 9/10/25: something broken with CoopOptimization so don't call it
      #self.TargetResolutionVector, coopProposed = self.CoopOptimization(
      #                                          timestamp, self.ConflictMatrix)
      self.TargetResolutionVector = self.Optimization(timestamp,
                                                      self.ConflictMatrix)

      # if we are not performing cooperation state deconfliction, use the
      # target resolution vector as the final one and proceed to dispatch
      if not self.coopStageFlag:
        prlog('>>> DeconflictSetpoints--bypassing COOPERATION stage')

        # Published IEEE Access Foundational Paper Reference:
        #   Step 3.2--Deconfliction Solution
        # RULES & HEURISTICS stage deconfliction done last
        # if there is no conflict and rules were just applied, there is no
        # need to apply them again since the ConflictMatrix has not changed
        if self.rulesStageLastFlag and not self.rulesStageFirstFlag:
          prlog('DeconflictSetpoints--bypassing COOPERATION applying final ' +
                'RULES & HEURISTICS stage deconfliction')
          self.RulesForBatteriesResolution(self.TargetResolutionVector,
                                           self.printAllRulesFlag)
          self.logResolutionTest('bypassing COOPERATION before last rules stage', self.TargetResolutionVector)
          self.RulesForRegulatorsResolution(self.TargetResolutionVector,
                                            self.printAllRulesFlag)
          self.logResolutionTest('bypassing COOPERATION after last rules stage', self.TargetResolutionVector)

        if printAllConflictsResolutionsFlag:
          prlog('DeconflictSetpoints--ResolutionVector (bypassing COOPERATION): ' + str(self.TargetResolutionVector))

        if self.testDeviceName:
          device = MethodUtil.NameToDevice[self.testDeviceName]
          if device in self.TargetResolutionVector:
            prlog('~TEST ResolutionVector (bypassing COOPERATION) for ' +
                  self.testDeviceName + ' setpoint: ' +
                  str(self.TargetResolutionVector[device][1]) +
                  ', timestamp: ' +
                  str(self.TargetResolutionVector[device][0]))
          else:
            prlog('~TEST ResolutionVector (bypassing COOPERATION) does not contain ' + self.testDeviceName)

        # Published IEEE Access Foundational Paper Reference:
        #   Step 4--Setpoint Validator
        self.SetpointValidatorForBatteries(self.TargetResolutionVector,
                                           self.printAllValidatorFlag)
        self.SetpointValidatorForRegulators(self.TargetResolutionVector,
                                            self.printAllValidatorFlag)

        # uncomment to output conflict metric values without cooperation
        if self.pltFlag:
          self.pltFile.write('conflict_metric,')
          diff = (datetime.now() - self.pltTZero).total_seconds()
          self.pltFile.write(str(diff))
          self.pltFile.write(',')
          self.pltFile.write(str(timestamp))
          self.pltFile.write(',')
          self.pltFile.write(str(self.startConflictMetric))
          self.pltFile.write(',')
          if self.rulesStageFirstFlag:
            self.pltFile.write(str(self.rulesFirstConflictMetric))
            self.pltFile.write(',')
          self.pltFile.write('BYPASS_COOPERATION')
          self.pltFile.write('\n')

        # Published IEEE Access Foundational Paper Reference:
        #   Step 5--Device Dispatcher
        dispatchCount = self.DeviceDispatcher(timestamp,
                                              self.TargetResolutionVector,
                                              self.printAllDispatchesFlag)
        prlog('>>> DeconflictSetpoints--invoked device dispatch, # ' +
              'devices dispatched: ' +str(dispatchCount))

        # update the current resolution to the new resolution to be ready for
        # the next dispatch
        self.ResolutionVector.clear()
        self.ResolutionVector = self.TargetResolutionVector

        prlog('DeconflictSetpoints--finished processing, timestamp: ' +
              str(timestamp))
        return

      # start cooperation
      # timers for scalability testing
      self.timerCoopStart = datetime.now()

      # Published IEEE Access Foundational Paper Reference:
      #   Step 3.2--Deconfliction Solution
      # COOPERATION stage deconfliction
      prlog('DeconflictSetpoints--applying COOPERATION stage ' +
            'deconfliction for meas message')

      # compute conflict metric for later comparison during later cooperation
      self.conflictMetric = self.ConflictMetricComputation(timestamp,
                                                       self.printAllMetricsFlag)

      # clear incentive weights before kicking off cooperation series because
      # we always start from scratch
      self.CooperationWeightsClear(timestamp, self.ConflictMatrix)

      # need to insure there is always a minimum conflict matrix as soon as the
      # target resolution vector is set in case we never hit the code before
      # the threshold check that normally sets it
      self.MinConflictMatrix = copy.deepcopy(self.ConflictMatrix)

      # publish this target resolution vector to the cooperation topic for
      # competing apps that support cooperation to respond to
      self.coopResponseCounter = 0
      self.coopConflictFlag = False
      self.coopCurrentSeries += 1
      self.coopCurrentFlag = True

      # can't serialize TargetResolutionVector that contains complex numbers
      # for SolarPV setpoints. Need to translate all of those to tuples
      coopProposed = copy.deepcopy(self.TargetResolutionVector)
      for device, value in coopProposed.items():
        if isinstance(value[1], complex):
          coopProposed[device] = (value[0], (value[1].real, value[1].imag))

      self.coopMsgID += 1
      coopMessage = {'time_sent': str(datetime.now()),
                     'coop_msgid': self.coopMsgID,
                     'coop_series': self.coopCurrentSeries,
                     'coop_proposed': coopProposed}
      self.gapps.send(self.coop_topic, json.dumps(coopMessage))
      if self.logMessagesFlag:
        msglog('requesting initial cooperation|msgid:' + str(self.coopMsgID) +
               '|series:' + str(self.coopCurrentSeries))
      prlog('>>> DeconflictSetpoints--kicked off new COOPERATION series, ' +
            'updated current series: ' + str(self.coopCurrentSeries))

      #self.logConflictReg('coop kickoff')
      #self.logConflictPV('coop kickoff')

      # set the cooperation timestamp to indicate when cooperation was initiated
      self.coopTimestamp = timestamp
      prlog('DeconflictSetpoints--finished processing, timestamp: ' +
            str(timestamp))
      return

    # coop message with conflict to get here
    prlog('DeconflictSetpoints--conflict found with COOP ' +
          'message, checking thresholds')

    # save the previous conflict metric for comparison
    prevConflictMetric = self.conflictMetric

    self.conflictMetric = self.ConflictMetricComputation(timestamp,
                                                       self.printAllMetricsFlag)

    # save the running minimum conflict metric and associated conflict matrix
    # during a cooperation series as the one to use when cooperation concludes
    if self.conflictMetric <= self.minConflictMetric:
      self.minConflictMetric = self.conflictMetric
      self.MinConflictMatrix = copy.deepcopy(self.ConflictMatrix)

    perConflictDelta = 100.0 # for no previous conflict metric value
    if prevConflictMetric > 0.0:
      perConflictDelta = 100.0 * (prevConflictMetric - self.conflictMetric)/ \
                                 prevConflictMetric

    prlog('>>> DeconflictSetpoints--thresholds, prev conflict metric: ' +
          str(prevConflictMetric) + ', new metric: ' + str(self.conflictMetric)+
          ', % change: ' + str(perConflictDelta) +
          ', min metric: ' + str(self.minConflictMetric) +
          ', cooperation responses: ' + str(self.coopResponseCounter))

    # check for NOT meeting thresholds
    # first, loop over all apps to check if max messages threshold is met
    coopMaxMessageFlag = False
    for app in self.AppCoopCount:
      if self.AppCoopCount[app] > self.coopMessagesThreshold:
        coopMaxMessageFlag = True
        break

    # thresholds for ending cooperation are either exceeding a per app
    # maximum for number of cooperation response messages, a conflict metric
    # value below a specified value or a (nonnegative) % conflict change between
    # 0 and a specifed value between iterations (cooperation responses).
    # The % conflict change needs to happen over two responses as controlled by
    # the self.coopConflictFlag as a single response can bail too soon.
    if self.coopResponseCounter<self.coopMinResponses or \
       ((not coopMaxMessageFlag) and \
        self.conflictMetric>self.conflictValueThreshold and \
        (((not self.coopConflictFlag) and \
          perConflictDelta>self.conflictPercentThreshold) or \
         perConflictDelta<0.0)):

      # initiate further cooperation
      prlog('>>> DeconflictSetpoints--thresholds NOT met, initiating ' +
            'further COOPERATION at response: ' + str(self.coopResponseCounter))

      # update incentive weights for every cooperation response
      self.CooperationWeightsUpdate(timestamp, self.ConflictMatrix,
                                    self.TargetResolutionVector)

      # start with a "target" resolution vector using the optimization code
      # that computes a weighted centroid per device
      # GDB 9/10/25: something broken with CoopOptimization so don't call it
      #newTargetResolutionVector, coopProposed =self.CoopOptimization(timestamp,
      #                                                     self.ConflictMatrix)
      coopProposed = self.Optimization(timestamp, self.ConflictMatrix)

      # can't serialize complex numbers for SolarPV setpoints so need to
      # translate all of those to tuples
      for device, value in coopProposed.items():
        if isinstance(value[1], complex):
          coopProposed[device] = (value[0], (value[1].real, value[1].imag))

      # publish this proposed setpoint vector to the cooperation topic for
      # competing apps that support cooperation to respond to
      self.coopMsgID += 1
      coopMessage = {'time_sent': str(datetime.now()),
                     'coop_msgid': self.coopMsgID,
                     'coop_series': self.coopCurrentSeries,
                     'coop_proposed': coopProposed}
      self.gapps.send(self.coop_topic, json.dumps(coopMessage))
      if self.logMessagesFlag:
        msglog('requesting more cooperation|msgid:' + str(self.coopMsgID) +
               '|series:' + str(self.coopCurrentSeries))
      prlog('DeconflictSetpoints--finished processing, timestamp: ' +
            str(timestamp))
      return

    # zero the cooperation timestamp to indicate no active cooperation
    self.coopTimestamp = 0
    self.coopCurrentFlag = False

    # flag for whether to conclude cooperation the first time the % conflict
    # change is below the threshold or if it needs to happen twice
    # Hardwire value to false so the first check can conclude cooperation and
    # set it to the expression if two checks are required
    #self.coopConflictFlag = False
    self.coopConflictFlag = perConflictDelta <= self.conflictPercentThreshold

    # thresholds for ending cooperation have been met to get here
    reason = 'None'
    if coopMaxMessageFlag:
      reason = 'Max_Cooperation_Responses'
      prlog('>>> DeconflictSetpoints---threshold YES met for max ' +
            'cooperation responses by an app, concluding COOPERATION with ' +
            'app response counts: ' + str(self.AppCoopCount))
    elif self.conflictMetric <= self.conflictValueThreshold:
      reason = 'Conflict_Metric_Value'
      prlog('>>> DeconflictSetpoints---threshold YES met for conflict ' +
            'metric value, concluding COOPERATION with conflict metric: ' +
            str(self.conflictMetric) + ', responses: ' +
            str(self.coopResponseCounter))
    else:
      reason = 'Conflict_Metric_Delta'
      prlog('>>> DeconflictSetpoints---threshold YES met for conflict ' +
            'metric % change, concluding COOPERATION with % change: ' +
            str(perConflictDelta) + ', responses: ' +
            str(self.coopResponseCounter))

    # replace running ConflictMatrix with the minimum conflict version and
    # we'll roll with that from this point on
    self.ConflictMatrix = copy.deepcopy(self.MinConflictMatrix)

    # set the final conflict metric value to the minimum achieved to
    # correspond to the minimum ConflictMatrix
    self.conflictMetric = min(self.conflictMetric, self.minConflictMetric)

    # Published IEEE Access Foundational Paper Reference:
    #   Step 3.2--Deconfliction Solution
    #   Step 3.3--Resolution
    # OPTIMIZATION stage deconfliction
    prlog('DeconflictSetpoints--applying OPTIMIZATION stage ' +
          'deconfliction to minimum conflict ConflictMatrix')
    # update incentive weights using minimum conflict matrix before final
    # optimization stage and device dispatch
    self.CooperationWeightsUpdate(timestamp, self.ConflictMatrix,
                                  self.TargetResolutionVector)

    self.logConflictTest('COOPERATION stage done before OPTIMIZATION')
    # start optimization triggered by cooperation concluding
    # timers for scalability testing
    coopFinish = datetime.now()
    self.timerCoop = (coopFinish - self.timerCoopStart).total_seconds()

    newResolutionVector = self.Optimization(timestamp, self.ConflictMatrix)
    self.logConflictTest('cooperation stage done after OPTIMIZATION')
    self.timerOpt = (datetime.now() - coopFinish).total_seconds()

    #self.logResolutionPV('coop done', newResolutionVector)

    # Published IEEE Access Foundational Paper Reference:
    #   Step 3.2--Deconfliction Solution
    # RULES & HEURISTICS stage deconfliction done last
    if self.rulesStageLastFlag:
      prlog('DeconflictSetpoints--applying final RULES & HEURISTICS ' +
            'stage deconfliction')
      self.RulesForBatteriesResolution(newResolutionVector,
                                       self.printAllRulesFlag)

      self.logResolutionTest('COOPERATION and OPTIMIZATION stages done before last rules stage', newResolutionVector)
      self.RulesForRegulatorsResolution(newResolutionVector,
                                        self.printAllRulesFlag)
      self.logResolutionTest('COOPERATION and OPTIMIZATION stages done after last rules stage', newResolutionVector)

    # Published IEEE Access Foundational Paper Reference:
    #   Step 4--Setpoint Validator
    self.SetpointValidatorForBatteries(newResolutionVector,
                                       self.printAllValidatorFlag)
    self.SetpointValidatorForRegulators(newResolutionVector,
                                        self.printAllValidatorFlag)

    if self.logMessagesFlag:
      msglog('SUCCESSFUL cooperation conclusion|responses:' +
             str(self.coopResponseCounter) + '|series:' +
             str(self.coopCurrentSeries))

    if self.pltFlag:
      self.pltFile.write('conflict_metric,')
      diff = (datetime.now() - self.pltTZero).total_seconds()
      self.pltFile.write(str(diff))
      self.pltFile.write(',')
      self.pltFile.write(str(timestamp))
      self.pltFile.write(',')
      self.pltFile.write(str(self.startConflictMetric))
      self.pltFile.write(',')
      if self.rulesStageFirstFlag:
        self.pltFile.write(str(self.rulesFirstConflictMetric))
        self.pltFile.write(',')
      self.pltFile.write(str(self.conflictMetric))
      self.pltFile.write(',')
      self.pltFile.write('Delta:')
      self.pltFile.write(str(perConflictDelta))
      self.pltFile.write(',Responses:' + str(self.coopResponseCounter))
      self.pltFile.write(',AppCounts:' + str(self.AppCoopCount))
      self.pltFile.write(',Series:')
      self.pltFile.write(str(self.coopCurrentSeries))
      self.pltFile.write(',Reason:')
      self.pltFile.write(reason)
      self.pltFile.write('\n')

    # Published IEEE Access Foundational Paper Reference:
    #   Step 5--Device Dispatcher
    # start dispatch triggered by cooperation concluding
    # logging for scalability testing
    self.PlotDispatch('CoopConcluded', newResolutionVector)
    dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                          self.printAllDispatchesFlag)
    prlog('>>> DeconflictSetpoints--invoked device dispatch, # ' +
          'devices dispatched: ' +str(dispatchCount))

    # update the current resolution to the new resolution to be ready for the
    # next dispatch
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # reset running minimum for conflict metric
    self.minConflictMetric = 1.0
    # reset running counts for cooperation messages
    self.AppCoopCount.clear()
    prlog('DeconflictSetpoints--finished processing, timestamp: ' +
          str(timestamp))


  def __init__(self, feeder_mrid, simulation_id, weights_base, interval):
    # GDB 9/22/25: Zero the plot timer as soon as possible
    self.pltTZero = datetime.now()

    # flag for whether simulation is run in real-time
    #self.realtimeFlag = True
    self.realtimeFlag = False

    # GDB 10/3/25: sim log messages overwhelm ActiveMQ causing severe
    # delays in message delivery in non-realtime mode
    self.simLogSubscribedFlag = True
    if not self.realtimeFlag:
      self.simLogSubscribedFlag = False
      self.simTimestampStart = None
      # match duration with simulation configuration
      # for some reason final expected timestamp isn't sent so subtract 2 off
      self.simDuration = 86400-120

    # flag for whether to log cooperation messages in a file
    self.logMessagesFlag = True

    self.messageQueue = Queue()

    # this is referenced in the MessageListener process so need to
    # set it before creating that
    self.printAllMessagesFlag = False

    # subscribe to simulation log and output messages in new process
    # since messages are just going on a queue, subscribe right away to
    # keep from missing any sent during pipeline initialization
    messageListener = Process(target=self.messageListenerProcess,
                              args=(simulation_id,))
    messageListener.start()

    self.gapps = GridAPPSD(simulation_id)
    assert self.gapps.connected

    if deconflictionAsServiceFlag:
      # service topic for sending DifferenceBuilder messages
      self.publish_topic = service_output_topic(
                           'gridappsd-app-deconfliction-service', simulation_id)

    else:
      # simulation topic for sending DifferenceBuilder messages
      self.publish_topic = simulation_input_topic(simulation_id)

    # service topic for sending target resolution messages to cooperating apps
    self.coop_topic = service_output_topic('deconfliction.cooperation',
                                           simulation_id)

    self.abort_topic = service_output_topic('deconfliction.abort',
                                            simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

    # test/debug settings
    # set this to the name of the device for detailed testing, e.g.,
    # 'BatteryUnit.battery1', or None to omit test output
    self.testDeviceName = None
    #self.testDeviceName = 'BatteryUnit.battery1'
    #self.testDeviceName = 'RatioTapChanger.reg4b'

    MethodUtil.sparql_mgr = SPARQLManager(self.gapps, feeder_mrid,
                     simulation_id, logFile=logDir+'deconfliction-pipeline.log')

    self.BatteriesInfo, BatteriesBus=AppUtil.getBatteries(MethodUtil.sparql_mgr)
    #prlog('Intialialization--starting BatteriesInfo: ' + str(self.BatteriesInfo))

    # dictionary of lists for the rolling time interval rules stage
    # deconfliction limiting the number of changes from charging to discharging
    # or vice versa for battery for asset health
    # I could make this another element in self.BatteriesInfo, but for now I'll
    # promote it as a separate top-level data structure
    self.BatteryHistory = {}
    for device in self.BatteriesInfo:
      self.BatteryHistory[device] = []

    self.Regulators = AppUtil.getRegulators(MethodUtil.sparql_mgr)
    #prlog('Initialization--starting Regulators: ' + str(self.Regulators))

    # dictionary of lists for the rolling time interval rules stage
    # deconfliction limiting the total number of steps changed for transformer
    # asset health
    # I could make this another element in self.Regulators, but for now I'll
    # promote it as a separate top-level data structure
    self.RegulatorHistory = {}
    for device in self.Regulators:
      self.RegulatorHistory[device] = []

    # for the app scalability task
    self.SolarPVsInfo, self.SolarPVs =AppUtil.getSolarPVs(MethodUtil.sparql_mgr)

    # deltaT is time between timesteps as fractional hours
    # optimization interval seconds is the number of simulation seconds
    # between triggering an optimization and must be a multiple of 3 for a
    # real-time simulation
    if self.realtimeFlag:
      #optIntervalSec = 3 # optimize every GridLAB-D timestamp
      # 15 seconds is a good number for a real-time simulation
      optIntervalSec = 15
      simLagSec = 0
    else:
      # if attempting non-real-time, something like 1800 is reasonable so
      # apps can complete optimizations safely within that interval
      optIntervalSec = 1800 # 30 minutes
      #optIntervalSec = 3600 # 1 hour
      simLagSec = 600

    if interval!=None and interval!='scalability':
      optIntervalSec = int(interval)

    # deltaT for non-realtime mode simulations needs a factor related to
    # the lag in measurements reflecting DifferenceBuilder messages
    self.deltaT = (optIntervalSec + simLagSec)/3600.0

    self.ConflictMatrix = {}
    self.ResolutionVector = {}
    self.TargetResolutionVector = {}

    # thresholds for concluding cooperation series
    # choose one of these groups of settings depending on  the desired
    # level of cooperation
    # note that driving more cooperation means more time needed, which
    # may impact scalability

    # 1) Drive minimum cooperation:
    #self.coopMessagesThreshold = 5
    #self.conflictPercentThreshold = 0.2
    #self.conflictValueThreshold = 0.20

    # 2) Drive mid-level cooperation:
    #self.coopMessagesThreshold = 10
    #self.conflictPercentThreshold = 0.1
    #self.conflictValueThreshold = 0.10

    # 3) Drive maximum cooperation:
    self.coopMessagesThreshold = 15
    self.conflictPercentThreshold = 0.05
    self.conflictValueThreshold = 0.05

    # multiple cooperation responses are required with a value > 1
    self.coopMinResponses = 2
    #self.coopMinResponses = 1

    # initialize conflict metric
    self.conflictMetric = 0.0
    # initialize combination cooperation timestamp and control flag
    self.coopTimestamp = 0
    self.coopResponseCounter = 0
    self.coopConflictFlag = False
    # initialize running cooperation minimums for conflict metric and matrix
    self.minConflictMetric = 1.0
    self.MinConflictMatrix = {}
    self.AppCoopCount = {}
    # initialize series counter used to uniquely identify cooperation messages
    self.coopCurrentSeries = 0
    self.coopCurrentFlag = False
    self.coopMsgID = 0

    self.simMessageCounter = 0

    # verbose logging control for various deconfliction pipeline aspects
    self.printAllFeasibilityFlag = False
    self.printAllRulesFlag = False
    self.printAllMetricsFlag = False
    self.printAllConflictsResolutionsFlag = False
    self.printAllValidatorFlag = False
    self.printAllDispatchesFlag = False

    # controls whether rules stage deconfliction is done as the first stage
    # using the ConflictMatrix and/or deferred until the last stage before
    # device dispatch using the ResolutionVector
    self.rulesStageFirstFlag = True
    self.rulesStageLastFlag = False
    self.noValidatorRulesFlag = True

    # controls where cooperation stage deconfliction is done
    self.coopStageFlag = True

    # APP SCALABILITY: to streamline the deconfliction workflow when focused
    # on running large numbers of apps rather than on the deconfliction
    # pipeline, it is best to turn off both rules and cooperation stages
    # deconfliction. To automate this the interval value is used to inform
    # the pipeline it is an app scalability test. Commenting out this code
    # block will invoke rules and cooperation stages based on the flag
    # settings above.
    # GDB 9/8/25: enable rules/cooperation for scalability runs
    #scalabilityFlag = interval!=None and interval=='scalability'
    #if scalabilityFlag:
    #  self.rulesStageFirstFlag = False
    #  self.rulesStageLastFlag = False
    #  self.coopStageFlag = False

    self.refCount = 0 # for debug/verification

    # rules settings for short simulations
    self.rulesBattTimeInterval = 60*15 # every 15 minutes
    # number of changes between charging and discharging, and vice versa,
    # allowed in the rolling time interval
    self.rulesBattSwitchesAllowed = 1
    self.rulesRegOuterTimeInterval = 60*10 # every 10 minutes
    self.rulesRegOuterStepsAllowed = 16
    self.rulesRegInnerTimeInterval = 30
    self.rulesRegInnerStepsAllowed = 1

    # assume a non-realtime simulation will be longer and adjust rules
    if not self.realtimeFlag:
      self.rulesBattTimeInterval = 60*60*4 # every 4 hours
      self.rulesBattSwitchesAllowed = 1
      self.rulesRegOuterTimeInterval = 60*60*6 # every 6 hours
      self.rulesRegOuterStepsAllowed = 8

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
          prlog('\nInitalization--applying optimization application weighting '+
                'factors in ' + appname + ': ' + str(self.OptAppWeights))
      except:
        appflag = True

      devname =  weights_base + "-dev.json"
      devflag = False
      try:
        with open(devname) as f:
          data = f.read()
          self.OptDevWeights = json.loads(data)
          prlog('\nInitialization--applying optimization device weighting ' +
                'factors in ' + devname + ': ' + str(self.OptDevWeights))
      except:
        devflag = True

      if appflag and devflag:
        prlog('\nInitialization *** WARNING: Could not find or load either ' +
              'optimization weighting factors files ' + appname +
              ' or ' + devname)

    else:
      prlog('\nInitialization--no file-based optimization weighting factors ' +
            'applied')

    self.pltFlag = True
    if self.pltFlag:
      self.pltFile = open(logDir + 'plot_data.csv', 'w')
      self.cmatFile = open(logDir + 'conflict_matrix.log', 'w')

    self.bypassDeconflictionFlag = False
    #self.bypassDeconflictionFlag = True
    self.instantSetpointUpdateFlag = False

    prlog('\nInitialization--finished, waiting for messages...\n')

    pendingDeconflictFlag = False
    pendingMeasMsgFlag = False

    notDoneFlag = True

    while notDoneFlag:
      while self.messageQueue.empty():
        # GDB 9/2/25: Warning: increasing the sleep duration above 0.1 such as
        # 0.5 can lead to bad things. With two processes sleeping on both ends
        # (apps and deconfliction pipeline) that's 4 sleep statements that are
        # part of processing messages leading to a potential 2 second total
        # delay (with 0.5 sleeps), which is horrible for cooperation messages.
        #sleep(0.1)
        sleep(0.05)

      # GDB 5/21/25: This is an "enhanced queue draining" design. It keeps
      # up with messages by doing the minimal work needed to take in new
      # simulation measurements and app setpoint requests, but then defers
      # initiating deconfliction until the queue is empty with all
      # messages processed. The enhanced aspect is that it also uses a counter
      # for the number of simulation measurement messages to determine whether
      # to initiate deconfliction because it takes potentially multiple
      # simulation measurements for any DifferenceBuilder messages that change
      # device setpoints to be reflected in measurements and performing
      # deconfliction before then could lead to making new requests based on
      # old data.
      while not self.messageQueue.empty():
        app_name, coop_series, timestamp, message = self.messageQueue.get()

        if 'processStatus' in message:
          notDoneFlag = False
          status = message['processStatus']
          if status == 'ABORT':
            prlog('ABORTING due to message delay that will lead to ' +
                  'imminent failure--delay seconds: ' + message['delaySeconds'])
            # tell all running apps to also abort by passing along the message
            self.gapps.send(self.abort_topic, json.dumps(message))

          else:
            prlog('Simulation ' + status + ' message received')

            if not self.simLogSubscribedFlag:
              self.gapps.send(self.abort_topic, json.dumps(message))

          break # done with all processing

        if app_name == None:
          self.ProcessSimulationMessage(message, timestamp,
                                        self.printAllMessagesFlag)

        else:
          meas_msg_flag = coop_series == None
          deconflictFlag = self.ProcessSetpointsMessage(message, timestamp,
                                          app_name, meas_msg_flag, coop_series,
                                          self.printAllConflictsResolutionsFlag)

          if deconflictFlag and not self.bypassDeconflictionFlag:
            # set the flag indicating there is pending deconfliction needed
            # since we need to drain the message queue before initiating that
            pendingDeconflictFlag = True
            if meas_msg_flag:
              pendingMeasMsgFlag = True

      # GDB 4/25/25: skip deconfliction if there haven't been a couple
      # measurement messages from the simulation since the most recent
      # "device dispatch" of new setpoints. This keeps new setpoints from
      # being dispatched before simulation measurements reflect the
      # previously dispatched setpoints.

      # GDB 8/31/25: check of simMessageCounter really messes up
      # processing for non-realtime simulations at least so be very
      # careful with this code.
      #if pendingDeconflictFlag and \
      #   (self.instantSetpointUpdateFlag or self.simMessageCounter>1):
      if pendingDeconflictFlag:
        # GDB 9/3/25: without this check to see if there is a current
        # cooperation series, device dispatches can happen multiple times
        # in quick succession for the same cooperation series
        if pendingMeasMsgFlag or self.coopCurrentFlag:
          self.DeconflictSetpoints(timestamp, pendingMeasMsgFlag,
                                   self.printAllConflictsResolutionsFlag)
        pendingDeconflictFlag = False
        pendingMeasMsgFlag = False

    if self.pltFlag:
      self.pltFile.close()
      self.cmatFile.close()

    # for SHIVA conflict metric
    #json_file = open('log/ConflictMatrix_' + basename + '.json', 'w')
    #json.dump(self.TimeConflictMatrix, json_file, indent=4)
    #json_file.close()
    #json_file = open('log/ResolutionVector_' + basename + '.json', 'w')
    #json.dump(self.TimeResolutionVector, json_file, indent=4)
    #json_file.close()

    messageListener.join()


def _main():
  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("--weights", nargs='?',
                      help="Optimization Weights Base Filename")
  parser.add_argument("--interval", nargs='?', help="Optimization Interval")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-deconfliction-pipeline'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  DeconflictionPipeline(feeder_mrid, opts.simulation_id, opts.weights,
                        opts.interval)

  prlog('Goodbye!')


if __name__ == "__main__":
  _main()

