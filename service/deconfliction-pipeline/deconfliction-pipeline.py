
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
FY23 Prototype: August 14, 2023
FY24 Service: September 19, 2024

@author: Gary Black
"""""

import sys
import os
import argparse
import json
import math
import copy
import queue
import time

from gridappsd import GridAPPSD
from gridappsd import DifferenceBuilder
from gridappsd.topics import simulation_output_topic, simulation_log_topic
from gridappsd.topics import simulation_input_topic, service_output_topic

# magic so all print statements flush without having to add flush=True
import functools
print = functools.partial(print, flush=True)

# for loading the shared modules just below
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
from sparql import SPARQLManager


class DeconflictionPipeline(GridAPPSD):

  def SetpointProcessor(self, app_name, timestamp, set_points,
                        printAllConflictsResolutionsFlag=False):
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

    if printAllConflictsResolutionsFlag:
      print('SetpointProcessor--ConflictMatrix: ' + str(self.ConflictMatrix))

    if self.testDeviceName:
      devid = MethodUtil.NameToDevice[self.testDeviceName]
      if devid in set_points:
        print('~TEST: set-points message with ' + self.testDeviceName +
              ' set-point: ' + str(set_points[devid]) +
              ', app: ' + app_name + ', timestamp: ' + str(timestamp))
        print('~TEST: ConflictMatrix for ' + self.testDeviceName + ': ' +
              str(self.ConflictMatrix[devid]))
      else:
        print('~TEST: set-points message does not contain ' +
              self.testDeviceName)


  def ConflictMetricComputation(self, timestamp, printAllMetricsFlag=False):
    centroid = {}
    apps = {}
    n_devices = len(self.ConflictMatrix)

    # GDB 5/21/24: Don't crash with an empty ConflictMatrix
    if n_devices == 0:
      print('ConflictMetricComputation--conflict metric undefined ' +
            '(no conflicts), timestamp: ' + str(timestamp))
      return 0.0

    for device in self.ConflictMatrix:
      n_apps_device = len(self.ConflictMatrix[device])

      device_setpoints = []

      for app in self.ConflictMatrix[device]:
        gamma_d_a = self.ConflictMatrix[device][app][1]
        if app not in apps:
          apps[app] = {}
        name = MethodUtil.DeviceToName[device]

        if printAllMetricsFlag:
          print('ConflictMetricComputation--device: ' + name + ', app: ' +
                app + ', setpoint: ' + str(gamma_d_a))

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
    print('ConflictMetricComputation--conflict metric: ' +
          str(conflict_metric) + ', timestamp: ' + str(timestamp))

    return conflict_metric


  def CooperationWeightsUpdate(self, timestamp, ConflictMatrix,
                               TargetResolutionVector):
    sigma_d_t = {}
    app_list = []
    minWeight = 1.0

    # find the sigma values for the target resolution vector first because
    # they are needed while iterating over each of the apps later
    for device in TargetResolutionVector:
      gamma_d_t = TargetResolutionVector[device][1]

      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        # Normalize setpoints using max charge and discharge possible
        sigma_d_t[device] = (gamma_d_t + self.Batteries[device]['prated']) / \
                            (2 * self.Batteries[device]['prated'])

      elif name.startswith('RatioTapChanger.'):
        # Normalize setpoints using highStep and lowStep
        sigma_d_t[device] = (gamma_d_t +
                             abs(self.Regulators[device]['highStep'])) / \
                            (self.Regulators[device]['highStep'] + \
                             abs(self.Regulators[device]['lowStep']))

      # while we are iterating over devices, build up a list of apps we
      # need to compute weights for since that's buried down a level within
      # the conflict matrix
      for app in ConflictMatrix[device]:
        if app not in app_list:
          app_list.append(app)

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
            sigma = (gamma_d_a + self.Batteries[device]['prated']) / \
                    (2 * self.Batteries[device]['prated'])
            sigma_d_a[device] = sigma
            centroid[device] = (sigma + sigma_d_t[device]) / 2

          elif name.startswith('RatioTapChanger.'):
            # Normalize setpoints using highStep and lowStep
            sigma = (gamma_d_a + abs(self.Regulators[device]['highStep'])) / \
                    (self.Regulators[device]['highStep'] + \
                     abs(self.Regulators[device]['lowStep']))
            sigma_d_a[device] = sigma
            centroid[device] = (sigma + sigma_d_t[device]) / 2

      # Distance vector:
      sum_dist_a = 0
      sum_dist_t = 0
      for device in centroid:
        sum_dist_a += (centroid[device] - sigma_d_a[device]) ** 2
        sum_dist_t += (centroid[device] - sigma_d_t[device]) ** 2

      dist_centroid_a = math.sqrt(sum_dist_a)
      dist_centroid_t = math.sqrt(sum_dist_t)

      # Compute conflict metric: average distance
      conflict_metric = (dist_centroid_a + dist_centroid_t) / 2
      # Ensuring 0 <= conflict_metric <= 1
      conflict_metric = conflict_metric * 2 / math.sqrt(len(sigma_d_a))

      # The lower the conflict metric, the higher the incentive weight value
      self.OptAppWeights[app] = 1.0 - conflict_metric
      #print('CooperationWeightsUpdate--timestamp: ' + str(timestamp) +
      #   ', app: ' + app + ', initial weight: ' + str(self.OptAppWeights[app]))

      # compute lowest weight over all apps to make adjustments later
      # block comment out from here to end of function to bypass adjustments
      minWeight = min(minWeight, self.OptAppWeights[app])

    # Adjust weights to give even more inventive for better cooperating apps
    weightLoss = 0.75 * minWeight # boost the incentive
    for app in app_list:
      self.OptAppWeights[app] -= weightLoss
      print('CooperationWeightsUpdate--timestamp: ' + str(timestamp) +
            ', app: ' + app + ', adj. weight: ' + str(self.OptAppWeights[app]))


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
      print('CooperationWeightsClear--timestamp: ' + str(timestamp) +
            ', app: ' + app)


  def ConflictIdentification(self, app_name, timestamp, set_points):
    # Determine if there is a new conflict
    for point in set_points:
      device = point['object']

      for app1 in self.ConflictMatrix[device]:
        if app1!=app_name and \
           (point['value']!=self.ConflictMatrix[device][app1][1] or \
            timestamp!=self.ConflictMatrix[device][app1][0]):
          # once a conflict is found, return immediately
          return True

    return False


  def RulesForBatteriesConflict(self, printAllRulesFlag=False):
    rollingTimeInterval = 60
    # number of changes between charging and discharging, and vice versa,
    # allowed in the rolling time interval
    rollingSwitchesAllowed = 1

    # set max/min allowable tap positions based on current position
    for devid in self.Batteries:
      histList = self.BatteryHistory[devid]
      if printAllRulesFlag:
        print('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[devid] + ', BatteryHistory: ' +
              str(histList))

      # iterate backwards through histList counting switches
      rollingSwitchCount = 0
      rollingStartTime = self.Batteries[devid]['timestamp'] - \
                         rollingTimeInterval
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingSwitchCount += 1

      if printAllRulesFlag:
        print('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', rolling charge/discharge switches: ' + str(rollingSwitchCount)+
              ', vs. allowed: ' + str(rollingSwitchesAllowed))

      if rollingSwitchCount >= rollingSwitchesAllowed:
        # save the final P_batt_inv in the history list since we need to make
        # sure not to allow the opposite direction in any setpoint requests
        self.Batteries[devid]['switch_P_batt_inv'] = \
                              self.BatteryHistory[devid][-1][1]
      else:
        self.Batteries[devid]['switch_P_batt_inv'] = None

    # find the maximum P_batt charge and discharge values per battery to
    # prevent overcharging or undercharging
    for devid in self.Batteries:
      chargeSoCMax = 0.9 - self.Batteries[devid]['SoC']
      self.Batteries[devid]['P_batt_charge_max'] = \
                         (chargeSoCMax*self.Batteries[devid]['ratedE']) / \
                         (self.Batteries[devid]['eff_c']*self.deltaT)
      if printAllRulesFlag:
        print('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', max charge SoC contribution: ' + str(chargeSoCMax) +
              ', max charge P_batt: ' +
              str(self.Batteries[devid]['P_batt_charge_max']))

      dischargeSoCMax = 0.2 - self.Batteries[devid]['SoC']
      self.Batteries[devid]['P_batt_discharge_max'] = \
                           (dischargeSoCMax*self.Batteries[devid]['ratedE']) / \
                           (1/self.Batteries[devid]['eff_d']*self.deltaT)
      if printAllRulesFlag:
        print('RulesForBatteriesConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', max discharge SoC contribution: ' + str(dischargeSoCMax) +
              ', max discharge P_batt: ' +
              str(self.Batteries[devid]['P_batt_discharge_max']))

    # iterate over all battery setpoints in ConflictMatrix to make sure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        for app in self.ConflictMatrix[device]:
          # check vs. battery rated power
          if abs(self.ConflictMatrix[device][app][1]) > \
             self.Batteries[device]['prated']:
            print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                  app + ', P_batt setpoint exceeds battery rated power: ' +
                  str(self.ConflictMatrix[device][app][1]))
            if self.ConflictMatrix[device][app][1] > 0:
              self.ConflictMatrix[device][app] = \
                                         (self.ConflictMatrix[device][app][0],
                                          self.Batteries[device]['prated'])
            else:
              self.ConflictMatrix[device][app] = \
                                         (self.ConflictMatrix[device][app][0],
                                          -self.Batteries[device]['prated'])
            print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                  app + ', P_batt setpoint reset to battery rated power: ' +
                  str(self.ConflictMatrix[device][app][1]))

          # check vs. battery SoC limits
          if self.ConflictMatrix[device][app][1] > \
             self.Batteries[device]['P_batt_charge_max']:
            print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                  app + ', P_batt setpoint above max charge P_batt: ' +
                  str(self.ConflictMatrix[device][app][1]))
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0],
                                self.Batteries[device]['P_batt_charge_max'])
            print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                  app + ', P_batt setpoint reset to max charge P_batt: ' +
                  str(self.ConflictMatrix[device][app][1]))

          elif self.ConflictMatrix[device][app][1] < \
             self.Batteries[device]['P_batt_discharge_max']:
            print('RulesForBatteriesConflict--device: ' + name + ', app ' +
                  app + ', P_batt setpoint below max discharge P_batt: ' +
                  str(self.ConflictMatrix[device][app][1]))
            self.ConflictMatrix[device][app]= \
                               (self.ConflictMatrix[device][app][0],
                                self.Batteries[device]['P_batt_discharge_max'])
            print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                  app + ', P_batt setpoint reset to max discharge P_batt: ' +
                  str(self.ConflictMatrix[device][app][1]))

          # check for switching between charge/discharge if over limit
          if self.Batteries[device]['switch_P_batt_inv'] != None:
            prev_P_batt_inv = self.Batteries[device]['switch_P_batt_inv']
            if prev_P_batt_inv>0 and self.ConflictMatrix[device][app][1]<0 or \
               prev_P_batt_inv<0 and self.ConflictMatrix[device][app][1]>0:
              print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                     app + ', P_batt setpoint attempted to change ' +
                     'charge/discharge state: ' +
                     str(self.ConflictMatrix[device][app][1]))
              # this is pretty harsh to force the setpoint request back to zero
              # to avoid a possible change in charge/discharge state, but no
              # other choice when the rule is applied before other stages
              self.ConflictMatrix[device][app]= \
                                  (self.ConflictMatrix[device][app][0], 0.0)
              print('RulesForBatteriesConflict--device: ' + name + ', app: ' +
                    app + ', P_batt setpoint reset to zero')


  def RulesForBatteriesResolution(self, newResolutionVector,
                                  printAllRulesFlag=False):
    rollingTimeInterval = 60
    # number of changes between charging and discharging, and vice versa,
    # allowed in the rolling time interval
    rollingSwitchesAllowed = 1

    # set max/min allowable tap positions based on current position
    for devid in self.Batteries:
      histList = self.BatteryHistory[devid]
      if printAllRulesFlag:
        print('RulesForBatteriesResolution--device: ' +
              MethodUtil.DeviceToName[devid] + ', BatteryHistory: ' +
              str(histList))

      # iterate backwards through histList counting switches
      rollingSwitchCount = 0
      rollingStartTime = self.Batteries[devid]['timestamp'] - \
                         rollingTimeInterval
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingSwitchCount += 1

      if printAllRulesFlag:
        print('RulesForBatteriesResolution--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', rolling charge/discharge switches: ' + str(rollingSwitchCount)+
              ', vs. allowed: ' + str(rollingSwitchesAllowed))

      if rollingSwitchCount >= rollingSwitchesAllowed:
        # save the final P_batt_inv in the history list since we need to make
        # sure not to allow the opposite direction in any setpoint requests
        self.Batteries[devid]['switch_P_batt_inv'] = \
                              self.BatteryHistory[devid][-1][1]
      else:
        self.Batteries[devid]['switch_P_batt_inv'] = None

    # find the maximum P_batt charge and discharge values per battery to
    # prevent overcharging or undercharging
    for devid in self.Batteries:
      chargeSoCMax = 0.9 - self.Batteries[devid]['SoC']
      self.Batteries[devid]['P_batt_charge_max'] = \
                            (chargeSoCMax*self.Batteries[devid]['ratedE']) / \
                            (self.Batteries[devid]['eff_c']*self.deltaT)
      if printAllRulesFlag:
        print('RulesForBatteriesResolution--device: ' +
            MethodUtil.DeviceToName[devid] + ', max charge SoC contribution: ' +
            str(chargeSoCMax) + ', max charge P_batt: ' +
            str(self.Batteries[devid]['P_batt_charge_max']))

      dischargeSoCMax = 0.2 - self.Batteries[devid]['SoC']
      self.Batteries[devid]['P_batt_discharge_max'] = \
                           (dischargeSoCMax*self.Batteries[devid]['ratedE']) / \
                           (1/self.Batteries[devid]['eff_d']*self.deltaT)
      if printAllRulesFlag:
        print('RulesForBatteriesResolution--device: ' +
         MethodUtil.DeviceToName[devid] + ', max discharge SoC contribution: ' +
         str(dischargeSoCMax) + ', max discharge P_batt: ' +
         str(self.Batteries[devid]['P_batt_discharge_max']))

    # iterate over all battery setpoints in newResolutionVector to insure they
    # fall within the acceptable P_batt range and set them to max values if not
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('BatteryUnit.'):
        # check vs. battery rated power
        if abs(newResolutionVector[device][1]) > \
           self.Batteries[device]['prated']:
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint exceeds battery rated power: ' +
                str(newResolutionVector[device][1]))
          if newResolutionVector[device][1] > 0:
            newResolutionVector[device] = (newResolutionVector[device][0],
                                           self.Batteries[device]['prated'])
          else:
            newResolutionVector[device] = (newResolutionVector[device][0],
                                           -self.Batteries[device]['prated'])
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint reset to battery rated power: ' +
                str(newResolutionVector[device][1]))

        # check vs. battery SoC limits
        if newResolutionVector[device][1] > \
           self.Batteries[device]['P_batt_charge_max']:
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint above max charge P_batt: ' +
                str(newResolutionVector[device][1]))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Batteries[device]['P_batt_charge_max'])
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint reset to max charge P_batt: ' +
                str(newResolutionVector[device][1]))

        elif newResolutionVector[device][1] < \
           self.Batteries[device]['P_batt_discharge_max']:
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint below max discharge P_batt: ' +
                str(newResolutionVector[device][1]))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Batteries[device]['P_batt_discharge_max'])
          print('RulesForBatteriesResolution--device: ' + name +
                ', P_batt setpoint reset to max discharge P_batt: ' +
                str(newResolutionVector[device][1]))

        # check for switching between charge/discharge if over limit
        if self.Batteries[device]['switch_P_batt_inv'] != None:
          prev_P_batt_inv = self.Batteries[device]['switch_P_batt_inv']
          if prev_P_batt_inv>0 and newResolutionVector[device][1]<0 or \
             prev_P_batt_inv<0 and newResolutionVector[device][1]>0:
            print('RulesForBatteriesResolution--device: ' + name +
              ', P_batt setpoint attempted to change charge/discharge state: ' +
              str(newResolutionVector[device][1]))
            # this is pretty harsh to force the setpoint request back to zero
            # to avoid a possible change in charge/discharge state, but no
            # other choice when the rule is applied before other stages
            newResolutionVector[device] = \
                                (newResolutionVector[device][0], 0.0)
            print('RulesForBatteriesResolution--device: ' + name +
                  ', P_batt setpoint reset to zero')


  def RulesForRegulatorsConflict(self, printAllRulesFlag=False):
    # comment out to disable per-timestamp limit on tap position changes
    #timestampTapBudget = 3

    rollingTimeInterval = 60
    rollingStepsAllowed = 6 # picked to trigger the rule a reasonable # of times

    # set max/min allowable tap positions based on current position
    for devid in self.Regulators:
      histList = self.RegulatorHistory[devid]
      if printAllRulesFlag:
        print('RulesForRegulatorsConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', RegulatorHistory: ' + str(histList))

      # iterate backwards through histList counting steps changed
      rollingStepCount = 0
      rollingStartTime = self.Regulators[devid]['timestamp'] - \
                         rollingTimeInterval
      previousStep = self.Regulators[devid]['step']
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingStepCount += abs(previousStep - hist[1])
        previousStep = hist[1]

      rollingTapBudget = max(0, rollingStepsAllowed - rollingStepCount)
      if printAllRulesFlag:
        print('RulesForRegulatorsConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', rolling steps: ' + str(rollingStepCount) +
              ', vs. allowed: ' + str(rollingStepsAllowed) +
              ', rolling budget: ' + str(rollingTapBudget))

      # comment out next line and uncoment the following on to disable
      # per-timestamp limit
      #tapBudget = min(timestampTapBudget, rollingTapBudget)
      tapBudget = rollingTapBudget

      #print('RulesForRegulatorsConflict--device: ' +
      #      MethodUtil.DeviceToName[devid] +
      #      ', per-timestamp tap budget: ' + str(timestampTapBudget) +
      #      ', rolling tap budget: ' + str(rollingTapBudget) +
      #      ', final tap budget: ' + str(tapBudget))

      # constrain by the overall tap budget and physical device limits
      self.Regulators[devid]['maxStep'] = min(self.Regulators[devid]['step'] + \
                                              tapBudget, 16)
      self.Regulators[devid]['minStep'] = max(self.Regulators[devid]['step'] - \
                                              tapBudget, -16)
      if printAllRulesFlag:
        print('RulesForRegulatorsConflict--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', min tap pos: ' + str(self.Regulators[devid]['minStep']) +
              ', max tap pos: ' + str(self.Regulators[devid]['maxStep']))

    # iterate over all regulator tap setpoints in ConflictMatrix to make sure
    # they fall within the acceptable maxTapBudget range of the current position
    for device in self.ConflictMatrix:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        for app in self.ConflictMatrix[device]:
          if self.ConflictMatrix[device][app][1] > \
             self.Regulators[device]['maxStep']:
            print('RulesForRegulatorsConflict--device: ' + name + ', app: ' +
                  app + '--tap pos setpoint above max allowable asset ' +
                  'health pos: ' + str(self.ConflictMatrix[device][app][1]))
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0],
                                self.Regulators[device]['maxStep'])
            print('RulesForRegulatorsConflict--device: ' + name + ', app: ' +
                  app + '--tap pos setpoint reset to max allowable asset ' +
                  'health pos: ' + str(self.ConflictMatrix[device][app][1]))

          elif self.ConflictMatrix[device][app][1] < \
               self.Regulators[device]['minStep']:
            print('RulesForRegulatorsConflict--device: ' + name + ', app :' +
                  app + '--tap pos setpoint below min allowable asset ' +
                  'health pos: ' + str(self.ConflictMatrix[device][app][1]))
            self.ConflictMatrix[device][app] = \
                               (self.ConflictMatrix[device][app][0],
                                self.Regulators[device]['minStep'])
            print('RulesForRegulatorsConflict--device: ' + name + ', app: ' +
                  app + '--tap pos setpoint reset to min allowable asset ' +
                  'health pos: ' + str(self.ConflictMatrix[device][app][1]))


  def RulesForRegulatorsResolution(self,newResolutionVector,
                                   printAllRulesFlag=False):
    # comment out to disable per-timestamp limit on tap position changes
    #timestampTapBudget = 3

    rollingTimeInterval = 60
    rollingStepsAllowed = 6 # picked to trigger the rule a reasonable # of times

    # set max/min allowable tap positions based on current position
    for devid in self.Regulators:
      histList = self.RegulatorHistory[devid]
      if printAllRulesFlag:
        print('RulesForRegulatorsResolution--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', RegulatorHistory: ' + str(histList))

      # iterate backwards through histList counting steps changed
      rollingStepCount = 0
      rollingStartTime = self.Regulators[devid]['timestamp'] - \
                         rollingTimeInterval
      previousStep = self.Regulators[devid]['step']
      for hist in reversed(histList):
        if hist[0] < rollingStartTime:
          break
        rollingStepCount += abs(previousStep - hist[1])
        previousStep = hist[1]

      rollingTapBudget = max(0, rollingStepsAllowed - rollingStepCount)
      if printAllRulesFlag:
        print('RulesForRegulatorsResolution--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', rolling steps: ' + str(rollingStepCount) +
              ', vs. allowed: ' + str(rollingStepsAllowed) +
              ', rolling budget: ' + str(rollingTapBudget))

      # comment out next line and uncoment the following on to disable
      # per-timestamp limit
      #tapBudget = min(timestampTapBudget, rollingTapBudget)
      tapBudget = rollingTapBudget

      #print('RulesForRegulatorsResolution--device: ' +
      #      MethodUtil.DeviceToName[devid] +
      #      ', per-timestamp tap budget: ' + str(timestampTapBudget) +
      #      ', rolling tap budget: ' + str(rollingTapBudget) +
      #      ', final tap budget: ' + str(tapBudget))

      # constrain by the overall tap budget and physical device limits
      self.Regulators[devid]['maxStep'] = min(self.Regulators[devid]['step'] + \
                                              tapBudget, 16)
      self.Regulators[devid]['minStep'] = max(self.Regulators[devid]['step'] - \
                                              tapBudget, -16)
      if printAllRulesFlag:
        print('RulesForRegulatorsResolution--device: ' +
              MethodUtil.DeviceToName[devid] +
              ', min tap pos: ' + str(self.Regulators[devid]['minStep']) +
              ', max tap pos: ' + str(self.Regulators[devid]['maxStep']))

    # iterate over all regulator tap setpoints in newResolutionVector to insure
    # they fall within the acceptable maxTapBudget range of the current position
    for device in newResolutionVector:
      name = MethodUtil.DeviceToName[device]
      if name.startswith('RatioTapChanger.'):
        if newResolutionVector[device][1] > \
           self.Regulators[device]['maxStep']:
          print('RulesForRegulatorsResolution--device: ' + name +
                ', tap pos setpoint above max allowable asset health pos: ' +
                str(newResolutionVector[device][1]))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Regulators[device]['maxStep'])
          print('RulesForRegulatorsResolution--device: ' + name +
                ', tap pos setpoint reset to max allowable asset health pos: ' +
                str(newResolutionVector[device][1]))

        elif newResolutionVector[device][1] < \
             self.Regulators[device]['minStep']:
          print('RulesForRegulatorsResolution--device: ' + name +
                ', tap pos setpoint below min allowable asset health pos: ' +
                str(newResolutionVector[device][1]))
          newResolutionVector[device] = \
                             (newResolutionVector[device][0],
                              self.Regulators[device]['minStep'])
          print('RulesForRegulatorsResolution--device: ' + name +
                ', tap pos setpoint reset to min allowable asset health pos: ' +
                str(newResolutionVector[device][1]))


  def OptimizationDeconflict(self, app_name, timestamp, ConflictMatrix):
    ResolutionVector = {}

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
          ResolutionVector[device] = (optTimestamp,
                                      round(optNumerator/optDenominator))
        else:
          ResolutionVector[device] = (optTimestamp, optNumerator/optDenominator)

    return ResolutionVector


  def DeviceDispatcher(self, timestamp, newResolutionVector,
                       printAllDispatchesFlag=False):
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

          switchStr = ''
          if value[1]>0 and self.Batteries[devid]['P_batt_inv']<0:
            switchStr = ' (SWITCH from discharging to charging)'
          elif value[1]<0 and self.Batteries[devid]['P_batt_inv']>0:
            switchStr = ' (SWITCH from charging to discharging)'

          if printAllDispatchesFlag:
            print('DeviceDispatcher--battery device: ' + name +
                  ', timestamp: ' + str(timestamp) + ', new value: ' +
                  str(value[1]) + ', old value: ' +
                  str(self.Batteries[devid]['P_batt_inv']) + switchStr)

          if self.testDeviceName and name==self.testDeviceName:
            print('~TEST: Dispatching to battery id: ' + devid +
                  ', device: ' + name + ', timestamp: ' + str(timestamp) +
                  ', new value: ' + str(value[1]) + ', old value: ' +
                  str(self.Batteries[devid]['P_batt_inv']) + switchStr)

        elif printAllDispatchesFlag:
          print('DeviceDispatcher--NO DISPATCH needed, battery device: ' +
                name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

      elif name.startswith('RatioTapChanger.'):
        # Dispatch regulator tap positions whenever they are different from the
        # current tap position
        if value[1] != self.Regulators[devid]['step']:
          # new value before old value for DifferenceBuilder
          self.difference_builder.add_difference(devid,
                   'TapChanger.step', value[1], self.Regulators[devid]['step'])
          diffCount += 1

          if printAllDispatchesFlag:
            print('DeviceDispatcher--regulator device: ' + name +
                  ', timestamp: ' + str(timestamp) + ', new value: ' +
                  str(value[1]) + ', old value: ' +
                  str(self.Regulators[devid]['step']))

          if self.testDeviceName and name==self.testDeviceName:
              print('~TEST: Dispatching to regulator id: ' + devid +
                    ', device: ' + name + ', timestamp: ' + str(timetstamp) +
                    ', new value: ' + str(value[1]) + ', old value: ' +
                    str(self.Regulators[devid]['step']))

        elif printAllDispatchesFlag:
          print('DeviceDispatcher--NO DISPATCH needed, regulator device: ' +
                name + ', timestamp: ' + str(timestamp) +
                ', same value: ' + str(value[1]))

    # it's also possible a device from the last resolution does not appear
    # in the new resolution.  In this case it's a "don't care" for the new
    # resolution and the device is left at the previous value with nothing sent
    if len(self.ResolutionVector) > len(newResolutionVector):
      for devid in self.ResolutionVector:
        if devid not in newResolutionVector:
          if printAllDispatchesFlag:
            print('DeviceDispatcher--deleted from resolution, device: ' +
                  MethodUtil.DeviceToName[devid])

          if self.testDeviceName and \
             MethodUtil.DeviceToName[devid]==self.testDeviceName:
            print('~TEST: deleted from resolution, id: ' + devid +
                  ', device: ' + MethodUtil.DeviceToName[devid])

    if diffCount > 0:
      dispatch_message = self.difference_builder.get_message()

      if printAllDispatchesFlag:
        print('DeviceDispatcher--sending device dispatch ' +
              'DifferenceBuilder message: ' + json.dumps(dispatch_message))
      else:
        print('DeviceDispatcher--sending device dispatch ' +
              'DifferenceBuilder message!')

      self.gapps.send(self.publish_topic, json.dumps(dispatch_message))
      self.difference_builder.clear()

    return diffCount


  def OnSimMessage(self, header, message):
    #print('OnSimMessage--received message: ' + str(message))
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      status = message['processStatus']
      if status=='COMPLETE' or status=='CLOSED':
        self.keepLoopingFlag = False
        print('OnSimMessage--simulation status message received, status: ' +
              status)

    else:
      self.messageQueue.put((None, None, None, message['message']))


  def pol2cart(self, mag, angle_deg):
        # Convert degrees to radians. GridAPPS-D spits angle in degrees
        angle_rad =  math.radians(angle_deg)
        p = mag * math.cos(angle_rad)
        q = mag * math.sin(angle_rad)
        return p, q


  def ProcessSimulationMessage(self, message, printAllMessagesFlag=False):
    if not printAllMessagesFlag:
      print('ProcessSimulationMessage--timestamp: ' + str(message['timestamp']))

    measurements = message['measurements']
    for devid in self.Batteries:
      measid = self.Batteries[devid]['SoC_measid']
      if measid in measurements:
        self.Batteries[devid]['SoC'] = measurements[measid]['value']/100.0
        MethodUtil.BatterySoC[devid] = self.Batteries[devid]['SoC']
        # comment this out and output it below with P_batt_inv to save space
        #if printAllMessagesFlag:
        #  print('ProcessSimulationMessage--timestamp: ' +
        #        str(message['timestamp']) + ', device: ' +
        #        self.Batteries[devid]['name'] +
        #        ', SoC: ' + str(self.Batteries[devid]['SoC']))

      measid = self.Batteries[devid]['P_batt_measid']
      if measid in measurements:
        # always update timestamp because it's needed for running history rule
        self.Batteries[devid]['timestamp'] = message['timestamp']

        p, q = self.pol2cart(measurements[measid]['magnitude'],
                             measurements[measid]['angle'])
        # negate the p value from the simulation so it is directly comparable
        # to the value that must be given to GridLAB-D in a DifferenceBuilder
        # message
        meas_P_batt_inv = -p

        if 'P_batt_inv' in self.Batteries[devid] and \
            meas_P_batt_inv!=self.Batteries[devid]['P_batt_inv']:
          if printAllMessagesFlag:
            print('ProcessSimulationMessage--BatteryHistory candidate,' +
                  'device: ' + self.Batteries[devid]['name'] +
                  ', old: ' + str(self.Batteries[devid]['P_batt_inv']) +
                  ', new: ' + str(meas_P_batt_inv))
          # check if this is a change from charging to discharging or vice versa
          if (meas_P_batt_inv>0 and self.Batteries[devid]['P_batt_inv']<0) or \
             (meas_P_batt_inv<0 and self.Batteries[devid]['P_batt_inv']>0):
            # append the timestamp, P_batt_inv to the running history
            self.BatteryHistory[devid].append((message['timestamp'],
                                               meas_P_batt_inv))
            print('ProcessSimulationMessage--BatteryHistory match, device: ' +
                  self.Batteries[devid]['name'] +
                  ', history: ' + str(self.BatteryHistory[devid]))

        self.Batteries[devid]['P_batt_inv'] = meas_P_batt_inv
        MethodUtil.BatteryP_batt_inv[devid] = meas_P_batt_inv
        if printAllMessagesFlag:
          print('ProcessSimulationMessage--timestamp: ' +
                str(message['timestamp']) + ', device: ' +
                self.Batteries[devid]['name'] + ', P_batt_inv: ' +
                str(self.Batteries[devid]['P_batt_inv']) + ', SoC: ' +
                str(self.Batteries[devid]['SoC']))

    for devid in self.Regulators:
      measid = self.Regulators[devid]['measid']
      if measid in measurements:
        # always update timestamp because it's needed for running history rule
        self.Regulators[devid]['timestamp'] = message['timestamp']

        # only update the rest if there is a value change
        if measurements[measid]['value'] != self.Regulators[devid]['step']:
          self.Regulators[devid]['step'] = measurements[measid]['value']
          MethodUtil.RegulatorPos[devid] = self.Regulators[devid]['step']
          if printAllMessagesFlag:
            print('ProcessSimulationMessage--timestamp: ' +
                  str(message['timestamp']) + ', device: ' +
                  self.Regulators[devid]['name'] + ', tap position: ' +
                  str(self.Regulators[devid]['step']))

          # append the timestamp, step to the running history
          self.RegulatorHistory[devid].append((message['timestamp'],
                                               self.Regulators[devid]['step']))

    if self.testDeviceName:
      devid = MethodUtil.NametoDevice[self.testDeviceName]
      if devid in self.Batteries:
        print('~TEST simulation updated SoC for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', SoC: ' + str(self.Batteries[devid]['SoC']))
        print('~TEST simulation updated P_batt_inv for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', P_batt_inv: ' + str(self.Batteries[devid]['P_batt_inv']))
      elif devid in self.Regulators:
        print('~TEST simulation updated tap position for device name: ' +
              self.testDeviceName + ', timestamp: ' + str(message['timestamp'])+
              ', pos: ' + str(self.Regulators[devid]['step']))


  def getAppName(self, header):
    app_info = header['destination']

    if app_info.startswith('/topic/goss.gridappsd.simulation.'):
      endind = app_info[33:].find('.')
      app_info = app_info[33:33+endind]

    return app_info.split(':')[0]


  def OnMeasSetpointsMessage(self, header, message):
    if self.printAllMessagesFlag:
      print('OnMeasSetpointsMessage--received message: ' + str(message))
      print('OnMeasSetpointsMessage--received header: ' + str(header))

    app_name = self.getAppName(header)
    self.messageQueue.put((app_name, True, None, message['input']['message']))


  def OnCoopSetpointsMessage(self, header, message):
    if self.printAllMessagesFlag:
      print('OnCoopSetpointsMessage--received message: ' + str(message))
      print('OnCoopSetpointsMessage--received header: ' + str(header))

    app_name = self.getAppName(header)
    self.messageQueue.put((app_name, False, message['cooperationIdentifier'],
                           message['input']['message']))


  def ProcessSetpointsMessage(self, message, app_name, meas_msg_flag, coop_id,
                              printAllConflictsResolutionsFlag):
    timestamp = message['timestamp']

    print('\nProcessSetpointsMessage--start processing, timestamp: ' +
          str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
          str(meas_msg_flag) + ', coop_id: ' + str(coop_id))

    if not meas_msg_flag and coop_id!=self.coopIdentifier:
      # discard any cooperation messages when not currently cooperating or
      # when from a previous cooperation phase
      print('ProcessSetpointsMessage--immediate discard of nonmatching coop ' +
            'message, coop_id: ' + coop_id + ', current coop_id: ' +
            str(self.coopIdentifier))
      print('ProcessSetpointsMessage--finished processing, timestamp: ' +
            str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
            str(meas_msg_flag) + ', coop_id: ' + str(coop_id))
      return

    if meas_msg_flag and self.coopIdentifier!=None:
      if self.coopTimestamp == timestamp:
        print('ProcessSetpointsMessage--special case skipping device ' +
              'dispatch for meas message with running cooperation initiated ' +
              'for same timestamp: ' + str(timestamp))

      else:
        # checking for coopTimestamp!=timestamp fixes a special case where we've
        # already ended the last phase of cooperation but then more meas
        # messages arrive and we don't want to immediately do further dispatches
        print('ProcessSetpointsMessage--conclude running cooperation phase ' +
              'with new meas message received, coopTimestamp: ' +
              str(self.coopTimestamp))

        # we were cooperating when a measurement message arrived so need to
        # conclude that cooperation before processing the new message

        # OPTIMIZATION stage deconfliction
        print('ProcessSetpointsMessage--applying OPTIMIZATION stage ' +
              'deconfliction for running cooperation')
        newResolutionVector = self.OptimizationDeconflict(app_name, timestamp,
                                                          self.ConflictMatrix)

        # Step 4: Setpoint Validator -- not implemented yet
        # Step 5: Device Dispatcher
        dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                              self.printAllDispatchesFlag)
        print('ProcessSetpointsMessage--invoked device dispatch for ' +
              'running cooperation, # devices dispatched: ' +str(dispatchCount))

        # update the current resolution to the new resolution to be ready for
        # the next dispatch
        self.ResolutionVector.clear()
        self.ResolutionVector = newResolutionVector

    # copy conflict matrix before updating since I need this if cooperation
    # iteration results in an increased conflict metric
    previousConflictMatrix = copy.deepcopy(self.ConflictMatrix)

    # set_points are the forward_differences part of the DifferenceBuilder
    # message with keys of object, attribute, and value
    set_points = message['forward_differences']

    # Step 1: Setpoint Processor
    print('ProcessSetpointsMessage--invoking setpoint processor')
    self.SetpointProcessor(app_name, timestamp, set_points,
                           printAllConflictsResolutionsFlag)

    # Step 2: Feasibility Maintainer -- not implemented yet

    # RULES & HEURISTICS stage deconfliction done first
    if self.rulesStageFirstFlag:
      print('ProcessSetpointsMessage--applying initial RULES & HEURISTICS ' +
            'stage deconfliction')
      self.RulesForBatteriesConflict(self.printAllRulesFlag)
      self.RulesForRegulatorsConflict(self.printAllRulesFlag)

    # Step 3: Deconflictor
    # Step 3.1: Conflict Identification
    print('ProcessSetpointsMessage--invoking conflict identification')
    conflictFlag = self.ConflictIdentification(app_name, timestamp, set_points)

    if not conflictFlag:
      print('ProcessSetpointsMessage--NO conflict found in conflict matrix')
      # zero conflict metric since by definition there is none
      self.conflictMetric = 0.0

      # start with a copy of the previous resolution
      newResolutionVector = copy.deepcopy(self.ResolutionVector)

      # copy the new set-points over top of the previous resolution
      for point in set_points:
        newResolutionVector[point['object']] = (timestamp, point['value'])

      # RULES & HEURISTICS stage deconfliction done last
      if not self.rulesStageFirstFlag:
        print('ProcessSetpointsMessage--applying final RULES & HEURISTICS ' +
              'stage deconfliction')
        self.RulesForBatteriesResolution(newResolutionVector,
                                         self.printAllRulesFlag)
        self.RulesForRegulatorsResolution(newResolutionVector,
                                          self.printAllRulesFlag)

      if printAllConflictsResolutionsFlag:
        print('ProcessSetpointsMessage--ResolutionVector (no conflict): ' +
              str(newResolutionVector))

      if self.testDeviceName:
        devid = MethodUtil.NameToDevice[self.testDeviceName]
        if devid in newResolutionVector:
          print('~TEST: ResolutionVector (no conflict) for ' +
                self.testDeviceName + ' setpoint: ' +
                str(newResolutionVector[devid][1]) +
                ', timestamp: ' +
                str(newResolutionVector[devid][0]))
        else:
          print('~TEST: ResolutionVector (no conflict) does not contain ' +
                self.testDeviceName)

      # Step 4: Setpoint Validator -- not implemented yet
      # Step 5: Device Dispatcher
      dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                            self.printAllDispatchesFlag)
      print('ProcessSetpointsMessage--invoked device dispatch, # devices ' +
            'dispatched: ' +str(dispatchCount))

      # update the current resolution to the new resolution to be ready for the
      # next dispatch
      self.ResolutionVector.clear()
      self.ResolutionVector = newResolutionVector

      # zero the cooperation timestamp to indicate no active cooperation
      self.coopTimestamp = 0
      self.coopIdentifier = None
      print('ProcessSetpointsMessage--finished processing, timestamp: ' +
            str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
            str(meas_msg_flag) + ', coop_id: ' + str(coop_id))
      return

    # conflict identified logic
    print('ProcessSetpointsMessage--YES conflict found in conflict matrix')
    if meas_msg_flag:
      # COOPERATION stage deconfliction
      print('ProcessSetpointsMessage--applying COOPERATION stage ' +
            'deconfliction for meas message')

      # compute conflict metric for later comparison during later cooperation
      self.conflictMetric = self.ConflictMetricComputation(timestamp,
                                                       self.printAllMetricsFlag)

      # clear incentive weights before kicking off cooperation phase because
      # we always start from scratch
      self.CooperationWeightsClear(timestamp, self.ConflictMatrix)

      # start with a "target" resolution vector using the optimization code
      # that computes a centroid/target per device
      self.TargetResolutionVector = self.OptimizationDeconflict(app_name,
                                                 timestamp, self.ConflictMatrix)

      # publish this target resolution vector to the cooperation topic for
      # competing apps that support cooperation to respond to
      self.coopCounter += 1
      self.coopIdentifier = 'COOP-' + str(self.coopCounter)
      coopMessage = {'cooperationIdentifier': self.coopIdentifier,
                     'targetResolutionVector': self.TargetResolutionVector}
      self.gapps.send(self.coop_topic, json.dumps(coopMessage))
      print('ProcessSetpointsMessage--kicked off new cooperation phase, ' +
            'updated coopIdentifier: ' + self.coopIdentifier)

      # set the cooperation timestamp to indicate when cooperation was initiated
      self.coopTimestamp = timestamp
      print('ProcessSetpointsMessage--finished processing, timestamp: ' +
            str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
            str(meas_msg_flag) + ', coop_id: ' + str(coop_id))
      return

    # coop message with conflict to get here
    print('ProcessSetpointsMessage--conflict found with with coop message, ' +
          'checking thresholds')

    # save the previous conflict metric for comparison
    prevConflictMetric = self.conflictMetric

    self.conflictMetric = self.ConflictMetricComputation(timestamp,
                                                       self.printAllMetricsFlag)

    perConflictDelta = 100.0 # for no previous conflict metric value
    if prevConflictMetric > 0.0:
      perConflictDelta = 100.0 * (prevConflictMetric - self.conflictMetric)/ \
                                 prevConflictMetric

    print('ProcessSetpointsMessage--thresholds, previous conflict metric: ' +
          str(prevConflictMetric) + ', new conflict metric: ' +
          str(self.conflictMetric) + ', % change: ' + str(perConflictDelta))

    # thresholds for ending cooperation are either a conflict metric value
    # below 0.2 or a % conflict change less than 2%
    # check for NOT meeting thresholds
    if self.conflictMetric>0.2 and perConflictDelta>2.0:
      # initiate further cooperation
      print('ProcessSetpointsMessage--NO thresholds met, initiating ' +
            'further cooperation')

      # update incentive weights for every cooperation iteration
      self.CooperationWeightsUpdate(timestamp, self.ConflictMatrix,
                                    self.TargetResolutionVector)

      # start with a "target" resolution vector using the optimization code
      # that computes a weighted centroid per device
      newTargetResolutionVector = self.OptimizationDeconflict(app_name,
                                                 timestamp, self.ConflictMatrix)

      # publish this target resolution vector to the cooperation topic for
      # competing apps that support cooperation to respond to
      coopMessage = {'cooperationIdentifier': self.coopIdentifier,
                     'targetResolutionVector': newTargetResolutionVector}
      self.gapps.send(self.coop_topic, json.dumps(coopMessage))
      print('ProcessSetpointsMessage--finished processing, timestamp: ' +
            str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
            str(meas_msg_flag) + ', coop_id: ' + str(coop_id))
      return

    # thresholds for ending cooperation have been met to get here
    print('ProcessSetpointsMessage---YES thresholds met, concluding ' +
          'cooperation')

    # OPTIMIZATION stage deconfliction
    if perConflictDelta < 0.0:
      print('ProcessSetpointsMessage--applying OPTIMIZATION stage ' +
            'deconfliction to previous conflict matrix due to increased ' +
            'conflict metric')
      # update incentive weights using previous conflict metric before final
      # optimization stage and device dispatch
      self.CooperationWeightsUpdate(timestamp, previousConflictMatrix,
                                    self.TargetResolutionVector)

      newResolutionVector = self.OptimizationDeconflict(app_name, timestamp,
                                                        previousConflictMatrix)
    else:
      print('ProcessSetpointsMessage--applying OPTIMIZATION stage ' +
            'deconfliction')
      # update incentive weights using current conflict metric before final
      # optimization stage and device dispatch
      self.CooperationWeightsUpdate(timestamp, self.ConflictMatrix,
                                    self.TargetResolutionVector)

      newResolutionVector = self.OptimizationDeconflict(app_name, timestamp,
                                                        self.ConflictMatrix)

    # RULES & HEURISTICS stage deconfliction done last
    if not self.rulesStageFirstFlag:
      print('ProcessSetpointsMessage--applying final RULES & HEURISTICS ' +
            'stage deconfliction')
      self.RulesForBatteriesResolution(newResolutionVector,
                                       self.printAllRulesFlag)
      self.RulesForRegulatorsResolution(newResolutionVector,
                                        self.printAllRulesFlag)

    # Step 4: Setpoint Validator -- not implemented yet
    # Step 5: Device Dispatcher
    dispatchCount = self.DeviceDispatcher(timestamp, newResolutionVector,
                                          self.printAllDispatchesFlag)
    print('ProcessSetpointsMessage--invoked device dispatch, # devices ' +
          'dispatched: ' +str(dispatchCount))

    # update the current resolution to the new resolution to be ready for the
    # next dispatch
    self.ResolutionVector.clear()
    self.ResolutionVector = newResolutionVector

    # zero the cooperation timestamp to indicate no active cooperation
    self.coopTimestamp = 0
    self.coopIdentifier = None
    print('ProcessSetpointsMessage--finished processing, timestamp: ' +
          str(timestamp) + ', app: ' + app_name + ', meas_msg_flag: ' +
          str(meas_msg_flag) + ', coop_id: ' + str(coop_id))


  def __init__(self, gapps, feeder_mrid, simulation_id, weights_base, interval):
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
      set_id[app+':meas'] = gapps.subscribe(service_output_topic(app + ':meas',
                                 simulation_id), self.OnMeasSetpointsMessage)
      set_id[app+':coop'] = gapps.subscribe(service_output_topic(app + ':coop',
                                 simulation_id), self.OnCoopSetpointsMessage)

    # subscribe to simulation log and output messages
    self.keepLoopingFlag = True
    out_id = gapps.subscribe(simulation_output_topic(simulation_id),
                             self.OnSimMessage)
    log_id = gapps.subscribe(simulation_log_topic(simulation_id),
                             self.OnSimMessage)

    # simulation topic for sending DifferenceBuilder messages
    self.publish_topic = simulation_input_topic(simulation_id)

    # service topic for sending target resolution messages to cooperating apps
    self.coop_topic = service_output_topic('gridappsd-deconflictor-app',
                                           simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

    # test/debug settings
    # set this to the name of the device for detailed testing, e.g.,
    # 'BatteryUnit.battery1', or None to omit test output
    self.testDeviceName = None
    #self.testDeviceName = 'BatteryUnit.battery1'

    MethodUtil.sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = AppUtil.getBatteries(MethodUtil.sparql_mgr)
    #print('Intialialization--starting Batteries: ' + str(self.Batteries))

    # dictionary of lists for the rolling time interval rules stage
    # deconfliction limiting the number of changes from charging to discharging
    # or vice versa for battery for asset health
    # I could make this another element in self.Batteries, but for now I'll
    # promote it as a separate top-level data structure
    self.BatteryHistory = {}
    for devid in self.Batteries:
      self.BatteryHistory[devid] = []

    self.Regulators = AppUtil.getRegulators(MethodUtil.sparql_mgr)
    #print('Initialization--starting Regulators: ' + str(self.Regulators))

    # dictionary of lists for the rolling time interval rules stage
    # deconfliction limiting the total number of steps changed for transformer
    # asset health
    # I could make this another element in self.Regulators, but for now I'll
    # promote it as a separate top-level data structure
    self.RegulatorHistory = {}
    for devid in self.Regulators:
      self.RegulatorHistory[devid] = []

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

    self.ConflictMatrix = {}
    self.ResolutionVector = {}
    self.TargetResolutionVector = {}

    # initialize conflict metric
    self.conflictMetric = 0.0
    # initialize combination cooperation timestamp and control flag
    self.coopTimestamp = 0
    # initialize counter used to uniquely identify cooperation messages
    self.coopCounter = 0
    self.coopIdentifier = None

    # verbose logging control for various deconfliction pipeline aspects
    self.printAllMessagesFlag = False
    self.printAllRulesFlag = False
    self.printAllDispatchesFlag = False
    self.printAllMetricsFlag = False
    self.printAllConflictsResolutionsFlag = False

    # controls whether rules stage deconfliction is done as the first stage
    # using the ConflictMatrix or deferred until the last stage before device
    # dispatch using the ResolutionVector
    self.rulesStageFirstFlag = True

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
          print('\nInitalization--applying optimization application weighting '+
                'factors in ' + appname + ': ' + str(self.OptAppWeights))
      except:
        appflag = True

      devname =  weights_base + "-dev.json"
      devflag = False
      try:
        with open(devname) as f:
          data = f.read()
          self.OptDevWeights = json.loads(data)
          print('\nInitialization--applying optimization device weighting ' +
                'factors in ' + devname + ': ' + str(self.OptDevWeights))
      except:
        devflag = True

      if appflag and devflag:
        print('\nInitialization *** WARNING: Could not find or load either ' +
              'optimization weighting factors files ' + appname +
              ' or ' + devname)

    else:
      print('\nInitialization--no file-based optimization weighting factors ' +
            'applied')

    print('\nInitialization--finished, waiting for messages...\n')

    while self.keepLoopingFlag:
      if self.messageQueue.qsize() == 0:
        time.sleep(0.1)
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
        app_name, meas_msg_flag, coop_id, message = self.messageQueue.get()
        if app_name != None:
          break

      if app_name == None:
        self.ProcessSimulationMessage(message, self.printAllMessagesFlag)

      else:
        self.ProcessSetpointsMessage(message, app_name, meas_msg_flag, coop_id,
                                     self.printAllConflictsResolutionsFlag)

    for id in set_id:
      gapps.unsubscribe(set_id[id])
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
  print('Starting deconfliction pipeline...')

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

  gapps = GridAPPSD(opts.simulation_id)
  assert gapps.connected

  DeconflictionPipeline(gapps, feeder_mrid, opts.simulation_id,
                        opts.weights, opts.interval)

  print('Goodbye!')


if __name__ == "__main__":
  _main()

