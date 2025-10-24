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
import copy
from time import sleep

# os.environ['MOSEKLM_LICENSE_FILE'] = '/home/mukh915/mosek/mosek.lic'
# import mosek  # Import mosek to ensure it's available

# GDB 8/27/25: Magic that puts message handling into its own process
# as the only way to keep up with simulation measurements when there
# are long-running optimizations
from multiprocessing import Process, Queue, Array

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

  def msglog(self, msg):
    logname = 'log/' + self.app_name + '-messages.log'
    try:
      with open(logname, 'a') as flog:
        flog.write(str(datetime.now()) + ': ' + msg + '\n')
    except:
      pass

  # start of message listener process methods

  # GDB 9/5/25 NOTE: I have been on the struggle bus for days regarding
  # cooperation message responses not being properly synchronized with
  # requests leading to cooperation being cutoff by new measurement setpoints.
  # This is why I put cooperation into a separate process than the main
  # process that does optimizations. But, it still was happening and my most
  # recent attempt to eliminate it was making sure there are no print
  # calls from either the message listener process or the coooperation handler
  # process just to make sure these prints weren't blocking and holding up
  # cooperation replies. These prints have double pound signs before them
  # instead of single pound signs for regular comments.

  def messageListenerProcess(self, simulation_id):
    # authenticate with GridAPPS-D Platform
    self.msg_gapps = GridAPPSD(simulation_id)
    assert self.msg_gapps.connected

    out_id = self.msg_gapps.subscribe(simulation_output_topic(simulation_id),
                                      self.OnSimOutputMessage)

    if self.simLogSubscribedFlag:
      log_id = self.msg_gapps.subscribe(simulation_log_topic(simulation_id),
                                        self.OnSimLogMessage)

    coop_id = self.msg_gapps.subscribe(service_output_topic(
                                       'deconfliction.cooperation',
                                       simulation_id), self.OnCoopMessage)
    abort_id = self.msg_gapps.subscribe(service_output_topic(
                                        'deconfliction.abort',
                                        simulation_id), self.OnAbortMessage)

    self.keepLoopingFlag = True

    while self.keepLoopingFlag:
      # GDB 9/2/25: Warning: increasing the sleep duration above 0.1 such as
      # 0.5 can lead to bad things. With two processes sleeping on both ends
      # (apps and deconfliction pipeline) that's 4 sleep statements that are
      # part of processing messages leading to a potential 2 second total
      # delay (with 0.5 sleeps), which is horrible for cooperation messages.
      #sleep(0.1)
      sleep(0.05)

    self.msg_gapps.unsubscribe(out_id)
    if self.simLogSubscribedFlag:
      self.msg_gapps.unsubscribe(log_id)
    self.msg_gapps.unsubscribe(coop_id)
    self.msg_gapps.unsubscribe(abort_id)


  def clearSimQueue(self):
    try:
      while True:
        self.simQueue.get(block=False)
    #except Queue.Empty:
    except:
      pass


  def clearCoopQueue(self):
    try:
      while True:
        self.coopQueue.get(block=False)
    #except Queue.Empty:
    except:
      pass


  def OnSimOutputMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    # if it's been optItervalSec since last optimization:
    ts_unix = int(message['message']['timestamp'])

    if self.realtimeFlag:
      # If doing real-time simulation must subtract 5 off timestamp to make it
      # evenly divisble by multiples of the 3 second GridLAB-D time interval
      if (ts_unix-5) % self.optIntervalSec == 0:
        if self.logMessagesFlag:
          self.msglog('received simulation measurements to queue at timestamp:' + str(ts_unix) + ', wall time: ' + str(datetime.utcfromtimestamp(ts_unix).time()))

        # only permit a single message at a time to be queued to not fall behind
        self.clearSimQueue()
        self.simQueue.put(message['message'])

    else:
      # If doing non-real-time simulation remove the 5 second offset because
      # GridLAB-D outputs at even 60 second intervals
      if ts_unix % self.optIntervalSec == 0:
        if self.logMessagesFlag:
          self.msglog('received simulation measurements to queue at timestamp:' + str(ts_unix) + ', wall time: ' + str(datetime.utcfromtimestamp(ts_unix).time()))

        # only permit a single message at a time to be queued to not fall behind
        self.clearSimQueue()
        self.simQueue.put(message['message'])


  def OnSimLogMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    status = message['processStatus']
    if status=='COMPLETE' or status=='CLOSED':
      self.keepLoopingFlag = False
      # both simulation and cooperation queues need this message

      self.clearSimQueue()
      self.simQueue.put(message)

      self.clearCoopQueue()
      self.coopQueue.put(message)


  def OnCoopMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    # GDB 9/29/25: Empty the queue before putting on the new message
    # to insure the app is not responding to a stale cooperation request
    if self.logMessagesFlag:
      time_sent = datetime.strptime(message['time_sent'],'%Y-%m-%d %H:%M:%S.%f')
      diff_sec = (datetime.now() - time_sent).total_seconds()

      self.msglog('received cooperation request|msgid:' +
                  str(message['coop_msgid']) + '|series:' +
                  str(message['coop_series']) + '|delay:' + str(diff_sec))

    # only permit a single message at a time to be queued to not fall behind
    self.clearCoopQueue()
    self.coopQueue.put(message)


  def OnAbortMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      if message['processStatus']=='COMPLETE' or \
         message['processStatus']=='ABORT':
        self.keepLoopingFlag = False
        self.simQueue.put(message)
        self.coopQueue.put(message)

  # end of message listener process methods

  # start of cooperation handler process methods

  def cooperationHandlerProcess(self, simulation_id):
    # authenticate with GridAPPS-D Platform
    self.coop_gapps = GridAPPSD(simulation_id)
    assert self.coop_gapps.connected

    # GDB 9/3/25: coopSeries keeps track of what cooperation series is the
    # one currently being processed in order to determine when to discard
    # "stale" cooperation messages associated with an earlier series
    self.coopSeries = 0
    self.coopMsgID = 0

    # coopCounter allows diminishing cooperation with each succeeding
    # cooperation message solicitation within a series
    self.coopCounter = 0

    while True:
      while self.coopQueue.empty():
        # GDB 9/2/25: Warning: increasing the sleep duration above 0.1 such as
        # 0.5 can lead to bad things. With two processes sleeping on both ends
        # (apps and deconfliction pipeline) that's 4 sleep statements that are
        # part of processing messages leading to a potential 2 second total
        # delay (with 0.5 sleeps), which is horrible for cooperation messages.
        #sleep(0.1)
        sleep(0.05)

      lastCoopMessage = None

      #print('Cooperation queue check start', flush=True)
      while not self.coopQueue.empty():
        message = self.coopQueue.get()

        if 'processStatus' in message: # simulation log message
          status = message['processStatus']
          ##if status == 'ABORT':
          ##  print('ABORTING due to message delay that will lead to ' +
          ##        'imminent failure--delay seconds: ' + message['delaySeconds'])
          ##else:
          ##  print('Simulation ' + status + ' message received', flush=True)

          return # done with all processing

        else:
          ##print('Cooperation message on queue with series: ' +
          ##      str(message['coop_series']), flush=True)
          lastCoopMessage = message
      ##print('Cooperation queue check finish', flush=True)

      if lastCoopMessage != None:
        if self.includeBatteriesFlag or self.includeRegulatorsFlag or \
           self.includeSolarPVsPFlag:
          checkSeries = lastCoopMessage['coop_series']
          self.coopMsgID = lastCoopMessage['coop_msgid']

          if checkSeries >= self.coopSeries:
            if checkSeries == self.coopSeries:
              # comment out incrementing coopCounter to not diminish
              # cooperation for each new solicitation during a series
              self.coopCounter += 1
            else:
              self.coopSeries = checkSeries
              self.coopCounter = 0

            ##print('Processing Cooperation message with series: ' +
            ##      str(checkSeries), flush=True)
            # respond to cooperation message
            self.processCoopMessage(lastCoopMessage)

          else:
            ##print('Discarding Cooperation message with stale series: ' +
            ##      str(checkSeries), flush=True)
            pass


  def processCoopMessage(self, message):
    # choose the desired level of app cooperation by uncommenting one of
    # the coopLevel settings
    #coopLevel = 4 # high cooperation
    coopLevel = 3 # medium-high cooperation
    #coopLevel = 2 # medium-low cooperation
    #coopLevel = 1 # low cooperation

    coopRatioDenom = 2.0 # for coopLevel 3
    if coopLevel == 1:
      coopRatioDenom = 1.0
    elif coopLevel == 2:
      coopRatioDenom = 1.5

    # message consists of a proposed dictionary with device mrid keys and
    # proposed set-point values
    coopProposed = message['coop_proposed']

    # except for SolarPVs the set-point values are tuples and they are
    # easier to work with as complex numbers so do that translation now
    coopTime = 0
    for mrid, value in coopProposed.items():
      coopTime = max(coopTime, value[0])
      # I create tuples for the complex SolarPV setpoints for serialization,
      # but JSON serializes those as lists so the reverse deserialization
      # needs to check for lists rather than tuples
      if isinstance(value[1], list):
        coopProposed[mrid] = (value[0], complex(value[1][0], value[1][1]))

    #for mrid in coopProposed:
    #  print('DECONFLICTOR COOPERATE mrid ' + mrid + ' proposed set-point: ' + str(coopProposed[mrid]), flush=True)

    if self.includeBatteriesFlag:
      # GDB 9/10/25: initialize proposed to None because there may be
      # missing devices in the proposed setpoints
      len_BatteriesInfo = len(self.BatteriesInfo)
      self.p_batt_proposed = [None] * len_BatteriesInfo

      for mrid in self.BatteriesInfo:
        if mrid in coopProposed:
          idx = self.BatteriesInfo[mrid]['idx']
          self.p_batt_proposed[idx] = -coopProposed[mrid][1]

    if self.includeRegulatorsFlag:
      # GDB 9/10/25: initialize proposed to None because there may be
      # missing devices in the proposed setpoints
      len_RegulatorsInfo = len(self.RegulatorsInfo)
      self.reg_proposed = [None] * len_RegulatorsInfo

      for reg in self.RegulatorsInfo:
        if reg in coopProposed:
          idx = self.RegulatorsInfo[reg]['idx']
          self.reg_proposed[idx] = coopProposed[reg][1]

    if self.includeSolarPVsPFlag:
      # GDB 9/10/25: initialize proposed to None because there may be
      # missing devices in the proposed setpoints
      len_SolarPVsInfo = len(self.SolarPVsInfo)
      self.pq_pv_proposed = [None] * len_SolarPVsInfo

      for mrid in self.SolarPVs:
        if mrid in coopProposed:
          idx = self.SolarPVs[mrid]['idx']
          self.pq_pv_proposed[idx] = -coopProposed[mrid][1]

    # Need to define the full optimization problem each time anything
    # changes for CVXPY to be happy
    # GDB 9/9/24: Can't do a new optimization for cooperation because
    # the objective function is non-linear/non-convex so we have an
    # alternative workflow implementation for supporting cooperation in
    # order to meet the FY24 deconfliction service deliverable

    print('BEFORE COOPERATION RESPONSE OPTIMIZATION', flush=True)
    self.optPerform(datetime.utcfromtimestamp(coopTime), cooperationFlag=True)
    #sleep(5)
    print('AFTER COOPERATION RESPONSE OPTIMIZATION', flush=True)

    #print('DECONFLICTOR COOPERATE p_batt_greedy: ' + str(self.p_batt_greedy), flush=True)
    #print('DECONFLICTOR COOPERATE p_batt_proposed: ' + str(self.p_batt_proposed), flush=True)

    # GDB 9/10/24: Here is the alternative support for cooperation via
    # ranking the differences between proposed and greedy setpoints:
    if self.includeBatteriesFlag:
      # first, create a list of differences
      p_batt_diff = [None] * len_BatteriesInfo
      for i in range(len_BatteriesInfo):
        if self.p_batt_proposed[i] != None:
          p_batt_diff[i] = abs(self.p_batt_greedy[i] - self.p_batt_proposed[i])

      #print('DECONFLICTOR COOPERATE p_batt_diff: ' + str(p_batt_diff), flush=True)

      # omit any setpoints where proposed == greeedy
      p_batt_sort = []
      for i in range(len_BatteriesInfo):
        if p_batt_diff[i]!=None and p_batt_diff[i]>0:
          p_batt_sort.append(p_batt_diff[i])

      # sorts in place
      p_batt_sort.sort()

      # GDB 3/25/25: Handle the case of only proposed == greedy
      diffMax = 0
      if len(p_batt_sort) > 0:
        coopCount = max(1, -(len(p_batt_sort)//-2)) # integer "ceiling" division

        # find the value associated with the last "cooperating" battery
        diffMax = p_batt_sort[coopCount-1]

        #print('DECONFLICTOR COOPERATE batteries coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
      #else:
        #print('DECONFLICTOR COOPERATE batteries coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

      if coopLevel == 4:
        for i in range(len_BatteriesInfo):
          # check if this is a "cooperating" battery
          if p_batt_diff[i]!=None and p_batt_diff[i]>0 and \
             p_batt_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            self.p_batt_greedy[i] = self.p_batt_proposed[i]

        #print('DECONFLICTOR COOPERATE p_batt_coop: ' + str(self.p_batt_greedy), flush=True)

      else:
        p_batt_denom = [] # just for diagnostic logging
        for i in range(len_BatteriesInfo):
          # check if this is a "cooperating" battery
          if p_batt_diff[i]!=None and p_batt_diff[i]>0 and \
             p_batt_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            #self.p_batt_greedy[i] = self.p_batt_proposed[i]
            # adjust cooperation level based on difference
            # find which entry this p_batt_diff is within p_batt_sort to
            # determine how much to cooperate. This is tricky code in that
            # a loop iterator varible is referenced after the loop.
            for ic in range(len(p_batt_sort)):
              if p_batt_diff[i] == p_batt_sort[ic]:
                break
            fcoop = float(ic/coopRatioDenom) + 1.0

            ratio = (self.p_batt_proposed[i] - self.p_batt_greedy[i])/ \
                    float(fcoop + self.coopCounter)
            self.p_batt_greedy[i] += ratio
            p_batt_denom.append((fcoop, self.coopCounter))
          else:
            p_batt_denom.append(None)

        #print('DECONFLICTOR COOPERATE p_batt_coop: ' + str(self.p_batt_greedy), flush=True)
        #print('DECONFLICTOR COOPERATE p_batt_denom: ' + str(p_batt_denom), flush=True)

      for mrid in self.BatteriesInfo:
        idx = self.BatteriesInfo[mrid]['idx']
        # new value before old value for DifferenceBuilder
        # note the p_batt value is negated for the GridLAB-D
        # DifferenceBuilder message
        if self.p_batt_proposed[idx] != None:
          self.difference_builder.add_difference(mrid,
               'PowerElectronicsConnection.p', -self.p_batt_greedy[idx], None)

    if self.includeSolarPVsPFlag:
      #print('DECONFLICTOR COOPERATE p_pv_greedy: ' + str(self.p_pv_greedy), flush=True)
      #print('DECONFLICTOR COOPERATE q_pv_greedy: ' + str(self.q_pv_greedy), flush=True)
      #print('DECONFLICTOR COOPERATE pq_pv_proposed: ' + str(self.pq_pv_proposed), flush=True)
      pq_pv_diff = [None] * len_SolarPVsInfo
      for i in range(len_SolarPVsInfo):
        # note this is the same difference code for SolarPVs as the others
        # even though the greedy and proposed vectors are complex
        if self.pq_pv_proposed[i] != None:
          pq_pv_diff[i] = abs(complex(self.p_pv_greedy[i], self.q_pv_greedy[i])\
                               - self.pq_pv_proposed[i])

      #print('DECONFLICTOR COOPERATE pq_pv_diff: ' + str(pq_pv_diff), flush=True)

      # omit any setpoints where proposed == greeedy
      pq_pv_sort = []
      for i in range(len_SolarPVsInfo):
        if pq_pv_diff[i]!=None and pq_pv_diff[i]>0:
          pq_pv_sort.append(pq_pv_diff[i])

      # sorts in place
      pq_pv_sort.sort()

      # handle the case of only proposed == greedy
      diffMax = 0
      if len(pq_pv_sort) > 0:
        coopCount = max(1, -(len(pq_pv_sort)//-2)) # integer "ceiling" division

        # find the value associated with the last "cooperating" battery
        diffMax = pq_pv_sort[coopCount-1]

        #print('DECONFLICTOR COOPERATE solarPVs coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
      #else:
        #print('DECONFLICTOR COOPERATE solarPVs coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

      if coopLevel == 4:
        for i in range(len_SolarPVsInfo):
          # check if this is a "cooperating" solarPV
          if pq_pv_diff[i]!=None and pq_pv_diff[i]>0 and pq_pv_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            self.p_pv_greedy[i] = self.pq_pv_proposed[i].real
            self.q_pv_greedy[i] = self.pq_pv_proposed[i].imag

        #print('DECONFLICTOR COOPERATE p_pv_coop: ' + str(self.p_pv_greedy), flush=True)
        #print('DECONFLICTOR COOPERATE q_pv_coop: ' + str(self.q_pv_greedy), flush=True)

      else:
        pq_pv_denom = [] # just for diagnostic logging
        for i in range(len_SolarPVsInfo):
          # check if this is a "cooperating" solarPV
          if pq_pv_diff[i]!=None and pq_pv_diff[i]>0 and pq_pv_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            #self.p_pv_greedy[i] = self.pq_pv_proposed[i].real
            #self.q_pv_greedy[i] = self.pq_pv_proposed[i].imag
            # adjust cooperation level based on difference
            # find which entry this p_batt_diff is within p_batt_sort to
            # determine how much to cooperate. This is tricky code in that
            # a loop iterator varible is referenced after the loop.
            for ic in range(len(pq_pv_sort)):
              if pq_pv_diff[i] == pq_pv_sort[ic]:
                break
            fcoop = float(ic/coopRatioDenom) + 1.0

            # again, these are complex numbers, but division by a scalar
            # is done to each of them giving a complex result that is then
            # added to the original complex number. This is equivalent to
            # breaking up the work into the real and imag components.
            ratio = (self.pq_pv_proposed[i] - \
                     complex(self.p_pv_greedy[i], self.q_pv_greedy[i]))/ \
                    float(fcoop + self.coopCounter)
            self.p_pv_greedy[i] += ratio.real
            self.q_pv_greedy[i] += ratio.imag
            pq_pv_denom.append((fcoop, self.coopCounter))
          else:
            pq_pv_denom.append(None)

        #print('DECONFLICTOR COOPERATE p_pv_coop: ' + str(self.p_pv_greedy), flush=True)
        #print('DECONFLICTOR COOPERATE q_pv_coop: ' + str(self.q_pv_greedy), flush=True)
        #print('DECONFLICTOR COOPERATE pq_pv_denom: ' + str(pq_pv_denom), flush=True)

      for mrid in self.SolarPVs:
        idx = self.SolarPVs[mrid]['idx']
        # new value before old value for DifferenceBuilder
        # note the p and q values are negated for the GridLAB-D
        # DifferenceBuilder message
        if self.pq_pv_proposed[idx] != None:
          self.difference_builder.add_difference(mrid,
           'PowerElectronicsConnection.p', self.p_pv_greedy[idx], None)
          self.difference_builder.add_difference(mrid,
           'PowerElectronicsConnection.q', self.q_pv_greedy[idx], None)

    if self.includeRegulatorsFlag:
      # now do the same for regulators
      #print('DECONFLICTOR COOPERATE reg_greedy: ' + str(self.reg_greedy), flush=True)
      #print('DECONFLICTOR COOPERATE reg_proposed: ' + str(self.reg_proposed), flush=True)

      reg_diff = [None] * len_RegulatorsInfo
      for i in range(len_RegulatorsInfo):
        if self.reg_proposed[i] != None:
          reg_diff[i] = abs(self.reg_greedy[i] - self.reg_proposed[i])

      #print('DECONFLICTOR COOPERATE reg_diff: ' + str(reg_diff), flush=True)

      # omit any setpoints where proposed == greeedy
      reg_sort = []
      for i in range(len_RegulatorsInfo):
        if reg_diff[i]!=None and reg_diff[i]>0:
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

        #print('DECONFLICTOR COOPERATE regulators coopCount: ' + str(coopCount) + ', diffMax: ' + str(diffMax), flush=True)
      #else:
        #print('DECONFLICTOR COOPERATE regulators coopCount: ALL, diffMax: ' + str(diffMax), flush=True)

      if coopLevel == 4:
        for i in range(len_RegulatorsInfo):
          # check if this is a "cooperating" regulator
          if reg_diff[i]!=None and reg_diff[i]>0 and reg_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            self.reg_greedy[i] = self.reg_proposed[i]

        #print('DECONFLICTOR COOPERATE reg_coop: ' + str(self.reg_greedy), flush=True)

      else:
        reg_denom = [] # just for diagnostic logging
        for i in range(len_RegulatorsInfo):
          # check if this is a "cooperating" regulator
          if reg_diff[i]!=None and reg_diff[i]>0 and reg_diff[i]<=diffMax:
            # full cooperation by setting the greedy value to proposed value
            #self.reg_greedy[i] = self.reg_proposed[i]
            # adjust cooperation level based on difference
            # find which entry this p_batt_diff is within p_batt_sort to
            # determine how much to cooperate. This is tricky code in that
            # a loop iterator varible is referenced after the loop.
            for ic in range(len(reg_sort)):
              if reg_diff[i] == reg_sort[ic]:
                break
            fcoop = float(ic/coopRatioDenom) + 1.0

            ratio = int((self.reg_proposed[i] - self.reg_greedy[i])/ \
                        (fcoop + self.coopCounter))
            self.reg_greedy[i] += ratio
            reg_denom.append((fcoop, self.coopCounter))
          else:
            reg_denom.append(None)

        #print('DECONFLICTOR COOPERATE reg_coop: ' + str(self.reg_greedy), flush=True)
        #print('DECONFLICTOR COOPERATE reg_denom: ' + str(reg_denom), flush=True)

      for reg in self.RegulatorsInfo:
        idx = self.RegulatorsInfo[reg]['idx']
        # new value before old value for DifferenceBuilder
        if self.reg_proposed[idx] != None:
          self.difference_builder.add_difference(reg, 'TapChanger.step',
                                                 self.reg_greedy[idx], None)

    if self.includeBatteriesFlag or self.includeSolarPVsPFlag or \
       self.includeRegulatorsFlag:
      # finally, send out the cooperation setpoints via DifferenceBuilder msg
      dispatch_message = self.difference_builder.get_message()
      dispatch_message['app_name'] = self.app_name
      dispatch_message['coop_series'] = self.coopSeries
      dispatch_message['coop_msgid'] = self.coopMsgID
      dispatch_message['time_sent'] = str(datetime.now())
      print('Sending Cooperation DifferenceBuilder message with msgid: ' +
            str(self.coopMsgID) + ', series: ' + str(self.coopSeries),
            flush=True)
      #print('Sending Cooperation DifferenceBuilder message: ' +
      #      json.dumps(dispatch_message), flush=True)
      self.coop_gapps.send(self.coop_publish_topic, json.dumps(dispatch_message))
      if self.logMessagesFlag:
        self.msglog('sending cooperation response|msgid:' + str(self.coopMsgID) + '|series:' + str(self.coopSeries))

      self.difference_builder.clear()

  # end of cooperation handler process methods

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


  def optPerform(self, ts_datetime, cooperationFlag=False):
    self.Constraints = []
    print('Starting Problem Formulation for App ... ',flush=True)
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
                                         self.q_pv_A, self.q_pv_B, self.q_pv_C,
                                         ts_datetime)

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
        objective += self.objectiveWeights[2] * self.optObjective3( self.SolarPVsInfo, self.BatteriesInfo, self.p_pv_A, self.p_pv_B, self.p_pv_C, self.p_batt, ts_datetime)

      if numWeights>3 and self.objectiveWeights[3]!=None:


        objective += self.objectiveWeights[3] * self.optObjective4( self.EnergySource, self.Psub,
                                                                    self.Psub_mod, self.p_flow_A,
                                                                    self.p_flow_B, self.p_flow_C)

      if numWeights>4 and self.objectiveWeights[4]!=None:
        objective += self.objectiveWeights[4] * self.optObjective5(self.BatteriesInfo, self.soc, 
                                                                   self.SolarPVsInfo, self.p_pv_A, self.p_pv_B, self.p_pv_C)

    else:
      if self.objectiveResilienceFlag:
        objective = self.optObjectiveForResilience(self.BatteriesInfo, self.soc,
                       self.SolarPVsInfo, self.p_pv_A, self.p_pv_B, self.p_pv_C)

      if self.objectiveCVRFlag:
        objective = self.optObjectiveForCVR(self.BusInfo,
                                            self.v_A, self.v_B, self.v_C)

      if self.objectiveMaxLocalFlag:
        # GDB 9/22/25: created separate 1 and 2 stage versions for the Max Local
        # objective since 2 stage doesn't work as desired for p_pv values

        # note max_local is a two stage optimization and the first stage
        # is run within the objectiveForMaxLocal function
        '''
        validFlag, objective = self.optObjectiveForMaxLocal2Stage(self.BusInfo,
                                    self.BatteriesInfo, self.EnergySource,
                                    self.Psub, self.Psub_mod,
                                    self.p_flow_A, self.p_flow_B, self.p_flow_C,
                                    self.p_batt, self.v_A, self.v_B, self.v_C)
        '''
        objective = self.optObjectiveForMaxLocal1Stage(self.BusInfo,
                                    self.BatteriesInfo, self.EnergySource,
                                    self.Psub, self.Psub_mod,
                                    self.p_flow_A, self.p_flow_B, self.p_flow_C)
        
    print('Starting Optimization for App ... ',flush=True)
    if validFlag and self.optDo(objective):
      if not cooperationFlag:
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

    if includeRegulatorsFlag:
      len_RegulatorsInfo = len(self.RegulatorsInfo)
      self.reg_taps = cp.Variable((len_RegulatorsInfo, 32), boolean=True,
                                  name='reg_taps')
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
      # GDB 8/25/25: Allow for tweaking the SoC limits based on whether it's
      # a realtime simulation or not
      if self.realtimeFlag:
        self.Constraints.append(soc[idx] >= 0.2)
        self.Constraints.append(soc[idx] <= 0.9)
      else:
        self.Constraints.append(soc[idx] >= 0.2)
        self.Constraints.append(soc[idx] <= 0.9)
        #self.Constraints.append(soc[idx] >= 0.25)
        #self.Constraints.append(soc[idx] <= 0.85)

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
                                    q_pv_A, q_pv_B, q_pv_C, ts_datetime):
    ts_target = pd.to_datetime(str(ts_datetime))
    idx_profile = abs(self.solar_profile['Timestamp'] - ts_target).idxmin()
    pv_profile_now = self.solar_profile['PV_profile'][idx_profile]

    for bus in SolarPVsInfo:
      idx = SolarPVsInfo[bus]['idx']

      numphases = len(SolarPVsInfo[bus]['phase'])
      if 'N' in SolarPVsInfo[bus]['phase']:
        numphases -= 1

      ratedS = pv_profile_now * (SolarPVsInfo[bus]['ratedS']/numphases)
      #ratedP = SolarPVsInfo[bus]['p']/numphases

      coeff = math.sqrt(2) - 1 ### Coefficient for Octagon Constraints

      if 'A' in SolarPVsInfo[bus]['phase']:
        self.Constraints.append(p_pv_A[idx] <= ratedS)
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
        self.Constraints.append(p_pv_B[idx] <= ratedS)
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
        self.Constraints.append(p_pv_C[idx] <= ratedS)
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


  def optObjectiveForResilience(self, BatteriesInfo, soc, SolarPVsInfo,
                                p_pv_A, p_pv_B, p_pv_C):
    # SHIVA magic scaling factor for SoC that causes the optmization to
    # come up with the correct results where -soc[i] doesn't.
    # Shiva will be investigating why this happens since we don't want
    # to be dependent on magic
    objective = sum(-100 * soc[i] for i in range(len(BatteriesInfo)))

    # MM 9/17/25
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

    objective -= objective_pv/1e+6

    return objective


  def optObjectiveForCVR(self, BusInfo, v_A, v_B, v_C):
    objective = sum((v_A[i] + v_B[i] + v_C[i]) for i in range(len(BusInfo)))
    return objective


  def optObjectiveForMaxLocal1Stage(self, BusInfo, BatteriesInfo, EnergySource,
                                    Psub, Psub_mod, p_flow_A,p_flow_B,p_flow_C):
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

    return objective


  def optObjectiveForMaxLocal2Stage(self, BusInfo, BatteriesInfo, EnergySource,
                                    Psub, Psub_mod, p_flow_A, p_flow_B,p_flow_C,
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
    #print('Adding Objective 1 for CVR at time {}'.format(ts_time))
    print('Adding Objective 1 for CVR')
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

    objective -= (objective_pv)/9960000
    return objective


  def optObjective2(self, EnergySource, Psub, Psub_mod, Qsub, Qsub_mod,
                    p_flow_A, p_flow_B, p_flow_C, q_flow_A, q_flow_B, q_flow_C):
    #print('Adding Objective 2 for PF at time {}'.format(ts_time))
    print('Adding Objective 2 for PF')
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
    objective = (Qsub_mod + Psub_mod) / 2000000
    return objective


  def optObjective3(self, SolarPVsInfo, BatteriesInfo, p_pv_A, p_pv_B, p_pv_C, p_batt, ts_datetime):
    cost = pd.read_csv('lmp_data.csv')
    cost['time'] = pd.to_datetime(cost['time'])
    ts_target = pd.to_datetime(str(ts_datetime))
    idx_cost = abs(cost['time'] - ts_target).idxmin()
    cost = cost['price'].values/1000
    average_cost = np.mean(cost)
    cost_now = cost[idx_cost]

    print('Adding Objective 3 for Arbitrage at time {} with current and average price {}, {}'.format(ts_target, cost_now, average_cost))
    cost_sign = math.copysign(1, (cost_now-average_cost))
    objective_batt = sum(cost_sign * cost_now* p_batt[i] for i in range(len(BatteriesInfo)))/1000

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
    objective = (objective_batt + objective_pv) * self.deltaT / 60

    return objective


  def optObjective4(self, EnergySource, Psub, Psub_mod, p_flow_A, p_flow_B, p_flow_C):

    #print('Adding Objective 4 for Peak Load at time {}'.format(ts_time))
    print('Adding Objective 4 for Peak Load')
    target_peak = 1.5e6
    self.Constraints.append(Psub_mod >=     target_peak - Psub)
    self.Constraints.append(Psub_mod >= -1*(target_peak - Psub))

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
    objective = Psub_mod /500000

    return objective


  def optObjective5(self, BatteriesInfo, soc, SolarPVsInfo, p_pv_A, p_pv_B, p_pv_C):
    #print('Adding Objective 5 for Resilience at time {}'.format(ts_time))
    print('Adding Objective 5 for Resilience')
    objective = sum(-100 * soc[i] for i in range(len(BatteriesInfo))) / (4.5* 100)

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

    objective -= (objective_pv)/16600000.0

    return objective


  def optDo(self, objective):
    problem = cp.Problem(cp.Minimize(objective), self.Constraints)
    startTime = datetime.now()
    #problem.solve(solver=cp.MOSEK, verbose=True) # commercial solver
    #problem.solve(solver=cp.CBC, verbose=False)
    # GDB 9/19/25: Set 15 second time limit before bailing because sometimes
    # it really gets stuck and falls so far behind that the results are
    # useless by the time they are computed and then the app is way behind
    # in missing all the data while it was stuck.

    problem.solve(solver=cp.GLPK_MI, abstol=1e-3, kktsolver='chol',
                  feastol=1e-3, max_iters=100, tm_lim=15000, verbose=False)

    
    # problem.solve(solver=cp.MOSEK, mosek_params={'MSK_DPAR_MIO_TOL_REL_GAP': 2e-2, 'MSK_IPAR_INTPNT_MAX_ITERATIONS': 100, 
    #                                       'MSK_DPAR_MIO_TOL_ABS_RELAX_INT': 1e-2, 'MSK_DPAR_OPTIMIZER_MAX_TIME': 15000},  verbose=False)

    print('Optimization status:', problem.status, flush=True)
    #print('Optimization value:', problem.value, flush=True)
    now = datetime.now()
    optTime = (now - startTime).total_seconds()
    optInterval= (now - self.lastTime).total_seconds()
    self.lastTime = now
    print('Optimization time: ' + str(optTime), flush=True)
    print('Optimization time interval: ' + str(optInterval), flush=True)
    print('Optimization Objective Value: ' + str(problem.value), flush=True)

    # GDB 9/19/25: Treat an optimal_inaccurate status the same as optimal
    return (problem.status.startswith('optimal'))


  def optDispatch(self, includeRegulatorsFlag, includeBatteriesFlag,
                  includeSolarPVsPFlag, includeVoltagesFlag):

    if includeVoltagesFlag:
      # volt_sum = sum((self.v_A[i].value + self.v_B[i].value + self.v_C[i].value) for i in range(len(self.BusInfo))) / (2401.77 ** 2)
      volt_sum = sum((self.v_A[i].value + self.v_B[i].value + self.v_C[i].value) for i in range(len(self.BusInfo))) / ((2401.77 ** 2) * (123 * 3))
      #print("Optimized sum of Voltages: {}".format(volt_sum))

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
                     tablefmt='psql'), flush=True)

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
             'PowerElectronicsConnection.p', total_p, None)
        total_q = self.q_pv_A[idx].value + self.q_pv_B[idx].value + \
                  self.q_pv_C[idx].value
        self.difference_builder.add_difference(mrid,
             'PowerElectronicsConnection.q', total_q, None)

        pq_pv_setpoints.append([name, bus, total_p/1000, total_q/1000])

        # set p_pv_greedy and q_pv_greedy with every optimization based
        # on measurements
        self.p_pv_greedy[idx] = total_p
        self.q_pv_greedy[idx] = total_q

      print(tabulate(pq_pv_setpoints,headers=['SolarPV', 'bus', 'Total p (kW)',
                     'Total q (kVAR)'], tablefmt='psql'), flush=True)

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
      dispatch_message['time_sent'] = str(datetime.now())
      print('Sending Measurements DifferenceBuilder message!', flush=True)
      #print('Sending Measurements DifferenceBuilder message: ' +
      #      json.dumps(dispatch_message), flush=True)

      # these can go either to the simulation or the deconfliction pipeline
      # based on the deconflictionAsServiceFlag value
      self.sim_gapps.send(self.sim_publish_topic, json.dumps(dispatch_message))
      if self.logMessagesFlag:
        self.msglog('sending new optimization setpoints')

      self.difference_builder.clear()


  def pol2cart(self, mag, angle_deg):
        # Convert degrees to radians. GridAPPS-D spits angle in degrees
        angle_rad =  math.radians(angle_deg)
        p = mag * np.cos(angle_rad)
        q = mag * np.sin(angle_rad)
        return p, q


  def updateEnergyConsumers(self, measurements):
    total_kW = 0; 
    total_kVAR = 0; 
    for bus in self.EnergyConsumers:
      for phase in self.EnergyConsumers[bus]['measid']:
        measid = self.EnergyConsumers[bus]['measid'][phase]
        if measid in measurements:
          p, q = self.pol2cart(measurements[measid]['magnitude'],
                               measurements[measid]['angle'])
          self.EnergyConsumers[bus]['kW'][phase] = p
          self.EnergyConsumers[bus]['kVar'][phase] = q
          total_kW += p 
          total_kVAR += q
    print('Updated EnergyConsumers - Current total kW : ' + str(round(total_kW/1000, 3)), flush=True)
    print('Updated EnergyConsumers - Current total kVAR : ' + str(round(total_kVAR/1000, 3)), flush=True)

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


  def processMeasMessage(self, measurements):
    # update the EnergyConsumers, etc. data structures with new
    # measurements
    if self.includeEnergyConsumersFlag:
      self.updateEnergyConsumers(measurements)
      #print('Updated EnergyConsumers: ' + json.dumps(self.EnergyConsumers, indent=2), flush=True)

    if self.includeSolarPVsPFlag:
      self.updateSolarPVs(measurements)
      #print('Updated SolarPVsInfo: ' + json.dumps(self.SolarPVsInfo, indent=2), flush=True)

    if self.includeBatteriesFlag:
      self.updateBatterySoC(measurements)
      #print('Updated BatteryInfo: ' + json.dumps(self.BatteriesInfo, indent=2), flush=True)

    # tap positions only need to be tracked when not solving for the
    # positions as part of the optimization problem
    if not self.includeRegulatorsFlag:
      self.updateRegulatorTaps(measurements)


  def __init__(self, opt_type, feeder_mrid, simulation_id, interval):
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

    # flag for whether simulation is run in real-time
    #self.realtimeFlag = True
    self.realtimeFlag = False

    self.simLogSubscribedFlag = True
    if not self.realtimeFlag:
      self.simLogSubscribedFlag = False

    # flag for whether to log cooperation messages in a file
    self.logMessagesFlag = True

    # deltaT is time between timesteps as fractional hours
    # optimization interval seconds is the number of simulation seconds
    # between triggering an optimization and must be a multiple of 3
    # for a real-time simulation
    if self.realtimeFlag:
      #self.optIntervalSec = 3 # optimize every GridLAB-D timestamp
      # 15 seconds is a good number for a real-time simulation
      self.optIntervalSec = 15
      simLagSec = 0
    else:
      # if attempting non-real-time, something like 1800 is reasonable
      # so the optimization time is safely shorter than the time between
      # optimizations--otherwise the queue draining won't work right.
      #self.optIntervalSec = 1800
      #self.optIntervalSec = 3600
      self.optIntervalSec = 7200
      simLagSec = 600

    if self.opt_type!='scalability' and interval!=None:
      self.optIntervalSec = int(interval)

    print('Optimization intervals: ' + str(self.optIntervalSec))
    # Add compensation factor to optIntervalSec in non-realtime mode
    # for computing deltaT because of the lag GridLAB-D is taking in
    # this mode for measurements to reflect DifferenceBuilder messages
    self.deltaT = (self.optIntervalSec + simLagSec)/3600.0

    if self.opt_type == 'scalability':
      # the interval value is actually the app_setup.csv line
      self.optPrelimScalability(interval)
    else:
      self.optPrelimClassic()

    # GDB 8/27/25: Magic IPC Queue class for sharing ActiveMQ messages
    # between different processes
    self.simQueue = Queue()
    self.coopQueue = Queue()

    # Subscribe to simulation and cooperation messages in new process
    # in order to handle messages in a timely fashion outside of the
    # processes that perform long-running numerical optimizations.
    messageListener = Process(target=self.messageListenerProcess,
                              args=(simulation_id,))
    messageListener.start()

    self.sim_gapps = GridAPPSD(simulation_id)
    assert self.sim_gapps.connected

    SPARQLManager = getattr(importlib.import_module('sparql'), 'SPARQLManager')
    sparql_mgr = SPARQLManager(self.sim_gapps, feeder_mrid, simulation_id)

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

    self.RegulatorsInfo, self.RegulatorsIdx = AppUtil.getCombineRegulators(sparql_mgr)

    # Need a way to map from a measid to the mrid for regulators in order
    # to process tap position changes in new measurements
    RegsForMeasID = AppUtil.getRegulators(sparql_mgr)
    for mrid in self.RegulatorsInfo:
      if mrid in RegsForMeasID:
        self.RegulatorsInfo[mrid]['measid'] = RegsForMeasID[mrid]['measid']

    print('RegulatorsInfo: ' + str(self.RegulatorsInfo), flush=True)
    print('RegulatorsIdx: ' + str(self.RegulatorsIdx), flush=True)

    # MM 9/19/25: load the solarPV profile data to be able to do quick
    # lookups for the current timestamp each time an optimization is done
    if self.includeSolarPVsPFlag:
      self.solar_profile = pd.read_csv('solar_profile.csv')
      self.solar_profile['Timestamp'] = pd.to_datetime(
                                           self.solar_profile['Timestamp'])

    # cooperation variables
    # for the greedy values these use multiprocessing shared memory
    len_BatteriesInfo = len(self.BatteriesInfo)
    self.p_batt_proposed = [None] * len_BatteriesInfo
    self.p_batt_greedy = Array('d', [0.0] * len_BatteriesInfo)

    len_RegulatorsInfo = len(self.RegulatorsInfo)
    self.reg_proposed = [None] * len_RegulatorsInfo
    self.reg_greedy = Array('i', [0] * len_RegulatorsInfo)

    len_SolarPVsInfo = len(self.SolarPVsInfo)
    self.pq_pv_proposed = [None] * len_SolarPVsInfo
    self.p_pv_greedy = Array('d', [0.0] * len_SolarPVsInfo)
    self.q_pv_greedy = Array('d', [0.0] * len_SolarPVsInfo)

    # topic for sending out cooperation responses
    self.coop_publish_topic = service_input_topic('deconfliction.cooperation',
                                                  simulation_id)

    # determine whether to send directly to simulation or the deconfliction
    # pipeline
    deconflictionAsServiceFlag = False
    # GDB 8/25/25: Set as service just to send to simulation for debugging
    # outside of running deconfliction pipeline
    #deconflictionAsServiceFlag = True
    if deconflictionAsServiceFlag:
      self.sim_publish_topic = simulation_input_topic(simulation_id)
    else:
      self.sim_publish_topic = service_input_topic('deconfliction.measurements',
                                                   simulation_id)

    # create DifferenceBuilder once and reuse it throughout the simulation
    self.difference_builder = DifferenceBuilder(simulation_id)

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

    self.b_i = np.arange(0.9, 1.1, 0.00625)

    self.optDefineVariables(self.includePFlowFlag, self.includeQFlowFlag,
                          self.includeVoltagesFlag, self.includeBatteriesFlag,
                          self.includeRegulatorsFlag, self.includeSolarPVsPFlag)

    # diagnostic for tracking time between optimizations
    self.lastTime = datetime.now()

    # Cooperation is handled in a third process so need to have everything
    # that code needs defined before creating this process such as the
    # device info dictionaries.
    cooperationHandler = Process(target=self.cooperationHandlerProcess,
                                 args=(simulation_id,))
    cooperationHandler.start()

    # start by discarding any messages that arrived during initialization
    # as we don't want to process anything that's stale
    print('Simulation queue check after initialization start', flush=True)
    while not self.simQueue.empty():
      message = self.simQueue.get()

      if 'processStatus' in message: # simulation log message
        # this would be weird to get this early, but it could happen
        status = message['processStatus']
        if status == 'ABORT':
          print('ABORTING due to message delay that will lead to ' +
                'imminent failure--delay seconds: ' + message['delaySeconds'])
        else:
          print('Simulation ' + status + ' message received', flush=True)

        # wait for messageListener process to finish
        messageListener.join()

        # wait for coperationHandler process to finish
        cooperationHandler.join()

        return # done with all processing

      if 'measurements' in message: # simulation output message
        print('Simulation measurements message on queue discarded with ' +
              'timestamp: ' + str(message['timestamp']), flush=True)
    print('Simulation queue check after initialization finish\n', flush=True)

    print('Initialized modularized ' + opt_type +
          ' CVXPY optimization competing app, waiting for messages...\n',
          flush=True)

    while True:
      while self.simQueue.empty():
        # GDB 9/2/25: Warning: increasing the sleep duration above 0.1 such as
        # 0.5 can lead to bad things. With two processes sleeping on both ends
        # (apps and deconfliction pipeline) that's 4 sleep statements that are
        # part of processing messages leading to a potential 2 second total
        # delay (with 0.5 sleeps), which is horrible for cooperation messages.
        #sleep(0.1)
        sleep(0.05)

      lastMeasMessage = None

      print('Simulation queue check start', flush=True)
      while not self.simQueue.empty():
        message = self.simQueue.get()

        if 'processStatus' in message: # simulation log message
          status = message['processStatus']
          if status == 'ABORT':
            print('ABORTING due to message delay that will lead to ' +
                  'imminent failure--delay seconds: ' + message['delaySeconds'])
          else:
            print('Simulation ' + status + ' message received', flush=True)

          # wait for messageListener process to finish
          messageListener.join()

          # wait for cooperationHandler process to finish
          cooperationHandler.join()

          return # done with all processing

        if 'measurements' in message: # simulation output message
          if self.logMessagesFlag:
            self.msglog('found simulation measurements message on queue with timestamp: ' + str(message['timestamp']) + ', wall time: ' + str(datetime.utcfromtimestamp(int(message['timestamp'])).time()))
          print('Simulation measurements message on queue with timestamp: ' +
                str(message['timestamp']), flush=True)
          lastMeasMessage = message
      print('Simulation queue check finish', flush=True)

      if lastMeasMessage != None:
        ts_unix = int(lastMeasMessage['timestamp'])
        global ts_time
        ts_time = datetime.utcfromtimestamp(ts_unix).time()

        if self.logMessagesFlag:
          self.msglog('simulation timestamp used for optimization: ' + str(ts_unix) + ', wall time: ' + str(ts_time))
        print('\nSimulation timestamp for optimization: ' + str(ts_unix) +
              ', wall time: ' + str(ts_time), flush=True)

        # process new measurements
        self.processMeasMessage(lastMeasMessage['measurements'])

        # perform optimization
        self.optPerform(datetime.utcfromtimestamp(ts_unix))


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

  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  competing_app = CompetingApp(opts.type, feeder_mrid, opts.simulation_id,
                               opts.interval)

  print('Goodbye!', flush=True)


if __name__ == "__main__":
  _main()

