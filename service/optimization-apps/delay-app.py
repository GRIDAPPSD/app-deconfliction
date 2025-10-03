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
import csv
import copy
from time import sleep

from gridappsd import GridAPPSD
from gridappsd.topics import simulation_input_topic, simulation_output_topic
from gridappsd.topics import simulation_log_topic
from gridappsd.topics import service_input_topic, service_output_topic

from datetime import datetime

# find and add shared directory to path hopefully wherever it is from here
'''
if (os.path.isdir('../shared')):
  sys.path.append('../shared')
elif (os.path.isdir('../competing-apps/shared')):
  sys.path.append('../competing-apps/shared')
elif (os.path.isdir('../../competing-apps/shared')):
  sys.path.append('../../competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')
'''

# 80 column ruler for continuation lines
#0000000011111111112222222222333333333344444444445555555555666666666677777777778
#2345678901234567890123456789012345678901234567890123456789012345678901234567890

class CompetingApp(GridAPPSD):

  def msglog(self, msg):
    print(msg, flush=True)
    '''
    try:
      with open('log/delay-app-messages.log', 'a') as flog:
        flog.write(str(datetime.now()) + ': ' + msg + '\n')
    except:
      pass
    '''

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
          self.msglog('received simulation measurements to queue at timestamp:' + str(ts_unix))

    else:
      # If doing non-real-time simulation remove the 5 second offset because
      # GridLAB-D outputs at even 60 second intervals
      if ts_unix % self.optIntervalSec == 0:
        if self.logMessagesFlag:
          self.msglog('received simulation measurements to queue at timestamp:' + str(ts_unix))


  def OnSimLogMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    status = message['processStatus']
    if status=='COMPLETE' or status=='CLOSED':
      self.keepLoopingFlag = False
      # both simulation and cooperation queues need this message


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


  def OnAbortMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    if 'processStatus' in message:
      if message['processStatus']=='COMPLETE' or \
         message['processStatus']=='ABORT':
        self.keepLoopingFlag = False


  def __init__(self, feeder_mrid, simulation_id):
    # flag for whether simulation is run in real-time
    #self.realtimeFlag = True
    self.realtimeFlag = False

    self.simLogSubscribedFlag = True
    if not self.realtimeFlag:
      self.simLogSubscribedFlag = False

    # flag for whether to log cooperation messages in a file
    self.logMessagesFlag = True

    if self.realtimeFlag:
      #self.optIntervalSec = 3 # optimize every GridLAB-D timestamp
      # 15 seconds is a good number for a real-time simulation
      self.optIntervalSec = 15
    else:
      # if attempting non-real-time, something like 1800 is reasonable
      # so the optimization time is safely shorter than the time between
      # optimizations--otherwise the queue draining won't work right.
      self.optIntervalSec = 1800
      #self.optIntervalSec = 3600

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

  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  competing_app = CompetingApp(feeder_mrid, opts.simulation_id)

  print('Goodbye!', flush=True)


if __name__ == "__main__":
  _main()

