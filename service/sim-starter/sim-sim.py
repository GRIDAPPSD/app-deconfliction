#!/usr/bin/env python3

# ------------------------------------------------------------------------------
# Copyright (c) 2020, Battelle Memorial Institute All rights reserved.
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
Created on Oct 16, 2026

@author: Gary D. Black
"""

__version__ = '0.1.0'

import sys
import time
import json
import os

# gridappsd-python module
from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic, simulation_output_topic

class SimSim:

  def OnSimOutputMessage(self, header, message):
    #print('header: ' + str(header), flush=True)
    #print('message: ' + str(message), flush=True)
    if not self.keepLoopingFlag:
      return

    # sleep between sending each measurements message
    time.sleep(0.5)

    self.gapps.send(self.publish_topic, json.dumps(message))

    simTimestamp = int(message['message']['timestamp'])
    #print('sim-sim published timestamp: ' + str(simTimestamp), flush=True)

    if self.simTimestampStart == None:
      self.simTimestampStart = simTimestamp

    if (simTimestamp - self.simTimestampStart) >= self.simDuration:
      self.keepLoopingFlag = False


  def __init__(self, sim_id):
    self.simTimestampStart = None
    self.simDuration = 86400-120

    self.gapps = GridAPPSD()

    self.publish_topic = service_output_topic('sim-sim', sim_id)

    self.keepLoopingFlag = True

    out_id = self.gapps.subscribe(simulation_output_topic(sim_id),
                                  self.OnSimOutputMessage)

    while self.keepLoopingFlag:
      time.sleep(0.05)

    #print('sim-sim finished', flush=True)

    self.gapps.unsubscribe(out_id)
    self.gapps.disconnect()


def _main():
    if len(sys.argv)<2 or '-help' in sys.argv:
        usestr =  '\nUsage: ' + sys.argv[0] + ' simid\n'
        usestr += '''
Optional command line arguments:
        -help: show this usage message
        '''
        print(usestr, file=sys.stderr, flush=True)
        exit()

    # authenticate with GridAPPS-D Platform
    os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-sim-sim'
    os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
    os.environ['GRIDAPPSD_USER'] = 'app_user'
    os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

    sim_id = sys.argv[1]

    sim_sim = SimSim(sim_id)


if __name__ == '__main__':
    _main()

