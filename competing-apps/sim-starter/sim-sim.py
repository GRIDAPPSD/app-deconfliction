#!/usr/bin/env python3

# imports for RepeatedTimer class

import threading
import time


class RepeatedTimer(object):
  def __init__(self, interval, function, *args, **kwargs):
    if interval > 0:
      print('Hit return to start sending messages...', end='', flush=True)
      input()

      self._timer = None
      self.interval = interval
      self.function = function
      self.args = args
      self.kwargs = kwargs
      self.is_running = False
      self.next_call = time.time()
      self.start()
    else:
      print('Hit return to send first message...', end='', flush=True)
      input()

      while function(*args, **kwargs):
        print('Hit return to send another message...', end='', flush=True)
        input()

  def _run(self):
    self.is_running = False
    self.start()
    # exit or continue based on return value of user function
    if not self.function(*self.args, **self.kwargs):
      self.stop()

  def start(self):
    if not self.is_running:
      self.next_call += self.interval
      self._timer = threading.Timer(self.next_call - time.time(), self._run)
      self._timer.start()
      self.is_running = True

  def stop(self):
    self._timer.cancel()
    self.is_running = False


# imports for sending GridAPPS-D messages via RepeatedTimer

import os, sys
import argparse
import importlib
import json
import csv
from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic

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


class SimSim(GridAPPSD):

  def on_message(self, headers, message):
    #print('headers: ' + str(headers), flush=True)
    #print('message: ' + str(message), flush=True)

    timestamp = message['timestamp']
    if timestamp != self.currentTimestamp:
      print('*** DISPATCH FALLING BEHIND--current timestamp: ' + str(self.currentTimestamp) + ', dispatch timestamp: ' + str(timestamp), flush=True)

    DispatchedDevices = message['dispatch']
    print('DispatchedDevices: ' + str(DispatchedDevices), flush=True)

    for device, value in DispatchedDevices.items():
      if device.startswith('BatteryUnit.'):
        self.Batteries[device]['P_batt'] = value


  def send_message(self):
    ret = True # return True to call this again the next time interval

    BatterySoC = {}

    # next() will throw an exception on EOF
    try:
      row = next(self.reader)

      for device, battery in self.Batteries.items():
        if battery['P_batt'] != 0.0:
          # only send out SoC values that have been updated
          contrib = AppUtil.contrib_SoC(battery['P_batt'], 1, battery,
                                        self.deltaT)
          battery['SoC'] += contrib
          print(device + ' P_batt: ' + str(battery['P_batt']),
                ', new SoC contribution: ' + str(contrib) +
                ', updated SoC: ' + str(battery['SoC']), flush=True)

        # unchanged SoC values also must be sent to apps
        BatterySoC[device] = battery['SoC']

    except:
      # send out one final message with end-of-data flag for timestamp
      row = ['', '', '', '']

      # returning False will exit from the timer-based calls
      ret = False

    message = {
      'timestamp': row[0],
      'loadshape': row[1],
      'solar': row[2],
      'price': row[3],
      'BatterySoC': BatterySoC
    }
    self.gapps.send(self.publish_topic, message)
    print(time.time(), ': ', str(message), flush=True)

    if row[0] != '':
      self.currentTimestamp = int(row[0])

    return ret


  def __init__(self, gapps, feeder_mrid, simulation_id):
    SPARQLManager = getattr(importlib.import_module('sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = AppUtil.getBatteries(sparql_mgr)

    # initialize P_batt to 0 for all batteries so there is something to
    # compute SoC from if there are no device dispatcher updates
    for device in self.Batteries:
      self.Batteries[device]['P_batt'] = 0.0

    print(self.Batteries, flush=True)

    self.deltaT = 0.25

    gapps.subscribe(service_output_topic(
              'gridappsd-deconfliction-pipeline-dispatch', simulation_id), self)

    self.publish_topic = service_output_topic('gridappsd-pseudo-sim',
                                              simulation_id)

    self.gapps = gapps

    fp = open('time-series.csv', 'r')
    self.reader = csv.reader(fp)
    next(self.reader) # skip header

    rt = RepeatedTimer(0, self.send_message)
    # 2 seconds seems to be right for resilience and decarbonization
    #rt = RepeatedTimer(2, self.send_message)
    # 8 seconds seems to be right for resilience, decarbonization, and profit
    #rt = RepeatedTimer(8, self.send_message)


def _main():
  print('Starting sim-sim simulated simulation...', flush=True)

  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-pseudo-sim'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD()
  assert gapps.connected

  SimSim(gapps, feeder_mrid, opts.simulation_id)


if __name__ == "__main__":
  _main()

