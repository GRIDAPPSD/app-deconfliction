#!/usr/bin/env python3

# imports for RepeatedTimer class

import threading
import time

import matplotlib # done from AppUtil so just making sure it's available here

class RepeatedTimer(object):
  def __init__(self, interval, function, get_dispatches_count, *args, **kwargs):
    if interval > 0:
      self._timer = None
      self.interval = interval
      self.function = function
      self.args = args
      self.kwargs = kwargs
      self.is_running = False
      self.next_call = time.time()
      self.start()

    elif interval < 0:
      dispatches_needed = -interval

      while function(*args, **kwargs):
        # magic that sends out new messages only when the number of device
        # dispatch messages received is the number of running competing apps
        while get_dispatches_count() < dispatches_needed:
          time.sleep(0.1)

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
import MethodUtil

class SimSim(GridAPPSD):

  def on_message(self, headers, message):
    #print('headers: ' + str(headers), flush=True)
    #print('message: ' + str(message), flush=True)

    timestamp = message['timestamp']
    if timestamp != self.currentTimestamp:
      print('*** DISPATCH FALLING BEHIND--current timestamp: ' + str(self.currentTimestamp) + ', dispatch timestamp: ' + str(timestamp), flush=True)
    else:
      self.dispatches_count += 1

    DispatchedDevices = message['dispatch']
    print('DispatchedDevices: ' + str(DispatchedDevices), flush=True)

    for device, value in DispatchedDevices.items():
      self.DeviceSetpoints[device] = value
      if device in MethodUtil.DeviceToName and \
         MethodUtil.DeviceToName[device].startswith('BatteryUnit.'):
        self.Batteries[device]['P_batt'] = value


  def get_dispatches_count(self):
    return self.dispatches_count


  def send_message(self):
    ret = True # return True to call this again the next time interval

    BatterySoC = {}

    # next() will throw a StopIteration exception on EOF
    try:
      row = next(self.reader)

      for device, battery in self.Batteries.items():
        if battery['P_batt'] != 0.0:
          # only send out SoC values that have been updated
          contrib = AppUtil.contrib_SoC(battery['P_batt'], 1, battery,
                                        self.deltaT)
          battery['SoC'] += contrib

          # constrain to range 0.2 <= SoC <= 0.9
          if battery['SoC'] > 0.9:
            battery['SoC'] = 0.9
          elif battery['SoC'] < 0.2:
            battery['SoC'] = 0.2

          print(device + ' P_batt: ' + str(battery['P_batt']),
                ', new SoC contribution: ' + str(contrib) +
                ', updated SoC: ' + str(battery['SoC']), flush=True)

        # unchanged SoC values also must be sent to apps
        BatterySoC[device] = battery['SoC']

      # for plotting
      datetime = AppUtil.to_datetime(row[0])
      self.t_plot.append(datetime)
      for device, battery in self.Batteries.items():
        self.p_batt_plot[device].append(battery['P_batt'])
        self.soc_plot[device].append(battery['SoC'])

    except StopIteration:
      # for plotting
      # make sure output directory exists since that's where results go
      if not os.path.isdir('output'):
        os.makedirs('output')

      AppUtil.make_plots('Deconfliction Resolution', 'deconfliction',
                   self.Batteries, self.t_plot, self.p_batt_plot, self.soc_plot)

      # send out one final message with end-of-data flag for timestamp
      row = ['', '', '', '']

      # returning False will exit from the timer-based calls
      ret = False

    message = {
      'timestamp': row[0],
      'loadshape': row[1],
      'solar': row[2],
      'price': row[3],
      'DeviceSetpoints': self.DeviceSetpoints,
      'BatterySoC': BatterySoC
    }
    self.gapps.send(self.publish_topic, message)
    print(time.time(), ': ', str(message), flush=True)

    # clear set-points so they accumulate over multiple deconflictor messages
    self.DeviceSetpoints.clear()

    if row[0] != '':
      self.currentTimestamp = int(row[0])

    self.dispatches_count = 0

    return ret


  def __init__(self, gapps, feeder_mrid, simulation_id, delay):
    SPARQLManager = getattr(importlib.import_module('sparql'),
                            'SPARQLManager')
    sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

    self.Batteries = AppUtil.getBatteries(sparql_mgr)

    # initialize P_batt to 0 for all batteries so there is something to
    # compute SoC from if there are no device dispatcher updates
    for device in self.Batteries:
      self.Batteries[device]['P_batt'] = 0.0

    print(self.Batteries, flush=True)

    self.DeviceSetpoints = {}

    self.deltaT = 0.25

    # for plotting
    self.t_plot = []
    self.soc_plot = {}
    self.p_batt_plot = {}
    for mrid in self.Batteries:
      self.soc_plot[mrid] = []
      self.p_batt_plot[mrid] = []

    gapps.subscribe(service_output_topic('gridappsd-deconfliction-pipeline',
                                         simulation_id), self)

    self.publish_topic = service_output_topic('gridappsd-sim-sim',
                                              simulation_id)

    self.gapps = gapps

    fp = open('time-series.csv', 'r')
    self.reader = csv.reader(fp)
    next(self.reader) # skip header

    rt = RepeatedTimer(int(delay), self.send_message, self.get_dispatches_count)


def _main():
  print('Starting sim-sim simulated simulation...', flush=True)

  parser = argparse.ArgumentParser()
  parser.add_argument("simulation_id", help="Simulation ID")
  parser.add_argument("request", help="Simulation Request")
  parser.add_argument("delay",
                      help="Delay in seconds between messages, 0=interactive")

  opts = parser.parse_args()

  sim_request = json.loads(opts.request.replace("\'",""))
  feeder_mrid = sim_request["power_system_config"]["Line_name"]

  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-sim-sim'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD()
  assert gapps.connected

  SimSim(gapps, feeder_mrid, opts.simulation_id, opts.delay)


if __name__ == "__main__":
  _main()

