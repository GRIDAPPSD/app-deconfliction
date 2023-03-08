#!/usr/bin/env python3


# imports for RepeatedTimer class

import threading
import time

class RepeatedTimer(object):
  def __init__(self, interval, function, *args, **kwargs):
    self._timer = None
    self.interval = interval
    self.function = function
    self.args = args
    self.kwargs = kwargs
    self.is_running = False
    self.next_call = time.time()
    self.start()

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

import os
import csv
from gridappsd import GridAPPSD
from gridappsd.topics import service_output_topic


def send_message(gapps, publish_topic, reader):
  ret = True # return True to call this again the next time interval

  # next() will throw an exception on EOF
  try:
    row = next(reader)
  except:
    # send out one final message with end-of-data flag for timestamp
    row = ['', '', '', '']

    # returning False will exit from the timer-based calls
    ret = False

  message = {
    'timestamp': row[0],
    'loadshape': row[1],
    'solar': row[2],
    'price': row[3]
  }
  gapps.send(publish_topic, message)
  print(time.time(), ': ', str(message), flush=True)

  return ret


def _main():
  # authenticate with GridAPPS-D Platform
  os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-pseudo-sim'
  os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
  os.environ['GRIDAPPSD_USER'] = 'app_user'
  os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

  gapps = GridAPPSD()
  assert gapps.connected

  # hardwire simulation_id as part of the topic
  publish_topic = service_output_topic('gridappsd-pseudo-sim', '0')
  #print(publish_topic)

  fp = open('time-series-data.csv', 'r')
  reader = csv.reader(fp)
  next(reader) # skip header

  rt = RepeatedTimer(.5, send_message, gapps, publish_topic, reader)
  #rt = RepeatedTimer(3, send_message, gapps, publish_topic, reader)


if __name__ == "__main__":
  _main()

