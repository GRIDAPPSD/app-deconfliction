"""Module for querying and parsing SPARQL through GridAPPS-D"""
import logging
import pandas as pd
import numpy as np
import re
from gridappsd import GridAPPSD, topics, utils

class AppUtil:
  """ Class for competing app utility functions
  """
    
  def discharge_batteries(Batteries, deltaT):
    for name in Batteries:
      Batteries[name]['state'] = 'discharging'
      if 'P_batt_d' in Batteries[name]:
        Batteries[name]['SoC'] -= 1/Batteries[name]['eff_d']*Batteries[name]['P_batt_d']*deltaT/Batteries[name]['ratedE']


  def charge_batteries(Batteries, deltaT):
    for name in Batteries:
      Batteries[name]['state'] = 'charging'
      if 'P_batt_c' in Batteries[name]:
        Batteries[name]['SoC'] += Batteries[name]['eff_c']*Batteries[name]['P_batt_c']*deltaT/Batteries[name]['ratedE']


