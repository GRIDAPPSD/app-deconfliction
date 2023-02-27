
import math
import numpy as np

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


  def economic_dispatch(P_sub, SynchronousMachines, P_def, price):
    P_sync_available = 0.0
    for name, sync in SynchronousMachines.items():
      # avail = math.sqrt(sync['ratedS']**2 - (sync['kW']**2 + sync['kVar']**2))
      sync['P_available'] = math.sqrt(sync['ratedS'] ** 2 - sync['kVar'] ** 2)
      P_sync_available += sync['P_available']

    # F_sub = P_sub * price
    # F_sync = 0.25 * P_sync ** 2 + 30 * P_sync_available + 150
    # Taking derivative and equating.
    # price = 0.00015 * P_sync_available + 0.0267......(1)
    # P_sub + P_sync_available = P_def.................(2)
    # Solving (1) and (2)
    if P_sub > 0:
      P_sync = min((price - 0.0152) / 0.00052, P_def)
      P_sub = P_def - P_sync
    else:
      P_sync = min(P_sync_available, P_def)
    print('Grid Power: ' + str(round(P_sub, 4)), flush=True)

    if P_sync_available + P_sub >= P_def:
      for name, sync in SynchronousMachines.items():
        P_dis = P_sync * sync['P_available'] / P_sync_available
        print('SynchronousMachine name: ' + name + ', P_dis: ' + str(round(P_dis, 4)), flush=True)
    else:
      for name, sync in SynchronousMachines.items():
        P_dis = sync['P_available']
        print('SynchronousMachine name: ' + name + ', P_dis: ' + str(round(P_dis,4)), flush=True)


