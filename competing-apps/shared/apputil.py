
import math
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime


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


  def dispatch_DGSs(Batteries, SynchronousMachines, deltaT, P_load, P_ren, P_sub, price=None):
    P_batt_total = 0.0
    for name in Batteries:
      if Batteries[name]['SoC'] > 0.2:
        P_batt_d = (Batteries[name]['SoC'] - 0.2)*Batteries[name]['ratedE'] / (1/Batteries[name]['eff_d'] * deltaT)
        Batteries[name]['P_batt_d'] = P_batt_d = min(P_batt_d, Batteries[name]['ratedkW'])
        P_batt_total += P_batt_d
      else:
        Batteries[name]['P_batt_d'] = P_batt_d = 0.0

    if P_batt_total > 0.0:
      if P_ren + P_batt_total > P_load:
        P_def = P_load - P_ren
        for name in Batteries:
          Batteries[name]['P_batt_d'] = P_def * Batteries[name]['P_batt_d'] / P_batt_total

      AppUtil.discharge_batteries(Batteries, deltaT)

    # only the profit app passes in price
    if price:
      P_def = P_load - (P_batt_total + P_ren)
      if P_def > 0.0:
        print('Power deficiency: ' + str(round(P_def, 4)), flush=True)
        P_sub = AppUtil.economic_dispatch(P_sub, SynchronousMachines, P_def, price)

    else:
      if P_batt_total + P_ren + P_sub <= P_load:
        P_def = P_load - (P_batt_total + P_ren + P_sub)
        print('Power deficiency: ' + str(round(P_def,4)), flush=True)

        P_sync_available = 0.0
        for name, sync in SynchronousMachines.items():
          #avail = math.sqrt(sync['ratedS']**2 - (sync['kW']**2 + sync['kVar']**2))
          sync['P_available'] = math.sqrt(sync['ratedS']**2 - sync['kVar']**2)
          P_sync_available += sync['P_available']
        print('SynchronousMachines available power: ' + str(round(P_sync_available,4)), flush=True)

        if P_sync_available > P_def:
          for name, sync in SynchronousMachines.items():
            P_dis = P_def*sync['P_available']/P_sync_available
            print('SynchronousMachine name: ' + name + ', P_dis: ' + str(round(P_dis,4)), flush=True)
        else:
          for name, sync in SynchronousMachines.items():
            P_dis = sync['P_available']
            print('SynchronousMachine name: ' + name + ', P_dis: ' + str(round(P_dis,4)), flush=True)


  def to_datetime(time):
    return datetime(1966, 8, 1, (time-1)//4, 15*((time-1) % 4), 0)


  def make_plots(title, prefix, Batteries, t_plot, p_batt_plot, soc_plot):
    for name in Batteries:
      plt.figure()
      fig, ax = plt.subplots()
      plt.title(title + ' P_batt:  ' + name, pad=15.0)
      plt.plot(t_plot, p_batt_plot[name])
      ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
      plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
      plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
      plt.xlabel('Time')
      plt.ylabel('P_batt  (kW)')
      plt.savefig('output/' + prefix + '_p_batt_' + name + '.png')
      #plot.show()

      plt.figure()
      fig, ax = plt.subplots()
      plt.title(title + ' SoC:  ' + name, pad=15.0)
      plt.plot(t_plot, soc_plot[name])
      ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
      plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
      plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
      plt.xlabel('Time')
      plt.ylabel('Battery SoC')
      plt.savefig('output/' + prefix + '_soc_' + name + '.png')
      #plot.show()


