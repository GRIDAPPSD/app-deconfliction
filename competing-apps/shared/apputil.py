
import math
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime


class AppUtil:
  """ Class for competing app utility functions
  """

  def getEnergyConsumers(sparql_mgr):
    EnergyConsumers = {}
    objs = sparql_mgr.obj_dict_export('EnergyConsumer')
    print('Count of EnergyConsumers Dict: ' + str(len(objs)), flush=True)
    for item in objs:
      name = item['IdentifiedObject.name']
      EnergyConsumers[name] = {}
      # Shiva HACK to force battery charging...
      #EnergyConsumers[name]['kW'] = 0.01*float(item['EnergyConsumer.p'])/1000.0
      EnergyConsumers[name]['kW'] = float(item['EnergyConsumer.p'])/1000.0
      EnergyConsumers[name]['kVar'] = float(item['EnergyConsumer.q'])/1000.0
      #print('EnergyConsumer name: ' + name + ', kW: ' + str(EnergyConsumers[name]['kW']) + ', kVar: ' + str(EnergyConsumers[name]['kVar']), flush=True)

    return EnergyConsumers

    
  def getSynchronousMachines(sparql_mgr):
    SynchronousMachines = {}
    objs = sparql_mgr.obj_dict_export('SynchronousMachine')
    print('Count of SynchronousMachines Dict: ' + str(len(objs)), flush=True)
    for item in objs:
      name = item['IdentifiedObject.name']
      SynchronousMachines[name] = {}
      SynchronousMachines[name]['kW'] = float(item['SynchronousMachine.p'])/1000.0
      SynchronousMachines[name]['kVar'] = float(item['SynchronousMachine.q'])/1000.0
      SynchronousMachines[name]['ratedS'] = float(item['SynchronousMachine.ratedS'])/1000.0
      print('SynchronousMachine name: ' + name + ', kW: ' + str(round(SynchronousMachines[name]['kW'],4)) + ', kVar: ' + str(round(SynchronousMachines[name]['kVar'],4)), flush=True)

    return SynchronousMachines


  def getBatteries(sparql_mgr):
    Batteries = {}
    bindings = sparql_mgr.battery_query()
    print('Count of Batteries: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = 'BatteryUnit:' + obj['name']['value']
      #bus = obj['bus']['value'].upper()
      Batteries[name] = {}
      Batteries[name]['ratedkW'] = float(obj['ratedS']['value'])/1000.0
      Batteries[name]['ratedE'] = float(obj['ratedE']['value'])/1000.0
      # Shiva HACK
      Batteries[name]['SoC'] = 0.5
      #Batteries[name]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
      # eff_c and eff_d don't come from the query, but they are used throughout
      # and this is a convenient point to assign them with query results
      Batteries[name]['eff_c'] = 0.975 * 0.86
      Batteries[name]['eff_d'] = 0.975 * 0.86
      print('Battery name: ' + name + ', ratedE: ' + str(round(Batteries[name]['ratedE'],4)) + ', SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)

    return Batteries

  def getSolarPVs(sparql_mgr):
    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    print('Count of SolarPV: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['name']['value']
      #bus = obj['bus']['value'].upper()
      #ratedS = float(obj['ratedS']['value'])
      #ratedU = float(obj['ratedU']['value'])
      SolarPVs[name] = {}
      SolarPVs[name]['kW'] = float(obj['p']['value'])/1000.0
      SolarPVs[name]['kVar'] = float(obj['q']['value'])/1000.0
      #print('SolarPV name: ' + name + ', kW: ' + str(SolarPVs[name]['kW']) + ', kVar: ' + str(SolarPVs[name]['kVar']), flush=True)

    return SolarPVs


  def getKitchenSink(sparql_mgr):
    pass
    # not needed now, but saving for possible future use

    #objs = sparql_mgr.obj_meas_export('EnergyConsumer')
    #print('Count of EnergyConsumers Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('EnergyConsumer: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    #print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('PowerElectronicsConnection: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_dict_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Dict: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_meas_export('LinearShuntCompensator')
    #print('Count of LinearShuntCompensators Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('LinearShuntCompensator: ' + str(item), flush=True)

    #objs = sparql_mgr.obj_meas_export('SynchronousMachine')
    #print('Count of SynchronousMachines Meas: ' + str(len(objs)), flush=True)
    #for item in objs:
    #  print('SynchronousMachine: ' + str(item), flush=True)


  def contrib_SoC(P_batt, timeDiff, Battery, deltaT):
    if P_batt >= 0:
      return Battery['eff_c']*P_batt*timeDiff*deltaT/Battery['ratedE']
    else:
      return 1/Battery['eff_d']*P_batt*timeDiff*deltaT/Battery['ratedE']


  timestampOld = {}
  P_battOld = {}
  SoCOld = {}
  timestampOlder = {}
  P_battOlder = {}
  SoCOlder = {}

  def new_SoC(name, P_batt, timestamp, Batteries, deltaT):
    if name not in AppUtil.timestampOld:
      AppUtil.timestampOlder[name] = AppUtil.timestampOld[name] = timestamp
      AppUtil.P_battOlder[name] = AppUtil.P_battOld[name] = P_batt
      AppUtil.SoCOlder[name] = AppUtil.SoCOld[name] = Batteries[name]['SoX']

    #print('\n~ZZZ new_SoC for device: ' + name + ', timestamp: ' + str(timestamp) + ', P_batt: ' + str(P_batt) + ', SoX: ' + str(Batteries[name]['SoX']), flush=True)
    #print('~ZZZ new_SoC for device: ' + name + ', timestampOld: ' + str(AppUtil.timestampOld[name]) + ', timestampOlder: ' + str(AppUtil.timestampOlder[name]), flush=True)
    #print('~ZZZ new_SoC for device: ' + name + ', P_battOld: ' + str(AppUtil.P_battOld[name]) + ', P_battOlder: ' + str(AppUtil.P_battOlder[name]), flush=True)
    #print('~ZZZ new_SoC for device: ' + name + ', SoCOld: ' + str(AppUtil.SoCOld[name]) + ', SoCOlder: ' + str(AppUtil.SoCOlder[name]), flush=True)

    actual = AppUtil.contrib_SoC(AppUtil.P_battOld[name], timestamp-AppUtil.timestampOld[name], Batteries[name], deltaT)

    projected = AppUtil.contrib_SoC(P_batt, 1, Batteries[name], deltaT)

    Batteries[name]['SoX'] = AppUtil.SoCOlder[name] + actual + projected

    print('~ZZZ new_SoC magic for device: ' + name + ', start SoC: ' + str(AppUtil.SoCOlder[name]) + ', actual SoC contrib: ' + str(actual) + ', projected SoC contrib: ' + str(projected) + ', new SoC: ' + str(Batteries[name]['SoX']), flush=True)

    if timestamp > AppUtil.timestampOld[name]:
      AppUtil.timestampOlder[name] = AppUtil.timestampOld[name]
      AppUtil.P_battOlder[name] = AppUtil.P_battOld[name]
      AppUtil.SoCOlder[name] = AppUtil.SoCOld[name]

    AppUtil.timestampOld[name] = timestamp
    AppUtil.P_battOld[name] = P_batt
    AppUtil.SoCOld[name] = Batteries[name]['SoX']


  def discharge_SoC(value, name, Batteries, deltaT):
    return 1/Batteries[name]['eff_d']*value*deltaT/Batteries[name]['ratedE']


  def discharge_batteries(Batteries, deltaT):
    for name in Batteries:
      Batteries[name]['state'] = 'discharging'
      if 'P_batt_d' in Batteries[name]:
        Batteries[name]['SoC'] -= AppUtil.discharge_SoC(Batteries[name]['P_batt_d'], name,
                                                        Batteries, deltaT)


  def charge_SoC(value, name, Batteries, deltaT):
    return Batteries[name]['eff_c']*value*deltaT/Batteries[name]['ratedE']


  def charge_batteries(Batteries, deltaT):
    for name in Batteries:
      Batteries[name]['state'] = 'charging'
      if 'P_batt_c' in Batteries[name]:
        Batteries[name]['SoC'] += AppUtil.charge_SoC(Batteries[name]['P_batt_c'], name,
                                                     Batteries, deltaT)


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
      batname = name[12:] # extract just the name for tidier plots
      plt.figure()
      fig, ax = plt.subplots()
      plt.title(title + ' P_batt:  ' + batname, pad=15.0)
      plt.plot(t_plot, p_batt_plot[name])
      ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
      plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
      plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
      plt.xlabel('Time')
      plt.ylabel('P_batt  (kW)')
      plt.savefig('output/' + prefix + '_p_batt_' + batname + '.png')
      #plot.show()

      plt.figure()
      fig, ax = plt.subplots()
      plt.title(title + ' SoC:  ' + batname, pad=15.0)
      plt.plot(t_plot, soc_plot[name])
      ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
      plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
      plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
      plt.xlabel('Time')
      plt.ylabel('Battery SoC')
      plt.savefig('output/' + prefix + '_soc_' + batname + '.png')
      #plot.show()


  def batt_to_solution(Batteries, solution):
    for name in Batteries:
      solution[name] = {}
      if Batteries[name]['state'] == 'charging':
        solution[name]['P_batt'] = Batteries[name]['P_batt_c']
      elif Batteries[name]['state'] == 'discharging':
        solution[name]['P_batt'] = -Batteries[name]['P_batt_d']
      else:
        solution[name]['P_batt'] = 0.0

      solution[name]['SoC'] = Batteries[name]['SoC']

