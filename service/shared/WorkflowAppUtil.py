
import math
import numpy as np

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime


class WorkflowAppUtil:
  """ Class for workflow-based competing app utility functions
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


  def discharge_SoC(value, mrid, Batteries, deltaT):
    return 1/Batteries[mrid]['eff_d']*value*deltaT/Batteries[mrid]['ratedE']


  def discharge_batteries(Batteries, deltaT):
    for mrid in Batteries:
      Batteries[mrid]['state'] = 'discharging'
      if 'P_batt_d' in Batteries[mrid]:
        Batteries[mrid]['SoC'] -= WorkflowAppUtil.discharge_SoC(
                           Batteries[mrid]['P_batt_d'], mrid, Batteries, deltaT)


  def charge_SoC(value, mrid, Batteries, deltaT):
    return Batteries[mrid]['eff_c']*value*deltaT/Batteries[mrid]['ratedE']


  def charge_batteries(Batteries, deltaT):
    for mrid in Batteries:
      Batteries[mrid]['state'] = 'charging'
      if 'P_batt_c' in Batteries[mrid]:
        Batteries[mrid]['SoC'] += WorkflowAppUtil.charge_SoC(
                           Batteries[mrid]['P_batt_c'], mrid, Batteries, deltaT)


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
    for mrid in Batteries:
      if Batteries[mrid]['SoC'] > 0.2:
        P_batt_d = (Batteries[mrid]['SoC'] - 0.2)*Batteries[mrid]['ratedE'] / (1/Batteries[mrid]['eff_d'] * deltaT)
        Batteries[mrid]['P_batt_d'] = P_batt_d = min(P_batt_d, Batteries[mrid]['ratedkW'])
        P_batt_total += P_batt_d
      else:
        Batteries[mrid]['P_batt_d'] = P_batt_d = 0.0

    if P_batt_total > 0.0:
      if P_ren + P_batt_total > P_load:
        P_def = P_load - P_ren
        for mrid in Batteries:
          Batteries[mrid]['P_batt_d'] = P_def * Batteries[mrid]['P_batt_d'] / P_batt_total

      WorkflowAppUtil.discharge_batteries(Batteries, deltaT)

    # only the profit app passes in price
    if price:
      P_def = P_load - (P_batt_total + P_ren)
      if P_def > 0.0:
        print('Power deficiency: ' + str(round(P_def, 4)), flush=True)
        P_sub = WorkflowAppUtil.economic_dispatch(P_sub, SynchronousMachines, P_def, price)

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


  def batt_to_solution(Batteries, solution):
    for mrid in Batteries:
      solution[mrid] = {}
      if Batteries[mrid]['state'] == 'charging':
        # Gary 6/23/23 Multiply P_batt by 1000 to scale units
        solution[mrid]['P_batt'] = 1000*Batteries[mrid]['P_batt_c']
        #solution[mrid]['P_batt'] = Batteries[mrid]['P_batt_c']
      elif Batteries[mrid]['state'] == 'discharging':
        # Gary 6/23/23 Multiply P_batt by 1000 to scale units
        solution[mrid]['P_batt'] = -1000*Batteries[mrid]['P_batt_d']
        #solution[mrid]['P_batt'] = -Batteries[mrid]['P_batt_d']
      else:
        solution[mrid]['P_batt'] = 0.0

      solution[mrid]['SoC'] = Batteries[mrid]['SoC']

