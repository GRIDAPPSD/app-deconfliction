
import math
import numpy as np

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime

import MethodUtil


class AppUtil:
  """ Class for competing app utility functions
  """

  def getRegulators(sparql_mgr):
    Regulators = {}
    bindings = sparql_mgr.regulator_query()
    #print('regulator_query results bindings: ' + str(bindings), flush=True)
    print('\nCount of Regulators: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      mrid = obj['pid']['value']
      name = 'RatioTapChanger.' + obj['pname']['value']
      if 'tname' in obj:
        mrid = obj['tid']['value']
        name = 'RatioTapChanger.' + obj['tname']['value']
      if 'phs' in obj:
        phases = obj['phs']['value']
      else:
        phases = 'ABC'
      Regulators[mrid] = {}
      Regulators[mrid]['phase'] = phases
      Regulators[mrid]['name'] = name
      Regulators[mrid]['rid'] = obj['rid']['value']
      Regulators[mrid]['step'] = int(obj['step']['value'])
      Regulators[mrid]['highStep'] = int(obj['highStep']['value'])
      Regulators[mrid]['lowStep'] = int(obj['lowStep']['value'])
      Regulators[mrid]['increment'] = float(obj['incr']['value'])
      print('Regulator mrid: ' + mrid + ', name: ' + name + ', rid: ' + Regulators[mrid]['rid'] + ', phase: ' + Regulators[mrid]['phase'] + ', step: ' + str(Regulators[mrid]['step']), flush=True)
      MethodUtil.DeviceToName[mrid] = name
      MethodUtil.NameToDevice[name] = mrid

    # Add measid key to Regulators for matching sim measurements
    objs = sparql_mgr.obj_meas_export('PowerTransformer')
    print('Count of PowerTransformer Meas: ' + str(len(objs)), flush=True)
    matches = 0
    attempts = 0
    for item in objs:
      if item['type']=='Pos':
        #print('Attempting to match PowerTransformer measurement: ' + str(item), flush=True)
        attempts += 1
        # GDB 6/25/24: This is ideally how matches should be done, but this
        # doesn't work with our current regulator query so improvising...
        #if item['eqid'] in Regulators:
        #  Regulators[item['eqid']]['measid'] = item['measid']
        #  matches += 1
        nameToMatch = 'RatioTapChanger.' + item['eqname']
        for mrid in Regulators:
          shortName = Regulators[mrid]['name'][:-1]
          if Regulators[mrid]['name'] == nameToMatch:
            Regulators[mrid]['measid'] = item['measid']
            matches += 1
            #print('Matched full name Regulator dictionary item: ' + str(Regulators[mrid]), flush=True)
            break
          elif shortName==nameToMatch and \
               Regulators[mrid]['phase']==item['phases']:
            Regulators[mrid]['measid'] = item['measid']
            matches += 1
            #print('Matched short name Regulator dictionary item: ' + str(Regulators[mrid]), flush=True)
            break

    print('Matching Regulator measurement attempts: ' + str(attempts) + ', matches: ' + str(matches), flush=True)

    return Regulators


  def getCombineRegulators(sparql_mgr):
    Regulators = {}
    RegIdx = {}
    bindings = sparql_mgr.regulator_combine_query()
    print('\nCount of Combine Regulators: ' + str(len(bindings)), flush=True)
    reg_idx = 0
    for obj in bindings:
      pname = obj['pname']['value']
      if 'phs' in obj:
        phases = obj['phs']['value']
      else:
        phases = 'ABC'

      if 'tname' in obj:
        mrid = obj['tid']['value']
        name = 'RatioTapChanger.' + obj['tname']['value']
      else:
        mrid = obj['pid']['value']
        name = 'RatioTapChanger.' + pname

      Regulators[mrid] = \
              {'pname': pname, 'name': name, 'idx': reg_idx, 'phases': phases}
      MethodUtil.DeviceToName[mrid] = name
      MethodUtil.NameToDevice[name] = mrid

      for char in phases:
        RegIdx[pname+'.'+char] = reg_idx

      reg_idx += 1

    return (Regulators, RegIdx)


  def getBatteries(sparql_mgr):
    Batteries = {}
    bindings = sparql_mgr.battery_query()
    #print('battery_query results bindings: ' + str(bindings), flush=True)
    print('\nCount of Batteries: ' + str(len(bindings)), flush=True)
    idx = 0
    for obj in bindings:
      mrid = obj['pecid']['value']
      name = 'BatteryUnit.' + obj['name']['value']
      Batteries[mrid] = {}
      Batteries[mrid]['idx'] = idx
      Batteries[mrid]['name'] = name
      Batteries[mrid]['id'] = obj['id']['value']
      Batteries[mrid]['bus'] = obj['bus']['value']
      Batteries[mrid]['phase'] = obj['phases']['value']
      Batteries[mrid]['ratedkW'] = float(obj['ratedS']['value'])/1000.0
      Batteries[mrid]['prated'] = float(obj['ratedS']['value'])
      Batteries[mrid]['ratedE'] = float(obj['ratedE']['value'])
      Batteries[mrid]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
      # eff_c and eff_d don't come from the query, but they are used throughout
      # and this is a convenient point to assign them along with query results
      Batteries[mrid]['eff'] = 0.975 * 0.86
      Batteries[mrid]['eff_c'] = 0.975 * 0.86
      Batteries[mrid]['eff_d'] = 0.975 * 0.86
      print('Battery mrid: ' + mrid + ', name: ' + name + ', ratedE: ' + str(round(Batteries[mrid]['ratedE'],4)) + ', SoC: ' + str(round(Batteries[mrid]['SoC'],4)), flush=True)
      idx += 1
      MethodUtil.DeviceToName[mrid] = name
      MethodUtil.NameToDevice[name] = mrid

    # Add measid key to Batteries for matching sim measurements
    objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    for item in objs:
      if item['eqid'] in Batteries:
        if item['type'] == 'VA':
          Batteries[item['eqid']]['P_batt_measid'] = item['measid']
        elif item['type'] == 'SoC':
          Batteries[item['eqid']]['SoC_measid'] = item['measid']

    return Batteries


  def getEnergyConsumers(sparql_mgr):
    #feeder_power = {'p': {'A': 0, 'B': 0, 'C': 0},
    #                'q': {'A': 0, 'B': 0, 'C': 0}}
    EnergyConsumers = {}
    bindings = sparql_mgr.energyconsumer_query()
    for obj in bindings:
      bus = obj['bus']['value'].upper()
      if bus not in EnergyConsumers:
        EnergyConsumers[bus] = {}
        EnergyConsumers[bus]['kW'] = {}
        EnergyConsumers[bus]['kVar'] = {}
        EnergyConsumers[bus]['measid'] = {}

      phases = obj['phases']['value']
      if phases == '':
        pval = float(obj['p']['value']) / 3.0
        qval = float(obj['q']['value']) / 3.0
        EnergyConsumers[bus]['kW']['A'] = pval
        EnergyConsumers[bus]['kW']['B'] = pval
        EnergyConsumers[bus]['kW']['C'] = pval
        EnergyConsumers[bus]['kVar']['A'] = qval
        EnergyConsumers[bus]['kVar']['B'] = qval
        EnergyConsumers[bus]['kVar']['C'] = qval
        #feeder_power['p']['A'] += pval
        #feeder_power['p']['B'] += pval
        #feeder_power['p']['C'] += pval
        #feeder_power['q']['A'] += qval
        #feeder_power['q']['B'] += qval
        #feeder_power['q']['C'] += qval
      else:
        pval = float(obj['p']['value'])
        qval = float(obj['q']['value'])
        EnergyConsumers[bus]['kW'][phases] = pval
        EnergyConsumers[bus]['kVar'][phases] = qval
        #feeder_power['p'][phases] += pval
        #feeder_power['q'][phases] += qval

    # Add measid key to EnergyConsumers for matching sim measurements
    objs = sparql_mgr.obj_meas_export('EnergyConsumer')
    print('Count of EnergyConsumers Meas: ' + str(len(objs)), flush=True)
    for item in objs:
      if item['type'] == 'VA':
        EnergyConsumers[item['bus']]['measid'][item['phases']] = item['measid']

    return EnergyConsumers


  def getSolarPVs(sparql_mgr):
    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    print('\nCount of SolarPV: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = obj['name']['value']
      bus = obj['bus']['value'].upper()
      ratedS = float(obj['ratedS']['value'])
      #ratedU = float(obj['ratedU']['value'])
      SolarPVs[bus] = {}
      SolarPVs[bus]['kW'] = float(obj['p']['value'])/1000.0
      SolarPVs[bus]['kVar'] = float(obj['q']['value'])/1000.0
      SolarPVs[bus]['p'] = float(obj['p']['value'])
      SolarPVs[bus]['phase'] = obj['phases']['value']
      SolarPVs[bus]['ratedS'] = float(obj['ratedS']['value'])
      #print('SolarPV name: ' + name + ', kW: ' + str(SolarPVs[name]['kW']) + ', kVar: ' + str(SolarPVs[name]['kVar']), flush=True)

    # Add measid key to SolarPVs for matching sim measurements
    objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
    print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)),
          flush=True)
    for item in objs:
      if item['type']=='VA' and item['bus'] in SolarPVs:
        SolarPVs[item['bus']]['measid'] = item['measid']

    return SolarPVs


  def getEnergySource(sparql_mgr):
    EnergySource = {}
    bindings = sparql_mgr.energysource_query()
    for obj in bindings:
      EnergySource['name'] = obj['name']['value']
      EnergySource['bus'] = obj['bus']['value'].upper()
      EnergySource['basev'] = float(obj['basev']['value'])
      EnergySource['nomv'] = float(obj['basev']['value'])

    return EnergySource


  def contrib_SoC(P_batt, timeDiff, Battery, deltaT):
    if P_batt >= 0:
      return Battery['eff_c']*P_batt*timeDiff*deltaT/Battery['ratedE']
    else:
      return 1/Battery['eff_d']*P_batt*timeDiff*deltaT/Battery['ratedE']


  def to_datetime(time):
    return datetime(1966, 8, 1, (int(time)-1)//4, 15*((int(time)-1) % 4), 0)


  def make_plots(title, prefix, Batteries, t_plot, p_batt_plot, soc_plot):
    matplotlib.use('agg')

    for mrid in Batteries:
      name = MethodUtil.DeviceToName[mrid]
      batname = name[12:] # extract just the name for tidier plots
      plt.figure()
      fig, ax = plt.subplots()
      plt.title(title + ' P_batt:  ' + batname, pad=15.0)
      plt.plot(t_plot, p_batt_plot[mrid])
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
      plt.plot(t_plot, soc_plot[mrid])
      ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
      plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
      plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
      plt.xlabel('Time')
      plt.ylabel('Battery SoC')
      plt.savefig('output/' + prefix + '_soc_' + batname + '.png')
      #plot.show()

