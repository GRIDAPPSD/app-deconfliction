
import math
import numpy as np

import cimgraph.data_profile.cimhub_2023 as cim
import MethodUtil


def prlog(msg, logFile):
  try:
    print(msg, flush=True)
    if logFile != None:
      with open(logFile, 'a') as flog:
        flog.write(msg + '\n')
  except:
    pass


class AppUtil:
  """ Class for competing app utility functions
  """

  def getRegulators(sparql_mgr):
    RegulatorMap = {}
    Regulators = {}
    bindings = sparql_mgr.regulator_query()
    #prlog('regulator_query results bindings: ' + str(bindings), sparql_mgr.logFile)
    prlog('\nCount of Regulators: ' + str(len(bindings)), sparql_mgr.logFile)
    for obj in bindings:
      devid = obj['rid']
      eqid = obj['pid']
      RegulatorMap[eqid] = devid

      name = 'RatioTapChanger.' + obj['pname']
      if 'tname' in obj:
        eqid = obj['tid']
        name = 'RatioTapChanger.' + obj['tname']
      if 'phs' in obj:
        phases = obj['phs']
      else:
        phases = 'ABC'

      Regulators[devid] = {}
      Regulators[devid]['phase'] = phases
      Regulators[devid]['name'] = name
      Regulators[devid]['step'] = int(obj['step'])
      Regulators[devid]['highStep'] = int(obj['highStep'])
      Regulators[devid]['lowStep'] = int(obj['lowStep'])
      Regulators[devid]['increment'] = float(obj['incr'])
      Regulators[devid]['measid'] = obj['measid']
      prlog('Regulator devid: ' + devid + ', name: ' + name + ', phase: ' + Regulators[devid]['phase'] + ', step: ' + str(Regulators[devid]['step']), sparql_mgr.logFile)
      MethodUtil.DeviceToName[devid] = name
      MethodUtil.NameToDevice[name] = devid

    return Regulators


  def getCombineRegulators(sparql_mgr):
    Regulators = {}
    RegIdx = {}
    bindings = sparql_mgr.regulator_combine_query()
    prlog('\nCount of Combine Regulators: ' + str(len(bindings)), sparql_mgr.logFile)
    reg_idx = 0
    for obj in bindings:
      devid = obj['rid']
      pname = obj['pname']
      if 'phs' in obj:
        phases = obj['phs']
      else:
        phases = 'ABC'

      if 'tname' in obj:
        #mrid = obj['tid']['value']
        name = 'RatioTapChanger.' + obj['tname']
      else:
        #mrid = obj['pid']['value']
        name = 'RatioTapChanger.' + pname

      Regulators[devid] = {'pname': pname, 'name': name, \
                           'idx': reg_idx, 'phases': phases}
      MethodUtil.DeviceToName[devid] = name
      MethodUtil.NameToDevice[name] = devid

      for char in phases:
        RegIdx[pname+'.'+char] = reg_idx

      reg_idx += 1

    return (Regulators, RegIdx)


  def getBatteries(sparql_mgr):
    BatteryMap = {}
    BatteriesInfo = {}
    BatteriesBus = {}
    bindings = sparql_mgr.battery_query()
    prlog('battery_query results bindings: ' + str(bindings), sparql_mgr.logFile)
    prlog('\nCount of Batteries: ' + str(len(bindings)), sparql_mgr.logFile)
    idx = 0
    for obj in bindings:
      devid = obj['id']
      eqid = obj['pecid']
      BatteryMap[eqid] = devid

      BatteriesInfo[devid] = {}
      name = 'BatteryUnit.' + obj['name']
      BatteriesInfo[devid]['name'] = name
      BatteriesInfo[devid]['idx'] = idx
      bus = obj['bus']
      BatteriesInfo[devid]['bus'] = bus
      phase = obj['phases'][0]
      BatteriesInfo[devid]['phase'] = phase
      BatteriesInfo[devid]['ratedkW'] = float(obj['ratedS'])/1000.0
      BatteriesInfo[devid]['prated'] = float(obj['ratedS'])
      BatteriesInfo[devid]['ratedE'] = float(obj['ratedE'])
      BatteriesInfo[devid]['SoC'] = float(obj['storedE'])/float(obj['ratedE'])
      BatteriesInfo[devid]['P_bat_measid'] = obj["P_batt_measid"]
      BatteriesInfo[devid]['SoC_measid'] = obj["SoC_batt_measid"]
      # eff_c and eff_d don't come from the query, but they are used throughout
      # and this is a convenient point to assign them along with query results
      BatteriesInfo[devid]['eff'] = 0.975 * 0.86
      BatteriesInfo[devid]['eff_c'] = 0.975 * 0.86
      BatteriesInfo[devid]['eff_d'] = 0.975 * 0.86
      prlog('Battery devid: ' + devid + ', name: ' + name + ', ratedE: ' + str(round(BatteriesInfo[devid]['ratedE'],4)) + ', SoC: ' + str(round(BatteriesInfo[devid]['SoC'],4)), sparql_mgr.logFile)
      # need a bus-indexed batteries dictionary as well
      BatteriesBus[bus] = {}
      BatteriesBus[bus]['mrid'] = devid
      BatteriesBus[bus]['phase'] = phase

      idx += 1
      MethodUtil.DeviceToName[devid] = name
      MethodUtil.NameToDevice[name] = devid

    return (BatteriesInfo, BatteriesBus)


  def getEnergyConsumers(sparql_mgr):
    #feeder_power = {'p': {'A': 0, 'B': 0, 'C': 0},
    #                'q': {'A': 0, 'B': 0, 'C': 0}}
    EnergyConsumers = {}
    bindings = sparql_mgr.energyconsumer_query()
    for obj in bindings:
      bus = obj['bus'].upper()
      if bus not in EnergyConsumers:
        EnergyConsumers[bus] = {}
        EnergyConsumers[bus]['kW'] = {}
        EnergyConsumers[bus]['kVar'] = {}
        EnergyConsumers[bus]['measid'] = {}

      phases = obj['phases']
      if phases == '':
        pval = float(obj['p']) / 3.0
        qval = float(obj['q']) / 3.0
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
        pval = float(obj['p'])
        qval = float(obj['q'])
        EnergyConsumers[bus]['kW'][phases] = pval
        EnergyConsumers[bus]['kVar'][phases] = qval
        #feeder_power['p'][phases] += pval
        #feeder_power['q'][phases] += qval

      EnergyConsumers[bus]['measid'] = obj['measid']

    return EnergyConsumers


  def getSolarPVs(sparql_mgr):
    SolarPVsInfo = {}
    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    prlog('\nCount of SolarPV: ' + str(len(bindings)), sparql_mgr.logFile)
    idx = 0
    for obj in bindings:
      name = 'PhotovoltaicUnit.' + obj['name']
      bus = obj['bus'].upper()
      devid = obj['id']
      #ratedU = float(obj['ratedU'])
      ratedS = float(obj['ratedS'])
      SolarPVsInfo[bus] = {}
      SolarPVsInfo[bus]['kW'] = float(obj['p'])/1000.0
      SolarPVsInfo[bus]['kVar'] = float(obj['q'])/1000.0
      SolarPVsInfo[bus]['p'] = float(obj['p'])
      SolarPVsInfo[bus]['phase'] = obj['phases'][0]
      SolarPVsInfo[bus]['ratedS'] = ratedS
      SolarPVsInfo[bus]['mrid'] = devid
      SolarPVsInfo[bus]['name'] = name
      SolarPVsInfo[bus]['idx'] = idx
      prlog('SolarPV name: ' + name + ', kW: ' + str(SolarPVsInfo[bus]['kW']) + ', kVar: ' + str(SolarPVsInfo[bus]['kVar']), sparql_mgr.logFile)
      SolarPVs[devid] = {}
      SolarPVs[devid]['PQ_pv_inv'] = None
      SolarPVs[devid]['idx'] = idx
      SolarPVs[devid]['ratedS'] = ratedS
      idx += 1
      MethodUtil.DeviceToName[devid] = name
      MethodUtil.NameToDevice[name] = devid
      SolarPVsInfo[bus]['measid'] = obj['measid']

    return (SolarPVsInfo, SolarPVs)


  def getEnergySource(sparql_mgr):
    EnergySource = {}
    bindings = sparql_mgr.energysource_query()
    for obj in bindings:
      EnergySource['name'] = obj['name']['value']
      EnergySource['bus'] = obj['bus']['value'].upper()
      EnergySource['basev'] = float(obj['basev']['value'])
      EnergySource['nomv'] = float(obj['basev']['value'])

    return EnergySource

