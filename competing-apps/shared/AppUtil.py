
import math
import numpy as np

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime


class AppUtil:
  """ Class for competing app utility functions
  """

  def getRegulators(sparql_mgr):
    Regulators = {}
    bindings = sparql_mgr.regulator_query()
    #print('regulator_query results bindings: ' + str(bindings), flush=True)
    print('Count of Regulators: ' + str(len(bindings)), flush=True)
    for obj in bindings:
      name = 'RatioTapChanger.' + obj['pname']['value']
      if 'tname' in obj:
        name = 'RatioTapChanger.' + obj['tname']['value']
        Regulators[name] = {}
      else:
        Regulators[name] = {}
      if 'phs' in obj:
        phases = obj['phs']['value']
      else:
        phases = 'ABC'
      Regulators[name]['phase'] = phases
      Regulators[name]['id'] = obj['id']['value']
      Regulators[name]['step'] = float(obj['step']['value'])
      Regulators[name]['highStep'] = float(obj['highStep']['value'])
      Regulators[name]['lowStep'] = float(obj['lowStep']['value'])
      Regulators[name]['increment'] = float(obj['incr']['value'])
      print('Regulator name: ' + name + ', id: ' + Regulators[name]['id'] + ', phase: ' + Regulators[name]['phase'] + ', step: ' + str(round(Regulators[name]['step'],4)), flush=True)

    return Regulators


  def getBatteries(sparql_mgr):
    Batteries = {}
    bindings = sparql_mgr.battery_query()
    #print('battery_query results bindings: ' + str(bindings), flush=True)
    print('Count of Batteries: ' + str(len(bindings)), flush=True)
    idx = 0
    for obj in bindings:
      name = 'BatteryUnit.' + obj['name']['value']
      #bus = obj['bus']['value'].upper()
      Batteries[name] = {}
      Batteries[name]['idx'] = idx
      Batteries[name]['id'] = obj['id']['value']
      Batteries[name]['bus'] = obj['bus']['value']
      Batteries[name]['phase'] = obj['phases']['value']
      Batteries[name]['ratedkW'] = float(obj['ratedS']['value'])/1000.0
      Batteries[name]['prated'] = float(obj['ratedS']['value'])
      Batteries[name]['ratedE'] = float(obj['ratedE']['value'])
      # Shiva HACK
      # Batteries[name]['SoC'] = 0.5
      Batteries[name]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
      # eff_c and eff_d don't come from the query, but they are used throughout
      # and this is a convenient point to assign them with query results
      Batteries[name]['eff'] = 0.975 * 0.86
      Batteries[name]['eff_c'] = 0.975 * 0.86
      Batteries[name]['eff_d'] = 0.975 * 0.86
      print('Battery name: ' + name + ', id: ' + Batteries[name]['id'] + ', ratedE: ' + str(round(Batteries[name]['ratedE'],4)) + ', SoC: ' + str(round(Batteries[name]['SoC'],4)), flush=True)
      idx += 1
    return Batteries


  def getSolarPVs(sparql_mgr):
    SolarPVs = {}
    bindings = sparql_mgr.pv_query()
    print('Count of SolarPV: ' + str(len(bindings)), flush=True)
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

