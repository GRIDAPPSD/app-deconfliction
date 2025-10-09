# Copyright (c) 2025, Battelle Memorial Institute All rights reserved.
# Battelle Memorial Institute (hereinafter Battelle) hereby grants permission
# to any person or entity lawfully obtaining a copy of this software and
# associated documentation files (hereinafter the Software) to redistribute and
# use the Software in source and binary forms, with or without modification.
# Such person or entity may use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and may permit others to do
# so, subject to the following conditions:
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimers.
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# Other than as used herein, neither the name Battelle Memorial Institute or
# Battelle may be used in any form whatsoever without the express written
# consent of Battelle.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL BATTELLE OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# General disclaimer for use with OSS licenses
#
# This material was prepared as an account of work sponsored by an agency of
# the United States Government. Neither the United States Government nor the
# United States Department of Energy, nor Battelle, nor any of their employees,
# nor any jurisdiction or organization that has cooperated in the development
# of these materials, makes any warranty, express or implied, or assumes any
# legal liability or responsibility for the accuracy, completeness, or
# usefulness or any information, apparatus, product, software, or process
# disclosed, or represents that its use would not infringe privately owned
# rights.
#
# Reference herein to any specific commercial product, process, or service by
# trade name, trademark, manufacturer, or otherwise does not necessarily
# constitute or imply its endorsement, recommendation, or favoring by the
# United States Government or any agency thereof, or Battelle Memorial
# Institute. The views and opinions of authors expressed herein do not
# necessarily state or reflect those of the United States Government or any
# agency thereof.
#
# PACIFIC NORTHWEST NATIONAL LABORATORY operated by BATTELLE for the
# UNITED STATES DEPARTMENT OF ENERGY under Contract DE-AC05-76RL01830
# ------------------------------------------------------------------------------

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import dates as md
from matplotlib.ticker import MaxNLocator
from datetime import datetime

# need to do some magic with the time axis if it's not a realtime simulation
realtimeFlag = False

plotDPI = 200
labelSize = 16
legendSize = 14
tickSize = 12
simColor = 'black'
resilColor = 'red'
maxLocalColor = 'green'
cvrColor = 'blue'
withCoopColor = 'cyan'
withoutCoopColor = 'magenta'
legendLoc = 'lower right'
legendProp = {'weight': 'bold', 'size': legendSize}

def to_datetime(time):
  return datetime(1966, 8, 1, (int(time)-1)//4, 15*((int(time)-1) % 4), 0)


def make_p_batt_plots(title, prefix, Batteries, t_plot_w, p_batt_plot_w, t_plot_wo, p_batt_plot_wo):
  for name in Batteries:
    if len(t_plot_w) != len(p_batt_plot_w[name]):
      print('*** Mismatched data points for cooperation plot ' + title + ' P_batt ' + name + ', time len: ' + str(len(t_plot_w)) + ', p_batt len: ' + str(len(p_batt_plot_w[name])), flush=True)

    if len(t_plot_wo) != len(p_batt_plot_wo[name]):
      print('*** Mismatched data points for no cooperation plot ' + title + ' P_batt ' + name + ', time len: ' + str(len(t_plot_wo)) + ', p_batt len: ' + str(len(p_batt_plot_wo[name])), flush=True)

    batname = name[12:] # extract just the name for tidier plots
    plt.figure(dpi=plotDPI)
    #plt.title(title + ' P_batt:  ' + batname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    if realtimeFlag:
      plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
    else:
      plt.xlim([0, 24])
      plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
      #plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

    plt.yticks(fontweight='bold', fontsize=tickSize)
    plt.ylabel('BESS Output (kW)', fontweight='bold', fontsize=labelSize)
    plt.plot(t_plot_w[:len(p_batt_plot_w[name])], p_batt_plot_w[name], color=withCoopColor, label='With Cooperation')
    plt.plot(t_plot_wo[:len(p_batt_plot_wo[name])], p_batt_plot_wo[name], color=withoutCoopColor, label='Without Cooperation')
    plt.legend(prop=legendProp, loc=legendLoc)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('log/' + prefix + '_p_batt_' + batname + '.png')
    #plot.show()
    plt.close()


def make_soc_plots(title, prefix, Batteries, t_plot_w, soc_plot_w, t_plot_wo, soc_plot_wo):
  for name in Batteries:
    if len(t_plot_w) != len(soc_plot_w[name]):
      print('*** Mismatched data points for cooperation plot ' + title + ' SoC ' + name + ', time len: ' + str(len(t_plot_w)) + ', soc len: ' + str(len(soc_plot_w[name])), flush=True)

    if len(t_plot_wo) != len(soc_plot_wo[name]):
      print('*** Mismatched data points for no cooperation plot ' + title + ' SoC ' + name + ', time len: ' + str(len(t_plot_wo)) + ', soc len: ' + str(len(soc_plot_wo[name])), flush=True)

    batname = name[12:] # extract just the name for tidier plots
    plt.figure(dpi=plotDPI)
    #plt.title(title + ' SoC:  ' + batname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    if realtimeFlag:
      plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
    else:
      plt.xlim([0, 24])
      plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
      #plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

    plt.yticks(fontweight='bold', fontsize=tickSize)
    plt.ylabel('BESS Output (SoC)', fontweight='bold', fontsize=labelSize)
    plt.plot(t_plot_w[:len(soc_plot_w[name])], soc_plot_w[name], color=withCoopColor, label='With Cooperation')
    plt.plot(t_plot_wo[:len(soc_plot_wo[name])], soc_plot_wo[name], color=withoutCoopColor, label='Without Cooperation')
    plt.legend(prop=legendProp, loc=legendLoc)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('log/' + prefix + '_soc_' + batname + '.png')
    #plot.show()
    plt.close()


def make_reg_plots(title, prefix, Regulators, t_plot_w, reg_plot_w, t_plot_wo, reg_plot_wo):
  for name in Regulators:
    if len(t_plot_w) != len(reg_plot_w[name]):
      print('*** Mismatched data points for cooperation plot ' + title + ' ' + name + ', time len: ' + str(len(t_plot_w)) + ', reg len: ' + str(len(reg_plot_w[name])), flush=True)

    if len(t_plot_wo) != len(reg_plot_wo[name]):
      print('*** Mismatched data points for no cooperation plot ' + title + ' ' + name + ', time len: ' + str(len(t_plot_wo)) + ', reg len: ' + str(len(reg_plot_wo[name])), flush=True)

    regname = name[16:] # extract just the name for tidier plots
    plt.figure(dpi=plotDPI)
    #plt.title(title + ' Tap Pos:  ' + regname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    if realtimeFlag:
      plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
    else:
      plt.xlim([0, 24])
      plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
      plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

    plt.ylim([-16, 16])
    plt.yticks([-16, -12, -8, -4, 0, 4, 8, 12, 16], fontweight='bold', fontsize=tickSize)
    plt.ylabel('Regulator Taps', fontweight='bold', fontsize=labelSize)
    plt.plot(t_plot_w[:len(reg_plot_w[name])], reg_plot_w[name], color=withCoopColor, label='With Cooperation')
    plt.plot(t_plot_wo[:len(reg_plot_wo[name])], reg_plot_wo[name], color=withoutCoopColor, label='Without Cooperation')
    plt.legend(prop=legendProp, loc=legendLoc)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('log/' + prefix + '_tap_' + regname + '.png')
    #plot.show()
    plt.close()


def make_p_pv_plots(title, prefix, SolarPVs, t_plot_w, p_pv_plot_w, t_plot_wo, p_pv_plot_wo):
  for name in SolarPVs:
    # just bail if there is no SolarPV data
    if len(p_pv_plot_w[name]) == 0:
      return

    if len(t_plot_w) != len(p_pv_plot_w[name]):
      print('*** Mismatched data points for cooperation plot ' + title + ' p_pv ' + name + ', time len: ' + str(len(t_plot_w)) + ', p_pv len: ' + str(len(p_pv_plot_w[name])), flush=True)

    if len(t_plot_wo) != len(p_pv_plot_wo[name]):
      print('*** Mismatched data points for no cooperation plot ' + title + ' p_pv ' + name + ', time len: ' + str(len(t_plot_wo)) + ', p_pv len: ' + str(len(p_pv_plot_wo[name])), flush=True)

    pvname = name[17:] # extract just the name for tidier plots
    plt.figure(dpi=plotDPI)
    #plt.title(title + ' p_pv:  ' + pvname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    if realtimeFlag:
      plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
    else:
      plt.xlim([0, 24])
      plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
      #plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

    plt.yticks(fontweight='bold', fontsize=tickSize)
    plt.ylabel('PV Output (kW)', fontweight='bold', fontsize=labelSize)
    plt.plot(t_plot_w[:len(p_pv_plot_w[name])], p_pv_plot_w[name], color=withCoopColor, label='With Cooperation')
    plt.plot(t_plot_wo[:len(p_pv_plot_wo[name])], p_pv_plot_wo[name], color=withoutCoopColor, label='Without Cooperation')
    plt.legend(prop=legendProp, loc=legendLoc)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('log/' + prefix + '_p_pv_' + pvname + '.png')
    #plot.show()
    plt.close()


def make_q_pv_plots(title, prefix, SolarPVs, t_plot_w, q_pv_plot_w, t_plot_wo, q_pv_plot_wo):
  for name in SolarPVs:
    # just bail if there is no SolarPV data
    if len(q_pv_plot_w[name]) == 0:
      return

    if len(t_plot_w) != len(q_pv_plot_w[name]):
      print('*** Mismatched data points for cooperation plot ' + title + ' q_pv ' + name + ', time len: ' + str(len(t_plot_w)) + ', q_pv len: ' + str(len(q_pv_plot_w[name])), flush=True)

    if len(t_plot_wo) != len(q_pv_plot_wo[name]):
      print('*** Mismatched data points for no cooperation plot ' + title + ' q_pv ' + name + ', time len: ' + str(len(t_plot_wo)) + ', q_pv len: ' + str(len(q_pv_plot_wo[name])), flush=True)

    pvname = name[17:] # extract just the name for tidier plots
    plt.figure(dpi=plotDPI)
    #plt.title(title + ' q_pv:  ' + pvname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    if realtimeFlag:
      plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
    else:
      plt.xlim([0, 24])
      plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
      #plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

    plt.yticks(fontweight='bold', fontsize=tickSize)
    plt.ylabel('PV Output (kVAR)', fontweight='bold', fontsize=labelSize)
    plt.plot(t_plot_w[:len(q_pv_plot_w[name])], q_pv_plot_w[name], color=withCoopColor, label='With Cooperation')
    plt.plot(t_plot_wo[:len(q_pv_plot_wo[name])], q_pv_plot_wo[name], color=withoutCoopColor, label='Without Cooperation')
    plt.legend(prop=legendProp, loc=legendLoc)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('log/' + prefix + '_q_pv_' + pvname + '.png')
    #plot.show()
    plt.close()


def _main():
  print('Starting plotter...', flush=True)

  matplotlib.use('agg')

  Batteries = ['BatteryUnit.battery1','BatteryUnit.battery2','BatteryUnit.battery3','BatteryUnit.battery4','BatteryUnit.battery5']

  Regulators = ['RatioTapChanger.reg1a','RatioTapChanger.reg2a','RatioTapChanger.reg3a','RatioTapChanger.reg3c','RatioTapChanger.reg4a','RatioTapChanger.reg4b','RatioTapChanger.reg4c']

  SolarPVs = ['PhotovoltaicUnit.dg_12','PhotovoltaicUnit.dg_18','PhotovoltaicUnit.dg_30','PhotovoltaicUnit.dg_36','PhotovoltaicUnit.dg_42','PhotovoltaicUnit.dg_48','PhotovoltaicUnit.dg_54','PhotovoltaicUnit.dg_6','PhotovoltaicUnit.dg_60','PhotovoltaicUnit.dg_66','PhotovoltaicUnit.dg_72','PhotovoltaicUnit.dg_78','PhotovoltaicUnit.dg_84','PhotovoltaicUnit.dg_90']

  # _w is with cooperation; _wo is without cooperation--just rules + optimize
  t_plot_w = []
  p_batt_plot_w = {}
  soc_plot_w = {}
  reg_plot_w = {}
  p_pv_plot_w = {}
  q_pv_plot_w = {}

  t_plot_wo = []
  p_batt_plot_wo = {}
  soc_plot_wo = {}
  reg_plot_wo = {}
  p_pv_plot_wo = {}
  q_pv_plot_wo = {}

  for batt in Batteries:
    p_batt_plot_w[batt] = []
    soc_plot_w[batt] = []
    p_batt_plot_wo[batt] = []
    soc_plot_wo[batt] = []

  for reg in Regulators:
    reg_plot_w[reg] = []
    reg_plot_wo[reg] = []

  for pv in SolarPVs:
    p_pv_plot_w[pv] = []
    q_pv_plot_w[pv] = []
    p_pv_plot_wo[pv] = []
    q_pv_plot_wo[pv] = []

  # Jan 1, midnight timestamp:
  timex_start = 1704067200.0

  app = 'SIMULATION'
  prefix = 'concurrent'

  simhits = 0
  with open('log/threeplots/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == app:
        simhits += 1
        t_plot_w.append((float(tokens[2]) - timex_start)/3600.0)

        start = 3
        finish = start + len(Batteries)*3
        for it in range(start, finish, 3):
          batt = tokens[it]
          p_batt_plot_w[batt].append(float(tokens[it+1])/1000.0)
          soc_plot_w[batt].append(float(tokens[it+2]))

        start = finish
        finish = start + len(Regulators)*2
        for it in range(start, finish, 2):
          reg = tokens[it]
          reg_plot_w[reg].append(int(tokens[it+1]))

        start = finish
        finish = start + len(SolarPVs)*2
        for it in range(start, finish, 2):
          pv = tokens[it]
          cmplx = complex(tokens[it+1])/1000.0
          p_pv_plot_w[pv].append(cmplx.real)
          q_pv_plot_w[pv].append(cmplx.imag)

  print(app + ' with cooperation hits: ' + str(simhits), flush=True)

  simhits = 0
  with open('log/threeplots_nocoop/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == app:
        simhits += 1
        t_plot_wo.append((float(tokens[2]) - timex_start)/3600.0)

        start = 3
        finish = start + len(Batteries)*3
        for it in range(start, finish, 3):
          batt = tokens[it]
          p_batt_plot_wo[batt].append(float(tokens[it+1])/1000.0)
          soc_plot_wo[batt].append(float(tokens[it+2]))

        start = finish
        finish = start + len(Regulators)*2
        for it in range(start, finish, 2):
          reg = tokens[it]
          reg_plot_wo[reg].append(int(tokens[it+1]))

        start = finish
        finish = start + len(SolarPVs)*2
        for it in range(start, finish, 2):
          pv = tokens[it]
          cmplx = complex(tokens[it+1])/1000.0
          p_pv_plot_wo[pv].append(cmplx.real)
          q_pv_plot_wo[pv].append(cmplx.imag)

  print(app + ' without cooperation hits: ' + str(simhits), flush=True)

  app = 'Simulation'

  make_p_batt_plots(app, prefix, Batteries, t_plot_w, p_batt_plot_w, t_plot_wo, p_batt_plot_wo)
  make_soc_plots(app, prefix, Batteries, t_plot_w, soc_plot_w, t_plot_wo, soc_plot_wo)
  make_reg_plots(app, prefix, Regulators, t_plot_w, reg_plot_w, t_plot_wo, reg_plot_wo)
  make_p_pv_plots(app, prefix, SolarPVs, t_plot_w, p_pv_plot_w, t_plot_wo, p_pv_plot_wo)
  make_q_pv_plots(app, prefix, SolarPVs, t_plot_w, q_pv_plot_w, t_plot_wo, q_pv_plot_wo)

  t_plot_w.clear()
  t_plot_wo.clear()

  for batt in Batteries:
    p_batt_plot_w[batt].clear()
    soc_plot_w[batt].clear()
    p_batt_plot_wo[batt].clear()
    soc_plot_wo[batt].clear()

  for reg in Regulators:
    reg_plot_w[reg].clear()
    reg_plot_wo[reg].clear()

  for pv in SolarPVs:
    p_pv_plot_w[pv].clear()
    q_pv_plot_w[pv].clear()
    p_pv_plot_wo[pv].clear()
    q_pv_plot_wo[pv].clear()

  print('Goodbye!')


if __name__ == "__main__":
  _main()

