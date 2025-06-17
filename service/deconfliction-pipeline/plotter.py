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


def to_datetime(time):
  return datetime(1966, 8, 1, (int(time)-1)//4, 15*((int(time)-1) % 4), 0)


def make_cm_plot(cm_t_plot, cm_start_plot, cm_rules_plot, cm_coop_plot):
  plt.title('Conflict Metric', pad=15.0)
  #ax = plt.figure().gca()
  #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
  #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
  #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
  plt.xlabel('Time (sec)')
  plt.ylabel('Conflict Metric')
  plt.plot(cm_t_plot, cm_start_plot, label='starting metric')
  plt.plot(cm_t_plot, cm_rules_plot, label='post-rules')
  plt.plot(cm_t_plot, cm_coop_plot, label='post-cooperation')
  plt.legend()
  plt.grid(True)
  plt.savefig('log/conflict_metric.png')
  #plot.show()
  plt.close()


def make_p_batt_plots(title, prefix, Batteries, t_plot, p_batt_plot):
  for name in Batteries:
    if len(t_plot) != len(p_batt_plot[name]):
      print('*** Mismatched data points for plot ' + title + ' P_batt ' + name + ', time len: ' + str(len(t_plot)) + ', p_batt len: ' + str(len(p_batt_plot[name])), flush=True)
    batname = name[12:] # extract just the name for tidier plots
    plt.title(title + ' P_batt:  ' + batname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time (sec)')
    plt.ylabel('P_batt (kW)')
    plt.plot(t_plot[:len(p_batt_plot[name])], p_batt_plot[name])
    plt.savefig('log/' + prefix + '_p_batt_' + batname + '.png')
    #plot.show()
    plt.close()


def make_soc_plots(title, prefix, Batteries, t_plot, soc_plot):
  for name in Batteries:
    if len(t_plot) != len(soc_plot[name]):
      print('*** Mismatched data points for plot ' + title + ' SoC ' + name + ', time len: ' + str(len(t_plot)) + ', soc len: ' + str(len(soc_plot[name])), flush=True)
    batname = name[12:] # extract just the name for tidier plots
    plt.title(title + ' SoC:  ' + batname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time (sec)')
    plt.ylabel('Battery SoC')
    plt.plot(t_plot[:len(soc_plot[name])], soc_plot[name])
    plt.savefig('log/' + prefix + '_soc_' + batname + '.png')
    #plot.show()
    plt.close()


def make_reg_plots(title, prefix, Regulators, t_plot, reg_plot):
  for name in Regulators:
    if len(t_plot) != len(reg_plot[name]):
      print('*** Mismatched data points for plot ' + title + ' ' + name + ', time len: ' + str(len(t_plot)) + ', reg len: ' + str(len(reg_plot[name])), flush=True)
    regname = name[16:] # extract just the name for tidier plots
    plt.title(title + ' Tap Pos:  ' + regname, pad=15.0)
    ax = plt.figure().gca()
    # integer y-axis number labels except for the position never changing
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time (sec)')
    plt.ylabel('Regulator Tap Pos')
    plt.plot(t_plot[:len(reg_plot[name])], reg_plot[name])
    plt.savefig('log/' + prefix + '_tap_' + regname + '.png')
    #plot.show()
    plt.close()


def make_p_pv_plots(title, prefix, SolarPVs, t_plot, p_pv_plot):
  for name in SolarPVs:
    # just bail if there is no SolarPV data
    if len(p_pv_plot[name]) == 0:
      return

    if len(t_plot) != len(p_pv_plot[name]):
      print('*** Mismatched data points for plot ' + title + ' p_pv ' + name + ', time len: ' + str(len(t_plot)) + ', p_pv len: ' + str(len(p_pv_plot[name])), flush=True)
    pvname = name[17:] # extract just the name for tidier plots
    plt.title(title + ' p_pv:  ' + pvname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time (sec)')
    plt.ylabel('p_pv (kW)')
    plt.plot(t_plot[:len(p_pv_plot[name])], p_pv_plot[name])
    plt.savefig('log/' + prefix + '_p_pv_' + pvname + '.png')
    #plot.show()
    plt.close()


def make_q_pv_plots(title, prefix, SolarPVs, t_plot, q_pv_plot):
  for name in SolarPVs:
    # just bail if there is no SolarPV data
    if len(q_pv_plot[name]) == 0:
      return

    if len(t_plot) != len(q_pv_plot[name]):
      print('*** Mismatched data points for plot ' + title + ' q_pv ' + name + ', time len: ' + str(len(t_plot)) + ', q_pv len: ' + str(len(q_pv_plot[name])), flush=True)
    pvname = name[17:] # extract just the name for tidier plots
    plt.title(title + ' q_pv:  ' + pvname, pad=15.0)
    #ax = plt.figure().gca()
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time (sec)')
    plt.ylabel('q_pv (kW)')
    plt.plot(t_plot[:len(q_pv_plot[name])], q_pv_plot[name])
    plt.savefig('log/' + prefix + '_q_pv_' + pvname + '.png')
    #plot.show()
    plt.close()


def _main():
  print('Starting plotter...', flush=True)

  matplotlib.use('agg')

  Batteries = ['BatteryUnit.battery1','BatteryUnit.battery2','BatteryUnit.battery3','BatteryUnit.battery4','BatteryUnit.battery5']

  Regulators = ['RatioTapChanger.reg1a','RatioTapChanger.reg2a','RatioTapChanger.reg3a','RatioTapChanger.reg3c','RatioTapChanger.reg4a','RatioTapChanger.reg4b','RatioTapChanger.reg4c']

  SolarPVs = ['PhotovoltaicUnit.dg_12','PhotovoltaicUnit.dg_18','PhotovoltaicUnit.dg_30','PhotovoltaicUnit.dg_36','PhotovoltaicUnit.dg_42','PhotovoltaicUnit.dg_48','PhotovoltaicUnit.dg_54','PhotovoltaicUnit.dg_6','PhotovoltaicUnit.dg_60','PhotovoltaicUnit.dg_66','PhotovoltaicUnit.dg_72','PhotovoltaicUnit.dg_78','PhotovoltaicUnit.dg_84','PhotovoltaicUnit.dg_90']

  t_plot = []
  p_batt_plot = {}
  soc_plot = {}
  reg_plot = {}
  p_pv_plot = {}
  q_pv_plot = {}

  for batt in Batteries:
    p_batt_plot[batt] = []
    soc_plot[batt] = []

  for reg in Regulators:
    reg_plot[reg] = []

  for pv in SolarPVs:
    p_pv_plot[pv] = []
    q_pv_plot[pv] = []

  cm_t_plot = []
  cm_start_plot = []
  cm_rules_plot = []
  cm_coop_plot = []

  app = 'SIMULATION'
  prefix = 'sim'

  simhits = 0
  cmhits = 0
  with open('log/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == app:
        simhits += 1
        t_plot.append(float(tokens[1]))

        start = 3
        finish = start + len(Batteries)*3
        for it in range(start, finish, 3):
          batt = tokens[it]
          p_batt_plot[batt].append(float(tokens[it+1])/1000.0)
          soc_plot[batt].append(float(tokens[it+2]))

        start = finish
        finish = start + len(Regulators)*2
        for it in range(start, finish, 2):
          reg = tokens[it]
          reg_plot[reg].append(int(tokens[it+1]))

        start = finish
        finish = start + len(SolarPVs)*2
        for it in range(start, finish, 2):
          pv = tokens[it]
          cmplx = complex(tokens[it+1])/1000.0
          p_pv_plot[pv].append(cmplx.real)
          q_pv_plot[pv].append(cmplx.imag)

      elif tokens[0] == 'conflict_metric':
        cmhits += 1
        cm_t_plot.append(float(tokens[1]))
        cm_start_plot.append(float(tokens[3]))
        cm_rules_plot.append(float(tokens[4]))
        cm_coop_plot.append(float(tokens[5]))

  print(app + ' hits: ' + str(simhits), flush=True)
  print('conflict_metric hits: ' + str(cmhits), flush=True)

  make_p_batt_plots(app, prefix, Batteries, t_plot, p_batt_plot)
  make_soc_plots(app, prefix, Batteries, t_plot, soc_plot)
  make_reg_plots(app, prefix, Regulators, t_plot, reg_plot)
  make_p_pv_plots(app, prefix, SolarPVs, t_plot, p_pv_plot)
  make_q_pv_plots(app, prefix, SolarPVs, t_plot, q_pv_plot)

  t_plot.clear()

  for batt in Batteries:
    p_batt_plot[batt].clear()
    soc_plot[batt].clear()

  for reg in Regulators:
    reg_plot[reg].clear()

  for pv in SolarPVs:
    p_pv_plot[pv].clear()
    q_pv_plot[pv].clear()

  make_cm_plot(cm_t_plot, cm_start_plot, cm_rules_plot, cm_coop_plot)

  cm_t_plot.clear()
  cm_start_plot.clear()
  cm_rules_plot.clear()
  cm_coop_plot.clear()

  #app_list = ['resilience-app', 'decarbonization-app', 'cvr-app']
  app_list = ['resilience-app', 'decarbonization-app']
  #app_list = ['resilience-app']
  #app_list = ['decarbonization-app']
  #prefix_list = ['resil', 'decarb', 'cvr']
  prefix_list = ['resil', 'decarb']
  #prefix_list = ['resil']
  #prefix_list = ['decarb']

  for iapp in range(len(app_list)):
    hits = 0
    with open('log/plot_data.csv', 'r') as file:
      for line in file:
        tokens = line.split(',')
        if tokens[0] == app_list[iapp]:
          hits += 1
          t_plot.append(float(tokens[1]))

          numdev = len(tokens)
          for it in range(3, numdev, 2):
            dev = tokens[it]
            if dev.startswith('BatteryUnit.'):
              p_batt_plot[dev].append(float(tokens[it+1])/1000.0)
            elif dev.startswith('RatioTapChanger.'):
              reg_plot[dev].append(int(tokens[it+1]))
            elif dev.startswith('PhotovoltaicUnit.'):
              cmplx = complex(tokens[it+1])/1000.0
              p_pv_plot[dev].append(cmplx.real)
              q_pv_plot[dev].append(cmplx.imag)

    print(app_list[iapp] + ' hits: ' + str(hits), flush=True)

    make_p_batt_plots(app_list[iapp], prefix_list[iapp], Batteries, t_plot, p_batt_plot)
    make_reg_plots(app_list[iapp], prefix_list[iapp], Regulators, t_plot, reg_plot)
    make_p_pv_plots(app_list[iapp], prefix_list[iapp], SolarPVs, t_plot, p_pv_plot)
    make_q_pv_plots(app_list[iapp], prefix_list[iapp], SolarPVs, t_plot, q_pv_plot)

    t_plot.clear()

    for batt in Batteries:
      p_batt_plot[batt].clear()

    for reg in Regulators:
      reg_plot[reg].clear()

    for pv in SolarPVs:
      p_pv_plot[pv].clear()
      q_pv_plot[pv].clear()

  print('Goodbye!')


if __name__ == "__main__":
  _main()

