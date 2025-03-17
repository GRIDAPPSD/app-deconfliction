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
from datetime import datetime


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
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time')
    plt.ylabel('P_batt  (kW)')
    plt.savefig('log/' + prefix + '_p_batt_' + batname + '.png')
    #plot.show()

    plt.figure()
    fig, ax = plt.subplots()
    plt.title(title + ' SoC:  ' + batname, pad=15.0)
    plt.plot(t_plot, soc_plot[name])
    #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
    #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
    #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
    plt.xlabel('Time')
    plt.ylabel('Battery SoC')
    plt.savefig('log/' + prefix + '_soc_' + batname + '.png')
    #plot.show()


def _main():
  print('Starting plotter...')


  app = 'SIMULATION'
  prefix = 'sim'

  Batteries = {}
  Batteries['BatteryUnit.battery1'] = True
  Batteries['BatteryUnit.battery2'] = True
  Batteries['BatteryUnit.battery3'] = True
  Batteries['BatteryUnit.battery4'] = True
  Batteries['BatteryUnit.battery5'] = True

  t_plot = []
  p_batt_plot = {}
  soc_plot = {}

  for batt in Batteries:
    p_batt_plot[batt] = []
    soc_plot[batt] = []

  hits = 0
  with open('log/hour_plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == app:
        hits += 1
        t_plot.append(float(tokens[1]))

        for it in range(3, 18, 3):
          batt = tokens[it]
          p_batt_plot[batt].append(float(tokens[it+1]))
          soc_plot[batt].append(float(tokens[it+2]))

  print('Hits: ' + str(hits))

  make_plots(app, prefix, Batteries, t_plot, p_batt_plot, soc_plot)

  print('Goodbye!')


if __name__ == "__main__":
  _main()

