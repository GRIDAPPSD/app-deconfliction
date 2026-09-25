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
legendLoc = 'lower right'
legendProp = {'weight': 'bold', 'size': legendSize}

def to_datetime(time):
  return datetime(1966, 8, 1, (int(time)-1)//4, 15*((int(time)-1) % 4), 0)

def make_resp_plot(disp_t_plot_b, disp_val_plot_b, disp_t_plot_l, disp_val_plot_l, disp_t_plot_2, disp_val_plot_2):
  plt.figure(figsize=(8,4), dpi=plotDPI)
  #plt.title('Conflict Metric', pad=15.0)

  #ax = plt.figure().gca()
  #ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
  #plt.xlim([AppUtil.to_datetime(1), AppUtil.to_datetime(96)])
  #plt.xticks([AppUtil.to_datetime(1), AppUtil.to_datetime(25), AppUtil.to_datetime(49), AppUtil.to_datetime(73), AppUtil.to_datetime(96)])
  if realtimeFlag:
    plt.xlabel('Time (sec)', fontweight='bold', fontsize=labelSize)
  else:
    plt.xlim([0, 24])
    plt.xticks([0, 4, 8, 12, 16, 20, 24], fontweight='bold', fontsize=tickSize)
    plt.xlabel('Time (hours of day)', fontweight='bold', fontsize=labelSize)

  #plt.ylim([-0.05, 1.0])
  #plt.yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0], fontweight='bold', fontsize=tickSize)
  plt.ylabel('Cooperation Responses', fontweight='bold', fontsize=labelSize)
  plt.plot(disp_t_plot_b, disp_val_plot_b, color='cyan', label='5 Apps Baseline')
  plt.plot(disp_t_plot_l, disp_val_plot_l, color='magenta', label='5 Apps Lower Thresholds')
  plt.plot(disp_t_plot_2, disp_val_plot_2, color='green', label='2 Apps Baseline')

  #plt.legend(prop=legendProp, loc=legendLoc)
  plt.legend(prop=legendProp, loc='upper left')
  plt.grid(True)
  plt.tight_layout()
  plt.savefig('coop_plots/coop_responses.png')
  #plot.show()
  plt.close()

def _main():
  print('Starting plotter...', flush=True)

  matplotlib.use('agg')

  disp_t_plot_b = []
  disp_val_plot_b = []
  disp_t_plot_l = []
  disp_val_plot_l = []
  disp_t_plot_2 = []
  disp_val_plot_2 = []

  # Jan 1, midnight timestamp:
  timex_start = 1704067200.0

  cmhits = 0
  baseline_resps = 0
  with open('../review_runs/baseline/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == 'conflict_metric':
        cmhits += 1
        resps = int(tokens[8].split(':')[1])
        if resps > 0:
          baseline_resps += resps
          disp_t_plot_b.append((float(tokens[2]) - timex_start)/3600.0)
          disp_val_plot_b.append(resps)

  print('baseline conflict_metric hits: ' + str(cmhits), flush=True)
  print('baseline total cooperation responses: ' + str(baseline_resps), flush=True)
  print('baseline responses/dispatch: ' + str(float(baseline_resps/cmhits)), flush=True)

  cmhits = 0
  lesscoop_resps = 0
  with open('../review_runs/less_coop/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == 'conflict_metric':
        cmhits += 1
        resps = int(tokens[8].split(':')[1])
        if resps > 0:
          lesscoop_resps += resps
          disp_t_plot_l.append((float(tokens[2]) - timex_start)/3600.0)
          disp_val_plot_l.append(resps)

  print('\nlesscoop conflict_metric hits: ' + str(cmhits), flush=True)
  print('lesscoop total cooperation responses: ' + str(lesscoop_resps), flush=True)
  print('lesscoop responses/dispatch: ' + str(float(lesscoop_resps/cmhits)), flush=True)

  cmhits = 0
  twoapps_resps = 0
  with open('../review_runs/two_apps/plot_data.csv', 'r') as file:
    for line in file:
      tokens = line.split(',')
      if tokens[0] == 'conflict_metric':
        cmhits += 1
        resps = int(tokens[8].split(':')[1])
        if resps > 0:
          twoapps_resps += resps
          disp_t_plot_2.append((float(tokens[2]) - timex_start)/3600.0)
          disp_val_plot_2.append(resps)

  print('\ntwoapps conflict_metric hits: ' + str(cmhits), flush=True)
  print('twoapps total cooperation responses: ' + str(twoapps_resps), flush=True)
  print('twoapps responses/dispatch: ' + str(float(twoapps_resps/cmhits)), flush=True)

  make_resp_plot(disp_t_plot_b, disp_val_plot_b, disp_t_plot_l, disp_val_plot_l, disp_t_plot_2, disp_val_plot_2)

  disp_t_plot_b.clear()
  disp_val_plot_b.clear()
  disp_t_plot_l.clear()
  disp_val_plot_l.clear()
  disp_t_plot_2.clear()
  disp_val_plot_2.clear()

  print('Goodbye!')


if __name__ == "__main__":
  _main()

