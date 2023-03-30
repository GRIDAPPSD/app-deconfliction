# Copyright (c) 2023, Battelle Memorial Institute All rights reserved.
# Battelle Memorial Institute (hereinafter Battelle) hereby grants permission to any person or entity
# lawfully obtaining a copy of this software and associated documentation files (hereinafter the
# Software) to redistribute and use the Software in source and binary forms, with or without modification.
# Such person or entity may use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and may permit others to do so, subject to the following conditions:
# Redistributions of source code must retain the above copyright notice, this list of conditions and the
# following disclaimers.
# Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
# the following disclaimer in the documentation and/or other materials provided with the distribution.
# Other than as used herein, neither the name Battelle Memorial Institute or Battelle may be used in any
# form whatsoever without the express written consent of Battelle.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# BATTELLE OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
# General disclaimer for use with OSS licenses
#
# This material was prepared as an account of work sponsored by an agency of the United States Government.
# Neither the United States Government nor the United States Department of Energy, nor Battelle, nor any
# of their employees, nor any jurisdiction or organization that has cooperated in the development of these
# materials, makes any warranty, express or implied, or assumes any legal liability or responsibility for
# the accuracy, completeness, or usefulness or any information, apparatus, product, software, or process
# disclosed, or represents that its use would not infringe privately owned rights.
#
# Reference herein to any specific commercial product, process, or service by trade name, trademark, manufacturer,
# or otherwise does not necessarily constitute or imply its endorsement, recommendation, or favoring by the United
# States Government or any agency thereof, or Battelle Memorial Institute. The views and opinions of authors expressed
# herein do not necessarily state or reflect those of the United States Government or any agency thereof.
#
# PACIFIC NORTHWEST NATIONAL LABORATORY operated by BATTELLE for the
# UNITED STATES DEPARTMENT OF ENERGY under Contract DE-AC05-76RL01830
# ------------------------------------------------------------------------------
"""
Created on October 31, 2022

@author: Gary Black and Shiva Poudel
"""""

import sys
import time
import os
import argparse
import json
import importlib
import math
import pprint
import numpy as np
import csv
from pulp import *

from gridappsd import GridAPPSD

from matplotlib import pyplot as plt
from matplotlib import dates as md
from datetime import datetime

class CompetingApp(GridAPPSD):
    energy_credits = {}

    def on_message(self, headers, message):
        reply_to = headers['reply-to']

        if message['requestType'] == 'GET_SNAPSHOT_YBUS':
            # hold up responses until we know the ybus reflects the regulator
            # tap positions for the simulation
            while not self.simRap.ybusInitFlag:
                time.sleep(0.1)

            lowerUncomplex = self.simRap.lowerUncomplex(self.simRap.Ybus)
            message = {
                'feeder_id': self.simRap.feeder_mrid,
                'simulation_id': self.simRap.simulation_id,
                'timestamp': self.simRap.timestamp,
                'ybus': lowerUncomplex
            }
            print('Sending Ybus snapshot response for timestamp: ' + str(self.simRap.timestamp), flush=True)
            self.simRap.gapps.send(reply_to, message)

        else:
            message = "No valid requestType specified"
            self.simRap.gapps.send(reply_to, message)

    def resiliency(self, EnergyConsumers, SynchronousMachines, Batteries,
                   SolarPVs, time, load_mult, pv_mult, price, deltaT,
                   emergencyState):

        P_load = 0.0
        for name in EnergyConsumers:
            P_load += load_mult * EnergyConsumers[name]['kW']

        P_ren = 0.0
        for name in SolarPVs:
            P_ren += pv_mult * SolarPVs[name]['kW']

        print('time: ' + str(time), flush=True)
        print('Total EnergyConsumers P_load: ' + str(round(P_load, 4)), flush=True)
        print('Total SolarPVs P_ren: ' + str(round(P_ren, 4)), flush=True)

        # Implement simple PuLP optimization for resilience
        # Objective function: Max: SoC of the Battery
        n_batt = len(Batteries)
        soc = LpVariable.dicts("soc", (i for i in range(n_batt)), lowBound=0.2, upBound=0.9, cat='Continuous')
        pbatt_c = LpVariable.dicts("pbatt_c", (i for i in range(n_batt)), lowBound=0, upBound=250, cat='Continuous')
        pbatt_d = LpVariable.dicts("pbatt_d", (i for i in range(n_batt)), lowBound=-250, upBound=0, cat='Continuous')
        lamda_c = LpVariable.dicts("lamda_c", (i for i in range(n_batt)), lowBound=0, upBound=1, cat='Binary')
        lamda_d = LpVariable.dicts("lamda_d", (i for i in range(n_batt)), lowBound=0, upBound=1, cat='Binary')
        P_load_v = LpVariable("P_load_v", lowBound=0, upBound=P_load, cat='Continuous')
        Psub = LpVariable("Psub", lowBound=-10000, upBound=10000, cat='Continuous')

        prob = LpProblem("Energy_Stored", LpMinimize)
        if 72 < time < 84:
            prob += lpSum(soc[n] for n in range(n_batt))
            prob += P_load_v <= P_load
            prob += Psub == 0
        else:
            # prob += lpSum(-P_load_v)
            prob += lpSum(- soc[n] for n in range(n_batt))
            prob += P_load_v == P_load

        # SoC constraints of the battery
        idx = 0
        for name in Batteries:
            Batteries[name]['state'] = 'idling'
            prob += soc[idx] == Batteries[name]['SoC'] + \
                    Batteries[name]['eff_c'] * pbatt_c[idx] * deltaT / Batteries[name]['ratedE'] + \
                    1 / Batteries[name]['eff_d'] * pbatt_d[idx] * deltaT / Batteries[name]['ratedE']
            prob += pbatt_c[idx] <= lamda_c[idx] * 250
            prob += pbatt_d[idx] >= - lamda_d[idx] * 250
            prob += lamda_c[idx] + lamda_d[idx] <= 1
            idx += 1

        # Energy Balance equations
        prob += Psub == -P_ren + P_load_v + pbatt_c[0] + pbatt_c[1] + pbatt_d[0] + pbatt_d[1]

        # print('Now solving the resilience application.......')
        prob.solve(PULP_CBC_CMD(msg=0))
        prob.writeLP('Resilience.lp')
        # print('Status:', LpStatus[prob.status])
        # print('Status:', LpStatus[prob.status])
        # print('Batt Power:', pbatt_c[0].varValue, pbatt_c[1].varValue, pbatt_d[0].varValue, pbatt_d[1].varValue)
        # print('Soc: ', soc[0].varValue, soc[1].varValue)
        idx = 0
        for name in Batteries:
            Batteries[name]['SoC'] = soc[idx].varValue
            if pbatt_c[idx].varValue >= 0.01:
                Batteries[name]['P_batt_c'] = pbatt_c[idx].varValue
                Batteries[name]['state'] = 'charging'
            if pbatt_d[idx].varValue <= - 0.01:
                Batteries[name]['P_batt_d'] = - pbatt_d[idx].varValue
                Batteries[name]['state'] = 'discharging'
            idx += 1

        return

    def decarbonization(self, EnergyConsumers, SynchronousMachines, Batteries,
                        SolarPVs, time, load_mult, pv_mult, profit, deltaT,
                        emergencyState):

        P_load = 0.0
        for name in EnergyConsumers:
            P_load += load_mult * EnergyConsumers[name]['kW']

        P_ren = 0.0
        for name in SolarPVs:
            P_ren += pv_mult * SolarPVs[name]['kW']

        print('time: ' + str(time), flush=True)
        print('Total EnergyConsumers P_load: ' + str(round(P_load, 4)), flush=True)
        print('Total SolarPVs P_ren: ' + str(round(P_ren, 4)), flush=True)

        # Implement simple PuLP optimization for resilience
        # Objective function: Max: SoC of the Battery
        n_batt = len(Batteries)
        soc = LpVariable.dicts("soc", (i for i in range(n_batt)), lowBound=0.2, upBound=0.9, cat='Continuous')
        pbatt_c = LpVariable.dicts("pbatt_c", (i for i in range(n_batt)), lowBound=0, upBound=250, cat='Continuous')
        pbatt_d = LpVariable.dicts("pbatt_d", (i for i in range(n_batt)), lowBound=-250, upBound=0, cat='Continuous')
        lamda_c = LpVariable.dicts("lamda_c", (i for i in range(n_batt)), lowBound=0, upBound=1, cat='Binary')
        lamda_d = LpVariable.dicts("lamda_d", (i for i in range(n_batt)), lowBound=0, upBound=1, cat='Binary')
        Psub = LpVariable("Psub", lowBound=-10000, upBound=10000, cat='Continuous')
        P_load_v = LpVariable("P_load_v", lowBound=0, upBound=P_load, cat='Continuous')
        Psub_mod = LpVariable("Psub_mod", lowBound=-10000, upBound=10000, cat='Continuous')

        prob = LpProblem("Dirty_Generations", LpMinimize)
        if 72 < time < 84:
            prob += lpSum(soc[n] for n in range(n_batt))
            prob += P_load_v <= P_load
            prob += Psub == 0
        else:
            prob += lpSum(- soc[n] for n in range(n_batt))
            prob += P_load_v == P_load

        # SoC constraints of the battery
        idx = 0
        for name in Batteries:
            Batteries[name]['state'] = 'idling'
            prob += soc[idx] == Batteries[name]['SoC'] + \
                    Batteries[name]['eff_c'] * pbatt_c[idx] * deltaT / Batteries[name]['ratedE'] + \
                    1 / Batteries[name]['eff_d'] * pbatt_d[idx] * deltaT / Batteries[name]['ratedE']
            prob += pbatt_c[idx] <= lamda_c[idx] * 250
            prob += pbatt_d[idx] >= - lamda_d[idx] * 250
            prob += lamda_c[idx] + lamda_d[idx] <= 1
            idx += 1

        # Converting modulus to linear problem
        prob += Psub_mod >= Psub
        prob += Psub_mod >= - Psub

        # Energy Balance equations
        prob += Psub == -P_ren + P_load_v + pbatt_c[0] + pbatt_c[1] + pbatt_d[0] + pbatt_d[1]

        # print('Now solving the decarbonization application.......')
        prob.solve(PULP_CBC_CMD(msg=0))
        prob.writeLP('Decarbonization.lp')
        # print('Status:', LpStatus[prob.status])
        # print('Batt Power:', pbatt_c[0].varValue, pbatt_c[1].varValue, pbatt_d[0].varValue, pbatt_d[1].varValue)
        # print('Soc: ', soc[0].varValue, soc[1].varValue)
        idx = 0
        for name in Batteries:
            Batteries[name]['SoC'] = soc[idx].varValue
            if pbatt_c[idx].varValue >= 0.01:
                Batteries[name]['P_batt_c'] = pbatt_c[idx].varValue
                Batteries[name]['state'] = 'charging'
            if pbatt_d[idx].varValue <= -0.01:
                Batteries[name]['P_batt_d'] = - pbatt_d[idx].varValue
                Batteries[name]['state'] = 'discharging'
            idx += 1
        return

    def to_datetime(self, time):
        return datetime(1966, 8, 1, (time - 1) // 4, 15 * ((time - 1) % 4), 0)

    def make_plots(self, title, prefix, Batteries, t_plot, p_batt_plot, soc_plot):
        for name in Batteries:
            plt.figure()
            fig, ax = plt.subplots()
            plt.title(title + ' P_batt:  ' + name, pad=15.0)
            plt.plot(t_plot, p_batt_plot[name])
            ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
            plt.xlim([self.to_datetime(1), self.to_datetime(96)])
            plt.xticks([self.to_datetime(1), self.to_datetime(25), self.to_datetime(49), self.to_datetime(73),
                        self.to_datetime(96)])
            plt.xlabel('Time')
            plt.ylabel('P_batt  (kW)')
            plt.savefig('output/' + prefix + '_p_batt_' + name + '.png')
            # plot.show()

            plt.figure()
            fig, ax = plt.subplots()
            plt.title(title + ' SoC:  ' + name, pad=15.0)
            plt.plot(t_plot, soc_plot[name])
            ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))
            plt.xlim([self.to_datetime(1), self.to_datetime(96)])
            plt.xticks([self.to_datetime(1), self.to_datetime(25), self.to_datetime(49), self.to_datetime(73),
                        self.to_datetime(96)])
            plt.xlabel('Time')
            plt.ylabel('Battery SoC')
            plt.savefig('output/' + prefix + '_soc_' + name + '.png')
            # plot.show()

    def conflict_matrix(self, solution):
        d_c = []
        centroid = []
        apps = {}
        n_devices = len(solution.keys())
        for device in solution:
            n_apps = len(solution[device])
            device_setpoints = []
            for app in solution[device]:
                if app not in apps:
                    apps[app] = []
                apps[app].append((solution[device][app] + 250) / 500)
                device_setpoints.append((solution[device][app] + 250) / 500)
            # Find centroid
            centroid.append(sum(device_setpoints) / n_apps)

        # Distance matrix
        for app in apps:
            s = 0
            for k in range(len(centroid)):
                s += (centroid[k] - apps[app][k]) ** 2
            d_c.append(math.sqrt(s))

        conflict_metric = sum(d_c) / n_apps
        conflict_metric = conflict_metric * 2 / math.sqrt(n_devices)
        return conflict_metric

    def __init__(self, gapps, feeder_mrid, simulation_id, state):
        SPARQLManager = getattr(importlib.import_module('shared.sparql'), 'SPARQLManager')
        sparql_mgr = SPARQLManager(gapps, feeder_mrid, simulation_id)

        # Competing Apps Start

        emergencyState = state.startswith('e') or state.startswith('E')

        EnergyConsumers = {}
        objs = sparql_mgr.obj_dict_export('EnergyConsumer')
        print('Count of EnergyConsumers Dict: ' + str(len(objs)), flush=True)
        for item in objs:
            name = item['IdentifiedObject.name']
            EnergyConsumers[name] = {}
            # Shiva HACK to force battery charging...
            # EnergyConsumers[name]['kW'] = 0.01*float(item['EnergyConsumer.p'])/1000.0
            EnergyConsumers[name]['kW'] = float(item['EnergyConsumer.p']) / 1000.0
            EnergyConsumers[name]['kVar'] = float(item['EnergyConsumer.q']) / 1000.0
            # print('EnergyConsumer name: ' + name + ', kW: ' + str(EnergyConsumers[name]['kW']) + ', kVar: ' + str(EnergyConsumers[name]['kVar']), flush=True)

        # objs = sparql_mgr.obj_meas_export('EnergyConsumer')
        # print('Count of EnergyConsumers Meas: ' + str(len(objs)), flush=True)
        # for item in objs:
        #  print('EnergyConsumer: ' + str(item), flush=True)

        # objs = sparql_mgr.obj_meas_export('PowerElectronicsConnection')
        # print('Count of PowerElectronicsConnections Meas: ' + str(len(objs)), flush=True)
        # for item in objs:
        #  print('PowerElectronicsConnection: ' + str(item), flush=True)

        # objs = sparql_mgr.obj_dict_export('LinearShuntCompensator')
        # print('Count of LinearShuntCompensators Dict: ' + str(len(objs)), flush=True)
        # for item in objs:
        #  print('LinearShuntCompensator: ' + str(item), flush=True)

        # objs = sparql_mgr.obj_meas_export('LinearShuntCompensator')
        # print('Count of LinearShuntCompensators Meas: ' + str(len(objs)), flush=True)
        # for item in objs:
        #  print('LinearShuntCompensator: ' + str(item), flush=True)

        SynchronousMachines = {}
        objs = sparql_mgr.obj_dict_export('SynchronousMachine')
        print('Count of SynchronousMachines Dict: ' + str(len(objs)), flush=True)
        for item in objs:
            name = item['IdentifiedObject.name']
            SynchronousMachines[name] = {}
            SynchronousMachines[name]['kW'] = float(item['SynchronousMachine.p']) / 1000.0
            SynchronousMachines[name]['kVar'] = float(item['SynchronousMachine.q']) / 1000.0
            SynchronousMachines[name]['ratedS'] = float(item['SynchronousMachine.ratedS']) / 1000.0
            print('SynchronousMachine name: ' + name + ', kW: ' + str(
                round(SynchronousMachines[name]['kW'], 4)) + ', kVar: ' + str(
                round(SynchronousMachines[name]['kVar'], 4)), flush=True)

        objs = sparql_mgr.obj_meas_export('SynchronousMachine')
        # print('Count of SynchronousMachines Meas: ' + str(len(objs)), flush=True)
        # for item in objs:
        #  print('SynchronousMachine: ' + str(item), flush=True)

        Batteries = {}
        bindings = sparql_mgr.battery_query()
        print('Count of Batteries: ' + str(len(bindings)), flush=True)
        for obj in bindings:
            name = obj['name']['value']
            # bus = obj['bus']['value'].upper()
            Batteries[name] = {}
            Batteries[name]['ratedkW'] = float(obj['ratedS']['value']) / 1000.0
            Batteries[name]['ratedE'] = float(obj['ratedE']['value']) / 1000.0
            # Shiva HACK
            Batteries[name]['SoC'] = 0.5
            # Batteries[name]['SoC'] = float(obj['storedE']['value'])/float(obj['ratedE']['value'])
            # eff_c and eff_d don't come from the query, but they are used throughout
            # and this is a convenient point to assign them with query results
            Batteries[name]['eff_c'] = 0.975 * 0.86
            Batteries[name]['eff_d'] = 0.975 * 0.86
            print('Battery name: ' + name + ', ratedE: ' + str(round(Batteries[name]['ratedE'], 4)) + ', SoC: ' + str(
                round(Batteries[name]['SoC'], 4)), flush=True)

        SolarPVs = {}
        bindings = sparql_mgr.pv_query()
        print('Count of SolarPV: ' + str(len(bindings)) + '\n \n', flush=True)
        for obj in bindings:
            name = obj['name']['value']
            # bus = obj['bus']['value'].upper()
            # ratedS = float(obj['ratedS']['value'])
            # ratedU = float(obj['ratedU']['value'])
            SolarPVs[name] = {}
            SolarPVs[name]['kW'] = float(obj['p']['value']) / 1000.0
            SolarPVs[name]['kVar'] = float(obj['q']['value']) / 1000.0
            # print('SolarPV name: ' + name + ', kW: ' + str(SolarPVs[name]['kW']) + ', kVar: ' + str(SolarPVs[name]['kVar']), flush=True)

        # make sure output directory exists since that's where results go
        if not os.path.isdir('output'):
            os.makedirs('output')

        # Invoke Competing Apps
        solution = {}
        solutions = {}
        with open('../sim-starter/time-series.csv', 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header

            deltaT = 0.25  # timestamp interval in fractional hours, 0.25 = 15 min.

            for name in Batteries:
                Batteries[name]['initial_soc'] = Batteries[name]['SoC']

            for row in reader:
                time = int(row[0])
                # loadshape = float(row[1])
                # solar = float(row[2])
                loadshape = 0.1728
                solar = 0.951665
                price = float(row[3])
                # time = 73
                # Resilience App
                print('Resilience App...', flush=True)
                self.resiliency(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, time, loadshape, solar,
                                price, deltaT, emergencyState)

                for name in Batteries:
                    solution[name] = {}
                    if Batteries[name]['state'] == 'charging':
                        print('Battery name: ' + name + ', ratedkW: ' + str(
                            round(Batteries[name]['ratedkW'], 4)) + ', P_batt_c: ' + str(
                            round(Batteries[name]['P_batt_c'], 4)) + ', updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)),
                              flush=True)
                        solution[name]['resilience'] = Batteries[name]['P_batt_c']
                    elif Batteries[name]['state'] == 'discharging':
                        print('Battery name: ' + name + ', ratedkW: ' + str(
                            round(Batteries[name]['ratedkW'], 4)) + ', P_batt_d: ' + str(
                            round(Batteries[name]['P_batt_d'], 4)) + ', updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)),
                              flush=True)
                        solution[name]['resilience'] = -Batteries[name]['P_batt_d']
                    else:
                        print('Battery name: ' + name + ', P_batt_c = P_batt_d = 0.0, updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)), flush=True)
                        solution[name]['resilience'] = 0.0

                # To make sure batteries start with same initial conditions for decarbonization app
                for name in Batteries:
                    Batteries[name]['SoC'] = Batteries[name]['initial_soc']

                # Decarbonization App
                print('\nDecarbonization App...', flush=True)
                self.decarbonization(EnergyConsumers, SynchronousMachines, Batteries, SolarPVs, time, loadshape, solar,
                                     price, deltaT, emergencyState)

                for name in Batteries:
                    if Batteries[name]['state'] == 'charging':
                        print('Battery name: ' + name + ', ratedkW: ' + str(
                            round(Batteries[name]['ratedkW'], 4)) + ', P_batt_c: ' + str(
                            round(Batteries[name]['P_batt_c'], 4)) + ', updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)),
                              flush=True)
                        solution[name]['decarbonization'] = Batteries[name]['P_batt_c']
                    elif Batteries[name]['state'] == 'discharging':
                        print('Battery name: ' + name + ', ratedkW: ' + str(
                            round(Batteries[name]['ratedkW'], 4)) + ', P_batt_d: ' + str(
                            round(Batteries[name]['P_batt_d'], 4)) + ', updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)),
                              flush=True)
                        solution[name]['decarbonization'] = -Batteries[name]['P_batt_d']
                    else:
                        print('Battery name: ' + name + ', P_batt_c = P_batt_d = 0.0, updated SoC: ' + str(
                            round(Batteries[name]['SoC'], 4)), flush=True)
                        solution[name]['decarbonization'] = 0.0
                        
                print('\nSolution from apps', solution)
                # Quantify the conflict. Can be useful for incentive-based scheme
                conflict_metric = self.conflict_matrix(solution)
                print('\nConflict Metric: ', conflict_metric, flush=True)

                solutions[time] = {}
                solutions[time]['solution'] = solution
                solutions[time]['conflict'] = conflict_metric
                
                # Invoke cooperative solution here....
                # exit()

        json_fp = open('output/' + 'conflict' + '_solution.json', 'w')
        json.dump(solutions, json_fp, indent=2)
        json_fp.close()

        return


def _main():
    print('Starting app code...', flush=True)

    # for loading modules
    if (os.path.isdir('shared')):
        sys.path.append('.')
    elif (os.path.isdir('../shared')):
        sys.path.append('..')
    elif (os.path.isdir('app-deconfliction/competing-apps/shared')):
        sys.path.append('app-deconfliction/competing-apps')
    else:
        sys.path.append('/gridappsd/services/app-deconfliction/competing-apps')

    # sim_request = json.loads(opts.request.replace("\'", ""))
    feeder_mrid = "_AAE94E4A-2465-6F5E-37B1-3E72183A4E44"
    simulation_id = 0.0
    state = 'a'

    # authenticate with GridAPPS-D Platform
    os.environ['GRIDAPPSD_APPLICATION_ID'] = 'gridappsd-competing-app'
    os.environ['GRIDAPPSD_APPLICATION_STATUS'] = 'STARTED'
    os.environ['GRIDAPPSD_USER'] = 'app_user'
    os.environ['GRIDAPPSD_PASSWORD'] = '1234App'

    gapps = GridAPPSD(simulation_id)
    assert gapps.connected

    competing_app = CompetingApp(gapps, feeder_mrid, simulation_id, state)


if __name__ == "__main__":
    _main()
