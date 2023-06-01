import opendssdirect as dss
from opendssdirect.utils import Iterator
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Direct the DSS files
dss.run_command('Redirect 123Bus/Run_IEEE123Bus.dss')
dss.run_command('Compile Run_IEEE123Bus.dss')
dss.run_command('set loadmult = 1.0')

# Script to control PV system
dss.run_command('PVSystem.' + 'dg_6' + '.Pmpp =' + str(100))

# Script to control Transformer taps
dss.run_command('Transformer.reg1a.Taps=[1.0 1.01875]')
# dss.run_command('Transformer.reg2a.Taps=[1.0 0.99375]')
# dss.run_command('Transformer.reg3a.Taps=[1.0 0.9875]')
# dss.run_command('Transformer.reg3c.Taps=[1.0 1.01875]')
# dss.run_command('Transformer.reg4a.Taps=[1.0 1.025]')
# dss.run_command('Transformer.reg4b.Taps=[1.0 1.01875]')
# dss.run_command('Transformer.reg4c.Taps=[1.0 1.05]')
dss.run_command('Set Controlmode=OFF')

# Solve the circuit
# dss.run_command('solve')
dss.Solution.Solve()
# dss.run_command('show voltages LN Nodes')

# Extract nodal voltages
node_names = dss.Circuit.AllNodeNames()
nodeA_names = []
nodeB_names = []
nodeC_names = []
for node in node_names:
    if ".1" in node:
        nodeA_names.append(node)
    elif ".2" in node:
        nodeB_names.append(node)
    elif ".3" in node:
        nodeC_names.append(node)

bus_voltages = {}
bus_A = []
bus_B = []
bus_C = []
phases = [1, 2, 3]
for p in phases:
    for idx, voltage in enumerate(dss.Circuit.AllNodeVmagPUByPhase(p)):
        if p == 1:
            bus_A.append(voltage)
            bus_voltages[nodeA_names[idx]] = voltage
        elif p == 2:
            bus_B.append(voltage)
            bus_voltages[nodeB_names[idx]] = voltage
        elif p == 3:
            bus_C.append(voltage)
            bus_voltages[nodeC_names[idx]] = voltage

# Extract line flows. Example is shown for a line connected to source bus
print('\n..........Substation Flow.............')
line_flow_sensors = ['l115']
for line in line_flow_sensors:
    element = 'Line.' + line
    dss.Circuit.SetActiveElement(element)
    print(complex(dss.CktElement.Powers()[0], dss.CktElement.Powers()[1]))
    print(complex(dss.CktElement.Powers()[2], dss.CktElement.Powers()[3]))
    print(complex(dss.CktElement.Powers()[4], dss.CktElement.Powers()[5]))

# print transformers tap
print('\n..........Transformer Taps.............')
for i in Iterator(dss.Transformers, 'Name'):
    print(dss.Transformers.Name(), dss.Transformers.Tap())

# Min, Max voltage and profile plot
print('\n..........Voltage Min-Max.............')
print(max(bus_A), max(bus_B), max(bus_C))
print(min(bus_A), min(bus_B), min(bus_C))
plt.scatter(range(len(bus_A)), bus_A)
plt.scatter(range(len(bus_B)), bus_B)
plt.scatter(range(len(bus_C)), bus_C)


plt.ylim([0.9, 1.1])
plt.xlabel('Bus Index')
plt.ylabel('Voltage (p.u.)')
plt.legend(['Phase-A', 'Phase-B', 'Phase-C'])
plt.plot(np.ones(100)*1.05, 'r--')
plt.plot(np.ones(100)*0.95, 'r--')
plt.show()
