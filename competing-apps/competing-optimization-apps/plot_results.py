import os
import json
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':


    data_MCDM_file_name =   'output/MCDM_solution.json'
    data_MOEAD_file_name = 'output/MOEAD_solution.json'
    data_incentive_file_name = 'output/Incentives_solution.json'

    data = {}
    print("Collecting Data from output folder")
    data['MCDM'] = json.loads(open(data_MCDM_file_name, "r").read())
    data['MOEAD']  = json.loads(open(data_MOEAD_file_name, "r").read())
    data['Incentive'] = json.loads(open(data_incentive_file_name, "r").read())

    fig, ax = plt.subplots(2,1, figsize=(10, 6))
    print("Plotting Data from output folder")
    for tech in data:
        data_tech = data[tech]
        time = np.zeros(97)
        Batt1_SOC = np.zeros(97)
        Batt2_SOC = np.zeros(97)

        Batt1_SOC[0] = 0.5
        Batt2_SOC[0] = 0.5
        idx = 0
        for key in data_tech:
            time[int(key)] = float(key) * 15/ 60
            Batt1_SOC[int(key)] = (data_tech[key]['battery1']['SoC'])
            Batt2_SOC[int(key)] = (data_tech[key]['battery2']['SoC'])

        ax[0].plot(time, Batt1_SOC, label=tech)
        ax[0].set_xlabel('Time of day')
        ax[0].set_ylabel('Battery 1 SOC')
        ax[0].legend()
        ax[0].grid()
        ax[1].plot(time, Batt2_SOC, label=tech)
        ax[1].set_xlabel('Time of day')
        ax[1].set_ylabel('Battery 2 SOC')
        ax[1].legend()
        ax[1].grid()
    plt.show()


