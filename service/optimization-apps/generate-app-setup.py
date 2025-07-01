import csv
import os
import pandas as pd
import random

def randomIntegersSumN(count=5, target_sum=10):
    breakpoints = sorted([0] + [random.randint(0, target_sum) for _ in range(count-1)] + [target_sum])
    return [breakpoints[i+1] - breakpoints[i] for i in range(count)]

def randomRealsSum1(count=5):
    return [round((1.0/10) * i, 1) for i in randomIntegersSumN(count, 10)]

def readSpecificColumnsFromCSV(readPath = '.', columnsList = None):
    try:
        tempInput = pd.read_csv(readPath, header = 0, usecols = columnsList)
    except FileNotFoundError:
        return pd.DataFrame()

    if columnsList == None:
        return tempInput

    if not (len(columnsList) == list(tempInput.columns.isin(columnsList)).count(True)):
        return pd.DataFrame()

    return tempInput

def main():
    numberOfRows = 10
    rootPath = r"."

    target01 = f"app_setup"
    targetType = f".csv"

    targetFilename01 = f"{target01}{targetType}"
    targetFilename02 = f"{target01}_{numberOfRows}{targetType}"

    topData = readSpecificColumnsFromCSV(os.path.join(rootPath, targetFilename01))
    if topData.empty: 
        print(f"Empty dataframe for file in: {rootPath}") 
        return 0

    while topData.shape[0] < 10:
        newAppName = f"app{random.randint(10, 999)}"
        if newAppName in topData['AppName']:
            continue
        newList = randomRealsSum1()
        newListStr = "[" + ' '.join(map(str, newList)) + "]"

        newRow = pd.DataFrame({
            'AppName': newAppName,
            'Objective': [newListStr],
            'includeEnergyConsumersFlag': 1,
            'includeSolarPVsFlag': random.randint(0, 1),
            'includeSolarPVsQFlag': random.randint(0, 1),
            'includeBatteriesFlag': random.randint(0, 1),
            'includeRegulatorsFlag': random.randint(0, 1),
            'includePFlowFlag': random.randint(0, 1),
            'includeQFlowFlag': random.randint(0, 1),
            'includeVoltagesFlag': random.randint(0, 1)
        })
        if newRow['includeVoltagesFlag'].iloc[0] < 0.5: 
            if newList[0] > 0.00001:
                continue
        if newRow['includeVoltagesFlag'].iloc[0] > 0.5:
            if newRow['includePFlowFlag'].iloc[0] < 0.5:
                continue
            if newRow['includeQFlowFlag'].iloc[0] < 0.5:
                continue
        if newRow['includeSolarPVsQFlag'].iloc[0] > 0.5: 
            if newRow['includeSolarPVsFlag'].iloc[0] < 0.5:
                continue

        topData = pd.concat([topData, newRow], ignore_index=True)

    topData.to_csv(targetFilename02, index=False)

    return 0

if __name__ == '__main__':
    main()