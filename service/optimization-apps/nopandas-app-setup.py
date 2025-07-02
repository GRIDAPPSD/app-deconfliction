import random
import sys
import os

def randomIntegersSumN(count=5, target_sum=10):
  breakpoints = sorted([0] + [random.randint(0, target_sum) for _ in range(count-1)] + [target_sum])
  return [breakpoints[i+1] - breakpoints[i] for i in range(count)]


def randomRealsSum1(count=5):
  return [round((1.0/10) * i, 1) for i in randomIntegersSumN(count, 10)]


def main():
  numRows = 10
  baseFile = 'app_setup'
  templateFile = baseFile + '.csv'

  if len(sys.argv) > 1:
    numRows = int(sys.argv[1])

    if len(sys.argv) > 2:
      templateFile = sys.argv[2]
      baseFile = os.path.splitext(templateFile)[0]

  outname = baseFile + '_' + str(numRows) + '.csv'

  with open(outname, 'w') as outfile:
    countRows = 0
    with open(templateFile, 'r') as infile:
      for line in infile:
        outfile.write(line)
        countRows += 1

    for irow in range(countRows, numRows+1):
      objList = randomRealsSum1()
      objStr = "[" + ' '.join(map(str, objList)) + "]"

      includeEnergyConsumersFlag = 1
      includeSolarPVsPFlag = random.randint(0, 1)
      includeSolarPVsQFlag = random.randint(0, 1)
      includeBatteriesFlag = random.randint(0, 1)
      includeRegulatorsFlag = random.randint(0, 1)
      includePFlowFlag = random.randint(0, 1)
      includeQFlowFlag = random.randint(0, 1)
      includeVoltagesFlag = random.randint(0, 1)

      # CVR objective needs voltages
      if objList[0] > 0.00001:
        includeVoltagesFlag = 1
      # voltages needs pflow and qflow
      if includeVoltagesFlag == 1:
        includePFlowFlag = 1
        includeQFlowFlag = 1
      # qflow needs pflow
      if includeSolarPVsQFlag == 1:
        includeSolarPVsPFlag = 1

      outfile.write(f"app{irow},{objStr},{includeEnergyConsumersFlag},{includeSolarPVsPFlag},{includeSolarPVsQFlag},{includeBatteriesFlag},{includeRegulatorsFlag},{includePFlowFlag},{includeQFlowFlag},{includeVoltagesFlag}\n")

if __name__ == '__main__':
  main()
