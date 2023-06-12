
# Resilience-Decarbonization-Profit compromise deconfliction methodology

import os, sys

# find and add shared directory to path hopefully wherever it is from here
if (os.path.isdir('../shared')):
  sys.path.append('../shared')
elif (os.path.isdir('../competing-apps/shared')):
  sys.path.append('../competing-apps/shared')
elif (os.path.isdir('../../competing-apps/shared')):
  sys.path.append('../../competing-apps/shared')
else:
  sys.path.append('/gridappsd/services/app-deconfliction/competing-apps/shared')

import MethodUtil  # shared directory needs to be in path to find this


class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictSbuMatrix = MethodUtil.ConflictSubMatrix


  def deconflict(self):
    ResolutionSubVector = {}
    ResolutionSubVector['setpoints'] = {}
    ResolutionSubVector['timestamps'] = {}

    for device in self.ConflictSubMatrix['setpoints']:
      compCount = 0
      compTotal = 0.0
      compTimestamp = 0
      otherCount = 0
      otherTotal = 0.0
      otherTimestamp = 0

      for app in self.ConflictSubMatrix['setpoints'][device]:
        if app=='resilience-app' or app=='decarbonization-app' or \
           app=='profit_cvr-app':
          compCount += 1
          compTotal += self.ConflictSubMatrix['setpoints'][device][app]
          compTimestamp = max(compTimestamp,
                              self.ConflictSubMatrix['timestamps'][app])
          if compCount == 3:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictSubMatrix['setpoints'][device][app]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictSubMatrix['timestamps'][app])

      if compCount > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionSubVector['setpoints'][device] = round(compTotal/compCount)
        else:
          ResolutionSubVector['setpoints'][device] = compTotal/compCount
        ResolutionSubVector['timestamps'][device] = compTimestamp
      elif otherCount > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionSubVector['setpoints'][device] =round(otherTotal/otherCount)
        else:
          ResolutionSubVector['setpoints'][device] = otherTotal/otherCount
        ResolutionSubVector['timestamps'][device] = otherTimestamp

    return (False, ResolutionSubVector)

