
# Resilience-Decarbonization compromise deconfliction methodology

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
    self.ConflictSubMatrix = MethodUtil.ConflictSubMatrix


  def deconflict(self):
    # if needed, battery SoC values are in the MethodUtil.BatterySoC dictionary

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
        if app=='resilience-app' or app=='decarbonization-app':
          compCount += 1
          compTotal += self.ConflictSubMatrix['setpoints'][device][app]
          compTimestamp = max(compTimestamp,
                              self.ConflictSubMatrix['timestamps'][app])
          if compCount == 2:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictSubMatrix['setpoints'][device][app]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictSubMatrix['timestamps'][app])

      if compCount > 0:
        # for resolution with only batteries comment out the next line,
        # comment out the first line under the RatioTapChanger if block,
        # uncomment the first line under the else block, then repeat those
        # steps under the elif below
        ResolutionSubVector['timestamps'][device] = compTimestamp
        if device.startswith('RatioTapChanger.'):
          ResolutionSubVector['setpoints'][device] = round(compTotal/compCount)
          pass
        else:
          #ResolutionSubVector['timestamps'][device] = compTimestamp
          ResolutionSubVector['setpoints'][device] = compTotal/compCount
      elif otherCount > 0:
        ResolutionSubVector['timestamps'][device] = otherTimestamp
        if device.startswith('RatioTapChanger.'):
          ResolutionSubVector['setpoints'][device] =round(otherTotal/otherCount)
          pass
        else:
          #ResolutionSubVector['timestamps'][device] = otherTimestamp
          ResolutionSubVector['setpoints'][device] = otherTotal/otherCount

    return (False, ResolutionSubVector)

