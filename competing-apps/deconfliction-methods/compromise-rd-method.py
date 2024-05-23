
# Resilience-Decarbonization compromise deconfliction methodology

import MethodUtil

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
    # if needed, battery SoC values are in the MethodUtil.BatterySoC dictionary
    ResolutionVector = {}

    for device in self.ConflictMatrix:
      compCount = 0
      compTotal = 0.0
      compTimestamp = 0
      otherCount = 0
      otherTotal = 0.0
      otherTimestamp = 0

      for app in self.ConflictMatrix[device]:
        if app=='resilience-app' or app=='decarbonization-app':
          compCount += 1
          compTotal += self.ConflictMatrix[device][app][1]
          compTimestamp = max(compTimestamp,
                              self.ConflictMatrix[device][app][0])
          if compCount == 2:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictMatrix[device][app][1]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictMatrix[device][app][0])

      if compCount > 0:
        # for resolution with only batteries comment out the next line,
        # comment out the first line under the RatioTapChanger if block,
        # uncomment the first line under the else block, then repeat those
        # steps under the elif below
        name = MethodUtil.DeviceToName[device]
        if name.startswith('RatioTapChanger.'):
          ResolutionVector[device] = (compTimestamp, round(compTotal/compCount))
          pass
        else:
          ResolutionVector[device] = (compTimestamp, compTotal/compCount)
      elif otherCount > 0:
        name = MethodUtil.DeviceToName[device]
        if name.startswith('RatioTapChanger.'):
          ResolutionVector[device] = \
                                  (otherTimestamp, round(otherTotal/otherCount))
          pass
        else:
          ResolutionVector[device] = (otherTimestamp, otherTotal/otherCount)

    return ResolutionVector

