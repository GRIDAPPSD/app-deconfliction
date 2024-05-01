
# Resilience-Decarbonization compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
    # if needed, battery SoC values are in the MethodUtil.BatterySoC dictionary

    ResolutionVector = {}
    ResolutionVector['setpoints'] = {}
    ResolutionVector['timestamps'] = {}

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
        ResolutionVector['timestamps'][device] = compTimestamp
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(compTotal/compCount)
          pass
        else:
          #ResolutionVector['timestamps'][device] = compTimestamp
          ResolutionVector['setpoints'][device] = compTotal/compCount
      elif otherCount > 0:
        ResolutionVector['timestamps'][device] = otherTimestamp
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(otherTotal/otherCount)
          pass
        else:
          #ResolutionVector['timestamps'][device] = otherTimestamp
          ResolutionVector['setpoints'][device] = otherTotal/otherCount

    return ResolutionVector

