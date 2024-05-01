
# Resilience-Decarbonization-Profit compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
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
        if app=='resilience-app' or app=='decarbonization-app' or \
           app=='profit_cvr-app':
          compCount += 1
          compTotal += self.ConflictMatrix[device][app][1]
          compTimestamp = max(compTimestamp,
                              self.ConflictMatrix[device][app][0])
          if compCount == 3:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictMatrix[device][app][1]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictMatrix[device][app][0])

      if compCount > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(compTotal/compCount)
        else:
          ResolutionVector['setpoints'][device] = compTotal/compCount
        ResolutionVector['timestamps'][device] = compTimestamp
      elif otherCount > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(otherTotal/otherCount)
        else:
          ResolutionVector['setpoints'][device] = otherTotal/otherCount
        ResolutionVector['timestamps'][device] = otherTimestamp

    return ResolutionVector

