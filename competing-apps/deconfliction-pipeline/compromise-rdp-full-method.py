
# Resilience-Decarbonization-Profit compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
    ResolutionVector = {}
    ResolutionVector['setpoints'] = {}
    ResolutionVector['timestamps'] = {}

    for device in self.ConflictMatrix['setpoints']:
      compCount = 0
      compTotal = 0.0
      compTimestamp = 0
      otherCount = 0
      otherTotal = 0.0
      otherTimestamp = 0

      for app in self.ConflictMatrix['setpoints'][device]:
        if app=='resilience-app' or app=='decarbonization-app' or \
           app=='profit_cvr-app':
          compCount += 1
          compTotal += self.ConflictMatrix['setpoints'][device][app]
          compTimestamp = max(compTimestamp,
                              self.ConflictMatrix['timestamps'][app])
          if compCount == 3:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictMatrix['setpoints'][device][app]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictMatrix['timestamps'][app])

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

