
# Resilience-Decarbonization compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix, ConflictOnlyMatrix,
               fullResolutionFlag=True):
    if fullResolutionFlag:
      self.ConflictMatrix = ConflictMatrix
    else:
      self.ConflictMatrix = ConflictOnlyMatrix

    self.fullResolutionFlag = fullResolutionFlag


  def deconflict(self):
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
        if app=='resilience-app' or app=='decarbonization-app':
          compCount += 1
          compTotal += self.ConflictMatrix['setpoints'][device][app]
          compTimestamp = max(compTimestamp,
                              self.ConflictMatrix['timestamps'][app])
          if compCount == 2:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictMatrix['setpoints'][device][app]
          otherTimestamp = max(otherTimestamp,
                               self.ConflictMatrix['timestamps'][app])

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

    return self.fullResolutionFlag, ResolutionVector

