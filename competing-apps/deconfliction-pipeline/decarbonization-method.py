
# Decarbonization exclusivity deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix, ConflictSubMatrix, BatterySoC,
               fullResolutionFlag=True):
    if fullResolutionFlag:
      self.ConflictMatrix = ConflictMatrix
    else:
      self.ConflictMatrix = ConflictSubMatrix

    self.BatterySoC = BatterySoC

    self.fullResolutionFlag = fullResolutionFlag



  def deconflict(self):
    ResolutionVector = {}
    ResolutionVector['setpoints'] = {}
    ResolutionVector['timestamps'] = {}

    for device in self.ConflictMatrix['setpoints']:
      count = 0
      total = 0.0
      timestamp = 0

      for app in self.ConflictMatrix['setpoints'][device]:
        if app == 'decarbonization-app':
          ResolutionVector['setpoints'][device] = \
                                   self.ConflictMatrix['setpoints'][device][app]
          ResolutionVector['timestamps'][device] = \
                                   self.ConflictMatrix['timestamps'][app]
          count = 0
          break
        else:
          count += 1
          total += self.ConflictMatrix['setpoints'][device][app]
          timestamp = max(timestamp, self.ConflictMatrix['timestamps'][app])

      if count > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(total/count)
        else:
          ResolutionVector['setpoints'][device] = total/count
        ResolutionVector['timestamps'][device] = timestamp

    return self.fullResolutionFlag, ResolutionVector

