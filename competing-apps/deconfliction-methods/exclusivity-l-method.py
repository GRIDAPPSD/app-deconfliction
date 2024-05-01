
# Load shedding exclusivity deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
    ResolutionVector = {}
    ResolutionVector['setpoints'] = {}
    ResolutionVector['timestamps'] = {}

    for device in self.ConflictMatrix:
      count = 0
      total = 0.0
      timestamp = 0

      for app in self.ConflictMatrix[device]:
        if app == 'load_shed-app':
          ResolutionVector['setpoints'][device] = \
                                   self.ConflictMatrix[device][app][1]
          ResolutionVector['timestamps'][device] = \
                                   self.ConflictMatrix[device][app][0]
          count = 0
          break
        else:
          count += 1
          total += self.ConflictMatrix[device][app][1]
          timestamp = max(timestamp, self.ConflictMatrix[device][app][0])

      if count > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionVector['setpoints'][device] = round(total/count)
        else:
          ResolutionVector['setpoints'][device] = total/count
        ResolutionVector['timestamps'][device] = timestamp

    return ResolutionVector

