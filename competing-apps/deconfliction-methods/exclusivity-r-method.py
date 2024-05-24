
# Resilience exclusivity deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictMatrix):
    self.ConflictMatrix = ConflictMatrix


  def deconflict(self, app_name, timestamp):
    ResolutionVector = {}

    for device in self.ConflictMatrix:
      count = 0
      total = 0.0
      timestamp = 0

      for app in self.ConflictMatrix[device]:
        if app == 'resilience-app':
          ResolutionVector[device] = self.ConflictMatrix[device][app]
          count = 0
          break
        else:
          count += 1
          total += self.ConflictMatrix[device][app][1]
          timestamp = max(timestamp, self.ConflictMatrix[device][app][0])

      if count > 0:
        if device.startswith('RatioTapChanger.'):
          ResolutionVector[device] = (timestamp, round(total/count))
        else:
          ResolutionVector[device] = (timestamp, total/count)

    return ResolutionVector

