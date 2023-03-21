
# Resilience-Decarbonization compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictSetpoints, ConflictTimestamps):
    self.ConflictSetpoints = ConflictSetpoints
    self.ConflictTimestamps = ConflictTimestamps


  def deconflict(self):
    SolutionSetpoints = {}
    SolutionTimestamps = {}

    for device in self.ConflictSetpoints:
      compCount = 0
      compTotal = 0.0
      compTimestamp = 0
      otherCount = 0
      otherTotal = 0.0
      otherTimestamp = 0

      for app in self.ConflictSetpoints[device]:
        if app=='resilience-app' or app=='decarbonization-app':
          compCount += 1
          compTotal += self.ConflictSetpoints[device][app]
          compTimestamp = max(compTimestamp, self.ConflictTimestamps[app])
          if compCount == 2:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictSetpoints[device][app]
          otherTimestamp = max(otherTimestamp, self.ConflictTimestamps[app])

      if compCount > 0:
        SolutionSetpoints[device] = compTotal/compCount
        SolutionTimestamps[device] = compTimestamp
      elif otherCount > 0:
        SolutionSetpoints[device] = otherTotal/otherCount
        SolutionTimestamps[device] = otherTimestamp

    return SolutionSetpoints, SolutionTimestamps

