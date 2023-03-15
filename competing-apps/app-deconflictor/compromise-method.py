
# Resilience-Decarbonization compromise deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictSetpoints, ConflictTimestamps):
    self.ConflictSetpoints = ConflictSetpoints
    self.ConflictTimestamps = ConflictTimestamps


  def deconflict(self):
    Solution = {}

    for device in self.ConflictSetpoints:
      compCount = 0
      compTotal = 0.0
      otherCount = 0
      otherTotal = 0.0

      for app in self.ConflictSetpoints[device]:
        if app=='resilience-app' or app=='decarbonization-app':
          compCount += 1
          compTotal += self.ConflictSetpoints[device][app]
          if compCount == 2:
            break
        else:
          otherCount += 1
          otherTotal += self.ConflictSetpoints[device][app]

      if compCount > 0:
        Solution[device] = compTotal/compCount
      elif otherCount > 0:
        Solution[device] = otherTotal/otherCount

    return Solution

