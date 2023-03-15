
# Resilience exclusivity deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictSetpoints, ConflictTimestamps):
    self.ConflictSetpoints = ConflictSetpoints
    self.ConflictTimestamps = ConflictTimestamps


  def deconflict():
    Solution = {}

    for device in self.ConflictSetpoints:
      count = 0
      total = 0.0
      for app in self.ConflictSetpoints[device]:
        if app == 'resilience-app':
          Solution[device] = self.ConflictSetpoints[device][app]
          count = 0
          break
        else:
          count += 1
          total += self.ConflictSetpoints[device][app]

      if count > 0:
        Solution[device] = total/count

    return Solution

