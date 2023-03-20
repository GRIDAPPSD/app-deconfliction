
# Resilience exclusivity deconfliction methodology

class DeconflictionMethod:

  def __init__(self, ConflictSetpoints, ConflictTimestamps):
    self.ConflictSetpoints = ConflictSetpoints
    self.ConflictTimestamps = ConflictTimestamps


  def deconflict(self):
    SolutionSetpoints = {}
    SolutionTimestamps = {}

    for device in self.ConflictSetpoints:
      count = 0
      total = 0.0
      timestamp = 0

      for app in self.ConflictSetpoints[device]:
        if app == 'resilience-app':
          SolutionSetpoints[device] = self.ConflictSetpoints[device][app]
          SolutionTimestamps[device] = self.ConflictTimestamps[app]
          count = 0
          break
        else:
          count += 1
          total += self.ConflictSetpoints[device][app]
          timestamp = max(timestamp, self.ConflictTimestamps[app])

      if count > 0:
        SolutionSetpoints[device] = total/count
        SolutionTimestamps[device] = timestamp

    return SolutionSetpoints, SolutionTimestamps

