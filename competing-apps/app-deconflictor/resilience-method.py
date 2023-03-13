
# Resilience exclusivity deconfliction methodology

class DeconflictionMethod:
  def __init__(self, AppIncludeList, AppExcludeList,
               ConflictSetpoints, ConflictTimestamps, Solution):
    self.ConflictSetpoints = ConflictSetpoints
    self.ConflictTimestamps = ConflictTimestamps
    self.Solution = Solution

    AppIncludeList.append('resilience-app')


  def deconflict():
    print('*** ERROR: deconflict method should never be called for Resilience Exclusivity deconfliction methodology!\n', flush=True)

