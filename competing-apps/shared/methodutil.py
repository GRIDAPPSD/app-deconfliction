

class MethodUtil:
  ConflictSubMatrix = {}
  ConflictSubMatrix['setpoints'] = {}
  ConflictSubMatrix['timestamps'] = {}

  BatterySoC = {}


  def getConflictSubMatrix():
    return MethodUtil.ConflictSubMatrix


  def getBatterySoC():
    return MethodUtil.BatterySoC

