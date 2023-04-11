from typing import List
from enum import Enum
import numpy as np
from math import dist
import pulp

INITIAL_SOC = 0.5
MAX_SOC = 0.9
MIN_SOC = 0.2
MAX_POWER_SUB = 10000

class State(Enum):
    NORMAL = 1
    EMERGENCY = 2
    
class Consumers(object):
    def __init__(self) -> None:
        pass

class PV(object):
    def __init__(self) -> None:
        pass
    
class Nameplate(object):
    def __init__(self, power: int, energy: int, efficiency: float) -> None:
        self.power = power
        self.energy = energy
        self.efficiency = efficiency
    
class Battery(object):        
    def __init__(self, name: str, nameplate: Nameplate) -> None:
        self.name = name
        self.rated = nameplate
        self.state_of_charge = INITIAL_SOC
        self.power = 0
        
    def charge_capacity(self) -> float:
        return self.rated.energy * (MAX_SOC - self.state_of_charge)
        
    def discharge_capacity(self) -> float:
        return -self.rated.energy * (self.state_of_charge - MIN_SOC)

    def update(self, delta_time: float) -> None:
        if self.power > 0:
            energy = self.charge(self.power, delta_time)
            self.state_of_charge += energy / self.rated.energy
            
            if self.state_of_charge > MAX_SOC:
                self.state_of_charge = MAX_SOC
                
        if self.power < 0:
            energy = self.discharge(self.power, delta_time)
            self.state_of_charge += energy / self.rated.energy
        
            if self.state_of_charge < MIN_SOC:
                self.state_of_charge = MIN_SOC

    def charge(self, power: float, delta_time: float) -> float:
        eta = self.rated.efficiency
        return eta * power * delta_time

    def discharge(self, power: float, delta_time: float) -> float:   
        eta = 1 / self.rated.efficiency
        return eta * power * delta_time

class Decarbonization(object):
    def __init__(self) -> None:
        pass
    
    def dispatch(self,
                 batteries: List[Battery],
                 advise: List[Battery],
                 load: float,
                 renewable: float,
                 delta_time: float, 
                 state: State) -> List[Battery]:
        
        #feasability we must curtail renewable if we cannot store excess
        total_power = np.sum([battery.rated.power for battery in batteries])
        if renewable - load > total_power:
            renewable = total_power + load
            
        # problem
        prob = pulp.LpProblem('dirty_generation', pulp.LpMinimize)
        
        # operating variables
        power_dirty = pulp.LpVariable('power_dirty', 0, MAX_POWER_SUB)
        power_battery = [pulp.LpVariable(f'{battery.name}_power', -battery.rated.power, battery.rated.power) for battery in batteries]
        
        # BASE: constraints and objective  
        if state is State.EMERGENCY:
            prob += load == renewable - pulp.lpSum(power_battery)
            prob += power_dirty == 0
        
        prob += power_dirty == load - renewable + pulp.lpSum(power_battery)
                  
        for i,battery in enumerate(batteries):
            prob += battery.charge(power_battery[i], delta_time) <= battery.charge_capacity()
            prob += battery.discharge(power_battery[i], delta_time) >= battery.discharge_capacity()
        
        # ADVISE: constraints and objective
        if advise and not State.EMERGENCY:
            prob += pulp.lpSum(power_battery) <= renewable
            prob += pulp.lpSum([dist([p],[advise[i].power]) for i,p in enumerate(power_battery)])
        
        # PRESS: constraints and objective
        # if press and not State.EMERGENCY:

        # COMPENSATE: constraints and objective
        # if compensate and not State.EMERGENCY:
            
        # objective

            
        prob += power_dirty

        prob.writeLP('Decarbonization.lp')
        prob.solve(pulp.PULP_CBC_CMD(msg=0))
        print(f'Status: {pulp.LpStatus[prob.status]}')
        
        for p in power_battery:
            print(p.varValue)
        
        return batteries
    
class Resilience(object):
    def __init__(self) -> None:
        pass
    
    def dispatch(self,
                 batteries: List[Battery],
                 advise: List[Battery],
                 load: float,
                 renewable: float,
                 delta_time: float, 
                 state: State) -> List[Battery]:
        
        #feasability we must curtail renewable if we cannot store excess
        total_power = np.sum([battery.rated.power for battery in batteries])
        if renewable - load > total_power:
            renewable = total_power + load            
        
        # problem
        prob = pulp.LpProblem('power_delivered', pulp.LpMaximize)
        
        # operating variables
        power_delivered = pulp.LpVariable('power_delivered', 0, MAX_POWER_SUB)
        power_battery = [pulp.LpVariable(f'{battery.name}_power', -battery.rated.power, battery.rated.power) for battery in batteries]
        
        # BASE: constraints and objective  
        if state is State.EMERGENCY:
            prob += load == renewable - pulp.lpSum(power_battery)
            prob += power_delivered == 0
            
        prob += power_delivered == load - renewable + pulp.lpSum(power_battery)
                  
        for i,battery in enumerate(batteries):
            prob += battery.charge(power_battery[i], delta_time) <= battery.charge_capacity()
            prob += battery.discharge(power_battery[i], delta_time) >= battery.discharge_capacity()
        
        # ADVISE: constraints and objective
        if advise and not State.EMERGENCY:
            prob += pulp.lpSum(power_battery) <= renewable
            prob += pulp.lpSum([dist([p],[advise[i].power]) for i,p in enumerate(power_battery)])
        
        # PRESS: constraints and objective
        # if press and not State.EMERGENCY:

        # COMPENSATE: constraints and objective
        # if compensate and not State.EMERGENCY:
            
        # objective

            
        prob += power_delivered

        prob.writeLP('Resilience.lp')
        prob.solve(pulp.PULP_CBC_CMD(msg=0))
        print(f'Status: {pulp.LpStatus[prob.status]}')
        
        for p in power_battery:
            print(p.varValue)
        
        return batteries