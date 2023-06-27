import apps
from typing import List

if __name__ == '__main__':
    nameplate = apps.Nameplate(250, 10000, 0.975*0.86)
    
    batteries = []
    for i in range(2):
        batteries.append(apps.Battery(str(i), nameplate))
    
    decarb_app = apps.Decarbonization()
    resil_app = apps.Resilience()
    
    load = 4000
    green = 3888
    state = apps.State.NORMAL
    
    for battery in batteries:
        battery.power = 100
    decarb_app.dispatch(batteries, batteries, load, green, 60/15, state)
    resil_app.dispatch(batteries, [], load, green, 60/15, state)