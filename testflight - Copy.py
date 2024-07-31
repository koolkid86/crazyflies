import time
from decimal import Decimal
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import CachedCfFactory, Swarm

# URI to the Crazyflie to connect to

uri = 'radio://0/80/2M/E7E7E7E7E6'

# 0 = leader, 1 = 1st follower, 2 = 2nd follower ...
params0 = {'s': 0}
params1 = {'s': 1}

BLACK = [0, 0, 0]
X1 = 0.75
Y1 = 0.75
Z1 = 0.5

def run_sequence(cf, pc, X1, Y1, Z1, params):
    if Z1 > 0.7:
        Z1 = 0.7
    if Z1 < 0.1:
        Z1 = 0.1
    if Y1 > 1.15:
        Y1 = 1.15
    if Y1 < 0.05:
        Y1 = 0.05
    if X1 > 1.5:
        X1 = 1.5
    if X1 < 0.05:
        X1 = 0.05

    time.sleep(0.1)
    print("Reached target coordinates!")
    time.sleep(0.3)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Connect to each Crazyflie separately
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                with PositionHlCommander(scf, default_height=0.5, controller=PositionHlCommander.CONTROLLER_PID) as pc:

                    while True:
                        
                        
                        X1 = float(input("X1 coord: "))
                        Y1 = float(input("Y1 coord: "))
                        Z1 = float(input("Z1 coord: "))

                        run_sequence(scf.cf, pc, X1, Y1, Z1)
