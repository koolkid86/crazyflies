#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example of a synchronized swarm choreography using the High level
commander.

The swarm takes off and flies a synchronous choreography before landing.
The take-of is relative to the start position but the Goto are absolute.
The sequence contains a list of commands to be executed at each step.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class to achieve synchronous sequences.
"""
import threading
import time
from collections import namedtuple
from queue import Queue

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.position_hl_commander import PositionHlCommander

# Time for one step in second
STEP_TIME = 1
#Default Drone Variables (These get changed by user imput)
X1 = 0.5
Y1 = 0.8
Z1 = 0.3
X2 = 1
Y2 = 0.5
Z2 = 0.6
S1 = 2
S2 = 2
SY1 = 0.5
SY2 = 0.5
PrevS1 = 2
PrevS2 = 2


# Possible commands, all times are in seconds
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Goto = namedtuple('Goto', ['x', 'y', 'z', 'time'])
# RGB [0-255], Intensity [0.0-1.0]
Ring = namedtuple('Ring', ['r', 'g', 'b', 'intensity', 'time'])
# Reserved for the control loop, do not use in sequence
Quit = namedtuple('Quit', [])
Wait = namedtuple('Wait', ['time'])


uris = [
    'radio://0/80/2M/E7E7E7E7E6',  # cf_id 0, startup position [-0.5, -0.5]
    'radio://0/80/2M/E7E7E7E7E8',  # cf_id 1, startup position [ 0, 0]
    # Add more URIs if you want more copters in the swarm
]

def activate_mellinger_controller(scf, use_mellinger):
    controller = PositionHlCommander.CONTROLLER_PID
    if use_mellinger:
        controller = PositionHlCommander.CONTROLLER_PID
    scf.cf.param.set_value('stabilizer.controller', str(controller))

def set_ring_color(cf, r, g, b, intensity, time):
    cf.param.set_value('ring.fadeTime', str(time))

    r *= intensity
    g *= intensity
    b *= intensity

    color = (int(r) << 16) | (int(g) << 8) | int(b)

    cf.param.set_value('ring.fadeColor', str(color))

def crazyflie_control(scf):
    cf = scf.cf
    control = controlQueues[uris.index(cf.link_uri)]

    activate_mellinger_controller(scf, False)

    commander = scf.cf.high_level_commander

    # Set fade to color effect and reset to Led-ring OFF
    set_ring_color(cf, 0, 0, 0, 0, 0)
    cf.param.set_value('ring.effect', '14')

    commander.takeoff(0.3, 3)

    while True:
        command = control.get()
        
        if type(command) is Quit:
            commander.go_to(0.5, 0.5, 0.3, 0, 2)
            commander.go_to(0.75, 0.75, 0.7, 0, 2)
            commander.go_to(0.5, 0.5, 0.1, 0, 2)
            commander.go_to(0.75, 0.75, 0.1, 0, 2)
            commander.land (0.0)

            break
        elif type(command) is Land:
            commander.land(0.0, command.time)
        elif type(command) is Wait:
            time.sleep(command.time)
        else:
            commander.go_to(command.x, command.y, command.z, 0, command.time)

    commander.land(0.0, 2)  # Ensure the drone lands before quitting
    time.sleep(2)

def inputs():
    #Default Drone Variables (These get changed by user imput)
    global X1
    global Y1
    global Z1
    global X2
    global Y2
    global Z2

    #Drone Speed
    global S1
    global S2
    global SY1
    global SY2
    global PrevS1
    global PrevS2
    PrevS1 = S1
    PrevS2 = S2

    #Previous Variable Storage
    prevX1 = X1
    prevY1 = Y1
    prevZ1 = Z1
    prevX2 = X2
    prevY2 = Y2
    prevZ2 = Z2
    
    #Imput Command
    X1 = float(input("X1 coord: "))
    Y1 = float(input("Y1 coord: "))
    Z1 = float(input("Z1 coord: "))
    SY1 = float(input("Speed 1 in m/s "))


    #Quick Math
    S1 = (SY1 - 0.625078) / -0.0675648
    S2 = (SY2 - 0.625078) / -0.0675648
    #Quick Math Limiter
    if S1 >= 9:
        S1 = 9
    if S1 <= -35.15:
        S1 = -35.15
    if S2 >= 9:
        S2 = 9
    if S2 <= -35.15:
        S2 = -35.15


    #Landing Command
    if X1 == 100:

        return [
        (2, 0, Goto(prevX1, prevY1, 0.7, 5)),
        (2, 1, Goto(prevX2, prevY2, 0.4, 5)),
        (3, 0, Wait(2)),
        (3, 1, Wait(2)),
        (4, 0, Goto(0.57, -0.2, 0.7, 5)),
        (4, 1, Goto(0.85, -0.2, 0.4, 5)),
        (5, 0, Wait(2)),
        (5, 1, Wait(2)),
        (6, 0, Goto(0.57, -0.2, 0.1, 5)),
        (6, 1, Goto(0.85, -0.2, 0.1, 5)),
        (7, 0, Wait(2)),
        (7, 1, Wait(2)),
        (8, 0, Land(2)),
        (8, 1, Land(2)),
    ]
    #Drone Flight Command Info
    else:
        #Drone 1 Bounding Box
        if X1 > 1.5:
            X1 = 1.5
        if X1 < 0.05:
            X1 = 0.05
        if Y1 > 1.15:
            Y1 = 1.15
        if Y1 < 0.05:
            Y1 = 0.05
        if Z1 > 0.7:
            Z1 = 0.7
        if Z1 < 0.1:
            Z1 = 0.1

        #Drone 2 Bounding Box
        if X2 > 1.5:
            X2 = 1.5
        if X2 < 0.05:
            X2 = 0.05
        if Y2 > 1.15:
            Y2 = 1.15
        if Y2 < 0.05:
            Y2 = 0.05
        if Z2 > 0.7:
            Z2 = 0.7
        if Z2 < 0.1:
            Z2 = 0.1
        return [
        (2, 0, Goto(1.38, 0.1, 0.5, S1)),
        (2, 1, Goto(1.47, 0.6, 0.5, S1)),
        (3, 1, Goto(1.38, 0.1, 0.5, S1)),
        (3, 0, Goto(0.88, 0.11, 0.5, S1)),
        (4, 1, Goto(0.88, 0.11, 0.5, S1)),
        (4, 0, Goto(0.5, 0.1, 0.5, S1)),
        (5, 1, Goto(0.5, 0.1, 0.5, S1)),
        (5, 0, Goto(0.09, 0.09, 0.5, S1)),
        (6, 1, Goto(0.09, 0.09, 0.5, S1)),
        (6, 0, Goto(0.2, 0.44, 0.5, S1)),
        (7, 1, Goto(0.2, 0.44, 0.5, S1)),
        (7, 0, Goto(0.15, 0.87, 0.5, S1)),
        (8, 1, Goto(0.15, 0.87, 0.5, S1)),
        (8, 0, Goto(0.17, 1.13, 0.5, S1)),
        (9, 1, Goto(0.17, 1.13, 0.5, S1)),
        (9, 0, Goto(0.52, 1.12, 0.5, S1)),
        (10, 1, Goto(0.52, 1.12, 0.5, S1)),
        (10, 0, Goto(1.02, 1.02, 0.5, S1)),
        (11, 1, Goto(1.02, 1.02, 0.5, S1)),
        (11, 0, Goto(1.44, 0.4, 0.5, S1)),
        (12, 1, Goto(1.44, 0.4, 0.5, S1)),
        (12, 0, Goto(0.84, 0.57, 0.5, S1)),
        (13, 1, Goto(0.84, 0.57, 0.5, S1)),
        (13, 0, Goto(0.84, 0.30, 0.5, S1)),
        (14, 1, Goto(0.84, 0.30, 0.5, S1)),
        (14, 0, Goto(0.62, 0.57, 0.5, S1)),
        (15, 1, Goto(0.62, 0.57, 0.5, S1)),
        (15, 0, Goto(0.85, 0.85, 0.5, S1)),
        (16, 1, Goto(0.85, 0.85, 0.5, S1)),
        (16, 0, Goto(0.84, 0.58, 0.5, S1)),
        (17, 1, Goto(0.84, 0.58, 0.5, S1)),
        (17, 0, Goto(0.61, 0.57, 0.5, S1)),
        (18, 1, Goto(0.61, 0.57, 0.5, S1)),
        (18, 0, Goto(0.88, 0.11, 0.5, S1)),
        (19, 1, Goto(0.88, 0.11, 0.5, S1)),
        (19, 0, Goto(0.31, 0.23, 0.5, S1)),
        (20, 1, Goto(0.31, 0.23, 0.5, S1)),
        (20, 0, Goto(0.33, 0.58, 0.5, S1)),
        (21, 1, Goto(0.33, 0.58, 0.5, S1)),
        (21, 0, Goto(0.36, 0.88, 0.5, S1)),
        (22, 1, Goto(0.36, 0.88, 0.5, S1)),
        (22, 0, Goto(0.79, 1.03, 0.5, S1)),
        (23, 1, Goto(0.79, 1.03, 0.5, S1)),
        (23, 0, Goto(1.31, 1.04, 0.5, S1)),
    ]

    

def control_thread():
    while True:
        sequence = inputs()
        pointer = 0
        step = 0
        stop = False

        while not stop:
            while sequence[pointer][0] <= step:
                cf_id = sequence[pointer][1]
                command = sequence[pointer][2]

                controlQueues[cf_id].put(command)
                pointer += 1

                if pointer >= len(sequence):
                    pointer = 0
                    step = 0
                    stop = True
                    break

                
            step += 1
            time.sleep(STEP_TIME)

            

if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.reset_estimators()

        print('Starting sequence!')

        control_thread_instance = threading.Thread(target=control_thread)
        control_thread_instance.start()

        swarm.parallel_safe(crazyflie_control)

        control_thread_instance.join()  # Wait for the control thread to finish

        print('Sequence finished!')