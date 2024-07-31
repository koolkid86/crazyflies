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

Modified by Sid ///  and Richard ////
email:
date: 7/30/2024


This modified code generates points and creates a leader-follower system
Leader moves to a randomly generated point
Follower goes to where the leader was.

TODO: avoid collisions

"""
import threading
import random
import time
import math
from collections import namedtuple
from queue import Queue

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig

# Time for one step in second
STEP_TIME = 1
#Default Drone Variables (These get changed by user imput)
X1 = 0.5   #Drone 1 X,Y,Z
Y1 = 0.8
Z1 = 0.3
X2 = 1
Y2 = 0.5
Z2 = 0.6
S1 = 2  # speed variables submitted to drones
S2 = 2
SY1 = 0.5  #Speed in m/s supplied by the user
SY2 = 0.5
PrevS1 = 2
PrevS2 = 2
RX1 = 0.5  # Random X,Y,Z positions
RY1 = 0.5
RZ1 = 0.5
prevRX1 = 1
prevRY1 = 1
prevRZ1 = 0.7
pointnum = 0  # counter for how many points the leader goes to.
random.seed(6544421)

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
    'radio://0/80/2M/E7E7E7E7E8',  # cf_id 0, startup position [-0.5, -0.5]
    'radio://0/80/2M/E7E7E7E7E6',  # cf_id 1, startup position [ 0, 0]
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


def sqDist2D(px,py,   qx,qy):
    return  (px-qx)**2 + (py-qy)**2

def minDistSimMovement( p0x,p0y, p1x,p1y, q0x,q0y, q1x,q1y, pSpeed, qSpeed):
    # As the drones p and q follow a path p0 -> p1, and q0->q1 (all in meters) at speeds pSpeed and qSpeed (m/s), 
    # returns the minimum distance from center points at the closest approach.
    minSqDist = sqDist2D(p0x,p0y,   q0x,q0y)
    distp     = sqDist2D(p0x,p0y,   p1x,p1y)
    distq     = sqDist2D(q0x,q0y,   q1x,q1y)
    pTime = distp/pSpeed
    qTime = distq/qSpeed
    maxTime = max(pTime,qTime)
    steps = 100  #only variable that controls resolution
    for i in range(steps):
        t = i*maxTime/steps  # in seconds
        if t<pTime:
            pix = p0x + pSpeed*t*(p1x-p0x)
            piy = p0y + pSpeed*t*(p1y-p0y)
        else:
            pix = p1x
            piy = p1y
        if t<qTime:
            qix = q0x + qSpeed*t*(q1x-q0x)
            qiy = q0y + qSpeed*t*(q1y-q0y)
        else:
            qix = q1x
            qiy = q1y
        
        iSqDist = sqDist2D(pix,piy,    qix,qiy)
        if iSqDist <  minSqDist:
            minSqDist = iSqDist
    return minSqDist**0.5


def inputs():
    #Default Drone Variables (These get changed by user imput)
    global X1
    global Y1
    global Z1
    global X2
    global Y2
    global Z2
    global RX1
    global RY1
    global RZ1
    global prevRX1
    global prevRY1
    global prevRZ1

    #Drone Speed
    global S1
    global S2
    global SY1
    global SY2
    global PrevS1
    global PrevS2
    PrevS1 = S1
    PrevS2 = S2
    global pointnum

    #Previous Variable Storage
    prevX1 = X1
    prevY1 = Y1
    prevZ1 = Z1
    prevX2 = X2
    prevY2 = Y2
    prevZ2 = Z2
    prevprevRX1 = prevRX1
    prevprevRY1 = prevRY1
    prevprevRZ1 = prevRZ1
    prevRX1 = RX1
    prevRY1 = RY1
    prevRZ1 = RZ1

    #Imput Command


    #Quick Math
    S1 = (SY1 - 0.625078) / -0.0675648
    S2 = (SY2 - 0.625078) / -0.0675648

    dumdum = 1.1

    #Randomization Unbiaser
    if RX1 <= 0.05:
        RX1 = RX1 + ((random.randint(1,100) / 100) * dumdum)
    elif RX1 >= 1.5:
        RX1 = RX1 + ((random.randint(-100,-1) / 100) * dumdum)
    else:
        RX1 = RX1 + ((random.randint(-100,100) / 100) * dumdum)
    
    if RY1 <= 0.05:
        RY1 = RY1 + ((random.randint(1,100) / 100) * dumdum)
    elif RY1 >= 1.15:
        RY1 = RY1 + ((random.randint(-100,-1) / 100) * dumdum)
    else:
        RY1 = RY1 + ((random.randint(-100,100) / 100) * dumdum)
    
    if RZ1 <= 0.2:
        RZ1 = RZ1 + ((random.randint(1,100) / 100) * dumdum)
    elif RZ1 >= 0.7:
        RZ1 = RZ1 + ((random.randint(-100,-1) / 100) * dumdum)
    else:
        RZ1 = RZ1 + ((random.randint(-100,100) / 100) * dumdum)

    #Quick Math Limiter
    if S1 >= 9:
        S1 = 9
    if S1 <= -35.15:
        S1 = -35.15
    if S2 >= 9:
        S2 = 9
    if S2 <= -35.15:
        S2 = -35.15

    if  prevRX1 - RX1 >= 0 and prevRX1 - RX1 <= 0.2:
        RX1 = 0.2
    if prevRX1 - RX1 <= 0 and prevRX1 - RX1 >= -0.2:
        RX1 = -0.2
    if prevRY1 - RY1 >= 0 and prevRY1 - RY1 <= 0.2:
        RY1 = 0.2
    if prevRY1 - RY1 <= 0 and prevRY1 - RY1 >= -0.2:
        RY1 = -0.2
    if prevRZ1 - RZ1 >= 0 and prevRZ1 - RZ1 <= 0.2:
        RZ1 = 0.2
    if prevRZ1 - RZ1 <= 0 and prevRZ1 - RZ1 >= -0.2:
        RZ1 = -0.2

    if pointnum == 5:
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
    else:
        pointnum = pointnum + 1

    #Landing Command
    if X1 == 100:

        return [
        (2, 0, Goto(prevX1, prevY1, 0.7, 1.85)),
        (2, 1, Goto(prevX2, prevY2, 0.4, 1.85)),
        (3, 0, Wait(2)),
        (3, 1, Wait(2)),
        (4, 0, Goto(0.57, -0.2, 0.7, 1.85)),
        (4, 1, Goto(0.85, -0.2, 0.4, 1.85)),
        (5, 0, Wait(2)),
        (5, 1, Wait(2)),
        (6, 0, Goto(0.57, -0.2, 0.1, 1.85)),
        (6, 1, Goto(0.85, -0.2, 0.1, 1.85)),
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

        if RX1 > 1.5:
            RX1 = 1.5
        if RX1 < 0.05:
            RX1 = 0.05
        if RY1 > 1.15:
            RY1 = 1.15
        if RY1 < 0.05:
            RY1 = 0.05
        if RZ1 > 0.7:
            RZ1 = 0.7
        if RZ1 < 0.2:
            RZ1 = 0.2
        
        pX1 = [prevRX1, prevRY1]
        pX2 = [RX1, RY1]
        pX3 = [prevprevRX1, prevprevRY1]
        pX4 = [prevRX1, prevRY1]


    if minDistSimMovement(prevRX1, prevRY1, RX1, RY1, prevprevRX1, prevprevRY1, prevRX1, prevRY1, S1, S2) <= 0.3:

        return [
        (2, 0, Wait(2)),
        (2, 1, Wait(2)),

        (3, 0, Goto(prevRX1, prevRY1, 0.7, S1)),
        (3, 1, Goto(prevprevRX1, prevprevRY1, 0.3, S2)),

        (4, 0, Wait(1)),
        (4, 1, Wait(1)),

        (5, 0, Goto(RX1, RY1, 0.7, S1)),
        (5, 1, Land(2)),

        (6, 0, Wait(1)),
        (6, 1, Wait(1)),

        (9, 0, Wait(2)),
        (9, 1, Goto(prevprevRX1, prevprevRY1, 0.3, S2)),

        (10, 0, Wait(1)),
        (10, 1, Wait(1)),

        (11, 0, Wait(2)),
        (11, 1, Goto(prevRX1, prevRY1, 0.3, S2)),

        (12, 0, Wait(1)),
        (12, 1, Wait(1)),

        (13, 0, Goto(RX1, RY1, RZ1, S1)),
        (13, 1, Goto(prevRX1, prevRY1, prevRZ1, S2)),
    ]

    else:

        return [
        (2, 0, Wait(2)),
        (2, 1, Wait(2)),

        (3, 0, Goto(prevRX1, prevRY1, 0.7, S1)),
        (3, 1, Goto(prevprevRX1, prevprevRY1, 0.3, S2)),

        (4, 0, Wait(2)),
        (4, 1, Wait(2)),

        (5, 0, Goto(RX1, RY1, 0.7, S1)),
        (5, 1, Goto(prevRX1, prevRY1, 0.3, S2)),

        (6, 0, Wait(2)),
        (6, 1, Wait(2)),

        (7, 0, Goto(RX1, RY1, RZ1, S1)),
        (7, 1, Goto(prevRX1, prevRY1, prevRZ1, S2)),
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
