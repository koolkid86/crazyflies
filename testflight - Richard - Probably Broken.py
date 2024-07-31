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
Example of a follow the leader synchronized swarm choreography using the High level
commander.

The leader takes commands from the user, and the followers follow the leader
with a specified offset.
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
LEADER_STEP_TIME = 1.5
FOLLOWER_DELAY = 1  # Delay for followers to move to the leader's previous position

# Possible commands, all times are in seconds
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Goto = namedtuple('Goto', ['x', 'y', 'z', 'time'])
# RGB [0-255], Intensity [0.0-1.0]
Ring = namedtuple('Ring', ['r', 'g', 'b', 'intensity', 'time'])
# Reserved for the control loop, do not use in sequence
Quit = namedtuple('Quit', [])

uris = [
    'radio://0/80/2M/E7E7E7E7E6',  # Leader cf_id 0
    'radio://0/80/2M/E7E7E7E7E7',  # Follower cf_id 1
]

leader_position = [0.0, 0.0, 0.0]
previous_leader_position = [0.0, 0.0, 0.0]

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
            break
        elif type(command) is Land:
            commander.land(0.0, command.time)
        else:
            commander.go_to(command.x, command.y, command.z, 0, command.time)

    commander.land(0.0, 2)  # Ensure the drone lands before quitting
    time.sleep(2)

def leader_control():
    global leader_position, previous_leader_position

    X1 = float(input("X1 coord: "))
    Y1 = float(input("Y1 coord: "))
    Z1 = float(input("Z1 coord: "))

    previous_leader_position = leader_position.copy()
    leader_position = [X1, Y1, Z1]

    return [
        (0, 0, Goto(X1, Y1, Z1, LEADER_STEP_TIME)),
    ]

def follower_control():
    global previous_leader_position

    X1 = previous_leader_position[0]
    Y1 = previous_leader_position[1]
    Z1 = previous_leader_position[2]

    return [
        (1, 0, Goto(X1, Y1, Z1, LEADER_STEP_TIME + FOLLOWER_DELAY)),
    ]

def control_thread():
    while True:
        leader_sequence = leader_control()
        follower_sequence = follower_control()

        for seq in leader_sequence:
            cf_id = seq[1]
            command = seq[2]
            controlQueues[cf_id].put(command)

        time.sleep(FOLLOWER_DELAY)

        for seq in follower_sequence:
            cf_id = seq[1]
            command = seq[2]
            controlQueues[cf_id].put(command)

        time.sleep(LEADER_STEP_TIME)

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
