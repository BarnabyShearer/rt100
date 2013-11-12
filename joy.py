#! /usr/bin/python2.7 -OO
# -*- coding: utf-8 -*-
"""
Joystick interface

©2013 b@Zi.iS GPLv2
"""
from __future__ import division, absolute_import, print_function, unicode_literals

from rt100 import *
import simplejoy
import time
import copy

AXIS = [
    [0, SHOULDER],
    [1, ZED],
    [2, ELBOW],
    [3, YAW]
]
BUTTON = [
    [1, GRIPPER, 1],
    [2, GRIPPER, -1],
    [4, WRIST1, 1],
    [6, WRIST1, -1],
    [5, WRIST2, 1],
    [7, WRIST2, -1]
]

def scale(ctrl, max = 1):
    if ctrl == ZED:
        max = -max
    return (CTRLS[ctrl][3] - CTRLS[ctrl][2]) / 20 / max

def main(r):
    pos = []
    for ctrl in CTRLS:
        pos.append(0)
            
    joystick = simplejoy.Joystick()
    joystick.start()

    while True:
        time.sleep(.1)
        old_pos = copy.copy(pos)
        for axis in AXIS:
            pos[axis[1]] += int(joystick.axis[axis[0]] * scale(axis[1], 32767))
        for but in BUTTON:
            pos[but[1]] += int(joystick.button[but[0]] * but[2] * scale(but[1], 1))
        i = 0
        for ctrl in CTRLS:
            if pos[i] < ctrl[2]:
                pos[i] = ctrl[2]
            if pos[i] > ctrl[3]:
                pos[i] = ctrl[3]
            if pos[i] != old_pos[i]:
                r.numeric(ctrl, pos[i])
            i += 1
        r.start()
        print(pos)

if __name__ == "__main__":
    r = Rtx()
    main(r)
