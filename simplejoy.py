#! /usr/bin/python2.7 -OO
# -*- coding: utf-8 -*-
"""
A thread read joystick events till they block.

©2013 b@Zi.iS GPLv2
"""
from __future__ import division, absolute_import, print_function, unicode_literals
import struct
import threading
import sys

class Joystick(threading.Thread):
    DEVICE = "/dev/input/js%s"
    FORMAT = b"IhBB" # unsigned int, short, unsigned char, unsigned char
    SIZE = struct.calcsize(FORMAT)

    daemon = True

    axis = {}
    button = {}

    def __init__(self, device = 0):
        super(Joystick, self).__init__()
        self.device = open(self.DEVICE % device)

    def run(self):
        while True:
            self.read()

    def read(self):
        _, value, type, number = struct.unpack(self.FORMAT, self.device.read(self.SIZE))
        if type & ~0x80 == 0x01:
            self.button[number] = value
        if type & ~0x80 == 0x02:
            self.axis[number] = value
