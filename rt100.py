#! /usr/bin/python2.7 -OO
# -*- coding: utf-8 -*-
"""
UMI RT100 controller

©2013 b@Zi.iS GPLv2
"""
from __future__ import division, absolute_import, print_function, unicode_literals

DEBUG = False

import sys
import time
import struct
import serial

DELAY = 0.01

STOP_POWERED, \
FORWARDS, \
BACKWARDS, \
STOP_UNPOWERED = range(4)

DEAD_STOP, \
RAMP_STOP, \
FREE_STOP, \
FREE_OFF = range(4)

CP_ERROR, \
CURRENT_POSITION, \
ERROR_LIMIT, \
NEW_POSITION, \
SPEED, \
KP, \
KI, \
KD, \
DEAD_BAND, \
OFFSET, \
MAX_FORCE, \
CURRENT_FORCE, \
ACCELERATION_TIME, \
USER_RAM, \
USER_IO, \
ACTUAL_POSITION = range(16)

CTRLS = [ #IP, CTRL, Min, Max
    [0, 2, -4000, 4000], #WRIST1
    [0, 3, -4000, 4000], #WRIST2
    [1, 0, -2630, 2206], #ELBOW
    [1, 1, -2630, 2630], #SHOULDER
    [1, 2, -3554, 0],    #ZED
    [1, 3, -1080, 1080], #YAW
    [1, 4, -30, 1200],   #GRIPPER
]

WRIST1, \
WRIST2, \
ELBOW, \
SHOULDER, \
ZED, \
YAW, \
GRIPPER = range(7)

class Rtx(object):
    """API for controlling the arm"""

    def __init__(self):
        """"""
        self.s = serial.Serial(
            port = "/dev/ttyUSB0",
            baudrate = 9600,
            bytesize = serial.EIGHTBITS,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            xonxoff = False,
            rtscts = False
        )
        self.s.write(chr(0x0))
        time.sleep(DELAY)
        while self.s.inWaiting():
            self.s.read()
        self._ip = None
        self._start = time.time()
        self.home()

    def home(self):
        """Move all joints to maximum then center"""

        #Allow power
        self.stop(FREE_OFF)

        #Set params
        for ctrl in CTRLS:
            self._deferredWrite(ctrl, SPEED, 150);
            self._deferredWrite(ctrl, MAX_FORCE, 30);

        self.home_ctrl(CTRLS[GRIPPER])        
        self.home_ctrl(CTRLS[ZED])
        self.home_ctrl(CTRLS[SHOULDER])
        self.home_ctrl(CTRLS[YAW])
        self.home_ctrl(CTRLS[ELBOW])
        self.home_wrist()

        print("HOME")
        
    def home_wrist(self):
        """Home both controllers in wrist"""

        for retry in range(3):
            self.manual(wrist1 = FORWARDS, wrist2 = FORWARDS)
            self.wait()
        for retry in range(3):
            self.manual(wrist1 = FORWARDS, wrist2 = BACKWARDS)
            self.wait()
        for retry in range(3):
            self.manual(wrist1 = FORWARDS, wrist2 = FORWARDS)
            self.wait()

        #Set pos
        self._immediateWrite(CTRLS[WRIST1], 2500)
        self._immediateWrite(CTRLS[WRIST2], -2500)

        self.manual() #stop

        #Zero
        self.numeric(CTRLS[WRIST1], 0)
        self.numeric(CTRLS[WRIST2], 0)
        self.start()
        self.wait()

    def home_ctrl(self, ctrl):
        """Home given controller"""

        moves = []
        for c in CTRLS:
            if c == ctrl:
                moves.append(FORWARDS)
            else:
                moves.append(STOP_POWERED)
        
        for retry in range(3):
            self.manual(*moves)
            self.wait()

        #Set to max
        self._immediateWrite(ctrl, ctrl[3])

        self.manual() #stop

        #Zero
        self.numeric(ctrl, 0)
        self.start()
        self.wait()

    def manual(
        self,
        wrist1 = STOP_POWERED,
        wrist2 = STOP_POWERED,
        elbow = STOP_POWERED,
        shoulder = STOP_POWERED,
        zed = STOP_POWERED,
        yaw = STOP_POWERED,
        gripper = STOP_POWERED,
    ):
        """Control manual movment"""
        ctrls = [0,0]
        ctrls[CTRLS[ZED][0]] += zed << (CTRLS[ZED][1] *2)
        ctrls[CTRLS[SHOULDER][0]] += shoulder << (CTRLS[SHOULDER][1] *2)
        ctrls[CTRLS[ELBOW][0]] += elbow << (CTRLS[ELBOW][1] *2)
        ctrls[CTRLS[YAW][0]] += yaw << (CTRLS[YAW][1] *2)
        ctrls[CTRLS[GRIPPER][0]] += gripper << (CTRLS[GRIPPER][1] *2)
        ctrls[CTRLS[WRIST1][0]] += wrist1 << (CTRLS[WRIST1][1] *2)
        ctrls[CTRLS[WRIST2][0]] += wrist2 << (CTRLS[WRIST2][1] *2)

        if self._cmd(
            0,
            0x80,
            ord(struct.pack(b'h', ctrls[0])[0]), ord(struct.pack(b'h', ctrls[0])[1])
        )[0] != 0:
            raise Exception("Move failed")

        if self._cmd(
            1,
            0x80,
            ord(struct.pack(b'h', ctrls[1])[0]), ord(struct.pack(b'h', ctrls[1])[1])
        )[0] != 0:
            raise Exception("Move failed")

    def wait(self):
        """Wait untill the robot isn't moving"""
        while self._cmd(0, 0x17)[1] & 1:
            time.sleep(DELAY*5)
        while self._cmd(1, 0x17)[1] & 1:
            time.sleep(DELAY*5)

    def mode(self, force_pos = 0, abs_rel = 1):
        #Dosn't seem to be working
        for ctrl in CTRLS:
            while self._cmd(ctrl[0], 0x10 + ctrl[1])[1] & 0x00001000 != (abs_rel << 4):
                print("Toggle ABS/REL")
                self._cmd(ctrl[0], 0x19, ctrl[1])
            while self._cmd(ctrl[0], 0x10 + ctrl[1])[1] & 0x00010000 != (force_pos << 5):
                print("Toggle Force/Pos")
                self._cmd(ctrl[0], 0x18, ctrl[1])

    def numeric(self, ctrl, value):
        """Set new destination for numeric movment"""
        if value < ctrl[2] or value > ctrl[3]:
            raise Exception("Move exceeds bounds")
        self._deferredWrite(ctrl, NEW_POSITION, value)

    def start(self):
        """Start the numeric movement"""
        #Note start dosn't allwas return success
        self._cmd(0, 0xAC)
        self._cmd(1, 0xBF)

    def stop(self, value):
        """Stop numeric Movement"""
        if self._cmd(0, 0x24 + value)[0] != 0:
            raise Exception("Stop failed")
        if self._cmd(1, 0x24 + value)[0] != 0:
            raise Exception("Stop failed")
        
    def _deferredWrite(self, ctrl, param, value):
        """Set a paramater using two frames"""
        if self._cmd(ctrl[0], 0x70 + ctrl[1], param, 0xCC)[0] != 0xE0:
            print("Defered Write failed")
        buf = self._cmd(ctrl[0], 0x78 + ctrl[1], ord(struct.pack(b'h', value)[0]), ord(struct.pack(b'h', value)[1]))
        if buf[0] != (self._checksum(value) + 0b11110000):
            raise Exception("Checksum Fail %s != %s" % (bin(buf[0])[2:].zfill(8), bin(self._checksum(value) + 0b11110000)[2:].zfill(8)))
        

    def _immediateWrite(self, ctrl, value):
        """Set a current pos using a single frame"""
        buf = self._cmd(ctrl[0], 0x58 + ctrl[1], ord(struct.pack(b'h', value)[0]), ord(struct.pack('h', value)[1]))
        if buf[0] != (self._checksum(value) + 0b10100000):
            raise Exception("Checksum Fail %s != %s" % (bin(buf[0])[2:].zfill(8), bin(self._checksum(value) + 0b10100000)[2:].zfill(8)))

    def _immediateRead(self, ctrl):
        """Read current pos using a single frame"""
        buf = b"".join(chr(c) for c in self._cmd(ctrl[0], 0x48 + ctrl[1])[1:])
        return struct.unpack(b'h', buf)[0]
    
    def _checksum(self, value):
        return (ord(struct.pack(b'h', value)[0]) & 0xF) ^ \
            (ord(struct.pack(b'h', value)[0]) >> 4) ^ \
            (ord(struct.pack(b'h', value)[1]) & 0xF) ^ \
            (ord(struct.pack(b'h', value)[1]) >> 4) 

    def _cmd(self, ip, cmd, b1 = None, b2 = 0):
        while ip != self._ip:
            if DEBUG:
                print("== Switching ==")
            if self.s.write(chr(0x29)) != 1:
                raise Exception("Write failed")
            time.sleep(DELAY)
            if ord(self.s.read()) != 0:
                raise Exception("Switch failed")
            if self.s.write(chr(0x01)) != 1:
                raise Exception("Write failed")
            time.sleep(DELAY)
            self._ip = ord(self.s.read()) - 0x20
            if DEBUG:
                print("New IP: %d" % self._ip)
            
        if b1 != None:
            if DEBUG:
                print(">%d %x %x %x" % (self._ip, cmd, b1, b2))
            if self.s.write(chr(cmd) + chr(b1) + chr(b2)) != 3:
                raise Exception("Write failed")
        else:
            if DEBUG:
                print(">%d %x" % (self._ip, cmd))
            if self.s.write(chr(cmd)) != 1:
                raise Exception("Write failed")
        time.sleep(DELAY)
        buf = []
        if DEBUG:
            print("%02.6f " % (time.time() - self._start), end="")
        while self.s.inWaiting() > 0:
            buf.append(ord(self.s.read()))
            if DEBUG:
                print(bin(buf[-1])[2:].zfill(8), end="")
        if DEBUG:
            print()

        return buf
    
if __name__ == "__main__":
    r = Rtx()
