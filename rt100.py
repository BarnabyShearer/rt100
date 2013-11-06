#! /usr/bin/python2.7 -OO
# -*- coding: utf-8 -*-
"""
UMI RT100 controller

©2013 b@Zi.iS GPLv2
"""
from __future__ import division, absolute_import, print_function, unicode_literals

import sys
import time
import struct
import serial
import threading
import signal

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
    [0, 2, -3000, 3000], #WRIST1
    [0, 3, -3000, 3000], #WRIST2
    [1, 0, -3554, 0],    #ZED
    [1, 1, -2630, 2630], #SHOULDER
    [1, 2, -2630, 2206], #ELBOW
    [1, 3, -1806, 1806], #YAW
    [1, 4, -30, 1200],   #GRIPPER
]

WRIST1, \
WRIST2, \
ZED, \
SHOULDER, \
ELBOW, \
YAW, \
GRIPPER = range(7)

class Comms(threading.Thread):
    """
    Thread for handling serial communication
    Sends commands or NOOPs at 125Hz as per manual
    """
    def __init__(self, port, ips = 2):
        """Open a serial port"""
        threading.Thread.__init__(self)
        self.__running = True
        self.s = serial.Serial(port, baudrate = 9600, rtscts = True)
        self.s.write('\0')
        time.sleep(.01)
        self.s.read(self.s.inWaiting())
        self.__cmd = None
        self.__ip = 0
        self.__ips = ips
        self.ready = threading.Event()
        print("Comms ready")

    def stop(self):
        """Cleanly exit thread"""
        self.__running = False

    def cmd(self, ip, cmd, b1 = None, b2 = 0):
        self.ready.clear()
        self.__cmd = [ip, cmd, b1, b2]

    def run(self):
        """Communication loop"""
        next = 0
        while self.__running:
            tick = time.time()
            if not tick - next > 0:
                time.sleep(next - tick)
            next = time.time() + .125
            if self.__cmd == None or self.__cmd[0] != self.__ip:
                print(">%d NOOP" % self.__ip)
                self.s.write('\x29')
                self.__ip = ord(self.s.read()) - 0x20
                print("<! %s" % hex(self.s.read(self.s.inWaiting())))
            else:
                if self.__cmd[2] != None:
                    print(">%d %x %x %x" % ([self.__ip] +  self.__cmd[1:]))
                    if self.s.write(chr(self.__cmd[1]) + chr(self.__cmd[2]) + chr(self.__cmd[3])) != 3:
                        print("Write failed")
                    time.sleep(.01) #TODO: check
                    buf = self.s.read()
                else:
                    print(">%d %x" % (self.__ip, self.__cmd[1]))
                    if self.s.write(chr(self.__cmd[1])) != 1:
                        print("Write failed")
                    time.sleep(.01) #TODO: check
                    buf = self.s.read(1)
                    buf += self.s.read(self.s.inWaiting())
                print("<%d" % self.__ip, end = "")
                buf = [ord(c) for c in buf]
                for c in buf:
                    print(bin(c), end = "")
                print()
                self.__cmd = None
                self.reply = buf
                self.__ip += 1
                self.ready.set()
            self.__ip %= self.__ips            

class Rtx(object):
    """API for controlling the arm"""

    def __init__(self, comms):
        """"""
        self.comms = comms

    def home(self):
        """Move all joints to maximum then center"""

        #Allow power
        self.stop(FREE_OFF)

        #Set params
        for ctrl in CTRLS:
            self.deferredWrite(ctrl, SPEED, 150);
            self.deferredWrite(ctrl, MAX_FORCE, 30);

        self.manual(
            FORWARDS,
            FORWARDS,
            FORWARDS,
            FORWARDS,           
            FORWARDS,
            FORWARDS,
            FORWARDS
        );

        self.wait()

        self.manual() #stop

        #Tell robot it is home
        self._cmd(0, 0x21)
        self._cmd(1, 0x21)
            
        #Set Pos to max
        self.pos = []
        for ctrl in CTRLS:
            self.pos.push(ctrl[3])
            self._immediateWrite(ctrl, CURRENT_POSITION, ctrl[3])

        #Go to 0
        for ctrl in CTRLS:
            self.numeric(ctrl, 0)
        self.start()

    def manual(
        self,
        zed = STOP_POWERED,
        sholder = STOP_POWERED,
        elbow = STOP_POWERED,
        yaw = STOP_POWERED,
        gripper = STOP_POWERED,
        wrist1 = STOP_POWERED,
        wrist2 = STOP_POWERED
    ):
        """Control manual movment"""
        self._cmd(
            0,
            0x80,
            wrist1 << 2 + wrist2
        );
        self._cmd(
            1,
            0x80 + gripper,
            yaw << 6 + elbow << 4 + shoulder << 2 + zed
        );

    def wait(self):
        """Wait untill the robot isn't moving"""
        op1 = False
        op2 = False
        while not(op1 and op2):
            op1 = not(self._cmd(0, 0x17)[2] & 1)
            op2 = not(self._cmd(1, 0x17)[2] & 1)

    def numeric(self, ctrl, value):
        """Set new destination for numeric movment"""
        if value < ctrl[2] or value > ctrl[3]:
            raise Exception("Move exceeds bounds")
        self.deferredWrite(ctrl, NEW_POSITION, value)

    def start(self):
        """Start the numeric movement"""
        self._cmd(0, 0xAC)
        self._cmd(1, 0xBF)

    def stop(self, value):
        """Stop numeric Movement"""
        self._cmd(0, 0x24 + value)
        self._cmd(1, 0x24 + value)
        
    def _deferredWrite(self, ctrl, param, value):
        """Set a paramater using two frames"""
        self._cmd(ctrl[0], 0x70 + ctrl[1], param, 0xCC)
        self._cmd(ctrl[0], 0x78 + ctrl[1], struct.pack('h', val)[0], struct.pack('h', val)[0], struct.pack('h', val)[1])

    def _immediateWrite(self, ctrl, val):
        """Set a paramater using a single frame"""
        self._cmd(ctrl[0], 0x58 + ctrl, struct.pack('h', val)[0], struct.pack('h', val)[0], struct.pack('h', val)[1])
       
    def _cmd(self, ip, cmd, b1 = None, b2 = 0):
        """Send a command"""
        self.comms.cmd(ip, cmd, b1, b2)
        self.comms.ready.wait()
        return self.comms.reply

if __name__ == "__main__":
    comms = Comms('/dev/ttyUSB0')
    def signal_handler(signal, frame):
        comms.stop()
        comms.join()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    comms.start()
    r = Rtx(comms)
    signal.pause()
