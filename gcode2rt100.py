#! /usr/bin/env python
import os
import select
import re
import time
import sys
import rt100
import math
import numpy

STOP_POWERED, FORWARDS, BACKWARDS, STOP_UNPOWERED = range(4)
WRIST1, WRIST2, ELBOW, SHOULDER, ZED, YAW, GRIPPER = range(7)


ZED_RANGE = 1000;
SHOULDER_RANGE = math.pi
ELBOW_RANGE = 1.8*math.pi
ARM_LENGTH = (290,290)

class Circle():
    def __init__(self, c, r):
        self.c = numpy.array(c)
        self.r = r

    def intersections(self, other):
        d = numpy.linalg.norm(self.c - other.c)
        a = (self.r**2 - other.r**2 + d**2)/(2*d)
        h = math.sqrt(self.r**2 - a**2)
        p2 = ((other.c - self.c) * (a/d)) + self.c
        return (
            p2 + ((1,-1) * ((other.c-self.c)[::-1]*h/d)),
            p2 + ((-1,1) * ((other.c-self.c)[::-1]*h/d))
        )

def q1(points, target=[0,0]):
    target = numpy.array(target)
    best = 999999
    best_point = None
    good = []
    for point in points:
        if point[0] > 0 and point[1] > 0:
            good.append(point)
    for point in good:
        if numpy.linalg.norm(point - target) < best:
            best = numpy.linalg.norm(point - target)
            best_point = point
    return best_point

def main():
    gcode = GCode(rt100.Rtx())

    input, slave = os.openpty()
    print("Send G-Code to:", os.ttyname(slave))

    line = ''
    while True:
        select.select([input],[],[input])
        while True:
            c = os.read(input, 1)
            if c == b'\x0A':
                break
            line += c.decode('ascii')

        line = line.lower()
        line = re.sub(r'\(.*\)', r'', line)
        line = re.sub(r';.*', r'', line)
        line = re.sub(r'^n[0-9-]+ ', r'', line)
        line = re.sub(r'\*[0-9]+', r'', line)

        line = line.replace('g', '|g')
        line = line.replace('m', '|m')
        print(line)

        for cmd in line.split('|'):
            if cmd!='':
                params = {}
                name = None
                value = ''
                for i in cmd:
                    if i == ' ':
                        continue
                    if i.isdigit() or i == '.' or i == '-':
                        value += i
                    else:
                        if name != None:
                            params[name] = float(value)
                        name = i
                        value = ''
                params[name] = float(value)
        
                if 'm' in params:
                    cmd = 'm%d' % params['m']
                    del params['m']
                elif 'g' in params:
                    cmd = 'g%d' % params['g']
                    del params['g']
                elif 't' in params:
                    cmd = 't%d' % params['t']
                    del params['t']
                else:
                    cmd = 'g1'
                code = getattr(gcode, cmd, None)
                if code != None:
                    os.write(input, (('ok %s\n' % code(**params) or '').encode('ascii')))
                else:
                    os.write(input, ('ok Not supported:%s\n' % cmd).encode('ascii'))
        line = ''
    
class GCode:

    metric = True
    absolute = True
    #Machine coords
    x = 200.0 #Assume at home
    y = ARM_LENGTH[1] -1.1
    z = 999.1
    #Working coords
    off_x = 0.0
    off_y = 0.0
    off_z = 0.0

    def __init__(self, printer):
        self.printer = printer

    def _units(self, v):
        if self.metric:
            return v
        else:
            return v * 0.0254

    def _ununits(self, v):
        if self.metric:
            return v
        else:
            return v/0.0254

    def m105(self):
        return 'T:0 /0 B:0 /0 @:0 B@:0'

    def g0(self, x=None, y=None, z=None, f=None, s=None, e=None):
        "Rapid move"
        self.g1(x, y, z, f, s)

    def g1(self, x=None, y=None, z=None, f=None, s=None, e=None):
        "Controlled move"
        #if f!=None:
        #    self.printer.feed(self._units(f))
        if x!=None:
            if self.absolute:
                self.x = self._units(x)
            else:
                 self.x += self._units(x)
        if y!=None:
            if self.absolute:
                self.y = self._units(y)
            else:
                 self.y += self._units(y)
        if z!=None:
            if self.absolute:
                self.z = self._units(z)
            else:
                 self.z += self._units(z)

        off_y = self.y + self.off_y + ARM_LENGTH[0]
        off_x = self.x + self.off_x
        off_z = self.z + self.off_z
        elbow_pos = q1(Circle((0, 0), ARM_LENGTH[0]).intersections(Circle((off_x, off_y), ARM_LENGTH[1])))
        print("Elbow: Z:%f X:%f Y%f" % (off_z, elbow_pos[0], elbow_pos[1]))
        shoulder = math.atan2(elbow_pos[1], elbow_pos[0])
        elbow = 2* math.pi - shoulder - math.atan2(off_y-elbow_pos[1], off_x-elbow_pos[0])

        print("Joints: a:%f b:%f" % (shoulder, elbow))
        self.printer.numeric(rt100.CTRLS[ZED], rt100.CTRLS[ZED][2] + (rt100.CTRLS[ZED][3] - rt100.CTRLS[ZED][2]) / ZED_RANGE * off_z)
        self.printer.numeric(rt100.CTRLS[SHOULDER], rt100.CTRLS[SHOULDER][2] + (rt100.CTRLS[SHOULDER][3] - rt100.CTRLS[SHOULDER][2]) / SHOULDER_RANGE * shoulder)
        self.printer.numeric(rt100.CTRLS[ELBOW], rt100.CTRLS[ELBOW][2] + (rt100.CTRLS[ELBOW][3] - rt100.CTRLS[ELBOW][2]) / ELBOW_RANGE * elbow)
        self.printer.start()
        self.printer.wait()
        pass

    def g4(self, p=None):
        """Dwell"""
        time.sleep(p/1000)
    
    def g20(self):
        "Set Units to Inches"
        self.metric = False

    def g21(self):
        "Set Units to Millimeters"
        self.metric = True

    def g28(self, x=None, y=None, z=None):
        "Home"
        if x!= None:
            x = 0.0
        if y!= None:
            y = 0.0
        if z!= None:
            z = 0.0
        if x==None and y==None and z==None:
            x = 0.0
            y = 0.0
            z = 0.0
    
        self.g90() #Absolute
        self.g21() #Metric
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.printer.home()

    def g40(self):
        "disable tool radius compensation"
        pass

    def g49(self):
        "disable tool length compensation"
        pass

    def g80(self):
        "cancel modal motion"
        pass

    def g54(self, x=None, y=None, z=None):
        "Work coordinate systems"
        if x != None or y != None or z != None:
            raise "Not implemented: G54 with values"

    def g61(self):
        "Exact path mode"
        pass

    def g80(self):
        "Cancel canned cycle"
        pass

    def g90(self):
        "Set to Absolute Positioning"
        self.absolute = True

    def g91(self):
        "Set to Relative Positioning"
        self.absolute = False

    def g92(self, x=None, y=None, z=None, e=None):
        "Set Position"
        if x != None:
            self.off_x = self.x + x
            self.x = 0
        if y != None:
            self.off_y = self.y + y
            self.y = 0
        if z != None:
            print("ZZZ", self.off_z, -self.z, z, -self.z + z)
            self.off_z = self.z + z
            self.z = 0

    def m0(self):
        "Stop"
        self.printer.stop(rt100.STOP_POWERED)

    def m1(self):
        "Optional Stop"
        pass

    def m2(self):
        "End of program"
        pass

    def m17(self):
        "Enable/Power all stepper motors"
        pass

    def m18(self):
        "Disable all stepper motors"
        self.printer.stop(rt100.STOP_UNPOWERED)

    def m110(self, n=None):
        "Set Current Line Number"
        pass

    def m111(self, s=None):
        "Set Debug Level"
        pass
    
    def m112(self):
        "Emergency Stop"
        self.printer.stop(rt100.STOP_UNPOWERED)

    def m114(self):
        "Get Current Position"
        return "C: X:%f Y:%f Z:%f E:0.00" % (self._ununits(self.x), self._ununits(self.y), self._ununits(self.z))

    def m115(self):
        "Get Firmware Version and Capabilities"
        return "PROTOCOL_VERSION:0.1 FIRMWARE_NAME:gcode2mint MACHINE_TYPE:rt100 EXTRUDER_COUNT:0" 
    
    def m116(self):
        "Wait"
        pass

    def t1(self):
        "Select Tool"
        #TODO        

if __name__ == "__main__":
    main()
