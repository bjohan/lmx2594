import serial
import sys
import time
import numpy as np


class ArduinoInterface:
    def __init__(self, port, baud):    
        self.s = serial.Serial(port, 115200, timeout = 1)
        self.regShadow = {}
        self.flush(to = 2)

    def flush(self, to = 1):
        tl = self.s.timeout
        self.s.timeout=to
        fd = []
        while True:
            d = self.s.read()
            if len(d) == 0:
                break
            fd+=fd
        if len(fd) > 0:
            print("Flushed: ", ''.join(fd))
        self.s.timeout=tl
        
    def parse(self, d):
        o = []
        l = d.decode().split(':')
        for t in l[1:]:
            o.append(int(t.strip().split()[0]))
        return o

    def read(self, ra):
        if self.regShadow[0]&0x4:
            self.write(0, self.regShadow[0]-0x4)

        cmd = b"r %d\r"%(ra)
        self.s.write(cmd)
        d = self.s.readline()
        if not "Read" in d.decode():
            raise IOError("Device did not reply to read command")
        v, r = self.parse(d)
        if r != ra:
            raise ValueError("Device responded with read for wrong register")
        self.regShadow[ra] = v
        return v
    
    def write(self, ra, v):
        cmd = b"w %d %d\r"%(ra, v)
        self.s.write(cmd)
        d = self.s.readline()
        #print d,ra, v
        if not "Wrote" in d.decode():
            raise IOError("Device did not reply to write command, got %s"%(d));
        v2, r = self.parse(d)
        if ra != r:
            raise ValueError("Device responded with write for wrong register address")
        if v2 != v:
            raise ValueError("Device reported writing wrong value to register");
        self.regShadow[ra] = v

    def chipEnable(self, state):
        if state:
            self.s.write(b"c 1\r");
        else:
            self.s.write(b"c 0\r");
        d = self.s.readline()
        if not "CE" in d.decode():
            raise IOError("Device did not respond to chip enable command"+str(d))
        s, = self.parse(d)
        s = (s == 1)
        if s != state:
            raise ValueError("Device reported wrong CE pin state")


    def reset(self):
        self.chipEnable(False)
        time.sleep(0.01)
        self.chipEnable(True)
        self.write(0, 1)
        time.sleep(0.01)
        self.write(0, 0)



class Lmx2594(ArduinoInterface):
    def __init__(self, port, fref=100e6):
        ArduinoInterface.__init__(self, port, 115200)
        #self.s = serial.Serial(port, 115200, timeout = 1)
        self.fref=fref
        #print("Flusing serial port")
        #self.s.timeout=2
        #while True:
        #    d = self.s.read()
        #    if len(d)==0:
        #        break
        #    print("Flushed: ", d)
        #self.s.timeout = 1
        print("Done")

    def applyConfig(self, fn):
        a = open(fn)
        t = a.read()
        for l in t.splitlines():
            r = l.split("0x")[-1]
            a = int(r[0:2],16)
            v = int(r[2:], 16)
            self.write(a, v)

    def enableLockDetect(self, state):
        if state:
            self.write(0, self.read(0)|0x4)
        else:
            self.write(0, self.read(0)&(0xFFFF-0x4))


    def readLockDetectStatus(self):
        return (self.read(110) >>9)&0x3

    def isLocked(self):
        return self.readLockDetectStatus()==2

    def getFpd(self):
        #F_osc*OSC_2X*MULT/(PLL_R_PRE*PLL_R)
        return 2*self.fref;

    def setRefDoubler(self, enabled):
        if enabled:
            self.assignBit(9, 1, 12)
        else:
            self.assignBit(9, 0, 12)

    def getRefDoubler(self):
        pass

    def setFrequency(self,f):
        chdivs = [2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 72, 96, 128, 192, 256, 384, 512, 768]
        #Set n divider
        #set PLL num and den
        #program fcal r0[3]=1
        self.assignBit(9, 1, 12)
        #F_vco = fpdX(N+NUM/DEN)
        chdiv = None
        if f < 7.5e9:
            for i in range(len(chdivs)):
                if f*chdivs[i] > 7.5e9:
                    f = f*chdivs[i]
                    chdiv = i
                    break
        if chdiv is not None:
            print("Corrected frequency and division factor", f, chdivs[chdiv])

        factor = f/self.getFpd()
        n = int(factor)
        num = int((factor-n)*0xFFFFFFFF)
        den = 0xFFFFFFFF
        self.write(36, n);
        self.write(38, den >> 16)
        self.write(39, den & 0xFFFF)
        self.write(42, num >> 16)
        self.write(43, num & 0xFFFF)
        self.write(0, self.read(0)|0x8)

        if chdiv is not None:
            if chdiv == 0:
                self.assignBit(31, 0, 14)
            else:
                self.assignBit(31, 1, 14)
            rv = self.read(75)
            rv = rv &0xF807| (chdiv<<6)
            self.write(75, rv)
            print("reg75 %x"%(rv))
            self.assignBit(45, 0, 11)
            self.assignBit(45, 0, 12)
            self.assignBit(46, 0, 0)
            self.assignBit(46, 0, 1)
            print("reg45 %x"%(self.read(45)))
            print("Actual frequency with divider: ", self.getFpd()*(n+num/float(den))/chdivs[chdiv])
        else:
            self.assignBit(45, 1, 11)
            self.assignBit(45, 0, 12)
            self.assignBit(46, 1, 0)
            self.assignBit(46, 0, 1)
            print("Actual frequency vco: ", self.getFpd()*(n+num/float(den)))
        

    def assignBit(self, ra, val, bit):
        rv = self.read(ra)
        mask = list('1'*16)
        mask[15-bit]='0'
        #print(mask)
        bm = int(''.join(mask),2)
        #print("mask %x"%(bm))
        rv = rv&bm|(val<<bit)
        self.write(ra, rv)


    def setPower(self, level):
        rv = self.read(44)
        level = level&0x3F
        rv = (rv & 0xFF)|(level <<8)
        self.write(44, rv)

    def setPowerB(self, level):
        self.assignBit(44, 0, 7)
        rv = self.read(45);
        rv = rv & 0xFFC0
        rv = rv | (level&0x3F) << 2;
        self.write(45, rv)

        rv = self.read(46);
        rv = rv & 0xFFFC;
        rv = rv | 1;
        self.write(46, rv)

print("Creating LMX obect")
lmx = Lmx2594(sys.argv[1], fref=109.325e6)
print("Resetting LMX")
lmx.reset()
print("Applying config")
lmx.applyConfig('HexRegisterValues.txt')
time.sleep(0.1)

lmx.enableLockDetect(True)
print("Is locked: ", lmx.isLocked())
lmx.enableLockDetect(True)
lmx.setPower(8*2*0+31)
lmx.setPowerB(8*2*0+31)
freqRange = np.linspace(0.01, 1.8, 501)
freqRange = np.linspace(1.7, 15, 501)
freqRange = [8.5]
for f in freqRange:#np.linspace(1.7, 15, 501):
    lmx.setFrequency(f*1e9)
    print("Is locked: ", lmx.isLocked())
    lmx.enableLockDetect(True)
    time.sleep(1.1)
input("enter to quit")
