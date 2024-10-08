import serial
import sys
import time
import numpy as np

class HardwareInterface:
    def __init__(self):
        self.regShadow = {}


    def read(self, ra):
        if self.regShadow[0]&0x4:
            self.write(0, self.regShadow[0]-0x4)
        v = self.readReg(ra)

        self.regShadow[ra] = v
        return v
    
    def write(self, ra, v):
        self.writeReg(ra, v);
        self.regShadow[ra] = v

    def chipEnable(self, state):
        print("Chip enable not yet implemented");

    def writeReg(self, ra, v):
        print("Unimplemented writeReg");

    def readReg(self, ra):
        print("Unimplemented readReg");

    def reset(self):
        self.chipEnable(False)
        time.sleep(0.1)
        self.chipEnable(True)
        self.write(0, 1)
        time.sleep(0.1)
        self.write(0, 0)




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
        if 0 in self.regShadow:
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



class Lmx2594:

    registerMap = {
        0: {"RAMP_EN": [15], "VCO_PHASE_SYNC": [14], "OUT_MUTE": [9], "FCAL_HPFD_ADJ":range(7,9), "FCAL_LPFD_ADJ":range(5,7), "FCAL_EN": [3], "MUX_OUT_LD_SEL":[2], "RESET":[1], "POWERDOWN":[0]},
        1: {"CAL_CLK_DIV":[0, 1, 2]},
        4: {"ACAL_CMP_DLY":range(8, 16)},
        7: {"OUT_FORCE":[14]},
        8: {"VCO_DACISET_FORCE":[14], "VCO_CAPCTRL_FORCE":[11]},
        9: {"OSC_2X":[12]},
        10: {"MULT":range(7,12)},
        11: {"PLL_R":range(4, 12)},
        12: {"PLL_R_PRE":range(0, 12)},
        14: {"CPG":range(4, 8)},
        16: {"VCO_DACISET":range(0, 9)},
        17: {"VCO_DACISET_STRT":range(0, 9)},
        19: {"VCP_CAPCTRL":range(0,8)},
        20: {"VCO_SEL":range(11, 14), "VCO_SEL_FORCE": [10]},
        31: {"CHDIV_DIV2":[14]},
        34: {"PLL_N_18_TO_16":range(0, 4)},
        36: {"PLL_N_15_TO_0":range(0, 16)},
        37: {"MASH_SEED_EN":[15], "PFD_DLY_SEL":range(8, 14)},
        38: {"PLL_DEN_31_TO_16":range(0, 16)},
        39: {"PLL_DEN_15_TO_0":range(0, 16)},
        40: {"MASH_SEED_31_TO_16":range(0, 16)},
        41: {"MASH_SEED_15_TO_0":range(0, 16)},
        42: {"PLL_NUM_31_TO_16":range(0, 16)},
        43: {"PLL_NUM_15_TO_0":range(0, 16)},
        44: {"OUTA_PWR":range(8,14), "OUTB_PD":[7], "OUTA_PD":[6], "MASH_RESET_N":[5], "MASH_ORDER":range(0, 4)},
        45: {"OUTA_MUX":range(11, 13), "OUT_ISET":range(9, 11), "OUTB_PWR":range(0, 6)},
        46: {"OUTB_MUX":range(0, 3)},
        58: {"INPIN_IGNORE":[15], "INPIN_HYST":[14], "INPIN_LVL":range(12, 14), "INPIN_FMT":range(9, 12)},
        59: {"LD_TYPE":[0]},
        60: {"LD_DLY":range(0, 16)},
        69: {"MASH_RST_COUNT_31_TO_16":range(0, 16)},
        69: {"MASH_RST_COUNT_15_TO_0":range(0, 16)},
        #ignoring sysref and ramping
        110: {"rb_LD_VTUNE":range(9, 11), "rb_VCO_SEL":range(5,8)},
        111: {"rb_VCO_CAPCTRL":range(0, 8)},
        112: {"rb_VCO_DACISET":range(0, 9)},
        }



    def __init__(self, port, fosc=100e6):
        #self.iface = interface
        #ArduinoInterface.__init__(self, port, 115200)
        self.iface=ArduinoInterface(port, 115200)
        self.fosc=fosc
        print("Created LMX object wit fosc", fosc/1e6)

    def applyConfig(self, fn):
        a = open(fn)
        t = a.read()
        for l in t.splitlines():
            sys.stdout.write('.')
            sys.stdout.flush()
            r = l.split("0x")[-1]
            a = int(r[0:2],16)
            v = int(r[2:], 16)
            self.iface.write(a, v)

    def enableLockDetect(self, state):
        if state:
            self.iface.write(0, self.iface.read(0)|0x4)
        else:
            self.iface.write(0, self.iface.read(0)&(0xFFFF-0x4))


    def readLockDetectStatus(self):
        return (self.iface.read(110) >>9)&0x3

    def isLocked(self):
        return self.readLockDetectStatus()==2

    def getFpd(self):
        ##F_osc*OSC_2X*MULT/(PLL_R_PRE*PLL_R)
        ##return 2*self.fosc;
        OSC_2X = self.getField('OSC_2X')
        MULT = self.getField('MULT')
        PLL_R_PRE = self.getField('PLL_R_PRE')
        PLL_R = self.getField('PLL_R')
        #print(OSC_2X, MULT, PLL_R_PRE, PLL_R)
        ##return self.fosc*(1+self.getField('OSC_2X'))*self.getField('MULT')/(self.getField('PLL_R_PRE')*self.getField('PLL_R'))
        return self.fosc*(1+OSC_2X)*MULT/(PLL_R_PRE*PLL_R)
        #return self.fosc

    def setFrequency(self,f):
        chdivs = [2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 72, 96, 128, 192, 256, 384, 512, 768]
        #Set n divider
        #set PLL num and den
        #program fcal r0[3]=1

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

        print("phase detector frequency", self.getFpd()/1e6)
        factor = f/self.getFpd()
        n = int(factor)
        num = int((factor-n)*0xFFFFFFFF)
        den = 0xFFFFFFFF
        self.iface.write(36, n);
        self.iface.write(38, den >> 16)
        self.iface.write(39, den & 0xFFFF)
        self.iface.write(42, num >> 16)
        self.iface.write(43, num & 0xFFFF)
        self.iface.write(0, self.iface.read(0)|0x8)

        if chdiv is not None:
            if chdiv == 0:
                self.assignBit(31, 0, 14)
            else:
                self.assignBit(31, 1, 14)
            rv = self.iface.read(75)
            rv = rv &0xF807| (chdiv<<6)
            self.iface.write(75, rv)
            #print("reg75 %x"%(rv))
            self.assignBit(45, 0, 11)
            self.assignBit(45, 0, 12)
            self.assignBit(46, 0, 0)
            self.assignBit(46, 0, 1)
            #print("reg45 %x"%(self.read(45)))
            print("Actual frequency with divider: ", self.getFpd()*(n+num/float(den))/chdivs[chdiv])
        else:
            self.assignBit(45, 1, 11)
            self.assignBit(45, 0, 12)
            self.assignBit(46, 1, 0)
            self.assignBit(46, 0, 1)
            print("Actual frequency vco: ", self.getFpd()*(float(n)+float(num)/float(den)))
        
    def assignBit(self, ra, val, bit):
        rv = self.iface.read(ra)
        mask = list('1'*16)
        mask[15-bit]='0'
        #print(mask)
        bm = int(''.join(mask),2)
        #print("mask %x"%(bm))
        rv = rv&bm|(val<<bit)
        self.iface.write(ra, rv)


    def setPower(self, level):
        rv = self.iface.read(44)
        level = level&0x3F
        rv = (rv & 0xFF)|(level <<8)
        self.iface.write(44, rv)

    def setPowerB(self, level):
        self.assignBit(44, 0, 7)
        rv = self.iface.read(45);
        rv = rv & 0xFFC0
        rv = rv | (level&0x3F) << 2;
        self.iface.write(45, rv)

        rv = self.read(46);
        rv = rv & 0xFFFC;
        rv = rv | 1;
        self.iface.write(46, rv)


    def getAllFieldNames(self):
        fields = []
        for r in Lmx2594.registerMap:
            theReg = Lmx2594.registerMap[r]
            for f in theReg:
                fields.append(f)
        return fields

    def findField(self, fieldname):
        for r in Lmx2594.registerMap:
            theReg = Lmx2594.registerMap[r]
            for f in theReg:
                theField = theReg[f]
                #print(f, theField)
                if fieldname == f:
                    return r, theField

    def toBinString(self, v):
        #regBits = bin(v)[2:]
        #regBits = ("0"*(16-len(regBits))+regBits)[::-1]
        #return regBits
        return '{0:016b}'.format(v)[::-1]

    def extractIndices(self, binStr, bits):
        result = ''
        for b in bits:
            result = result+binStr[b]
        return result

    def insertIndices(self, targetStr, bits, positions):
        i = 0
        targetList = list(targetStr)
        for p in positions:
            targetList[p]=bits[i]
            i+=1
        return "".join(targetList)

    def getField(self, name):
        reg, fieldBits = self.findField(name);
        vBits = self.toBinString(self.iface.read(reg))
        bitString = self.extractIndices(vBits, fieldBits)
        #print("vbits", vBits)
        #print("bitstring", bitString)
        return int(bitString[::-1], 2)

    def setField(self, name, value):
        reg, fieldBits = self.findField(name)
        if len('{0:b}'.format(value)) > len(fieldBits):
            raise ValueError("Value %d is larger than what fits in %d bits"%(value, len(fieldBits)))
        vBits = self.toBinString(self.iface.read(reg))
        vBitsNew = self.insertIndices(vBits, self.toBinString(value), fieldBits)
        #print(vBits, vBitsNew)
        self.iface.write(reg, int(vBitsNew[::-1], 2))

    def reset(self):
        self.iface.reset()
        




def getIndices(l, i):
    o = ''
    for j in i:
        o+=l[j]
    return o


if __name__ == '__main__':
    print("Creating LMX obect")
    lmx = Lmx2594(sys.argv[1], fosc=320e6)
    print("Resetting LMX")
    lmx.reset()
    print("Applying config")
    lmx.applyConfig('10GOut320MRef.txt')
    #time.sleep(1000)
    #quit()
    #lmx.setField('OSC_2X', 0)
    #lmx.setField('MULT', 1)
    #lmx.setField('PLL_R', 1)
    #lmx.setField('PLL_R_PRE', 2)
    #lmx.setField('OUT_MUTE', 0)
    #lmx.setField('OUT_FORCE', 1)
    #lmx.setField('MASH_ORDER', 0)
    lmx.setField('OUTB_PD', 0)
    lmx.setField('OUTA_MUX', 1)
    lmx.setField('OUTB_MUX', 1)
    time.sleep(0.1)
    lmx.setFrequency(10e9)
    time.sleep(0.1)
    print("Is locked: ", lmx.isLocked())
    lmx.enableLockDetect(True)
    for fn in lmx.getAllFieldNames():
        v = lmx.getField(fn)
        print("%s:"%(fn), v)
    time.sleep(10)
    #for i in range(112):
    #   regdata = lmx.read(i)
    #    if i in registerMap:
    #        print(i, "contains %d named fields"%(len(registerMap[i])))
    #        regBits = bin(regdata)[2:]
    #        regBits = ("0"*(16-len(regBits))+regBits)[::-1]
    #        print("regbits length", len(regBits));
    #        for f in registerMap[i]:
    #            bits = registerMap[i][f]
    #            #print(regBits, bits)
    #            print(f, getIndices(regBits, bits), int(getIndices(regBits, bits),2), end='      ')
    #        else:
    #            print()
    #        print('*'*10)
    #exit()
    #lmx.enableLockDetect(True)
    print("Is locked: ", lmx.isLocked())
    #lmx.enableLockDetect(True)
    #lmx.setPower(8*2*0+31)
    #lmx.setPowerB(8*2*0+31)
    #freqRange = np.linspace(0.01, 1.8, 501)
    freqRange = np.linspace(0.01, 15, 1001)[::-1]
    for f in freqRange:#np.linspace(1.7, 15, 501):
        lmx.setFrequency(f*1e9)
        print("rb_LD_VTUNE", lmx.getField("rb_LD_VTUNE"))
        #print("Is locked: ", lmx.isLocked())
        lmx.enableLockDetect(True)
        time.sleep(1)

