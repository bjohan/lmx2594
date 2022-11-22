import serial
import sys
import time
import numpy as np
import tqdm

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
    
    chdivs = [2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 72, 96, 128, 192, 256, 384, 512, 768]
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
        70: {"MASH_RST_COUNT_15_TO_0":range(0, 16)},
        75: {"CHDIV": range(6, 11)},
        #ignoring sysref and ramping
        110: {"rb_LD_VTUNE":range(9, 11), "rb_VCO_SEL":range(5,8)},
        111: {"rb_VCO_CAPCTRL":range(0, 8)},
        112: {"rb_VCO_DACISET":range(0, 9)},
        }



    def __init__(self, port, fosc=100e6, mash_order=3):
        ArduinoInterface.__init__(self, port, 115200)
        self.fosc=fosc
        self.mash_order = mash_order


        print("Created LMX object wit fosc", fosc/1e6)
        self.initialize()
        #self.setField('OSC_2X', 0)
        print("FPD is %f MHz"%(self.getFpd()/1e6))
    
    def initialize(self):
        print("Resetting LMX")
        self.reset()
        print("Applying config")
        self.applyConfig('10GOut320MRef.txt')
        self.setupReferencePath()
        self.setField('OUTB_PD', 0)
        self.setField('OUTA_MUX', 1)
        self.setField('OUTB_MUX', 1)
        self.setField('MASH_ORDER', self.mash_order)
        self.setField('OUTA_PWR', 3)
        self.setField('OUTB_PWR', 3)
        time.sleep(0.1)
        self.setFrequency(10e9)
        time.sleep(0.1)
        self.enableLockDetect(True)
        print("Is locked: ", self.isLocked())
        

    def setupReferencePath(self):
        ref=self.fosc
        if ref < 200e6:
            self.setField('OSC_2X',1)
            ref=ref*2
        else:
            self.setField('OSC_2X', 0)
        
        if ref > 250e6:
            div=int(ref/250.0e6+1)
        else: 
            div=1
        print("PRE DIVIDER", div)
        self.setField('PLL_R_PRE', div)
        ref=ref/div

        if ref<250e6/3:
            mul=int(250.0e6/ref+1)
            if mul > 7:
                mul = 7
        else:
            mul=1

        print("REF MULT", mul)
        self.setField('MULT', mul)
        ref=ref*mul

        self.setField('PLL_R', 1)
        if ref != self.getFpd():
            raise Exception("ERROR in setup reference path. expected %f, got %f"%(ref, self.getFpd()))

        self.setField('CAL_CLK_DIV', 3)

            


    def printConfig(self):
        for r in self.registerMap:
            fs=self.registerMap[r].keys();
            for f in fs:
                print(f, self.getField(f))

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
        #return 2*self.fosc;
        return self.fosc*(1+self.getField('OSC_2X'))*self.getField('MULT')/(self.getField('PLL_R_PRE')*self.getField('PLL_R'))

    def setCwMode(self):
        pass

    def setCwFreq(self,f):
        self.setFrequency(f)
        self.enableLockDetect(True)


    def computeChDivAndVcoFrequency(self, f):
        chdiv = None
        if f < 7.5e9:
            for i in range(len(Lmx2594.chdivs)):
                if f*Lmx2594.chdivs[i] > 7.5e9:
                    f = f*Lmx2594.chdivs[i]
                    chdiv = i
                    break
        return chdiv, f


    def raiseNExceptionIfNeeded(self, n, nmin):
                if n < nmin:
                    raise Exception("N must be greater than or equal to %d. Computed n was %d"%(nmin, n))

    def computeNAndPfdDlySel(self, f):
        factor = f/self.getFpd()
        n = int(factor)

        pfd_dly_sel = None
        if self.mash_order == 0:
            if f <= 12.5e9:
                pfd_dly_sel=1
                self.raiseNExceptionIfNeeded(n, 28)
            else:
                pfd_dly_sel = 2
                self.raiseNExceptionIfNeeded(n, 32)

        if self.mash_order == 1:
            if f <= 10e9:
                pfd_dly_sel=1
                self.raiseNExceptionIfNeeded(n, 28)
            elif f < 12.5e9:
                pfd_dly_sel=2
                self.raiseNExceptionIfNeeded(n, 32)
            else:
                pfd_dly_sel=3
                self.raiseNExceptionIfNeeded(n, 36)

        if self.mash_order == 2:
            if f <= 10e9:
                pfd_dly_sel=2
                self.raiseNExceptionIfNeeded(n, 32)
            else:
                pfd_dly_sel=3
                self.raiseNExceptionIfNeeded(n, 36)

        if self.mash_order == 3:
            if f <= 10e9:
                pfd_dly_sel=3
                self.raiseNExceptionIfNeeded(n, 36)
            else:
                pfd_dly_sel=4
                self.raiseNExceptionIfNeeded(n, 40)
        
        if self.mash_order == 4:
            if f <= 10e9:
                pfd_dly_sel=5
                self.raiseNExceptionIfNeeded(n, 44)
            else:
                pfd_dly_sel=6
                self.raiseNExceptionIfNeeded(n, 48)
        return n, pfd_dly_sel




    def setFrequency(self,f):
        #Set n divider
        #set PLL num and den
        #program fcal r0[3]=1

        #F_vco = fpdX(N+NUM/DEN)

        chdiv, f = self.computeChDivAndVcoFrequency(f)
        #print("CHDIV", chdiv, f)
        if chdiv is not None:
            if Lmx2594.chdivs[chdiv] > 6 and f > 11.5e9:
                raise Exception("chdiv > 6 and fvco > 11.5e9, chdiv is", Lmx2594.chdivs[chdiv], "and fvco is", f)


        n, pfd_dly_sel = self.computeNAndPfdDlySel(f)
        #print("N", n, "PFD_DLY_SEL", pfd_dly_sel)
        #print("phase detector frequency", self.getFpd()/1e6)
        factor = f/self.getFpd()
        #n = int(factor)
        num = int((factor-n)*0xFFFFFFFF)
        den = 0xFFFFFFFF
        #print("Factor", factor)
        self.write(36, n);
        self.write(38, den >> 16)
        self.write(39, den & 0xFFFF)
        self.write(42, num >> 16)
        self.write(43, num & 0xFFFF)
        self.write(0, self.read(0)|0x8)

        if chdiv is not None:
            if chdiv > 0:
                self.setField('CHDIV_DIV2', 1)
            else:
                self.setField('CHDIV_DIV2', 0)
            self.setField('CHDIV', chdiv)
            self.setField('OUTA_MUX', 0)
            self.setField('OUTB_MUX', 0)
            factual=self.getFpd()*(n+num/float(den))/Lmx2594.chdivs[chdiv]
            fvco=self.getFpd()*(n+num/float(den))
        else:
            self.setField('OUTA_MUX', 1)
            self.setField('OUTB_MUX', 1)
            factual=self.getFpd()*(n+num/float(den))
            fvco=self.getFpd()*(n+num/float(den))
            #print("Actual frequency vco: ", self.getFpd()*(n+num/float(den)))
        self.setField('PFD_DLY_SEL', pfd_dly_sel)
        self.setField('FCAL_EN', 1)
        #print("VCO frequency", fvco, "Actual frequency", factual)
        #print("Requested f:", f, "Actual f:", factual, "N:", n, "pfd_dly_sel:", pfd_dly_sel, "CHDIV:", chdiv, "factor:", factor)
        return factual, fvco, n

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
        vBits = self.toBinString(self.read(reg))
        bitString = self.extractIndices(vBits, fieldBits)
        #print("vbits", vBits)
        #print("bitstring", bitString)
        return int(bitString[::-1], 2)

    def setField(self, name, value):
        reg, fieldBits = self.findField(name)
        if len('{0:b}'.format(value)) > len(fieldBits):
            raise ValueError("Value %d is larger than what fits in %d bits"%(value, len(fieldBits)))
        vBits = self.toBinString(self.read(reg))
        vBitsNew = self.insertIndices(vBits, self.toBinString(value), fieldBits)
        #print(vBits, vBitsNew)
        self.write(reg, int(vBitsNew[::-1], 2))



if __name__ == '__main__':
    def getIndices(l, i):
        o = ''
        for j in i:
            o+=l[j]
        return o

    print("Creating LMX obect")
    lmx = Lmx2594(sys.argv[1], fosc=280e6, mash_order=4)
    lmx.enableLockDetect(True)
    print("Is locked: ", lmx.isLocked())
    if len(sys.argv)>=3:
        pwr=32
        if len(sys.argv)>= 4:
            pwr=int(sys.argv[3])
        lmx.setField('OUTA_PWR', pwr)
        lmx.setField('OUTB_PWR', pwr)
        lmx.setFrequency(float(sys.argv[2]))
        input("press enter to exit")
        quit()

    freqRange = np.linspace(1.8, 15, 1001)[::-1]
    pbar = tqdm.tqdm(freqRange)
    for f in pbar:#np.linspace(1.7, 15, 501):
        lmx.setFrequency(f*1e9)
        #print("Is locked: ", lmx.isLocked())
        pbar.set_description("frequency %f rb_LD_VTUNE"%(f)+str(lmx.getField("rb_LD_VTUNE")))
        pbar.refresh()
        lmx.enableLockDetect(True)
        time.sleep(1)

