#!/usr/bin/python3
import serial
import sys
import time
import numpy as np
import tqdm
import mpmath as mp
import math 
import fractions
mp.mp.dps=100
import default_config

class Lmx2594():
    
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


    def __init__(self, port, fosc=100e6, mash_order=3, timeout=5, macro=False):
        self.shadowMap={}
        if isinstance(port, str):
            self.d=serial.Serial(port, 115200, timeout=timeout);
        else:
            self.d=port

        print("Waiting for LMX2594 to boot");
        msg = self.readOutput(finalToken=b'READY');
        print("Booted successfully");

        if macro:
            print(self.macro(True).decode('ascii').strip())
        self.fosc=fosc
        self.mash_order = mash_order


        print("Created LMX object wit fosc", fosc/1e6)
        self.initialize()
        #self.setField('OSC_2X', 0)
        print("FPD is %f MHz"%(self.getFpd()/1e6))
   

    def write(self, cmd):
        if len(cmd)>15:
            raise ValueError("Command is too long, 16 digits is the max");
        self.d.write(cmd+b'\n')


    def readOutput(self, finalToken=b'DONE\r\n'):
        o=b'';
        while True:
            d=self.d.read(1)
            if len(d) == 0:
                raise Exception("Did not get expected response before timeout. Expected to get %s in \"%s\""%(finalToken, o))
            o+=d;
            if finalToken in o:
                return o[0:-len(finalToken)];

    def query(self, cmd):
        self.write(cmd);
        return self.readOutput()

    def macro(self, state):
        if state:
            return self.query(b"M 1");
        else:
            return self.query(b"M 0");

    def writeRegister(self, addr, value):
        cmdstr=b"W %d %d"%(addr, value)
        self.query(cmdstr)
        self.shadowMap[addr]=value


    def readRegister(self, addr, shadow=True):
        if shadow and addr in self.shadowMap:
            return self.shadowMap[addr]
        cmdstr=b'R %d'%(addr);
        resp=self.query(cmdstr).decode('ascii').split(': ')[1].split(' ')[0]
        print(resp)
        return int(resp)
    

    def locked(self):
        d=self.query(b'O')
        d=d.strip()
        s=d.split(b"PLL Output")[-1].strip()
        return s==b'1'


    def chipEnable(self, state):
        if state:
            self.query(b'E 1')
        else:
            self.query(b'E 0')

    def spiReset(self):
        self.writeRegister(0, 2);
        self.writeRegister(0, 0);

    def reset(self):
        self.chipEnable(False)
        self.chipEnable(True)
        self.spiReset()

    def initialize(self):
        #print("Resetting LMX")
        #print(self.readRegister(9))
        self.reset()
        #print(self.readRegister(9))
        #print("Applying config")
        #self.applyConfig('10GOut320MRef.txt')
        self.applyDefaultConfig()
        self.writeRegister(0, 0)
        self.setupReferencePath()
        self.setField('OUTB_PD', 0)
        self.setField('OUTA_MUX', 1)
        self.setField('OUTB_MUX', 1)
        self.setField('MASH_ORDER', self.mash_order)
        self.setField('OUTA_PWR', 3)
        self.setField('OUTB_PWR', 3)
        time.sleep(0.1)
        #self.setFrequency(10e9)
        time.sleep(0.1)
        self.enableLockDetect(True)
        #print("Is locked: ", self.isLocked())
        

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
        #print("PRE DIVIDER", div)
        self.setField('PLL_R_PRE', div)
        ref=ref/div

        if ref<250e6/3:
            mul=int(250.0e6/ref+1)
            if mul > 7:
                mul = 7
        else:
            mul=1

        #print("REF MULT", mul)
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
            self.writeRegister(a, v)

    def applyDefaultConfig(self):
        addrs = default_config.conf.keys();
        for a in addrs:
            self.writeRegister(a, default_config.conf[a])

    def enableLockDetect(self, state):
        if state:
            self.writeRegister(0, self.readRegister(0)|0x4)
        else:
            self.writeRegister(0, self.readRegister(0)&(0xFFFF-0x4))


    def readLockDetectStatus(self):
        return (self.readRegister(110, shadow=False) >>9)&0x3

    def isLocked(self):
        return self.readLockDetectStatus()==2

    def getFpd(self):
        #F_osc*OSC_2X*MULT/(PLL_R_PRE*PLL_R)
        #return 2*self.fosc;
        #print("2x", self.getField('OSC_2X'), "mult", self.getField('MULT'), "r pre",  self.getField('PLL_R_PRE'), "r", self.getField('PLL_R'))
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
   
    def getMinimumNAndPfdDlySel(self, fvco, mash_order):
        if mash_order == 0:
            if fvco <= 12.5e9:
                return 28, 1;
            return 32, 2;
        if mash_order == 1:
            if fvco <= 10e9:
                return 28, 1
            if fvco <= 12.5e9:
                return 32, 2
            return 36, 3
        if mash_order == 2:
            if fvco <= 10e9:
                return 32, 2
            return 36, 3
        if mash_order == 3:
            if fvco <= 10e6:
                return 36, 3
            return 40, 4
        if mash_order ==4:
            if fvco <= 10e9:
                return 44, 5
            return 48, 6
        raise Exception("Invalid mash order must be 0..4 and not %d"%(mash_order))


    def setFrequency2(self, f, mash_order=4):
        fpd = self.getFpd();
        fvco=f
        #determine vcoFrequency:
        if fvco > 15e9:
            raise ValueError("Frequency too high %f is more than 15e9"%(fvco))

        chdiv = None
        if fvco < 7.5e9:
            chdiv = 0;
            while True:
                fvco=f*self.chdivs[chdiv]
                if fvco >= 7.5e9:
                    break
                chdiv = chdiv + 1

        minimumN, pfd_dly_sel = self.getMinimumNAndPfdDlySel(fvco, mash_order)

        #print("Determined fvco", fvco, "chdiv", chdiv)
        multFrac = fractions.Fraction(int(fvco), int(fpd));

        if minimumN > multFrac:
            raise ValueError("Unable to set frequency, required multiplier is less than than minimum n, try reducing mash_order or fpd")

        #print("Multiplication fraction", multFrac)
        fracfrac = multFrac-minimumN
        #print("Fractional fraction", fracfrac)

        n = minimumN
        num = fracfrac.numerator
        den = fracfrac.denominator
        if num > den:
            extraN = int(int(num)/int(den));
        n = n + extraN
        num = num - den*extraN

        if den > 0xFFFFFFFF:
            print("WARNING, exact frequency is not possible using approximate frequency instead");
            n = int(fvco/fpd)
            f = float(fvco)/float(fpd)-n
            den = 0xFFFFFFFF;
            num = int(f*den)

        #n = fvco/fpd-1 #minimumN
        #num = 1 #fracfrac.numerator
        #den = 1 #fracfrac.denominator

        self.writeRegister(36, n);
        self.writeRegister(38, den >> 16)
        self.writeRegister(39, den & 0xFFFF)
        self.writeRegister(42, num >> 16)
        self.writeRegister(43, num & 0xFFFF)
        self.setField('PFD_DLY_SEL', pfd_dly_sel)
        self.setField('FCAL_EN', 1)
        self.setField('MASH_ORDER', mash_order)
        
        if chdiv is not None:
            if chdiv > 0:
                self.setField('CHDIV_DIV2', 1)
            else:
                self.setField('CHDIV_DIV2', 0)
            self.setField('CHDIV', chdiv)
            self.setField('OUTA_MUX', 0)
            self.setField('OUTB_MUX', 0)
        else:
            self.setField('OUTA_MUX', 1)
            self.setField('OUTB_MUX', 1)
            
           
        #print("N", n, "num", num, "den", den)
        factual=mp.mpf(fpd)*(mp.mpf(n)+mp.mpf(num)/mp.mpf(den))
        #print("Actual vco frequency", factual)
        #if chdiv is not None:
        #    print("Output frequency", factual/self.chdivs[chdiv], self.chdivs[chdiv])



    def setFrequency(self,f):
        #Set n divider
        #set PLL num and den
        #program fcal r0[3]=1

        #F_vco = fpdX(N+NUM/DEN)

        chdiv, f = self.computeChDivAndVcoFrequency(f)
        #if chdiv is not None:
        #    print("CHDIV", chdiv, f, self.chdivs[chdiv])
        if chdiv is not None:
            if Lmx2594.chdivs[chdiv] > 6 and f > 11.5e9:
                raise Exception("chdiv > 6 and fvco > 11.5e9, chdiv is", Lmx2594.chdivs[chdiv], "and fvco is", f)

        fpd = self.getFpd();
        n, pfd_dly_sel = self.computeNAndPfdDlySel(f)
        #print("N", n, "PFD_DLY_SEL", pfd_dly_sel)
        #print("phase detector frequency", self.getFpd()/1e6)
        factor = f/fpd
        #n = int(factor)
        den = int(f);
        den = math.gcd(int(f), int(fpd))
        #while den > 0xFFFFFFFF:
        #    den = den /10;
        #den = int(den)
        den = 280
        #print("denominator", den)
        #den = 0xFFFFFFFF
        num = int((factor-n)*den)
        #print("Factor", factor)
        self.writeRegister(36, n);
        self.writeRegister(38, den >> 16)
        self.writeRegister(39, den & 0xFFFF)
        self.writeRegister(42, num >> 16)
        self.writeRegister(43, num & 0xFFFF)
        self.writeRegister(0, self.readRegister(0)|0x8)

        if chdiv is not None:
            if chdiv > 0:
                self.setField('CHDIV_DIV2', 1)
            else:
                self.setField('CHDIV_DIV2', 0)
            self.setField('CHDIV', chdiv)
            self.setField('OUTA_MUX', 0)
            self.setField('OUTB_MUX', 0)
            factual=mp.mpf(fpd)*(mp.mpf(n)+mp.mpf(num)/mp.mpf(den))/mp.mpf(Lmx2594.chdivs[chdiv])
            fvco=fpd*(n+num/float(den))
        else:
            self.setField('OUTA_MUX', 1)
            self.setField('OUTB_MUX', 1)
            factual=mp.mpf(fpd)*(mp.mpf(n)+mp.mpf(num)/mp.mpf(den))
            fvco=fpd*(n+num/float(den))
            #print("Actual frequency vco: ", self.getFpd()*(n+num/float(den)))
        self.setField('PFD_DLY_SEL', pfd_dly_sel)
        self.setField('FCAL_EN', 1)
        #print("VCO frequency", fvco, "Actual frequency", factual)
        #print("Requested fvco:", f, "Actual fout(fvco/CHDIV):", factual, "N:", n, "pfd_dly_sel:", pfd_dly_sel, "CHDIV:", chdiv, "factor:", factor, "fpd", fpd)
        return factual, fvco, n

    def assignBit(self, ra, val, bit):
        rv = self.readRegister(ra)
        mask = list('1'*16)
        mask[15-bit]='0'
        #print(mask)
        bm = int(''.join(mask),2)
        #print("mask %x"%(bm))
        rv = rv&bm|(val<<bit)
        self.writeRegister(ra, rv)


    def setPower(self, level):
        rv = self.readRegister(44)
        level = level&0x3F
        rv = (rv & 0xFF)|(level <<8)
        self.writeRegister(44, rv)

    def setPowerB(self, level):
        self.assignBit(44, 0, 7)
        rv = self.readRegister(45);
        rv = rv & 0xFFC0
        rv = rv | (level&0x3F)# << 2;
        self.writeRegister(45, rv)

        rv = self.readRegister(46);
        rv = rv & 0xFFFC;
        rv = rv | 1;
        self.writeRegister(46, rv)


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
        vBits = self.toBinString(self.readRegister(reg))
        bitString = self.extractIndices(vBits, fieldBits)
        #print("vbits", vBits)
        #print("bitstring", bitString)
        return int(bitString[::-1], 2)

    def setField(self, name, value):
        reg, fieldBits = self.findField(name)
        if len('{0:b}'.format(value)) > len(fieldBits):
            raise ValueError("Value %d is larger than what fits in %d bits"%(value, len(fieldBits)))
        vBits = self.toBinString(self.readRegister(reg))
        vBitsNew = self.insertIndices(vBits, self.toBinString(value), fieldBits)
        #print(vBits, vBitsNew)
        self.writeRegister(reg, int(vBitsNew[::-1], 2))



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(prog="lmx2594.py", description="Program for controlling lmx2594 pll", epilog="Have fun!")
    parser.add_argument("port", help='For instance /dev/ttyUSB0')
    parser.add_argument('-f', '--frequency', help="Set frequency", type=float);
    parser.add_argument('-r', '--reference', help="Reference frequency", type=float);
    parser.add_argument('-mo', '--mash-order', help="Order of delta sigma", type=int);
    parser.add_argument('-m', '--macro', help="Record macro while setting frequency that will be executed at boot. This means that the device will set the specified frequency during power on even without a computer", action='store_true')
    parser.add_argument('-l', '--locked', help="Check lock status", action='store_true');
    args=parser.parse_args()


    if len(sys.argv) < 2:
        print("Specify serial port to use");
        quit(0)

    reference=280e6;
    if args.reference:
        reference=args.reference

    mash_order = 3
    if args.mash_order:
        mash_order = args.mash_order
        if mash_order not in range(5):
            print("Mash order must be in range 0..4")

    d=Lmx2594(args.port, fosc=reference, macro=args.macro, mash_order=mash_order)
    if args.frequency:
        #d.setFrequency(args.frequency)
        d.setFrequency2(args.frequency)
    if args.locked:
        import time
        d.enableLockDetect(True)
        time.sleep(1)
        print("Lock status is", d.locked())

        #print("Lock status is", d.locked())
    if args.macro:
        print(d.macro(False).decode('ascii').strip())
    input("press enter to continue");


