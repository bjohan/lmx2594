#!/usr/bin/python3
import sys
sys.path.append("yig_filter_bank_controller/py/")

import yig_controller

class VoltageGenerator:
    def __init__(self, dev):
        self.c = yig_controller.YigController(dev)
        self.setA(0)
        self.setB(0)

    def setA(self, value):
        self.c.yigA.set(6, int(32767*value))

    def setB(self, value):
        self.c.yigB.set(7, int(32767*value))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("Small tool for operating yig filter bank from the command line")
    parser.add_argument("port", help="Serial port to use for connecting to yig filter bank")
    parser.add_argument("-a", "--a-value", help="Set output a to this value [-1, 1]", metavar="VALUE", type=float)
    parser.add_argument("-b", "--b-value", help="Set output b to this value [-1, 1]", metavar="VALUE", type=float)

    args = parser.parse_args()

    vg = VoltageGenerator(args.port)

    if args.a_value:
        vg.setA(args.a_value)

    if args.b_value:
        vg.setB(args.b_value)

    input("Press enter to quit")

