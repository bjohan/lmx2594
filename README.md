# lmx2594
Test board for lmx2594. The PCB is connected to an Arduino nano which then can be controlled from a PC. The Arduino can be programmed so that the PLL always starts with a certain frequency. It is also possible to change frequency dynamically.

# PCB
Current version of PCB in pcb/lmx2594_v2/

# Arduino sketch
In lmx2594NewSketch/

#Python library commandline tool
In py/ 
```
/lmx2594cmd2.py --help
usage: lmx2594.py [-h] [-f FREQUENCY] [-r REFERENCE] [-mo MASH_ORDER] [-m] [-l] port

Program for controlling lmx2594 pll

positional arguments:
  port                  For instance /dev/ttyUSB0

options:
  -h, --help            show this help message and exit
  -f FREQUENCY, --frequency FREQUENCY
                        Set frequency
  -r REFERENCE, --reference REFERENCE
                        Reference frequency
  -mo MASH_ORDER, --mash-order MASH_ORDER
                        Order of delta sigma
  -m, --macro           Record macro while setting frequency that will be executed at boot. This means that the device will set the specified frequency during power on even without a
                        computer
  -l, --locked          Check lock status

Have fun!
```

Example:
Program arduino so it always starts at 1296MHz with 10MHz reference
```
./lmx2594cmd2.py /dev/ttyUSB0 -r 10e6 -f 1296e6 -m
```

Example:
Set the frequency to 144MHz with 10MHz reference
```
./lmx2594cmd2.py /dev/ttyUSB0 -r 10e6 -f 144e6
```