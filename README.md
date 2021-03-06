# SkeletonBeagle
Skeleton code for reading TSL1401 Line Sensor on Beagle Bone Blue for EE192

Using PRU0 for Fast A/D read (about 8.2 us)
Feb. 27, 2019.

1. Clone whole repo.
git clone https://github.com/ucb-ee192/SkeletonBeagle

2. Compile and install PRU code
i) cd pru_firmware
ii) make all
iii) sudo make install

3. compile and run LineCamera
i) cd LineCamera
ii) make 
iii) sudo ./LineCamera   (must be root as accesses /dev/mem, etc)
This reads TSL1401 through shared memory buffer and saves to file linescans.csv
LEDs USR0...USR3 will flash when reading A/D.
This particular .csv format is compatible with Python linescanplot.py

Notes: The PRU0 has direct access to GPIO addresses without any device driver protection.

UART1 RX is used as CLK.
UART1 TX is used as SI.

These GPIO pins are setup using rc_pinmux and rc_gpio_init, and should avoid conflicts. 

The analog output from the line camera is in the range 0...3.3 V. A voltage divider is needed to keep the input
voltage less than 1.8V on the A/D input.

See LineCamera/TSL1401-BBBLue-interface.png for wiriong connections.


