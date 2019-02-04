# SkeletonBeagle
Skeleton code for Beagle Bone Blue for EE192

Using PRU0 for Fast A/D read (about 8.2 us)
Feb. 3, 2019.

1. Clone whole repo.
git clone https://github.com/ucb-ee192/SkeletonBeagle

2. Compile and install PRU code
i) cd pru_firmware
ii) make all
iii) sudo make install

3. compile and run A to D test
i) cd AtodTestPRU
ii) make 
iii) sudo ./AtodTestPRU   (must be root as accesses /dev/mem, etc)
This reads shared memory buffer 128 times and prints.
LEDs USR0...USR3 will flash when reaing A/D.

Notes: The PRU0 has direct access to GPIO addresses without any device driver protection.
The output should be changed to another GP output port with a connector on the Beaglebone blue board.
Beware that a Linux process might be in IO conflict depending what other processes are running.
 

