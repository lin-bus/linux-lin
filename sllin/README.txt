Intro
=====
Sllin is TTY discipline enabling you to create LIN Master (and partially
LIN Slave) out of your computer.
Hardware needed is Hardware UART embedded into the computer + simple
voltage level LIN converter.


Compilation
===========
To successfully compile sllin, it is necessary to have source code
of Linux kernel actually running on the computer.

To compile, run
$ make


First steps
===========
To use sllin, it is necessary to set sllin TTY discipline to some
existing serial device.

It is possible to use slightly modified slcan_attach program --
particular patch from canutils-patches folder has to be applied.

After successful compilation and loading of sllin, patching and
compiling of slcan_attach, it is possible to run:

$ sudo slcan_attach -w /dev/ttyS0
attached tty /dev/ttyS0 to netdevice sllin0
Press any key to detach /dev/ttyS0 ...

# Run from another terminal
$ dmesg
[157600.564071] sllin: sllin_kwthread stopped.
[157600.572058] netconsole: network logging stopped, interface sllin0 unregistered
[157608.437260] sllin: serial line LIN interface driver
[157608.437267] sllin: 10 dynamic interface channels.
[157608.437271] sllin: Break is generated manually with tiny sleep.
[157610.513646] sllin: sllin_open() invoked
[157610.519502] sllin: sllin_kwthread started.

$ ip link show dev sllin0
11: sllin0: <NOARP> mtu 16 qdisc noop state DOWN qlen 10
    link/can

$ sudo ip link set sllin0 up

$ sudo ip link set sllin0 up
lisovros@pc-lisovy:~/src/lin/pcan_lin/sllin$ ip link show dev sllin0
11: sllin0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN qlen 10
    link/can

# state UNKNOWN in this case is considered as wanted


Real usage
==========
Communication with sllin is done by sending different types of CAN
frames into it.

* EFF non-RTR frame:
  Configuration on internal "frame cache".

* SFF RTR frame:
  Send LIN header with LIN ID corresponding to can_id in this
  particular CAN frame.

* SFF non-RTR frame:
  Send LIN header immediately (with LIN ID corresponding to can_id
  in this particular CAN frame) followed by LIN response containing
  same data as this particular CAN frame.


Module parameters
=================
* maxdev
   -- Optional
   -- Possible values: unsigned int
   -- Maximum number of sllin interfaces.
      When not set, maxdev = 4.
      When maxdev < 4, maxdev = 4.

* master
   -- Optional
   -- Possible values: 0 or 1
   -- Sets if LIN interface will be in Master mode (1 = Master, 0 = Slave).
      When not set, master = 1.

* baudrate
   -- Optional
   -- Possible values: unsigned int
   -- Baudrate used by LIN interface on LIN bus.
      When not set, baudrate = LIN_DEFAULT_BAUDRATE (19200).


Examples
========
# Some outputs might be slightly modified for more comfortable reading

$ ls sllin.c
sllin.c

$ make
make -C /lib/modules/2.6.36.2-00398-g504e6a6-dirty/build M=/h.../sllin modules
make[1]: Entering directory `/h.../kernel/build/glab-2.6.36'
make -C /h.../kernel/2.6.36 O=/h.../kernel/build/glab-2.6.36/. modules ARCH=i386
  CC [M]  /h.../sllin/sllin.o
  Building modules, stage 2.
  MODPOST 1 modules
  LD [M]  /h.../sllin/sllin.ko
make[1]: Leaving directory `/h.../kernel/build/glab-2.6.36'

$ sudo insmod ./sllin.ko

$ dmesg | tail -3
[158268.949289] sllin: serial line LIN interface driver
[158268.949296] sllin: 10 dynamic interface channels.
[158268.949300] sllin: Break is generated manually with tiny sleep.

# Run in another terminal
$ sudo slcan_attach -w /dev/ttyS0
attached tty /dev/ttyS0 to netdevice sllin0
Press any key to detach /dev/ttyS0 ...

$ sudo ip link set sllin0 up

$ ip link show dev sllin0
12: sllin0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN qlen 10
    link/can

# Run "candump sllin0" in another terminal


# ----- Simple RTR CAN frame -----
# Patched version of cangen
$ cangen sllin0 -r -I 1 -n 1 -L 0

# Output from candump
  sllin0    1  [0] remote request
  sllin0    1  [2] 00 00

# First line: RTR sent to sllin0
# Second line: Response obtained from LIN bus (there was LIN slave
#  device on the LIN bus answering to LIN ID 1with data 0x00 0x00).


# ----- RX_TIMEOUT -----
$ cangen sllin0 -r -I 8 -n 1 -L 0

# LIN_ERR_RX_TIMEOUT flag set -- nobody answered to our LIN header
#  or CAN RTR frame
  sllin0    8  [0] remote request
  sllin0      2000  [0]

$ ip -s link show dev sllin0
14: sllin0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN qlen 10
    link/can 
    RX: bytes  packets  errors  dropped overrun mcast   
    2          4        1       0       0       0      
    TX: bytes  packets  errors  dropped carrier collsns 
    0          4        0       0       0       0 



# ----- Configure frame cache -----
# Configure frame cache to answer on LIN ID = 8
$ cangen sllin0 -e -I 0x848 -n 1 -L 2 -D beef

  sllin0       848  [2] BE EF

# Try RTR CAN frame with ID = 8 again
$ cangen sllin0 -r -I 8 -n 1 -L 0

# Everything went better than expected
  sllin0    8  [0] remote request
  sllin0    8  [2] BE EF



# ----- non-RTR CAN frame -----
$ cangen sllin0 -I 7 -n 1 -L 2 -D f00f

  sllin0    7  [2] F0 0F
  sllin0    7  [2] F0 0F


# ----- Semi-slave mode -----
$ insmod ./sllin.ko master=0

$ sudo slcan_attach -w /dev/ttyS0
attached tty /dev/ttyS0 to netdevice sllin0
Press any key to detach /dev/ttyS0 ...

# run in another terminal
$ sudo ip link set sllin0 up

$ candump -t d sllin0
 (000.000000)  sllin0    2  [0] remote request
 (001.003734)  sllin0    1  [0] remote request
 (000.000017)  sllin0    1  [2] 08 80
 (000.996027)  sllin0    2  [0] remote request
 (001.003958)  sllin0    1  [0] remote request
 (000.000017)  sllin0    1  [2] 08 80
 (000.996049)  sllin0    2  [0] remote request
 (001.003930)  sllin0    1  [0] remote request
 (000.000016)  sllin0    1  [2] 08 80
 (000.996053)  sllin0    2  [0] remote request
 (001.003945)  sllin0    1  [0] remote request
 (000.000017)  sllin0    1  [2] 08 80

# There is one LIN header without response on the bus (= only RTR can frame)
# and another LIN header followed by a response (= RTR + non-RTR CAN frame
# with the same ID)

