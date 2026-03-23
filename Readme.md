# SocketLIN (sllin)

`sllin` is a TTY line discipline that allows a Linux system to act as a LIN master
and, to some extent, as a LIN slave.

Communication to userspace is exposed as a CAN netdevice (`sllin0`), so you can
use standard tools like `ip`, `candump`, and `cangen`.

---

## Requirements

- Linux with matching kernel headers/sources for the currently running kernel
- UART interface (for example `/dev/ttyS0`, `/dev/ttyUSB0`)
- LIN level shifter/transceiver (UART ↔ LIN)
- `can-utils` (for example `candump`, `cangen`)
- Root privileges for `insmod`, `ip link`, `ldattach`

Optional:
- `slcan_attach` or project-specific attach tools/scripts

---

## Build

Inside the `sllin` directory:

```bash
make
```

This builds the kernel module `sllin.ko`.

---

## Load the module

```bash
sudo insmod ./sllin.ko
```

With parameters (example):

```bash
sudo insmod ./sllin.ko master=1 baudrate=19200 maxdev=10
```

Important parameters:

- `master`: `1` = master, `0` = slave
- `baudrate`: LIN baud rate (driver default: `LIN_DEFAULT_BAUDRATE`, usually `19200`)
- `maxdev`: maximum number of dynamic interfaces (currently defaults to `10` in this driver)
- `break_by_baud`: generate break via temporary baud-rate switch (`0/1`)

Check status:

```bash
dmesg | tail -n 20
```

---

## Attach a TTY to sllin

### Option A: `ldattach`

```bash
sudo ldattach <ldisc_number> /dev/ttyS0
```

Note: historically, `28` (master) and `29` (slave) were often used.
On modern kernels, these numbers are already assigned (`N_MCTP`,
`N_DEVELOPMENT`). For out-of-tree drivers, the line discipline number
registered by the driver must match the running kernel setup.

### Option B: attach helper (`slcan_attach`-style)

If a compatible helper tool is available (need to be patched first with patch under slin/canutils-patches) :

```bash
sudo slcan_attach -w /dev/ttyS0
```

---

## Bring the interface up

```bash
ip link show dev sllin0
sudo ip link set sllin0 up
ip link show dev sllin0
```

`state UNKNOWN` is expected for this interface type.

---

## Cache runtime behavior

- LIN ID is always in the lower 6 bits (`can_id & 0x3f`)
- EFF (`CAN_EFF_FLAG`) marks a control/configuration frame for `sllin` frame cache
- Important control bits (above LIN ID):
	- `LIN_CACHE_RESPONSE` (`1 << 6` = `0x40`): enable cached response for this LIN ID
	- `LIN_CHECKSUM_EXTENDED` (`1 << 7` = `0x80`): use enhanced checksum
	- `LIN_SINGLE_RESPONSE` (`1 << 8` = `0x100`): one-shot cached response

One-shot cache response (LIN ID `0x10`):

```bash
# Configure one-shot cache response for LIN ID 0x10
# can_id = 0x10 (ID) + 0x40 (CACHE_RESPONSE) + 0x100 (SINGLE_RESPONSE) = 0x150
cangen sllin0 -e -I 0x150 -n 1 -L 2 -D a1b2

# Trigger with RTR
cangen sllin0 -R -I 0x10 -n 1 -L 0
```

cache response (LIN ID `0x10`):
```bash
# Configure cache response for LIN ID 0x10
# can_id = 0x10 (ID) + 0x40 (CACHE_RESPONSE) = 0x50
cangen sllin0 -e -I 0x50 -n 1 -L 2 -D a1b2

# Trigger with RTR
cangen sllin0 -R -I 0x10 -n 3 -L 0

# clear cache response
# can_id = 0x10 (ID) + 0x40 (CACHE_RESPONSE) = 0x50 
# BUT DLC = 0 -> Disable cache response
cangen sllin0 -e -I 0x50 -n 1 -L 0
```

## Practical examples

### Request LIN header/response (RTR)

```bash
cangen sllin0 -R -I 1 -n 1 -L 0
```

Monitor in parallel:

```bash
candump sllin0
```

### Demonstrate timeout behavior

```bash
cangen sllin0 -R -I 8 -n 1 -L 0
ip -s link show dev sllin0
```

If no slave responds, RX error counters increase.

### Configure frame cache

```bash
cangen sllin0 -e -I 0x848 -n 1 -L 2 -D beef
cangen sllin0 -R -I 8 -n 1 -L 0
```

### Send non-RTR frame

```bash
cangen sllin0 -I 7 -n 1 -L 2 -D f00f
```