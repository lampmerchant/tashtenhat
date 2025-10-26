# TashTenHat

Raspberry Pi hat with I2C interface for X10 (and bonus UART).


## I2C Interface

By default, the I2C base address is 0x58.


### X10 Data Stream (base address + 0, read/write, stream)

Read or write bytes at this address to receive or send bytes over the X10 interface.


### X10 Control Register (base address + 1, write, 1 byte)

| Bits | Function                |
| ---- | ----------------------- |
| 7-4  | Reserved                |
| 3-0  | Outbound frame priority |

Outbound frame priority:

| Value | Description                                                              |
| ----- | ------------------------------------------------------------------------ |
| `0x0` | Wait 6 bit times from idle line to transmit - highest priority (default) |
| `0x1` | Wait 7 bit times from idle line to transmit                              |
| `0x2` | Wait 8 bit times from idle line to transmit                              |
| `0x3` | Wait 9 bit times from idle line to transmit                              |
| `0x4` | Wait 10 bit times from idle line to transmit                             |
| `0x5` | Wait 11 bit times from idle line to transmit                             |
| `0x6` | Wait 12 bit times from idle line to transmit                             |
| `0x7` | Wait 13 bit times from idle line to transmit                             |
| `0x8` | Wait 14 bit times from idle line to transmit                             |
| `0x9` | Wait 15 bit times from idle line to transmit                             |
| `0xA` | Wait 16 bit times from idle line to transmit                             |
| `0xB` | Wait 17 bit times from idle line to transmit                             |
| `0xC` | Wait 18 bit times from idle line to transmit                             |
| `0xD` | Wait 19 bit times from idle line to transmit                             |
| `0xE` | Wait 20 bit times from idle line to transmit                             |
| `0xF` | Wait 21 bit times from idle line to transmit - lowest priority           |


### AC Cycle Count (base address + 1, read, 1 byte)

Increments once every second (every 120 zero crossings of the AC line).


### UART Data Stream (base address + 2, read/write, stream)

Read or write bytes at this address to receive or send bytes over the UART.


### UART Control Register (base address + 3, write, 1 byte)

| Bits | Function  |
| ---- | --------- |
| 7-4  | Reserved  |
| 3-0  | Baud rate |

Baud rate values:

| Value | Baud Rate           |
| ----- | ------------------- |
| `0x0` | 300 Hz              |
| `0x1` | 600 Hz              |
| `0x2` | 1200 Hz             |
| `0x3` | 1800 Hz             |
| `0x4` | 2400 Hz             |
| `0x5` | 4800 Hz             |
| `0x6` | 9600 Hz             |
| `0x7` | 19.2 kHz            |
| `0x8` | 28.8 kHz            |
| `0x9` | 38.4 kHz            |
| `0xA` | 57.6 kHz            |
| `0xB` | 78.8 kHz            |
| `0xC` | 115.2 kHz (default) |
| `0xD` | 500 kHz             |
| `0xE` | 1 MHz               |
| `0xF` | 2 MHz               |


### UART Status Register (base address + 3, read, 1 byte)

| Bits | Function           |
| ---- | ------------------ |
| 7-4  | Read queue length  |
| 3-0  | Write queue length |

If either queue length is read as `0xF`, the actual queue length is 15 or greater.


## Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
