# Fennec 6800
A Motorola 6800 Based Expandable Single Board Computer

## Introduction
The Fennec 6800 is a Motorola 6800 based single board computer (SBC) created with the inspiration of targeting 1977 components and features. It features an original Motorola 6800 processor running at whopping 921.6kHz, 8kB of the latest 2114 SRAM, 2kB of ROM using the brand new 2716 EPROM, and dual serial ports. An optional 40 pin expansion bus allows for connectivity to other modules, such as the Fennec VTI video terminal interface board, or to create your own attachments.

## Specifications
    * Motorola 6800 @ 921.6kHz
    * 1-8kB of onboard SRAM using pairs of 2114 (1k x 4 bit) SRAM chips. Using the optional RAM adapter board up to 32kB of RAM can be fitted with minimal modifications
    * 2kB, 2716 EPROM containing the Fennec Monitor. Optional support for a 4kB 2732 EPROM
    * Dual 6850 ACIA, RS-232 UART connected to dual 25 pin EIA-232 ports in DTE wiring.
    * Selectable serial baud rates via DIP switches from 600-9600 baud + 115,200 in default /16 mode, 150-2400+28,800 in /64 mode when configured. Non-standard baud rates are possible using a different master oscillator, but CPU speed will be affected.
    * Single rail design. Requires only a 5v regulated source of power.
    * 40 pin expansion bus with pre-decoded expansion board select signals, allowing for quite prototyping of custom expansions.
    * Beautiful blue PCB with exposed traces for a refined look.
    * Compact Board design, only 8.6×5.2 inches!

The Fennec 6800 includes a capable and expandable ROM monitor built in, requiring no tedious toggling of front panel switches to enter and begin executing code. Only a terminal is required to begin making use of the system, simply hook the terminal up to the bottom serial port (furthest from the power jack). This serial port is the local console port in the default system, all traffic is normally routed through this port, though inside the monitor certain commands may be redirected to the auxiliary port located above it.

The monitor provides facilities for examining memory, depositing values into memory, jumping to code in memory, receiving a binary over the XMODEM protocol into RAM, and dumping a block of RAM over the XMODEM protocol. With these facilities and an appropriate host system, program development can be done remotely and loaded into the Fennec to speed up initial system bring-up and testing, or development can be done in the traditional way on paper then loaded in in hexadecimal into the program memory and tested out using the computer.

The monitor also provides support for option ROMs. These option ROM images can be mapped into any of the expansion regions in upper memory starting at a multiple of 0x1000. If detected they can hook into the monitor to provide future expandability and extra features, such as the Fennec VTI board which uses an option ROM to redirect the default terminal from the first serial port to a video display and asci keyboard interface, allowing for standalone use. Future cards may also use this facility to add additional commands such as providing for a rudimentary disk operation system for example.

## Manual
The file Fennec6800_Manual.pdf contains a more comprehensive manual of the Fennec, including some theory of operation as well as assembly hints and guides. Importantly if building a REV 0 board the bodeges required for correct operation are described. The primary bodge required is changing how the data bus buffer is selected, instead of being controlled by only the CPU it is decoded in a way to allow expansion boards to enable the buffer. The KiCAD schematics reflect the changes already made, however the gerbers and rev0 pcb file don't contain the fix.

## BOM 
The complete BOM can be found in either the manual, or in the spreadsheet Fennec6800_BOM.csv 
The BOM contains the reference designators, quantity, a description of the part, as well as one or more suitable Digikey part numbers. The CPU, ACIA/UARTs, and 2716/28C16 EPROM/EEPROM however need to be aquired from secondary sources as they have not been manufactured in some time. The CPU needs to be a MC6800 or direct compatible, note a 6802 has a different pinout and is not drop in compatible. The ACIAs are MC68B50 or similar UARTs, the B variant is not needed for the lower speeds, however a Hitachi 63B50 (2MHz rated) would be required for 115,200 board operation to not be overclocking the chip slightly. This speed however is honestly faster than the Fennec 68000 is really designed for, and at 9600 baud any variant is fine. For the EPROM any speed grade 2716 or 28C16 or equivalent 2kB (8-bit) EPROM/EEPROM is suitable. 2732 EPROMs may also be used, and the extra space can be mapped by installing the jumper next to the ROM socket, however the monitor would need to be reassembled to take advantage of the extra space.


