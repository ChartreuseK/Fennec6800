# Fennec 6800 62256 RAM Adapter

This adapter allows the use of common 62256 32k x 8-bit SRAM chips with the Fennec 6800, replacing the vintage 2114 SRAM chips. The adapter plugs into the top two SRAM sockets as well as the 74LS138 socket (replacing the IC) and provides a full 8kB of SRAM to the Fennec. Optionally wires may be soldered down to the main board to grab A13 and A14 and soldered to the ram adapter. This modification would allow for the maximum of 32kB of RAM to be fitted to the system, more than normally possbile.

BOM:

* 1x 62256 SRAM - DIP 28
* 1x 74LS02 - DIP 14
* 1x 74LS08 - DIP 14
* 3x 0.1uF 50v X7R Ceramic Capacitors 0.1" pin spacing
* 2x 1k ohm 1/4W carbon film resistor, through hole
* 2x 1x40 pin male machine pin breakaway header. 
  * (16 + 18 + 18 + 9) = 61 pins required.
* Optional 1x40 male square pin header for jumpers. 2 + 3 + 3
* Optional 3x jumper shorting connectors
* 2x DIP 14 socket
* 1x DIP 28 wide (0.6") socket

