# 9front-qca9531
experimental 9front kernel for the qca9531

This will boot, and has drivers for the UART and both Ethernet interfaces.
The Switch is currently not configured, and may or may not work, 
depending on how u-boot configures it.

Tested on the UniElec U9531-01
in u-boot;
load kernel to 0x80060000
load plan9.ini to 0x80050000

An nvram file can be loaded into the kernel to get it to 
load off your local grid.

