Modified from https://github.com/keystone-enclave/freedom-u540-c000-bootloader to work with the VC707 FPGA board.

Now you can type:

	$ make vc707

to make the vc707zsbl.hex and vc707fsbl.bin for the VC707 FPGA board.

The vc707zsbl.hex is used in rom.v (it is the bootrom inside hardware).
The vc707fsbl.bin is copied to the 4th partition of the sd card.

To make this work, you have to copy this repo to under the folder of freedom/bootrom/

Note: the normal make is still kept by the command $ make
