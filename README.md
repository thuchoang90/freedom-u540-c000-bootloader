Modified from [KeyStone-encalve](https://github.com/keystone-enclave/freedom-u540-c000-bootloader) to work with the VC707 FPGA board.

From your VC707 hardware project folder:

	$ cd bootrom/
	$ git clone https://github.com/thuchoang90/freedom-u540-c000-bootloader.git
	$ cd freedom-u540-c000-bootloader/
	$ git submodule update --init --recursive

Remember to export the riscv64gc-elf-toolchain to path, and make:

	$ export PATH=/opt/riscv64gc-elf-tc/bin/:$PATH
	$ make clean
	$ make vc707
	
After this, the vc707zsbl.hex and vc707fsbl.bin will appear.

The vc707zsbl.hex is used in rom.v (it is the bootrom inside hardware) under the folder builds/vc707-u500devkit/

For the vc707fsbl.bin, you can write it to the 4th partition of the sd card

	$ sudo dd if=vc707fsbl.bin of=/dev/sdX4 bs=4096 conv=fsync
	where X4 is the partition 4 of the USB device
