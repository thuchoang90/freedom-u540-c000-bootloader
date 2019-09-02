Modified from [KeyStone-encalve](https://github.com/keystone-enclave/freedom-u540-c000-bootloader) to work with the VC707 FPGA board.

To get this to your VC707 hardware project folder:

	stands at the VC707 folder ( i.e, pwd = freedom/ )
	$ cd bootrom/
	$ git clone https://github.com/thuchoang90/freedom-u540-c000-bootloader.git
	$ cd freedom-u540-c000-bootloader/
	$ git submodule update --init --recursive

To 'make' (remember to export the riscv64 elf toolchain first):

	$ export PATH=/opt/riscv64gc-elf-tc/bin/:$PATH
	$ make clean
	$ make vc707
	
After this, the **vc707zsbl.hex** and **vc707fsbl.bin** will appear.

The **vc707zsbl.hex** is used in rom.v (it is the bootrom inside hardware) under the folder **builds/vc707-u500devkit/**

The **vc707fsbl.bin** is for the 4th partition of the SD card. To write it to the SD card:

	$ sudo dd if=vc707fsbl.bin of=/dev/sdX4 bs=4096 conv=fsync
	where X4 is the partition 4 of the USB device
