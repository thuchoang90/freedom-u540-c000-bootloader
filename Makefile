# Copyright (c) 2018 SiFive, Inc
# SPDX-License-Identifier: Apache-2.0
# SPDX-License-Identifier: GPL-2.0-or-later
# See the file LICENSE for further information

ifeq ($(BOARD),)
FPGA_FLAG=
else
FPGA_FLAG=-DFPGA
endif

ifeq ($(TEEHW),)
TEEHW_FLAG=
else
TEEHW_FLAG=-DTEEHW
endif

CROSSCOMPILE?=riscv64-unknown-elf-
CC=${CROSSCOMPILE}gcc
LD=${CROSSCOMPILE}ld
OBJCOPY=${CROSSCOMPILE}objcopy
OBJDUMP=${CROSSCOMPILE}objdump
CFLAGS=-I. -Ilib/ -O2 -ggdb -march=rv64imafdc -mabi=lp64d -Wall -mcmodel=medany -mexplicit-relocs $(FPGA_FLAG) $(TEEHW_FLAG)
CCASFLAGS=-I. -mcmodel=medany -mexplicit-relocs
LDFLAGS=-nostdlib -nostartfiles

dts := $(BUILD_DIR)/$(CONFIG_PROJECT).$(CONFIG).dts
clk := $(BUILD_DIR)/$(CONFIG_PROJECT).$(CONFIG).tl_clock.h

ifeq ($(TEEHW),)
lds=memory_freedom.lds
else
lds=memory_teehw.lds
endif

# This is broken up to match the order in the original zsbl
# clkutils.o is there to match original zsbl, may not be needed
LIB_ZS1_O=\
	spi/spi.o \
	uart/uart.o \
	lib/version.o

LIB_ZS2_O=\
	clkutils/clkutils.o \
	gpt/gpt.o \
	lib/memcpy.o \
	sd/sd.o \

LIB_FS_O= \
	fsbl/start.o \
	fsbl/main.o \
	$(LIB_ZS1_O) \
	fsbl/ux00boot.o \
	clkutils/clkutils.o \
	gpt/gpt.o \
	fdt/fdt.o \
	sd/sd.o \
	lib/memcpy.o \
	lib/memset.o \
	lib/strcmp.o \
	lib/strlen.o \
	fsbl/dtb.o \
	lib/sha3/sha3.o \
	lib/ed25519/fe.o \
	lib/ed25519/ge.o \
	lib/ed25519/keypair.o \
	lib/ed25519/sc.o \
	lib/ed25519/sign.o \
	lib/ed25519/verify.o


H=$(wildcard *.h */*.h)

all: zsbl.bin fsbl.bin

romgen: $(clk) FPGAzsbl.hex FPGAfsbl.bin
	cp FPGAzsbl.hex $(BUILD_DIR)/
ifeq ($(TEEHW),)
	$(rocketchip_dir)/scripts/vlsi_rom_gen $(ROMCONF) $(BUILD_DIR)/FPGAzsbl.hex > $(BUILD_DIR)/rom.v
else
	$(ROMGEN) $(ROM_CONF_FILE) FPGAzsbl.hex > $(ROM_FILE)
endif

$(clk): $(dts)
	awk '/tlclk {/ && !f{f=1; next}; f && match($$0, /^.*clock-frequency.*<(.*)>.*/, arr) { print "#define TL_CLK " arr[1] "UL"}' $< > tl_clock.h
	cp tl_clock.h $@

elf: zsbl.elf fsbl.elf

asm: zsbl.asm fsbl.asm

lib/version.c:
	echo "const char *gitid = \"$(shell git describe --always --dirty)\";" > lib/version.c
	echo "const char *gitdate = \"$(shell git log -n 1 --date=short --format=format:"%ad.%h" HEAD)\";" >> lib/version.c
	echo "const char *gitversion = \"$(shell git rev-parse HEAD)\";" >> lib/version.c
#	echo "const char *gitstatus = \"$(shell git status -s )\";" >> lib/version.c

zsbl/ux00boot.o: ux00boot/ux00boot.c
	$(CC) $(CFLAGS) -DUX00BOOT_BOOT_STAGE=0 -c -o $@ $^

zsbl.elf: zsbl/start.o zsbl/main.o $(LIB_ZS1_O) zsbl/ux00boot.o $(LIB_ZS2_O) memory.lds ux00_zsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) $(patsubst %, -T%, $(filter %.lds,$^))

FPGAzsbl.elf: zsbl/start.o zsbl/main.o $(LIB_ZS1_O) zsbl/ux00boot.o $(LIB_ZS2_O) $(lds) ux00_zsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) $(patsubst %, -T%, $(filter %.lds,$^))

fsbl/ux00boot.o: ux00boot/ux00boot.c
	$(CC) $(CFLAGS) -DUX00BOOT_BOOT_STAGE=1 -c -o $@ $^

fsbl.elf: $(LIB_FS_O) ememoryotp/ememoryotp.o memory.lds ux00_fsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) $(patsubst %, -T%, $(filter %.lds,$^))

FPGAfsbl.elf: $(LIB_FS_O) $(lds) ux00_fsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) $(patsubst %, -T%, $(filter %.lds,$^))

fsbl/dtb.o: fsbl/ux00_fsbl.dtb

zsbl/start.o: zsbl/ux00_zsbl.dtb

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.asm: %.elf
	$(OBJDUMP) -S $^ > $@

%.dtb: %.dts
ifeq ($(BOARD),)
	dtc $^ -o $@ -O dtb
else
	dtc $(dts) -o $@ -O dtb
endif

%.o: %.S
ifeq ($(BOARD),)
	$(CC) $(CFLAGS) $(CCASFLAGS) -c $< -o $@
else
	$(CC) $(CFLAGS) $(CCASFLAGS) -DSKIP_ECC_WIPEDOWN -c $< -o $@
endif

%.o: %.c $(H)
	$(CC) $(CFLAGS) -o $@ -c $<

%.hex: %.bin
	od -t x4 -An -w4 -v $< > $@

clean::
	rm -f */*.o */*.dtb zsbl.bin zsbl.elf zsbl.asm fsbl.bin fsbl.elf fsbl.asm lib/version.c
	rm -f FPGAzsbl.bin FPGAzsbl.elf FPGAzsbl.hex FPGAfsbl.bin FPGAfsbl.elf tl_clock.h $(clk)
