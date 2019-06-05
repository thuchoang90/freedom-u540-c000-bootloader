# Copyright (c) 2018 SiFive, Inc
# SPDX-License-Identifier: Apache-2.0
# SPDX-License-Identifier: GPL-2.0-or-later
# See the file LICENSE for further information

CROSSCOMPILE?=riscv64-unknown-elf-
CC=${CROSSCOMPILE}gcc
LD=${CROSSCOMPILE}ld
OBJCOPY=${CROSSCOMPILE}objcopy
OBJDUMP=${CROSSCOMPILE}objdump
CFLAGS=-I. -Ilib/ -O2 -ggdb -march=rv64imafdc -mabi=lp64d -Wall -mcmodel=medany -mexplicit-relocs
CCASFLAGS=-I. -mcmodel=medany -mexplicit-relocs
LDFLAGS=-nostdlib -nostartfiles

dts := $(BUILD_DIR)/$(CONFIG_PROJECT).$(CONFIG).dts
clk := $(BUILD_DIR)/$(CONFIG_PROJECT).$(CONFIG).tl_clock.h

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

LIB_FS_VC707=\
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
	lib/ed25519/verify.o \

LIB_FS_O=\
	$(LIB_FS_VC707) \
	ememoryotp/ememoryotp.o \


H=$(wildcard *.h */*.h)

all:
	cp zsbl/ux00_zsbl-sifive.dts ./zsbl/ux00_zsbl.dts
	cp fsbl/ux00_fsbl-sifive.dts ./fsbl/ux00_fsbl.dts
	echo "#define HiFive 1" >> board.h
	sed -i -e 's/INCLUDE\smemory_vc707.lds/INCLUDE memory.lds/g' ux00_zsbl.lds
	sed -i -e 's/INCLUDE\smemory_vc707.lds/INCLUDE memory.lds/g' ux00_fsbl.lds
	make zsbl.bin fsbl.bin

romgen: $(clk)
	cp $(dts) ./zsbl/ux00_zsbl.dts
	cp $(dts) ./fsbl/ux00_fsbl.dts
	echo "#define vc707 1\n#define SKIP_ECC_WIPEDOWN 1" >> board.h
	cp $(clk) ./tl_clock.h
	sed -i -e 's/INCLUDE\smemory.lds/INCLUDE memory_vc707.lds/g' ux00_zsbl.lds
	sed -i -e 's/INCLUDE\smemory.lds/INCLUDE memory_vc707.lds/g' ux00_fsbl.lds
	make vc707zsbl.hex vc707fsbl.bin
	cp vc707zsbl.hex $(BUILD_DIR)/
	$(rocketchip_dir)/scripts/vlsi_rom_gen $(ROMCONF) $(BUILD_DIR)/vc707zsbl.hex > $(BUILD_DIR)/rom.v

asm: zsbl.asm fsbl.asm

$(clk): $(dts)
	awk '/tlclk {/ && !f{f=1; next}; f && match($$0, /^.*clock-frequency.*<(.*)>.*/, arr) { print "#define TL_CLK " arr[1] "UL"}' $< > $@.tmp
	mv $@.tmp $@

lib/version.c:
	echo "const char *gitid = \"$(shell git describe --always --dirty)\";" > lib/version.c
	echo "const char *gitdate = \"$(shell git log -n 1 --date=short --format=format:"%ad.%h" HEAD)\";" >> lib/version.c
	echo "const char *gitversion = \"$(shell git rev-parse HEAD)\";" >> lib/version.c
#	echo "const char *gitstatus = \"$(shell git status -s )\";" >> lib/version.c

zsbl/ux00boot.o: ux00boot/ux00boot.c
	$(CC) $(CFLAGS) -DUX00BOOT_BOOT_STAGE=0 -c -o $@ $^

zsbl.elf: zsbl/start.o zsbl/main.o $(LIB_ZS1_O) zsbl/ux00boot.o $(LIB_ZS2_O) ux00_zsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) -T$(filter %.lds,$^)

vc707zsbl.elf: zsbl/start.o zsbl/main.o $(LIB_ZS1_O) zsbl/ux00boot.o $(LIB_ZS2_O) ux00_zsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) -T$(filter %.lds,$^)

fsbl/ux00boot.o: ux00boot/ux00boot.c
	$(CC) $(CFLAGS) -DUX00BOOT_BOOT_STAGE=1 -c -o $@ $^

fsbl.elf: $(LIB_FS_O) ux00_fsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) -T$(filter %.lds,$^)

vc707fsbl.elf: $(LIB_FS_VC707) ux00_fsbl.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(filter %.o,$^) -T$(filter %.lds,$^)

fsbl/dtb.o: fsbl/ux00_fsbl.dtb

zsbl/start.o: zsbl/ux00_zsbl.dtb

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.asm: %.elf
	$(OBJDUMP) -S $^ > $@

%.dtb: %.dts
	dtc $^ -o $@ -O dtb

%.o: %.S
	$(CC) $(CFLAGS) $(CCASFLAGS) -c $< -o $@

%.o: %.c $(H)
	$(CC) $(CFLAGS) -o $@ -c $<

%.hex: %.bin
	od -t x4 -An -w4 -v $< > $@

clean::
	rm -f */*.o */*.dtb zsbl/ux00_zsbl.dts fsbl/ux00_fsbl.dts lib/version.c tl_clock.h board.h
	rm -f zsbl.bin zsbl.elf zsbl.asm fsbl.bin fsbl.elf fsbl.asm
	rm -f vc707zsbl.bin vc707zsbl.elf vc707zsbl.asm vc707zsbl.hex vc707fsbl.bin vc707fsbl.elf vc707fsbl.asm
