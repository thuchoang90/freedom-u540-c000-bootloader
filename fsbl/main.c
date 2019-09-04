/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#include "board.h"

#include "encoding.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "fdt/fdt.h"
#include <uart/uart.h>
#include <stdio.h>
#ifndef vc707
  #include <ememoryotp/ememoryotp.h>
#else
  #include "tl_clock.h"
#endif
#include "regconfig-ctl.h"
#include "regconfig-phy.h"
#include "fsbl/ux00ddr.h"

#define DENALI_PHY_DATA ddr_phy_settings
#define DENALI_CTL_DATA ddr_ctl_settings
#include "ddrregs.h"

#define DDR_ADDR  MEMORY_MEM_ADDR
#define DDR_SIZE  MEMORY_MEM_SIZE /*this needs to get from the platform.h*/
#define DDRCTLPLL_F 55
#define DDRCTLPLL_Q 2

#include <sifive/platform.h>
#include <sifive/barrier.h>
#include <stdatomic.h>

#include <sifive/devices/ccache.h>
#include <sifive/devices/gpio.h>
#include <spi/spi.h>
#include <ux00boot/ux00boot.h>
#include <gpt/gpt.h>

#include "lib/sha3/sha3.h"
#include "lib/ed25519/ed25519.h"

#ifndef PAYLOAD_DEST
  #define PAYLOAD_DEST MEMORY_MEM_ADDR
#endif

#ifndef SPI_MEM_ADDR
  #ifndef SPI_NUM
    #define SPI_NUM 0
  #endif

  #define _CONCAT3(A, B, C) A ## B ## C
  #define _SPI_MEM_ADDR(SPI_NUM) _CONCAT3(SPI, SPI_NUM, _MEM_ADDR)
  #define SPI_MEM_ADDR _SPI_MEM_ADDR(SPI_NUM)
#endif

#ifdef vc707
  #ifdef TL_CLK
    #define F_CLK TL_CLK/1000
  #else
    #error Must define TL_CLK
  #endif
#endif

Barrier barrier = { {0, 0}, {0, 0}, 0}; // bss initialization is done by main core while others do wfi

extern const gpt_guid gpt_guid_sifive_bare_metal;
volatile uint64_t dtb_target;
unsigned int serial_to_burn = ~0;

uint32_t __attribute__((weak)) own_dtb = 42; // not 0xedfe0dd0 the DTB magic

#ifndef vc707
static const uintptr_t i2c_devices[] = {
  I2C_CTRL_ADDR,
};

static spi_ctrl* const spi_devices[] = {
  (spi_ctrl*) SPI0_CTRL_ADDR,
  (spi_ctrl*) SPI1_CTRL_ADDR,
  (spi_ctrl*) SPI2_CTRL_ADDR,
};

static const uintptr_t uart_devices[] = {
  UART0_CTRL_ADDR,
  UART1_CTRL_ADDR,
};
#endif

void handle_trap(uintptr_t sp) {
  ux00boot_fail((long) read_csr(mcause), 1);
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
}

int slave_main(int id, unsigned long dtb);
void secure_boot_main();

/**
 * Scale peripheral clock dividers before changing core PLL.
 */
#ifndef vc707
void update_peripheral_clock_dividers(unsigned int peripheral_input_khz) {
  unsigned int i2c_target_khz = 400;
  uint16_t prescaler = i2c_min_clk_prescaler(peripheral_input_khz, i2c_target_khz);
  for (size_t i = 0; i < sizeof(i2c_devices) / sizeof(i2c_devices[0]); i++) {
    _REG32(i2c_devices[i], I2C_PRESCALER_LO) = prescaler & 0xff;
    _REG32(i2c_devices[i], I2C_PRESCALER_HI) = (prescaler >> 8) & 0xff;
  }

  unsigned int spi_target_khz = 50000;
  unsigned int spi_div = spi_min_clk_divisor(peripheral_input_khz, spi_target_khz);
  for (size_t i = 0; i < sizeof(spi_devices) / sizeof(spi_devices[0]); i++) {
    spi_devices[i]->sckdiv = spi_div;
  }

  unsigned int uart_target_hz = 115200ULL;
  unsigned int uart_div = uart_min_clk_divisor(peripheral_input_khz * 1000ULL, uart_target_hz);
  for (size_t i = 0; i < sizeof(uart_devices) / sizeof(uart_devices[0]); i++) {
    _REG32(uart_devices[i], UART_REG_DIV) = uart_div;
  }
}

long nsec_per_cyc = 300; // 33.333MHz
void nsleep(long nsec) {
  long step = nsec_per_cyc*2; // 2 instructions per loop iteration
  while (nsec > 0) nsec -= step;
}
#endif

int puts(const char * str) {
	uart_puts((void *) UART0_CTRL_ADDR, str);
	return 1;
}

//HART 0 runs main
int main(int id, unsigned long dtb) {
  // PRCI init
  // Initialize UART divider for 33MHz core clock in case if trap is taken prior
  // to core clock bump.
#ifndef vc707
  unsigned long long uart_target_hz = 115200ULL;
  const uint32_t initial_core_clk_khz = 33000;
  unsigned long peripheral_input_khz;
#else
  unsigned long peripheral_input_khz = F_CLK;
#endif

#ifndef vc707
  if (UX00PRCI_REG(UX00PRCI_CLKMUXSTATUSREG) & CLKMUX_STATUS_TLCLKSEL) {
    peripheral_input_khz = initial_core_clk_khz;
  } else {
    peripheral_input_khz = initial_core_clk_khz / 2;
  }
  UART0_REG(UART_REG_DIV) = uart_min_clk_divisor(peripheral_input_khz * 1000ULL, uart_target_hz);

  // Check Reset Values (lock don't care)
  uint32_t pll_default =
    (PLL_R(PLL_R_default)) |
    (PLL_F(PLL_F_default)) |
    (PLL_Q(PLL_Q_default)) |
    (PLL_RANGE(PLL_RANGE_default)) |
    (PLL_BYPASS(PLL_BYPASS_default)) |
    (PLL_FSE(PLL_FSE_default));
  uint32_t lockmask = ~PLL_LOCK(1);
  uint32_t pllout_default =
    (PLLOUT_DIV(PLLOUT_DIV_default)) |
    (PLLOUT_DIV_BY_1(PLLOUT_DIV_BY_1_default)) |
    (PLLOUT_CLK_EN(PLLOUT_CLK_EN_default));

  if ((UX00PRCI_REG(UX00PRCI_COREPLLCFG)     ^ pll_default) & lockmask) return (__LINE__);
  if ((UX00PRCI_REG(UX00PRCI_COREPLLOUT)     ^ pllout_default))         return (__LINE__);
  if ((UX00PRCI_REG(UX00PRCI_DDRPLLCFG)      ^ pll_default) & lockmask) return (__LINE__);
  if ((UX00PRCI_REG(UX00PRCI_DDRPLLOUT)      ^ pllout_default))         return (__LINE__);
  if (((UX00PRCI_REG(UX00PRCI_GEMGXLPLLCFG)) ^ pll_default) & lockmask) return (__LINE__);
  if (((UX00PRCI_REG(UX00PRCI_GEMGXLPLLOUT)) ^ pllout_default))         return (__LINE__);

  //CORE pll init
  // If tlclksel is set for 2:1 operation,
  // Set corepll 33Mhz -> 1GHz
  // Otherwise, set corepll 33MHz -> 500MHz.

  if (UX00PRCI_REG(UX00PRCI_CLKMUXSTATUSREG) & CLKMUX_STATUS_TLCLKSEL){
    nsec_per_cyc = 2;
    peripheral_input_khz = 500000; // peripheral_clk = tlclk
    update_peripheral_clock_dividers(peripheral_input_khz);
    ux00prci_select_corepll_500MHz(&UX00PRCI_REG(UX00PRCI_CORECLKSELREG),
                                   &UX00PRCI_REG(UX00PRCI_COREPLLCFG),
                                   &UX00PRCI_REG(UX00PRCI_COREPLLOUT));
  } else {
    nsec_per_cyc = 1;
    peripheral_input_khz = (1000000 / 2); // peripheral_clk = tlclk
    update_peripheral_clock_dividers(peripheral_input_khz);

    ux00prci_select_corepll_1GHz(&UX00PRCI_REG(UX00PRCI_CORECLKSELREG),
                                 &UX00PRCI_REG(UX00PRCI_COREPLLCFG),
                                 &UX00PRCI_REG(UX00PRCI_COREPLLOUT));
  }

  //
  //DDR init
  //
  uint32_t ddrctlmhz =
    (PLL_R(0)) |
    (PLL_F(DDRCTLPLL_F)) |
    (PLL_Q(DDRCTLPLL_Q)) |
    (PLL_RANGE(0x4)) |
    (PLL_BYPASS(0)) |
    (PLL_FSE(1));
  UX00PRCI_REG(UX00PRCI_DDRPLLCFG) = ddrctlmhz;

  // Wait for lock
  while ((UX00PRCI_REG(UX00PRCI_DDRPLLCFG) & PLL_LOCK(1)) == 0) ;

  uint32_t ddrctl_out =
    (PLLOUT_DIV(PLLOUT_DIV_default)) |
    (PLLOUT_DIV_BY_1(PLLOUT_DIV_BY_1_default)) |
    (PLLOUT_CLK_EN(1));
  (UX00PRCI_REG(UX00PRCI_DDRPLLOUT)) = ddrctl_out;

  //Release DDR reset.
  UX00PRCI_REG(UX00PRCI_DEVICESRESETREG) |= DEVICESRESET_DDR_CTRL_RST_N(1);
  asm volatile ("fence"); // HACK to get the '1 full controller clock cycle'.
  UX00PRCI_REG(UX00PRCI_DEVICESRESETREG) |= DEVICESRESET_DDR_AXI_RST_N(1) | DEVICESRESET_DDR_AHB_RST_N(1) | DEVICESRESET_DDR_PHY_RST_N(1);
  asm volatile ("fence"); // HACK to get the '1 full controller clock cycle'.
  // These take like 16 cycles to actually propogate. We can't go sending stuff before they
  // come out of reset. So wait. (TODO: Add a register to read the current reset states, or DDR Control device?)
  for (int i = 0; i < 256; i++){
    asm volatile ("nop");
  }

  ux00ddr_writeregmap(UX00DDR_CTRL_ADDR,ddr_ctl_settings,ddr_phy_settings);
  ux00ddr_disableaxireadinterleave(UX00DDR_CTRL_ADDR);

  ux00ddr_disableoptimalrmodw(UX00DDR_CTRL_ADDR);

  ux00ddr_enablewriteleveling(UX00DDR_CTRL_ADDR);
  ux00ddr_enablereadleveling(UX00DDR_CTRL_ADDR);
  ux00ddr_enablereadlevelinggate(UX00DDR_CTRL_ADDR);
  if(ux00ddr_getdramclass(UX00DDR_CTRL_ADDR) == DRAM_CLASS_DDR4)
    ux00ddr_enablevreftraining(UX00DDR_CTRL_ADDR);
  //mask off interrupts for leveling completion
  ux00ddr_mask_leveling_completed_interrupt(UX00DDR_CTRL_ADDR);

  ux00ddr_mask_mc_init_complete_interrupt(UX00DDR_CTRL_ADDR);
  ux00ddr_mask_outofrange_interrupts(UX00DDR_CTRL_ADDR);
  ux00ddr_setuprangeprotection(UX00DDR_CTRL_ADDR,DDR_SIZE);
  ux00ddr_mask_port_command_error_interrupt(UX00DDR_CTRL_ADDR);
#endif

  const uint64_t ddr_size = DDR_SIZE;
  const uint64_t ddr_end = PAYLOAD_DEST + ddr_size;

#ifndef vc707
  ux00ddr_start(UX00DDR_CTRL_ADDR, PHYSICAL_FILTER_CTRL_ADDR, ddr_end);
  ux00ddr_phy_fixup(UX00DDR_CTRL_ADDR);

  //
  //GEMGXL init
  //
  uint32_t gemgxl125mhz =
    (PLL_R(0)) |
    (PLL_F(59)) |  /*4000Mhz VCO*/
    (PLL_Q(5)) |   /* /32 */
    (PLL_RANGE(0x4)) |
    (PLL_BYPASS(0)) |
    (PLL_FSE(1));
  UX00PRCI_REG(UX00PRCI_GEMGXLPLLCFG) = gemgxl125mhz;

  // Wait for lock
  while ((UX00PRCI_REG(UX00PRCI_GEMGXLPLLCFG) & PLL_LOCK(1)) == 0) ;

  uint32_t gemgxlctl_out =
    (PLLOUT_DIV(PLLOUT_DIV_default)) |
    (PLLOUT_DIV_BY_1(PLLOUT_DIV_BY_1_default)) |
    (PLLOUT_CLK_EN(1));
  UX00PRCI_REG(UX00PRCI_GEMGXLPLLOUT) = gemgxlctl_out;

  //Release GEMGXL reset (set bit DEVICESRESET_GEMGXL to 1)
  UX00PRCI_REG(UX00PRCI_DEVICESRESETREG) |= DEVICESRESET_GEMGXL_RST_N(1);

//#ifdef VSC8541_PHY
#define PHY_NRESET 0x1000
  // VSC8541 PHY reset sequence; leave pull-down active for 2ms
  nsleep(2000000);
  // Set GPIO 12 (PHY NRESET) to OE=1 and OVAL=1
  atomic_fetch_or(&GPIO_REG(GPIO_OUTPUT_VAL), PHY_NRESET);
  atomic_fetch_or(&GPIO_REG(GPIO_OUTPUT_EN),  PHY_NRESET);
  nsleep(100);
  // Reset PHY again to enter unmanaged mode
  atomic_fetch_and(&GPIO_REG(GPIO_OUTPUT_VAL), ~PHY_NRESET);
  nsleep(100);
  atomic_fetch_or(&GPIO_REG(GPIO_OUTPUT_VAL), PHY_NRESET);
  nsleep(15000000);
//#endif
  // Procmon => core clock
  UX00PRCI_REG(UX00PRCI_PROCMONCFG) = 0x1 << 24;
#endif

  // Copy the DTB and reduce the reported memory to match DDR
  dtb_target = ddr_end - 0x200000; // - 2MB

#ifndef SKIP_DTB_DDR_RANGE
#define DEQ(mon, x) ((cdate[0] == mon[0] && cdate[1] == mon[1] && cdate[2] == mon[2]) ? x : 0)

  const char *cdate = __DATE__;
  int month =
    DEQ("Jan", 1) | DEQ("Feb",  2) | DEQ("Mar",  3) | DEQ("Apr",  4) |
    DEQ("May", 5) | DEQ("Jun",  6) | DEQ("Jul",  7) | DEQ("Aug",  8) |
    DEQ("Sep", 9) | DEQ("Oct", 10) | DEQ("Nov", 11) | DEQ("Dec", 12);

  char date[11] = "YYYY-MM-DD";
  date[0] = cdate[7];
  date[1] = cdate[8];
  date[2] = cdate[9];
  date[3] = cdate[10];
  date[5] = '0' + (month/10);
  date[6] = '0' + (month%10);
  date[8] = cdate[4];
  date[9] = cdate[5];

  // Post the serial number and build info
  extern const char * gitid;
  UART0_REG(UART_REG_TXCTRL) = UART_TXEN;
  puts("\r\nSiFive FSBL:       ");
  puts(date);
  puts("-");
  puts(gitid);

  // If chiplink is connected and has a DTB, use that DTB instead of what we have
  // compiled-in. This will be replaced with a real bootloader with overlays in
  // the future
#ifndef vc707
  uint32_t *chiplink_dtb = (uint32_t*)0x2ff0000000UL;
  if (*chiplink_dtb == 0xedfe0dd0){
	dtb = (uintptr_t)chiplink_dtb;
	puts("\r\nUsing Chiplink DTB");
  } else if (own_dtb == 0xedfe0dd0){
	dtb = (uintptr_t)&own_dtb;
	puts("\r\nUsing FSBL DTB");
  }
#else
  dtb = (uintptr_t)&own_dtb;
  puts("\r\nUsing FSBL DTB");
#endif

  memcpy((void*)dtb_target, (void*)dtb, fdt_size(dtb));
  fdt_reduce_mem(dtb_target, ddr_size); // reduce the RAM to physically present only
  fdt_set_prop(dtb_target, "sifive,fsbl", (uint8_t*)&date[0]);

#ifndef SKIP_OTP_MAC
#define FIRST_SLOT	0xfe
#define LAST_SLOT	0x80

#ifndef vc707
  unsigned int serial = ~0;
  int serial_slot;
  ememory_otp_power_up_sequence();
  ememory_otp_begin_read();
  for (serial_slot = FIRST_SLOT; serial_slot >= LAST_SLOT; serial_slot -= 2) {
    unsigned int pos = ememory_otp_read(serial_slot);
    unsigned int neg = ememory_otp_read(serial_slot+1);
    serial = pos;
    if (pos == ~neg) break; // legal serial #
    if (pos == ~0 && neg == ~0) break; // empty slot encountered
  }
  ememory_otp_exit_read();

  void *uart = (void*)UART0_CTRL_ADDR;
  uart_puts(uart, "\r\nHiFive-U serial #: ");
  uart_put_hex(uart, serial);

  // Program the OTP?
  if (serial_to_burn != ~0 && serial != serial_to_burn && serial_slot > LAST_SLOT) {
    uart_puts(uart, "Programming serial: ");
    uart_put_hex(uart, serial_to_burn);
    uart_puts(uart, "\r\n");
    ememory_otp_pgm_entry();
    if (serial != ~0) {
      // erase the current serial
      uart_puts(uart, "Erasing prior serial\r\n");
      ememory_otp_pgm_access(serial_slot,   0);
      ememory_otp_pgm_access(serial_slot+1, 0);
      serial_slot -= 2;
    }
    ememory_otp_pgm_access(serial_slot,    serial_to_burn);
    ememory_otp_pgm_access(serial_slot+1, ~serial_to_burn);
    ememory_otp_pgm_exit();
    uart_puts(uart, "Resuming boot\r\n");
    serial = serial_to_burn;
  }
  ememory_otp_power_down_sequence();
#endif

  // SiFive MA-S MAC block; default to serial 0
  unsigned char mac[6] = { 0x70, 0xb3, 0xd5, 0x92, 0xf0, 0x00 };
#ifndef vc707
  if (serial != ~0) {
    mac[5] |= (serial >>  0) & 0xff;
    mac[4] |= (serial >>  8) & 0xff;
    mac[3] |= (serial >> 16) & 0xff;
  }
#endif
  fdt_set_prop(dtb_target, "local-mac-address", &mac[0]);
#endif
  puts("\r\n");
#endif

  puts("Loading boot payload");
  ux00boot_load_gpt_partition((void*) PAYLOAD_DEST, &gpt_guid_sifive_bare_metal, peripheral_input_khz);

  puts("\r\n\n");
  secure_boot_main();
  slave_main(0, dtb);

  //dead code
  return 0;
}

typedef unsigned char byte;

// Sanctum header fields in DRAM
extern byte sanctum_dev_public_key[32];
extern byte sanctum_dev_secret_key[64];
unsigned int sanctum_sm_size = 0x1ff000;
extern byte sanctum_sm_hash[64];
extern byte sanctum_sm_public_key[32];
extern byte sanctum_sm_secret_key[64];
extern byte sanctum_sm_signature[64];

/* Update this to generate valid entropy for target platform*/
inline byte random_byte(unsigned int i) {
#warning Bootloader does not have entropy source, keys are for TESTING ONLY
  return 0xac + (0xdd ^ i);
}

void hwsha3_init() {
  SHA3_REG(SHA3_REG_STATUS) = 1 << 24; // Reset, and also put 0 in size
}

void hwsha3_update(void* data, size_t size) {
  uint64_t tmp;
  byte* d = (byte*)data;
  byte* t = (byte*)&tmp;
  while(size >= 8) {
    for(int i = 0; i < 8; i++) {
      t[7-i] = d[i];
    }
    SHA3_REG64(SHA3_REG_DATA_0) = tmp;
    SHA3_REG(SHA3_REG_STATUS) = 0;
    SHA3_REG(SHA3_REG_STATUS) = 1 << 16;
    size -= 8;
    d += 8;
  }
  if(size > 0) {
    for(int i = 0; i < size; i++) {
      t[7-i] = d[i];
    }
    SHA3_REG64(SHA3_REG_DATA_0) = tmp;
    SHA3_REG(SHA3_REG_STATUS) = size & 0x7;
    SHA3_REG(SHA3_REG_STATUS) = 1 << 16;
  }
}

void hwsha3_final(byte* hash, void* data, size_t size) {
  uint64_t tmp;
  byte* d = (byte*)data;
  byte* t = (byte*)&tmp;
  while(size >= 8) {
    for(int i = 0; i < 8; i++) {
      t[7-i] = d[i];
    }
    size -= 8;
    SHA3_REG64(SHA3_REG_DATA_0) = tmp;
    SHA3_REG(SHA3_REG_STATUS) = 0;
    SHA3_REG(SHA3_REG_STATUS) = size?(1 << 16):(3 << 16);
    d += 8;
  }
  if(size > 0) {
    for(int i = 0; i < size; i++) {
      t[7-i] = d[i];
    }
    SHA3_REG64(SHA3_REG_DATA_0) = tmp;
    SHA3_REG(SHA3_REG_STATUS) = size & 0x7;
    SHA3_REG(SHA3_REG_STATUS) = 3 << 16;
  }
  while(SHA3_REG(SHA3_REG_STATUS) & (1 << 10));
  for(int i = 0; i < 64; i++) {
    *(((uint8_t*)hash) + 63 - i) = *(((uint8_t*)(SHA3_CTRL_ADDR+SHA3_REG_HASH_0)) + i);
  }
}

void secure_boot_main() {
  byte scratchpad[128];
  sha3_ctx_t hash_ctx;
  void *uart = (void*)UART0_CTRL_ADDR;
  uart_puts(uart, "Hello world, FSBL\r\n");
  
  // Test the hardware with the software SHA3
  byte hash[64];
  uint32_t* hs = (uint32_t*)hash;
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, (void*)"FOX1", 4);
  sha3_final(hash, &hash_ctx);
  for(int i = 0; i < 16; i++) 
     uart_put_hex(uart, *(hs+i));
  uart_puts(uart, "\r\n");
    
  hwsha3_init(&hash_ctx);
  hwsha3_final(hash, (void*)"FOX1", 4);
  for(int i = 0; i < 16; i++) 
     uart_put_hex(uart, *(hs+i));
  uart_puts(uart, "\r\n");
  
  //*sanctum_sm_size = 0x200;
  // Reserve stack space for secrets

  // TODO: on real device, copy boot image from memory. In simulator, HTIF writes boot image
  // ... SD card to beginning of memory.
  // sd_init();
  // sd_read_from_start(DRAM, 1024);

  /* Gathering high quality entropy during boot on embedded devices is
   * a hard problem. Platforms taking security seriously must provide
   * a high quality entropy source available in hardware. Platforms
   * that do not provide such a source must gather their own
   * entropy. See the Keystone documentation for further
   * discussion. For testing purposes, we have no entropy generation.
  */

  // Create a random seed for keys and nonces from TRNG
  for (unsigned int i=0; i<32; i++) {
    scratchpad[i] = random_byte(i);
  }

  /* On a real device, the platform must provide a secure root device
     keystore. For testing purposes we hardcode a known private/public
     keypair */
  // TEST Device key
  #include "use_test_keys.h"

  // Derive {SK_D, PK_D} (device keys) from a 32 B random seed
  //ed25519_create_keypair(sanctum_dev_public_key, sanctum_dev_secret_key, scratchpad);

  // Measure SM
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, (void*)DDR_ADDR, sanctum_sm_size);
  sha3_final(sanctum_sm_hash, &hash_ctx);

  // Combine SK_D and H_SM via a hash
  // sm_key_seed <-- H(SK_D, H_SM), truncate to 32B
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, sanctum_dev_secret_key, sizeof(*sanctum_dev_secret_key));
  sha3_update(&hash_ctx, sanctum_sm_hash, sizeof(*sanctum_sm_hash));
  sha3_final(scratchpad, &hash_ctx);
  // Derive {SK_D, PK_D} (device keys) from the first 32 B of the hash (NIST endorses SHA512 truncation as safe)
  ed25519_create_keypair(sanctum_sm_public_key, sanctum_sm_secret_key, scratchpad);

  // Endorse the SM
  memcpy(scratchpad, sanctum_sm_hash, 64);
  memcpy(scratchpad + 64, sanctum_sm_public_key, 32);
  // Sign (H_SM, PK_SM) with SK_D
  ed25519_sign(sanctum_sm_signature, scratchpad, 64 + 32, sanctum_dev_public_key, sanctum_dev_secret_key);

  // Clean up
  // Erase SK_D
  memset((void*)sanctum_dev_secret_key, 0, sizeof(*sanctum_sm_secret_key));

  // caller will clean core state and memory (including the stack), and boot.
  return;
}

/*
  HARTs 1..5 run slave_main
  slave_main is a weak symbol in crt.S
*/
int slave_main(int id, unsigned long dtb) {
  // Wait for the DTB location to become known
  while (!dtb_target) {}

  //wait on barrier, disable sideband then trap to payload at PAYLOAD_DEST
  write_csr(mtvec,PAYLOAD_DEST);

  register int a0 asm("a0") = id;
#ifdef SKIP_DTB_DDR_RANGE
  register unsigned long a1 asm("a1") = dtb;
#else
  register unsigned long a1 asm("a1") = dtb_target;
#endif
  // These next two guys must get inlined and not spill a0+a1 or it is broken!
  Barrier_Wait(&barrier, NUM_CORES);
#ifndef vc707
  ccache_enable_ways(CCACHE_CTRL_ADDR,14);
#endif
  asm volatile ("unimp" : : "r"(a0), "r"(a1));

  return 0;
}
