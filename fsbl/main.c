/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#include "encoding.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "fdt/fdt.h"
#include <ememoryotp/ememoryotp.h>
#include <uart/uart.h>
#include <stdio.h>

#include "regconfig-ctl.h"
#include "regconfig-phy.h"
#include "fsbl/ux00ddr.h"

#define DENALI_PHY_DATA ddr_phy_settings
#define DENALI_CTL_DATA ddr_ctl_settings
#include "ddrregs.h"

#define DDR_SIZE  MEMORY_MEM_SIZE
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
#ifdef TEEHW
#include "lib/aes/aes.h"
#include <sifive/plic_driver.h>
#include "usb/usbtest.h"
#endif

#ifdef FPGA
#include "tl_clock.h"
#define F_CLK TL_CLK/1000
#endif

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

Barrier barrier = { {0, 0}, {0, 0}, 0}; // bss initialization is done by main core while others do wfi

extern const gpt_guid gpt_guid_sifive_bare_metal;
volatile uint64_t dtb_target;
unsigned int serial_to_burn = ~0;

uint32_t __attribute__((weak)) own_dtb = 42; // not 0xedfe0dd0 the DTB magic

#ifndef FPGA
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

#ifdef TEEHW
// Structures for registering different interrupt handlers
// for different parts of the application.
void no_interrupt_handler (void) {};
function_ptr_t g_ext_interrupt_handlers[PLIC_NUM_INTERRUPTS];
function_ptr_t g_time_interrupt_handler = no_interrupt_handler;
plic_instance_t g_plic;// Instance data for the PLIC.
#define RTC_FREQ 1000000

void handle_m_ext_interrupt(){
  int int_num  = PLIC_claim_interrupt(&g_plic);
  if ((int_num >=1 ) && (int_num < PLIC_NUM_INTERRUPTS)) {
    g_ext_interrupt_handlers[int_num]();
  }
  else {
    ux00boot_fail((long) read_csr(mcause), 1);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
  }
  PLIC_complete_interrupt(&g_plic, int_num);
}

void handle_m_time_interrupt() {
  clear_csr(mie, MIP_MTIP);

  // Reset the timer for 1s in the future.
  // This also clears the existing timer interrupt.

  volatile uint64_t * mtime       = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIME);
  volatile uint64_t * mtimecmp    = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIMECMP);
  uint64_t now = *mtime;
  uint64_t then = now + RTC_FREQ;
  *mtimecmp = then;

  g_time_interrupt_handler();

  // Re-enable the timer interrupt.
  set_csr(mie, MIP_MTIP);
}

uintptr_t handle_trap(uintptr_t mcause, uintptr_t epc)
{
  // External Machine-Level interrupt from PLIC
  if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_EXT)) {
    handle_m_ext_interrupt();
    // External Machine-Level interrupt from PLIC
  } else if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_TIMER)){
    handle_m_time_interrupt();
  }
  else {
    ux00boot_fail((long) read_csr(mcause), 1);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
  }
  return epc;
}
#else
void handle_trap(uintptr_t sp)
{
  ux00boot_fail((long) read_csr(mcause), 1);
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
}
#endif

int slave_main(int id, unsigned long dtb);
void secure_boot_main();

/**
 * Scale peripheral clock dividers before changing core PLL.
 */
#ifndef FPGA
void update_peripheral_clock_dividers(unsigned int peripheral_input_khz)
{
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

int puts(const char * str){
	uart_puts((void *) UART0_CTRL_ADDR, str);
	return 1;
}

//HART 0 runs main

int main(int id, unsigned long dtb)
{
  // PRCI init

  // Initialize UART divider for 33MHz core clock in case if trap is taken prior
  // to core clock bump.
#ifndef FPGA
  unsigned long long uart_target_hz = 115200ULL;
  const uint32_t initial_core_clk_khz = 33000;
  unsigned long peripheral_input_khz;
#else
  unsigned long peripheral_input_khz = F_CLK;
  asm volatile("mv %0, a1" : "=r" (dtb));
  dtb = (uintptr_t)dtb;
#endif

#ifdef TEEHW
  // Disable the machine & timer interrupts until setup is done.
  clear_csr(mstatus, MSTATUS_MIE);
  clear_csr(mie, MIP_MEIP);
  clear_csr(mie, MIP_MTIP);

  for (int ii = 0; ii < PLIC_NUM_INTERRUPTS; ii ++) {
    g_ext_interrupt_handlers[ii] = no_interrupt_handler;
  }

  PLIC_init(&g_plic,
            PLIC_CTRL_ADDR,
            PLIC_NUM_INTERRUPTS,
            PLIC_NUM_PRIORITIES);
#endif

#ifndef FPGA
  if (UX00PRCI_REG(UX00PRCI_CLKMUXSTATUSREG) & CLKMUX_STATUS_TLCLKSEL){
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
#ifndef FPGA
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
#ifdef TEEHW
  UART0_REG(UART_REG_RXCTRL) = UART_RXEN;
#endif

  puts("\r\nSiFive FSBL:       ");
  puts(date);
  puts("-");
  puts(gitid);
  // If chiplink is connected and has a DTB, use that DTB instead of what we have
  // compiled-in. This will be replaced with a real bootloader with overlays in
  // the future
#ifndef FPGA
  uint32_t *chiplink_dtb = (uint32_t*)0x2ff0000000UL;
  if (*chiplink_dtb == 0xedfe0dd0){
	dtb = (uintptr_t)chiplink_dtb;
	puts("\r\nUsing Chiplink DTB");
  } else if (own_dtb == 0xedfe0dd0){
	dtb = (uintptr_t)&own_dtb;
	puts("\r\nUsing FSBL DTB");
  }
#else
  puts("\r\nUsing ZSBL DTB");
#endif

  memcpy((void*)dtb_target, (void*)dtb, fdt_size(dtb));
  fdt_reduce_mem(dtb_target, ddr_size); // reduce the RAM to physically present only
  fdt_set_prop(dtb_target, "sifive,fsbl", (uint8_t*)&date[0]);

#ifndef SKIP_OTP_MAC
#define FIRST_SLOT	0xfe
#define LAST_SLOT	0x80

#ifndef FPGA
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
#ifndef FPGA
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

#ifndef TEEHW
  puts("\r\n\n");
#else
  puts("\r\n\n\nWelcome to TEE-HW Bootloader\r\n\n");
#endif

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
#define DRAM_BASE 0x80000000

/* Update this to generate valid entropy for target platform*/
inline byte random_byte(unsigned int i) {
#warning Bootloader does not have entropy source, keys are for TESTING ONLY
  return 0xac + (0xdd ^ i);
}

#ifdef TEEHW
void hwsha3_init() {
  SHA3_REG(SHA3_REG_STATUS) = 1 << 24; // Reset, and also put 0 in size
}

void hwsha3_update(void* data, size_t size) {
  uint64_t* d = (uint64_t*)data;
  SHA3_REG(SHA3_REG_STATUS) = 0;
  while(size >= 8) {
    SHA3_REG64(SHA3_REG_DATA_0) = *d;
    SHA3_REG(SHA3_REG_STATUS) = 1 << 16;
    size -= 8;
    d += 1;
  }
  if(size > 0) {
    SHA3_REG64(SHA3_REG_DATA_0) = *d;
    SHA3_REG(SHA3_REG_STATUS) = size & 0x7;
    SHA3_REG(SHA3_REG_STATUS) = 1 << 16;
  }
}

void hwsha3_final(byte* hash, void* data, size_t size) {
  uint64_t* d = (uint64_t*)data;
  SHA3_REG(SHA3_REG_STATUS) = 0;
  while(size >= 8) {
    size -= 8;
    SHA3_REG64(SHA3_REG_DATA_0) = *d;
    SHA3_REG(SHA3_REG_STATUS) = 1 << 16;
    d += 1;
  }
  /*if(size > 0)*/ {
    if(size > 0) SHA3_REG64(SHA3_REG_DATA_0) = *d;
    SHA3_REG(SHA3_REG_STATUS) = size & 0x7;
    SHA3_REG(SHA3_REG_STATUS) = 3 << 16;
  }
  while(SHA3_REG(SHA3_REG_STATUS) & (1 << 10));
  for(int i = 0; i < 8; i++) {
    *(((uint64_t*)hash) + i) = *(((uint64_t*)(SHA3_CTRL_ADDR+SHA3_REG_HASH_0)) + i);
  }
}

void hwsha3_test() {
  #include "use_test_keys.h"
  //*sanctum_sm_size = 0x200;
  // Reserve stack space for secrets
  byte pad[128], swpad[64];
  uint32_t* hs = (uint32_t*)pad;
  sha3_ctx_t hash_ctx;
  unsigned long start_mcycle;
  unsigned long delta_mcycle;
  void *uart = (void*)UART0_CTRL_ADDR;
  unsigned long tclk = F_CLK; //at KHz
  tclk = 1000000/tclk; //at ns

  uart_puts(uart,"Begin SHA-3 hardware test:\r\n\n");

  start_mcycle = read_csr(mcycle);
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, (void*)DRAM_BASE, sanctum_sm_size);
  sha3_final(sanctum_sm_hash, &hash_ctx);
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, sanctum_dev_secret_key, sizeof(*sanctum_dev_secret_key));
  sha3_update(&hash_ctx, sanctum_sm_hash, sizeof(*sanctum_sm_hash));
  sha3_final(pad, &hash_ctx);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "Software: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i=16; i<32; i++)
    uart_put_hex(uart, *(hs+i));
  for(int i=0; i<64; i++)
    swpad[i] = pad[i+64];
  uart_puts(uart,"\r\n\n");

  start_mcycle = read_csr(mcycle);
  hwsha3_init();
  hwsha3_final(sanctum_sm_hash, (void*)DRAM_BASE, sanctum_sm_size);
  hwsha3_init();
  hwsha3_update(sanctum_dev_secret_key, sizeof(*sanctum_dev_secret_key));
  hwsha3_final(pad, sanctum_sm_hash, sizeof(*sanctum_sm_hash));
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "Hardware: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i=16; i<32; i++)
    uart_put_hex(uart, *(hs+i));
  uart_puts(uart,"\r\n\n");

  for(int i=0; i<64; i++)
    if(swpad[i]!=pad[i+64]) {
      uart_puts(uart,"SHA-3 hardware test fail!\r\n\n\n");
      return;
    }
  uart_puts(uart,"SHA-3 hardware test passed!\r\n\n\n");
}

void hwed25519_test() {
  byte scratchpad[128], scratchpad_hw[128];
  unsigned long start_mcycle;
  unsigned long delta_mcycle;
  void *uart = (void*)UART0_CTRL_ADDR;
  unsigned long tclk = F_CLK; //at KHz
  tclk = 1000000/tclk; //at ns
  byte sanctum_sm_public_key_hw[32];
  byte sanctum_sm_secret_key_hw[64];
  byte sanctum_sm_signature_hw[64];

  uart_puts(uart,"Begin ED25519 hardware test:\r\n\n");

  // Create a random seed for keys and nonces from TRNG
  for (unsigned int i=0; i<32; i++) {
    scratchpad[i] = random_byte(i);
    scratchpad_hw[i] = scratchpad[i];
  }

  start_mcycle = read_csr(mcycle);
  ed25519_create_keypair(sanctum_sm_public_key, sanctum_sm_secret_key, scratchpad);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "Software gen key: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\nPublic key: ");
  for(int i = 0; i < 8; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_public_key+i));
  uart_puts(uart, "\r\nPrivate key: ");
  for(int i = 0; i < 16; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_secret_key+i));

  start_mcycle = read_csr(mcycle);
  hw_ed25519_create_keypair(sanctum_sm_public_key_hw, sanctum_sm_secret_key_hw, scratchpad_hw);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "\r\n\nHardware gen key: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  uart_puts(uart,"Public key: ");
  for(int i = 0; i < 8; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_public_key_hw+i));
  uart_puts(uart, "\r\nPrivate key: ");
  for(int i = 0; i < 16; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_secret_key_hw+i));

  memcpy(scratchpad, sanctum_sm_hash, 64);
  memcpy(scratchpad + 64, sanctum_sm_public_key, 32);
  memcpy(scratchpad_hw, sanctum_sm_hash, 64);
  memcpy(scratchpad_hw + 64, sanctum_sm_public_key_hw, 32);

  start_mcycle = read_csr(mcycle);
  ed25519_sign(sanctum_sm_signature, scratchpad, 64 + 32, sanctum_dev_public_key, sanctum_dev_secret_key);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "\r\n\nSoftware sign: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i = 0; i < 16; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_signature+i));

  start_mcycle = read_csr(mcycle);
  hw_ed25519_sign(sanctum_sm_signature_hw, scratchpad_hw, 64 + 32, sanctum_dev_public_key, sanctum_dev_secret_key, 1);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "\r\n\nHardware sign: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i = 0; i < 16; i++)
    uart_put_hex(uart, *((uint32_t*)sanctum_sm_signature_hw+i));
  uart_puts(uart,"\r\n\n");

  for(int i=0; i<32; i++) {
    if(sanctum_sm_public_key[i]!=sanctum_sm_public_key_hw[i]) {
      uart_puts(uart,"ED25519 hardware test fail!\r\n\n\n");
      return;
    }
  }
  for(int i=0; i<64; i++) {
    if(sanctum_sm_secret_key[i]!=sanctum_sm_secret_key_hw[i]) {
      uart_puts(uart,"ED25519 hardware test fail!\r\n\n\n");
      return;
    }
  }
  for(int i=0; i<64; i++) {
    if(sanctum_sm_signature[i]!=sanctum_sm_signature_hw[i]) {
      uart_puts(uart,"ED25519 hardware test fail!\r\n\n\n");
      return;
    }
  }
  uart_puts(uart,"ED25519 hardware test passed!\r\n\n\n");
}

void hwaes_test() {
  unsigned long start_mcycle;
  unsigned long delta_mcycle;
  void *uart = (void*)UART0_CTRL_ADDR;
  unsigned long tclk = F_CLK; //at KHz
  tclk = 1000000/tclk; //at ns

  // Vectors extracted from the tiny AES test.c file in AES128 mode
  uint8_t aeskey[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
  //uint8_t aesout[] = { 0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66, 0xef, 0x97 };
  uint8_t aesin1[] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a };
  uint8_t aesin2[] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a };
  struct AES_ctx ctx;

  uart_puts(uart,"Begin AES hardware test:\r\n\n");

  // Software encrypt AES128
  start_mcycle = read_csr(mcycle);
  AES_init_ctx(&ctx, aeskey);
  AES_ECB_encrypt(&ctx, aesin1);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "Software: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i = 0; i < 4; i++)
    uart_put_hex(uart, *((uint32_t*)aesin1+i));

  // Hardware encrypt AES128
  start_mcycle = read_csr(mcycle);
  // Put the key (only 128 bits lower)
  for(int i = 0; i < 4; i++)
    AES_REG(AES_REG_KEY + i*4) = *((uint32_t*)aeskey+i);
  for(int i = 4; i < 8; i++)
    AES_REG(AES_REG_KEY + i*4) = 0; // Clean the "other part" of the key, "just in case"
  AES_REG(AES_REG_CONFIG) = 1; // AES128, encrypt
  AES_REG(AES_REG_STATUS) = 1; // Key Expansion Enable
  while(!(AES_REG(AES_REG_STATUS) & 0x4)); // Wait for ready
  // Put the data
  for(int i = 0; i < 4; i++)
    AES_REG(AES_REG_BLOCK + i*4) = *((uint32_t*)aesin2+i);
  AES_REG(AES_REG_STATUS) = 2; // Data enable
  while(!(AES_REG(AES_REG_STATUS) & 0x4)); // Wait for ready
  // Copy back the data to the pointer
  for(int i = 0; i < 4; i++)
    *((uint32_t*)aesin2+i) = AES_REG(AES_REG_RESULT + i*4);
  delta_mcycle = read_csr(mcycle) - start_mcycle;
  delta_mcycle = delta_mcycle*tclk/1000; //at us
  uart_puts(uart, "\r\n\nHardware: ");
  uart_put_dec(uart, delta_mcycle/1000000);
  uart_puts(uart,"s ");
  uart_put_dec(uart, (delta_mcycle%1000000)/1000);
  uart_puts(uart,"ms ");
  uart_put_dec(uart, delta_mcycle%1000);
  uart_puts(uart,"us\r\n");
  for(int i = 0; i < 4; i++)
    uart_put_hex(uart, *((uint32_t*)aesin2+i));
  uart_puts(uart,"\r\n\n");

  for(int i=0; i<16; i++) {
    if(aesin1[i]!=aesin2[i]) {
      uart_puts(uart,"AES hardware test fail!\r\n\n\n");
      return;
    }
  }
  uart_puts(uart,"AES hardware test passed!\r\n\n\n");
}

void hwrng_test() {
  void *uart = (void*)UART0_CTRL_ADDR;
  uart_puts(uart,"Begin RNG hardware test:\r\n");
  uart_puts(uart,"Now begin RNG 16 times in a row:\r\n");

  // Random number generation
  // Reset first the TRNG
  RANDOM_REG(RANDOM_RND_SOURCE_ENABLE) = 0;
  RANDOM_REG(RANDOM_TRNG_SW_RESET) = 1;
  while(RANDOM_REG(RANDOM_TRNG_BUSY));
  // Put the sampling to a odd number (remember that is a LFSR16 anyways)
  RANDOM_REG(RANDOM_SAMPLE_CNT1) = 29;
  for(int ii = 0; ii < 16; ii++) {
    // Enable the random number generator
    RANDOM_REG(RANDOM_RND_SOURCE_ENABLE) = 1;
    // Wait until we have the random number
    while(!RANDOM_REG(RANDOM_TRNG_VALID));
    // Get our random
    for(int i = 0; i < 6; i++)
      uart_put_hex(uart, RANDOM_REG(RANDOM_EHR_DATA0 + i*4));
    uart_puts(uart,"\r\n");
    // Reset the sampling counters only
    RANDOM_REG(RANDOM_RND_SOURCE_ENABLE) = 0;
    RANDOM_REG(RANDOM_RST_BITS_COUNTER) = 1;
    while(RANDOM_REG(RANDOM_TRNG_BUSY));
  }
  uart_puts(uart,"\n\n");
}
#endif

void secure_boot_main(){
  //*sanctum_sm_size = 0x200;
  // Reserve stack space for secrets
  byte scratchpad[128];
#ifndef TEEHW
  sha3_ctx_t hash_ctx;
#else
  void *uart = (void*)UART0_CTRL_ADDR;
  unsigned long start_mcycle;
  unsigned long delta_mcycle;
  char cod;

  while(1) {
    cod = 255;
    puts("Press 'a' to run ALL     hardware tests\r\n");
    puts("Press '1' to run SHA-3   hardware test\r\n");
    puts("Press '2' to run ED25519 hardware test\r\n");
    puts("Press '3' to run AES     hardware test\r\n");
    puts("Press '4' to run RNG     hardware test\r\n");
    puts("Press 'u' to run USB1.1  driver   test\r\n\n");
    puts("Press ENTER to boot Linux   ");
    for(int i=5; i>0; i--) {
      uart_put_dec(uart,i);
      start_mcycle = read_csr(mcycle);
      do {
        int32_t val = (int32_t) _REG32(uart, UART_REG_RXFIFO);
        if(val>=0) {
          cod = val & 0xFF;
          break;
        }
        delta_mcycle = read_csr(mcycle) - start_mcycle;
      } while(delta_mcycle < TL_CLK);
      if(cod==255) uart_putc(uart,'\b');
      else break;
    }
    switch(cod) {
      case 'a':
        puts("\r\n\n\nGot 'a'\r\n");
        hwsha3_test();
        hwed25519_test();
        hwaes_test();
        hwrng_test();
        continue;
      case '1':
        puts("\r\n\n\nGot '1'\r\n");
        hwsha3_test();
        continue;
      case '2':
        puts("\r\n\n\nGot '2'\r\n");
        hwed25519_test();
        continue;
      case '3':
        puts("\r\n\n\nGot '3'\r\n");
        hwaes_test();
        continue;
      case '4':
        puts("\r\n\n\nGot '4'\r\n");
        hwrng_test();
        continue;
      case 'u':
        puts("\r\n\n\nGot 'u'\r\n");
        uart_puts(uart, "Entering USB test\r\n");
        usb_test();
        continue;
      case 255:
        uart_putc(uart,'0');
    }
    puts("\r\n\n\n");
    break;
  }
#endif

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
#ifndef TEEHW
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, (void*)DRAM_BASE, sanctum_sm_size);
  sha3_final(sanctum_sm_hash, &hash_ctx);

  // Combine SK_D and H_SM via a hash
  // sm_key_seed <-- H(SK_D, H_SM), truncate to 32B
  sha3_init(&hash_ctx, 64);
  sha3_update(&hash_ctx, sanctum_dev_secret_key, sizeof(*sanctum_dev_secret_key));
  sha3_update(&hash_ctx, sanctum_sm_hash, sizeof(*sanctum_sm_hash));
  sha3_final(scratchpad, &hash_ctx);
#else
  hwsha3_init();
  hwsha3_final(sanctum_sm_hash, (void*)DRAM_BASE, sanctum_sm_size);

  // Combine SK_D and H_SM via a hash
  // sm_key_seed <-- H(SK_D, H_SM), truncate to 32B
  hwsha3_init();
  hwsha3_update(sanctum_dev_secret_key, sizeof(*sanctum_dev_secret_key));
  hwsha3_final(scratchpad, sanctum_sm_hash, sizeof(*sanctum_sm_hash));
#endif
  // Derive {SK_D, PK_D} (device keys) from the first 32 B of the hash (NIST endorses SHA512 truncation as safe)
#ifndef TEEHW
  ed25519_create_keypair(sanctum_sm_public_key, sanctum_sm_secret_key, scratchpad);
#else
  hw_ed25519_create_keypair(sanctum_sm_public_key, sanctum_sm_secret_key, scratchpad);
#endif

  // Endorse the SM
  memcpy(scratchpad, sanctum_sm_hash, 64);
  memcpy(scratchpad + 64, sanctum_sm_public_key, 32);

  // Sign (H_SM, PK_SM) with SK_D
#ifndef TEEHW
  ed25519_sign(sanctum_sm_signature, scratchpad, 64 + 32, sanctum_dev_public_key, sanctum_dev_secret_key);
#else
  hw_ed25519_sign(sanctum_sm_signature, scratchpad, 64 + 32, sanctum_dev_public_key, sanctum_dev_secret_key, 1);
#endif

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

int slave_main(int id, unsigned long dtb)
{
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
#ifndef FPGA
  ccache_enable_ways(CCACHE_CTRL_ADDR,14);
#endif
  asm volatile ("unimp" : : "r"(a0), "r"(a1));

  return 0;
}
