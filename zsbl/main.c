/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#include <sifive/barrier.h>
#include <sifive/platform.h>
#include <sifive/smp.h>
#include <ux00boot/ux00boot.h>
#include <gpt/gpt.h>
#include "encoding.h"

#include <uart/uart.h>

static Barrier barrier;
extern const gpt_guid gpt_guid_sifive_fsbl;


#ifndef FPGA
#define CORE_CLK_KHZ 33000
#else
#include "tl_clock.h"
#define CORE_CLK_KHZ TL_CLK/1000
#endif


void handle_trap(void)
{
  ux00boot_fail((long) read_csr(mcause), 1);
}


void init_uart(unsigned int peripheral_input_khz)
{
  unsigned long long uart_target_hz = 115200ULL;
  UART0_REG(UART_REG_DIV) = uart_min_clk_divisor(peripheral_input_khz * 1000ULL, uart_target_hz);
  UART0_REG(UART_REG_TXCTRL) = UART_TXEN;
}

/* no-op */
int puts(const char* str){
	uart_puts((void *) UART0_CTRL_ADDR, str);
	return 1;
}

void print_init(void){
  puts("------------------- in BootROM now --------------------\r\n");
  puts("__________________PHAM LAB ! 範研究室__________________\r\n");
  puts("| Hello !                              | こんにちは！ |\r\n");
  puts("| Tokyo, Japan                         | 東京  日本   |\r\n");
  puts("| University of Electro-Communications | 電気通信大学 |\r\n");
  puts("¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\r\n\n");
}

int main()
{
  if (read_csr(mhartid) == NONSMP_HART) {
    unsigned int peripheral_input_khz;
#ifndef FPGA
    if (UX00PRCI_REG(UX00PRCI_CLKMUXSTATUSREG) & CLKMUX_STATUS_TLCLKSEL) {
      peripheral_input_khz = CORE_CLK_KHZ; // perpheral_clk = tlclk
    } else {
      peripheral_input_khz = (CORE_CLK_KHZ / 2);
    }
#else
    //peripheral_input_khz = CORE_CLK_KHZ;
    GPIO1_REG(GPIO_INPUT_EN) = 0xFF;
    GPIO1_REG(GPIO_OUTPUT_EN) = 0x0;
    peripheral_input_khz = (GPIO1_REG(GPIO_INPUT_VAL)+1)*1000;
#endif
    init_uart(peripheral_input_khz);
    print_init();
    ux00boot_load_gpt_partition((void*) CCACHE_SIDEBAND_ADDR, &gpt_guid_sifive_fsbl, peripheral_input_khz);
  }

  Barrier_Wait(&barrier, NUM_CORES);

  puts("\nJump to FSBL now\r\n\n\n");
  return 0;
}
