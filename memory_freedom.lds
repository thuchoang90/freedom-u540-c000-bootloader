/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

MEMORY
{
  debug_ctrl (rwx) : ORIGIN = 0x0, LENGTH = 0x1000
  test_ctrl (rw) : ORIGIN = 0x4000, LENGTH = 0x1000
  maskrom_mem (rx) : ORIGIN = 0x78000000, LENGTH = 0x2000
  clint_ctrl (rw) : ORIGIN = 0x2000000, LENGTH = 0x10000
  ccache_sideband (rwx) : ORIGIN = 0x90000000, LENGTH = 0x1e0000
  plic_ctrl (rw) : ORIGIN = 0xc000000, LENGTH = 0x4000000
  uart0_ctrl (rw) : ORIGIN = 0x64000000, LENGTH = 0x1000
  spi0_ctrl (rw) : ORIGIN = 0x64001000, LENGTH = 0x1000
  gpio_ctrl (rw) : ORIGIN = 0x64002000, LENGTH = 0x1000
  error_mem (rwx) : ORIGIN = 0x3000, LENGTH = 0x1000
  memory_mem (rwx) : ORIGIN = 0x80000000, LENGTH = 0x40000000
}
