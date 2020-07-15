/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#include <sifive/bits.h>
#include <sifive/smp.h>
#ifdef TEEHW
#include <encoding.h>
#endif

  .section .text.init
  .globl _prog_start
_prog_start:
  .globl _start
_start:
  csrr	t0, misa
  andi	t0, t0, 32
  beqz	t0, 1f
  lui	t0, 0x6
  csrs  mstatus, t0
#ifdef __riscv_flen
  fssr  zero
#endif
1:
/* from common/crt.S, including to match existing fsbl */
  li  x1,  0
  li  x2,  0
  li  x3,  0
  li  x4,  0
  li  x5,  0
  li  x6,  0
  li  x7,  0
  li  x8,  0
  li  x9,  0
// save a0 and a1; arguments from previous boot loader stage:
//  li  x10, 0
//  li  x11, 0
#ifndef TEEHW
// if TEEHW: save a2 and a3 too; TL_CLK & NUM_CORES are in there
  li  x12, 0
  li  x13, 0
#endif
  li  x14, 0
  li  x15, 0
  li  x16, 0
  li  x17, 0
  li  x18, 0
  li  x19, 0
  li  x20, 0
  li  x21, 0
  li  x22, 0
  li  x23, 0
  li  x24, 0
  li  x25, 0
  li  x26, 0
  li  x27, 0
  li  x28, 0
  li  x29, 0
  li  x30, 0
  li  x31, 0

.option push
.option norelax
  la gp, __global_pointer$
.option pop

/* end from common/crt.S */
  la t0, trap_entry
  csrw mtvec, t0		// 0x800004a in fsbl-1.0

/* smp_pause jumps all SMP cpus forward to branch target 42f in smp_resume */
/* leaving us with only NONSMP_HART (generally 0) to do this work */
  smp_pause(s1, s2)
  // Load data section
  la t0, _data_lma
  la t1, _data
  beq t0, t1, 2f
  la t2, _edata
  bgeu t1, t2, 2f
1:
  LOAD t3, 0(t0)
  STORE t3, 0(t1)
  addi t0, t0, REGBYTES
  addi t1, t1, REGBYTES
  bltu t1, t2, 1b
2:

  // Zero BSS section
//used to have: #ifdef SKIP_ECC_WIPEDOWN
  la t0, _fbss
  la t1, _ebss
  bgeu t0, t1, 2f
1:
  STORE zero, 0(t0)
  addi t0, t0, REGBYTES
  bltu t0, t1, 1b
2:
//endif

  smp_resume(s1, s2)

  // Allocate 4 KiB stack for each hart
  la sp, _sp
  csrr t0, mhartid
//  slli t1, t0, 12
  slli t1, t0, 10
  sub sp, sp, t1

  li t1, NONSMP_HART
  bne t0, t1, 3f

  call main
  tail exit

.globl abort
abort:
  li 	a0, -1

.globl exit
exit:
  beqz	a0, _pass
  mv	t3, a0
_fail:
  lui	t0, 0x4
  li	t1, 0x3333
  slli	a0, a0, 0x10
  add 	a0, a0, t1
  sw	a0, 0(t0) # 4000
1:
  j 1b  // loop (0x8000128
_pass:
  fence
  lui	t0, 0x4
  li	t1, 0x5555
  sw	t1, 0(t0) // 4000
1:
  j 1b	// loop (0x800013c in fsbl-1.0)

3:
  csrr	a0, mhartid
  call	slave_main
  beqz	a0, _slave_pass
  mv	t3, a0
  lui	t0, 0x4
  li	t1, 0x3333
  slli	a0, a0, 0x10
  add	a0, a0, t1
  sw	a0,0(t0) # 4000
1:
  j 1b
_slave_pass:
.weak slave_main
slave_main:
1:
  wfi
  j 1b
  
  .align 4
trap_entry:
#if __riscv_xlen == 64
  addi  sp,sp,-256
  sd    ra,8(sp)
  sd    gp,24(sp)
  sd    tp,32(sp)
  sd    t0,40(sp)
  sd    t1,48(sp)
  sd    t2,56(sp)
  sd    s0,64(sp)
  sd    s1,72(sp)
  sd    a0,80(sp)
  sd    a1,88(sp)
  sd    a2,96(sp)
  sd    a3,104(sp)
  sd    a4,112(sp)
  sd    a5,120(sp)
  sd    a6,128(sp)
  sd    a7,136(sp)
  sd    s2,144(sp)
  sd    s3,152(sp)
  sd    s4,160(sp)
  sd    s5,168(sp)
  sd    s6,176(sp)
  sd    s7,184(sp)
  sd    s8,192(sp)
  sd    s9,200(sp)
  sd    s10,208(sp)
  sd    s11,216(sp)
  sd    t3,224(sp)
  sd    t4,232(sp)
  sd    t5,240(sp)
  sd    t6,248(sp)
#else
  addi  sp,sp,-128
  sw    ra,4(sp)
  sw    gp,12(sp)
  sw    tp,16(sp)
  sw    t0,20(sp)
  sw    t1,24(sp)
  sw    t2,28(sp)
  sw    s0,32(sp)
  sw    s1,36(sp)
  sw    a0,40(sp)
  sw    a1,44(sp)
  sw    a2,48(sp)
  sw    a3,52(sp)
  sw    a4,56(sp)
  sw    a5,60(sp)
  sw    a6,64(sp)
  sw    a7,68(sp)
  sw    s2,72(sp)
  sw    s3,76(sp)
  sw    s4,80(sp)
  sw    s5,84(sp)
  sw    s6,88(sp)
  sw    s7,92(sp)
  sw    s8,96(sp)
  sw    s9,100(sp)
  sw    s10,104(sp)
  sw    s11,108(sp)
  sw    t3,112(sp)
  sw    t4,116(sp)
  sw    t5,120(sp)
  sw    t6,124(sp)
#endif
#ifndef TEEHW
  mv    a0,sp
#else
  csrr a0, mcause
  csrr a1, mepc
  mv a2, sp
#endif
  jal   ra, handle_trap
#ifdef TEEHW
  csrw mepc, a0
  # Remain in M-mode after mret
  li t0, MSTATUS_MPP
  csrs mstatus, t0
#endif
#if __riscv_xlen == 64
  ld    ra,8(sp)
  ld    gp,24(sp)
  ld    tp,32(sp)
  ld    t0,40(sp)
  ld    t1,48(sp)
  ld    t2,56(sp)
  ld    s0,64(sp)
  ld    s1,72(sp)
  ld    a0,80(sp)
  ld    a1,88(sp)
  ld    a2,96(sp)
  ld    a3,104(sp)
  ld    a4,112(sp)
  ld    a5,120(sp)
  ld    a6,128(sp)
  ld    a7,136(sp)
  ld    s2,144(sp)
  ld    s3,152(sp)
  ld    s4,160(sp)
  ld    s5,168(sp)
  ld    s6,176(sp)
  ld    s7,184(sp)
  ld    s8,192(sp)
  ld    s9,200(sp)
  ld    s10,208(sp)
  ld    s11,216(sp)
  ld    t3,224(sp)
  ld    t4,232(sp)
  ld    t5,240(sp)
  ld    t6,248(sp)
  addi  sp,sp,256
#else
  lw    ra,4(sp)
  lw    gp,12(sp)
  lw    tp,16(sp)
  lw    t0,20(sp)
  lw    t1,24(sp)
  lw    t2,28(sp)
  lw    s0,32(sp)
  lw    s1,36(sp)
  lw    a0,40(sp)
  lw    a1,44(sp)
  lw    a2,48(sp)
  lw    a3,52(sp)
  lw    a4,56(sp)
  lw    a5,60(sp)
  lw    a6,64(sp)
  lw    a7,68(sp)
  lw    s2,72(sp)
  lw    s3,76(sp)
  lw    s4,80(sp)
  lw    s5,84(sp)
  lw    s6,88(sp)
  lw    s7,92(sp)
  lw    s8,96(sp)
  lw    s9,100(sp)
  lw    s10,104(sp)
  lw    s11,108(sp)
  lw    t3,112(sp)
  lw    t4,116(sp)
  lw    t5,120(sp)
  lw    t6,124(sp)
  addi  sp,sp,128
#endif
  mret
  csrr  t0,mcause
  add   a0,zero,t0
  j     _fail
