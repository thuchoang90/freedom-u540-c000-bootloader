/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#ifndef _SIFIVE_PLATFORM_H
#define _SIFIVE_PLATFORM_H

#ifdef FPGA
#include <sifive/platform_fpga.h>
#else
#include <sifive/platform_hifive.h>
#endif

#endif /* _SIFIVE_PLATFORM_H */
