/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#ifndef _SIFIVE_PLATFORM_H
#define _SIFIVE_PLATFORM_H

#ifdef FPGA
#ifdef TEEHW
#include <sifive/platform_teehw.h>
#else
#include <sifive/platform_freedom.h>
#endif
#else
#include <sifive/platform_hifive.h>
#endif

#endif /* _SIFIVE_PLATFORM_H */
