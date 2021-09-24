/*
 * platform_2079x.h: 2079x platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_BCM2079X_H_
#define _PLATFORM_BCM2079X_H_

/* NFC pins*/
#define NFC_INT  101 /*NFC_IRQ*/
#define NFC_WAKE  (102+28+10) /*NFC_WAKE*/
#define NFC_ENABLE (102+28+9)  /*NFC_EN*/
#define BCM_NFC_ADDR 0x77
#define BCM_NFC_BUS 2

#endif
