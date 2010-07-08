/*
 * This file is part of wl12xx
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Kalle Valo <kalle.valo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef _LINUX_SPI_WL12XX_H
#define _LINUX_SPI_WL12XX_H

#define WMPA_NUMBER_OF_SECTIONS        3
#define WMPA_NUMBER_OF_BUFFERS 160
#define WMPA_SECTION_HEADER    24
#define WMPA_SECTION_SIZE_0    (WMPA_NUMBER_OF_BUFFERS * 64)
#define WMPA_SECTION_SIZE_1    (WMPA_NUMBER_OF_BUFFERS * 256)
#define WMPA_SECTION_SIZE_2    (WMPA_NUMBER_OF_BUFFERS * 2048)

struct wl12xx_platform_data {
        int (*set_power)(bool enable);
        int (*set_reset)(bool enable);
        int (*set_carddetect)(bool enable);
        void *(*mem_prealloc)(int section, unsigned long size);
        int irq;

};

#endif
