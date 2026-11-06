/* SPDX-License-Identifier: GPL-2.0 */
#ifndef FM_WMT_SYMBOLS_H
#define FM_WMT_SYMBOLS_H

/*
 * Bind / release the WMT/STP functions exported by the vendor's
 * wmt_drv.ko. fm_wmt_symbols_get() must succeed before anything in this
 * driver calls into WMT; see plat/wmt_symbols.c.
 */
signed int fm_wmt_symbols_get(void);
void fm_wmt_symbols_put(void);

#endif /* FM_WMT_SYMBOLS_H */
