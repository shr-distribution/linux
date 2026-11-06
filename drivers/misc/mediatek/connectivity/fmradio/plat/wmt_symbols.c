// SPDX-License-Identifier: GPL-2.0
/*
 * Runtime binding of the WMT/STP functions fmradio_drv calls.
 *
 * They are exported by the vendor's wmt_drv.ko, which is not part of this
 * kernel tree. An in-tree module that references them directly cannot be
 * built: modpost does not know the symbols and, for in-tree modules, there
 * is no way to hand it the vendor's Module.symvers. So this file defines the
 * functions under their usual names - every call site stays unchanged - and
 * forwards each one through a pointer taken with symbol_get() when the
 * module loads. symbol_get() also pins wmt_drv for as long as we hold the
 * pointers.
 *
 * The prototypes come from the WMT headers in wmt/, so the compiler checks
 * every wrapper against them. What is lost is modversions' CRC check against
 * the wmt_drv actually loaded; the headers are from the same MediaTek release
 * as that driver.
 */

#include <linux/module.h>

#include "osal_typedef.h"
#include "stp_exp.h"
#include "wmt_exp.h"
#include "fm_dbg.h"
#include "fm_err.h"
#include "wmt_symbols.h"

#define WMT_FUNCS(X) \
	X(mtk_wcn_stp_send_data) \
	X(mtk_wcn_stp_receive_data) \
	X(mtk_wcn_stp_register_event_cb) \
	X(mtk_wcn_wmt_msgcb_reg) \
	X(mtk_wcn_wmt_func_on) \
	X(mtk_wcn_wmt_func_off) \
	X(mtk_wcn_wmt_ic_info_get) \
	X(mtk_wcn_wmt_chipid_query)

#define WMT_PTR(name) static typeof(&name) p_##name;
WMT_FUNCS(WMT_PTR)

INT32 mtk_wcn_stp_send_data(const PUINT8 buffer, const UINT32 length,
			    const UINT8 type)
{
	return p_mtk_wcn_stp_send_data(buffer, length, type);
}

INT32 mtk_wcn_stp_receive_data(PUINT8 buffer, UINT32 length, UINT8 type)
{
	return p_mtk_wcn_stp_receive_data(buffer, length, type);
}

INT32 mtk_wcn_stp_register_event_cb(INT32 type, MTK_WCN_STP_EVENT_CB func)
{
	return p_mtk_wcn_stp_register_event_cb(type, func);
}

INT32 mtk_wcn_wmt_msgcb_reg(ENUM_WMTDRV_TYPE_T eType, PF_WMT_CB pCb)
{
	return p_mtk_wcn_wmt_msgcb_reg(eType, pCb);
}

MTK_WCN_BOOL mtk_wcn_wmt_func_on(ENUM_WMTDRV_TYPE_T type)
{
	return p_mtk_wcn_wmt_func_on(type);
}

MTK_WCN_BOOL mtk_wcn_wmt_func_off(ENUM_WMTDRV_TYPE_T type)
{
	return p_mtk_wcn_wmt_func_off(type);
}

UINT32 mtk_wcn_wmt_ic_info_get(ENUM_WMT_CHIPINFO_TYPE_T type)
{
	return p_mtk_wcn_wmt_ic_info_get(type);
}

INT32 mtk_wcn_wmt_chipid_query(VOID)
{
	return p_mtk_wcn_wmt_chipid_query();
}

void fm_wmt_symbols_put(void)
{
#define WMT_PUT(name) \
	if (p_##name) { \
		symbol_put_addr(p_##name); \
		p_##name = NULL; \
	}
	WMT_FUNCS(WMT_PUT)
#undef WMT_PUT
}

/*
 * By name, not symbol_get(name): our own non-exported definitions above
 * share the names, and __symbol_get() only ever finds exported symbols, so
 * it resolves to wmt_drv's.
 */
signed int fm_wmt_symbols_get(void)
{
#define WMT_GET(name) \
	p_##name = (typeof(p_##name))__symbol_get(#name); \
	if (!p_##name) { \
		WCN_DBG(FM_ERR | MAIN, "%s: %s not found, is wmt_drv loaded?\n", \
			__func__, #name); \
		fm_wmt_symbols_put(); \
		return -ENODEV; \
	}
	WMT_FUNCS(WMT_GET)
#undef WMT_GET
	return 0;
}
