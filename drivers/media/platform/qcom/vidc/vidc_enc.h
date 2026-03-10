/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm VIDC 1080p Video Encoder driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 */

#ifndef __VIDC_ENC_H__
#define __VIDC_ENC_H__

#include "vidc_core.h"

/* Encoder registration */
int vidc_enc_register(struct vidc_core *core);
void vidc_enc_unregister(struct vidc_core *core);

#endif /* __VIDC_ENC_H__ */
