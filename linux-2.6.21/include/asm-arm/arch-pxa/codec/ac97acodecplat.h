/*
 * linux/sound/arm/codec/ac97acodecplat.h.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef ZY_AC97_ACODEC_PLAT_H
#define ZY_AC97_ACODEC_PLAT_H
#include "acodec.h"

extern acodec_error_t	zy_ac97_acodec_mfp_init(acodec_context_t *p_device_context);
extern acodec_error_t	zy_ac97_acodec_mfp_deinit(acodec_context_t *p_device_context);

extern void init_codec_state(acodec_context_t *p_device_context);
#endif
