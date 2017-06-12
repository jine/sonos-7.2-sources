/*
 * linux/sound/arm/codec/ac97acodecplat.c.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include    <asm/arch/codec/ac97acodecplat.h>
#include    <asm/arch/codec/ac97acodec.h>
#include    <asm/arch/hardware.h>
#include    <asm/arch/pxa-regs.h>

extern void pxa3xx_enable_ac97_pins(void);
extern acodec_error_t zy_ac97_acodec_init(acodec_context_t *p_ac97_ctxt);
extern acodec_error_t zy_ac97_acodec_read(acodec_context_t *codec_context,
					  unsigned short offset,
					  unsigned short *pdata);
acodec_error_t	zy_ac97_acodec_mfp_init(acodec_context_t *acodec_context)
{
	unsigned short codec_id;

	pxa3xx_mfp_set_afds(MFP_RSVD_AC97_SDATA_IN_0, MFP_AF0, MFP_DS03X);
	pxa3xx_enable_ac97_pins();
	zy_ac97_acodec_init(acodec_context);
    	if (zy_ac97_acodec_read(acodec_context, 0x0, &codec_id)){
		/*
		 * there is a bug on MonahansL/MonhansPL PC card: AC97_SDATA_IN is not connected to CODEC
		 * ECO 72: Connect PWM_0(MFP_RSVD_AC97_SDATA_IN_0) to CODEC as AC97_SDATA_IN
		 */
		pxa3xx_mfp_set_afds(MFP_RSVD_AC97_SDATA_IN_0, MFP_RSVD_AC97_SDATA_IN_0_AF, MFP_DS03X);
		pxa3xx_mfp_set_afds(MFP_AC97_SDATA_IN_0, MFP_AF0, MFP_DS01X);
	}

	return ACODEC_SUCCESS;
}

acodec_error_t	zy_ac97_acodec_mfp_deinit(acodec_context_t *acodec_context)
{
	/* do later: free all MFP resources. */
	return ACODEC_SUCCESS;
}
