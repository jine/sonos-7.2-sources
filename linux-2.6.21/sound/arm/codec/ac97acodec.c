/*
 * linux/sound/arm/codec/ac97acodec.c.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <asm/arch/codec/ac97acodec.h>

static int zy_ac97_acodec_link_lock(zy_ac97_acodec_t * p_ac97);
static acodec_error_t  zy_ac97_acodec_shut_down_aclink(
		zy_ac97_acodec_t * p_ac97_reg, int * p_ost_regs);

static acodec_error_t  zy_ac97_acodec_cold_reset(acodec_context_t * p_ac97_ctxt)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	p_zy_ac97acodec_t p_ac97 = (p_zy_ac97acodec_t)(p_ac97_ctxt->p_ctrl_reg);
	int		 pri_codec_ready;
	unsigned long		time_remaining;

	p_ac97->gcr = 0;
	p_ac97->gcr |= ZY_AC97_GCR_CLKBPB_MSK;
	/*  Hold reset active for a minimum time */
	udelay(ZY_AC97_COLD_HOLDTIME);
	p_ac97->gcr &= ~ZY_AC97_GCR_CLKBPB_MSK;

	/*  Deactivate cold reset condition */
	p_ac97->gcr |= (ZY_AC97_GCR_COLD_RESET_MSK | ZY_AC97_GCR_WARM_RESET_MSK);


	pri_codec_ready = 0;
	time_remaining = (p_ac97_ctxt->u_max_setup_time_out_ms) * 10;
	do  {
		udelay(1);
		if (p_ac97->gsr & ZY_AC97_GSR_PCRDY_MSK)
			pri_codec_ready = 1;
	} while (time_remaining-- && (pri_codec_ready == 0));

	/*  Timeout status if some of the devices weren't ready. */
	if (pri_codec_ready == 0) {
		status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
	}

	return (status);
}


/*******************************************************************************
 * FUNCTION:		 zy_ac97_acodec_init
 *
 * DESCRIPTION:	Perform a cold reset of the AC'97 controller and codec(s).
 *
 *		Note: Does not enable any interrupt types within the
 *		ACUNIT, so no interrupts should occur at this point.
 *
 * INPUT PARAMETERS:
 *	   p_ac97_ctxt  pointer to a acodec_context_t struct, which contains
 *			necessary setup information.
 * RETURNS:
 *	   Success:	0
 *	   Failure:	Refer to zy_ac97_acodec_cold_reset
 *
 * GLOBAL EFFECTS:
 *
 * ASSUMPTIONS:	- No other systems, such as debuggers, are using the
 *		  AC '97 Controller or codecs, or the AC Link.
 *		- The software system will be unharmed by a cold reset of
 *		  the entire AC '97 subsystem and any associated devices.
 *		- Ac97 Clock should be enabled before the function called
 *******************************************************************************/

acodec_error_t  zy_ac97_acodec_init(acodec_context_t *p_ac97_ctxt)
{
	acodec_error_t	status ;
	status = zy_ac97_acodec_cold_reset(p_ac97_ctxt);

	return (status);
}

/*******************************************************************************
 *
 * FUNCTION:		 zy_ac97_acodec_deinit
 *
 * DESCRIPTION:	  Shutdown AC97 unit clearly and thoroughly.
 *				   Release GPIOs used by AC97.
 *				   Disable clock to AC97 unit.
 *
 * INPUT PARAMETERS:
 *	   p_ac97_ctxt  pointer to a acodec_context_t struct, which contains
 *			necessary information.
 *
 * RETURNS:
 *	   Success:	0 (AC97_NO_ERROR)
 *	   Failure:	Refer to zy_ac97_acodec_shut_down_aclink
 *******************************************************************************/
acodec_error_t  zy_ac97_acodec_deinit(acodec_context_t * p_ac97_ctxt)
{
	acodec_error_t	status ;

	status = zy_ac97_acodec_shut_down_aclink(
			(p_zy_ac97acodec_t)(p_ac97_ctxt->p_ctrl_reg),
			p_ac97_ctxt->p_ost_regs);

	return (status);
}

acodec_error_t zy_ac97_acodec_write (acodec_context_t *codec_context,
		unsigned short offset, unsigned short data)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	int			got_link;
	unsigned long		time_remaining;
	volatile unsigned long *	p_codec_reg;
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(codec_context->p_ctrl_reg);
	unsigned long max_rw_time_out_us = (codec_context->u_max_read_write_time_out_ms) * 1000;


	if(offset == ZY_AC97_CR_E_MDM_GPIO_PIN_STAT) {
		/* it is a special register and sent out on slot 12 */
		p_codec_reg = &(p_ac97_reg->codec_regs_primary_mdm[0]);
		p_codec_reg += offset / ZY_AC97_CODEC_REGS_PER_WORD;
		/*  The data will be sent out on slot 12. */
		*p_codec_reg = (unsigned long)data;
		goto done;
	}

	/*  Point to specified register within area mapped to target codec regs */
	p_codec_reg = &(p_ac97_reg->codec_regs_primary_aud[0]);
	p_codec_reg += offset / ZY_AC97_CODEC_REGS_PER_WORD;

	/* Lock the ACLINK */
	time_remaining = ZY_AC97_LOCK_TIMEOUT_DEF;
	do {
		got_link = zy_ac97_acodec_link_lock(p_ac97_reg);
		if (0 == got_link)	/*  1 usec is a long time.  Skip delay if possible. */
		{
			udelay(1);
		}
	}		/*  Wait while time remaining and ACLINK not available */
	while (time_remaining-- && (0 == got_link));

	if (0 == got_link)	/*  Didn't get the ACLINK */
	{
		status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
	}
	else	/*  We got the link. Perform the write operation and don't wait. */
	{
		/*  First, clear old write status indication CDONE by writing a ONE to that bit. */
		p_ac97_reg->gsr = ZY_AC97_GSR_CDONE_MSK;

		*p_codec_reg = (unsigned long)data;	   /*  Now the write! */

		/* Wait until write cycle is complete. There should be a way
		 * to do this speculatively at the beginning of the procedure.
		 * Need to discover it. Too inefficient to always wait.
		 */

		time_remaining = max_rw_time_out_us;
		do
		{
			udelay(1);
		}	 /*  Wait while time remaining and command I/O still incomplete. */
		while ( (time_remaining--) && !(p_ac97_reg->gsr & ZY_AC97_GSR_CDONE_MSK));
		if (!(p_ac97_reg->gsr & ZY_AC97_GSR_CDONE_MSK))
		{
			status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
			p_ac97_reg->car = ZY_AC97_CAR_CAIP_CLEAR;
		}
	}  /*  Got AC link */

done:
	return(status);
} /*  Ac97CtrlCodecWrite() */


acodec_error_t zy_ac97_acodec_read  (acodec_context_t *codec_context, unsigned short offset,  unsigned short *pdata)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	int			got_link;
	unsigned long		time_remaining;
	volatile unsigned long *	p_codec_reg;
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(codec_context->p_ctrl_reg);
	unsigned long max_rw_time_out_us = (codec_context->u_max_read_write_time_out_ms) * 1000;

	/*  Point to specified register within area mapped to target codec regs */
	p_codec_reg = &(p_ac97_reg->codec_regs_primary_aud[0]);
	p_codec_reg += offset / ZY_AC97_CODEC_REGS_PER_WORD;

	/* Lock the ACLINK */
	time_remaining = ZY_AC97_LOCK_TIMEOUT_DEF;
	do
	{
		got_link = zy_ac97_acodec_link_lock(p_ac97_reg);
		if (0 == got_link)	/*  1 usec is a long time.  Skip delay if possible. */
		{
			udelay(1);
		}
	}		/*  Wait while time remaining and ACLINK not available */
	while (time_remaining-- && (0 == got_link));

	if (0 == got_link)	/*  Didn't get the ACLINK */
	{
		status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
	}
	else	/*  We got the link. Perform the write operation and don't wait. */
	{
		 /*  First, clear old read status indications. */
		p_ac97_reg->gsr = ZY_AC97_GSR_SDONE_MSK | ZY_AC97_GSR_RCS_ERR_MSK;

		*pdata = (unsigned short)(*p_codec_reg); /*  This is THE DUMMY READ. */

		 /*  Wait for read I/O with codec to complete before doing real read. */
		time_remaining = max_rw_time_out_us;
		do
		{
			udelay(1);
		}   /*  Wait while time remaining and read I/O still incomplete */
		while( (time_remaining--) && (!(p_ac97_reg->gsr & ZY_AC97_GSR_SDONE_MSK)) );

		if ((p_ac97_reg->gsr & ZY_AC97_GSR_SDONE_MSK) && (!(p_ac97_reg->gsr & ZY_AC97_GSR_RCS_ERR_MSK)) )
		{
			if (p_ac97_reg->gsr & ZY_AC97_GSR_RCS_ERR_MSK)
			{/* timeout indicated by RCS bit */
				status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
			}
			/*  succeed in reading. clear status bits first. */
			p_ac97_reg->gsr = ZY_AC97_GSR_SDONE_MSK | ZY_AC97_GSR_RCS_ERR_MSK;
			*pdata = (unsigned short)(*p_codec_reg);	/*  THE REAL READ. */
			if (*pdata == 0xffff)
			{/* timeout indicated by returned value */
				status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
			}
			/* check later: is second waiting really needed? */
	 		time_remaining = max_rw_time_out_us;
			do
			{
				udelay(1);
  			}   /*  Wait while time remaining and read I/O still incomplete */
			while( (time_remaining--) && (!(p_ac97_reg->gsr & ZY_AC97_GSR_SDONE_MSK)) );
		}
		else	/*  failed */
		{
			status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
			p_ac97_reg->car = ZY_AC97_CAR_CAIP_CLEAR;
		} /*  else  (OK to do real read) */

	} /*  else  (We got the link.  Perform the read operations.) */

	return (status);
}

static acodec_error_t  zy_ac97_acodec_shut_down_aclink(p_zy_ac97acodec_t p_ac97_reg, int * p_ost_regs)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned long time_remaining = ZY_AC97_LINKOFF_TIMEOUT_DEF;

	p_ac97_reg->gcr |= ZY_AC97_GCR_LINK_OFF_MSK;
	p_ac97_reg->gcr |= ZY_AC97_GCR_CLKBPB_MSK;

	while (!(p_ac97_reg->gsr & ZY_AC97_GSR_ACOFFD_MSK))
	{
		time_remaining --;
		if (0 == time_remaining)
		{
			status = ACODEC_CONTROLLER_INTERFACE_TIMEOUT;
			break;
		}
		udelay(1);
	}
	p_ac97_reg->gcr |= ZY_AC97_GCR_FRCRST_MSK;
	/* check later: any delay needed */
	p_ac97_reg->gcr &= ~ZY_AC97_GCR_FRCRST_MSK;
	p_ac97_reg->gcr &= ~ZY_AC97_GCR_CLKBPB_MSK;

	return(status);
}

static int zy_ac97_acodec_link_lock(p_zy_ac97acodec_t p_ac97_reg)
{
	int		status = 1;
	volatile unsigned long	car_tmp;

	car_tmp = p_ac97_reg->car;
	if (car_tmp & ZY_AC97_CAR_CAIP_MSK)	/*  "1" in CAIP bit means lock failed. */
	{
		status = 0;
	}
	return (status);
}


acodec_error_t zy_ac97_acodec_save_context (acodec_context_t *codec_context, zy_ac97_save_context_t *p_ac97_save_context)
{
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(codec_context->p_ctrl_reg);
	p_ac97_save_context->pocr = p_ac97_reg->pocr;
	p_ac97_save_context->picr = p_ac97_reg->picr;
	p_ac97_save_context->mccr = p_ac97_reg->mccr;
	p_ac97_save_context->pcscr = p_ac97_reg->pcscr;
	p_ac97_save_context->pcclcr = p_ac97_reg->pcclcr;
	p_ac97_save_context->mocr = p_ac97_reg->mocr;
	p_ac97_save_context->micr = p_ac97_reg->micr;
	p_ac97_save_context->gcr = p_ac97_reg->gcr;

	return ACODEC_SUCCESS;
}

acodec_error_t zy_ac97_acodec_restore_context (acodec_context_t *codec_context, zy_ac97_save_context_t *p_ac97_save_context)
{
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(codec_context->p_ctrl_reg);

	p_ac97_reg->pocr = p_ac97_save_context->pocr;
	p_ac97_reg->picr = p_ac97_save_context->picr;
	p_ac97_reg->mccr = p_ac97_save_context->mccr;
	p_ac97_reg->pcscr = p_ac97_save_context->pcscr;
	p_ac97_reg->pcclcr = p_ac97_save_context->pcclcr;
	p_ac97_reg->mocr = p_ac97_save_context->mocr;
	p_ac97_reg->micr = p_ac97_save_context->micr;
	/* check later: any side effect to restore ac97 GCR? */
	p_ac97_reg->gcr = p_ac97_save_context->gcr;

	return ACODEC_SUCCESS;
}


