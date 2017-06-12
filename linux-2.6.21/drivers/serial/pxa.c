/*
 *  linux/drivers/serial/pxa.c
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
 *  Author:	Nicolas Pitre
 *  Created:	Feb 20, 2003
 *  Copyright:	(C) 2003 Monta Vista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Note 1: This driver is made separate from the already too overloaded
 * 8250.c because it needs some kirks of its own and that'll make it
 * easier to add DMA support.
 *
 * Note 2: I'm too sick of device allocation policies for serial ports.
 * If someone else wants to request an "official" allocation of major/minor
 * for this driver please be my guest.  And don't forget that new hardware
 * to come from Intel might have more than 3 or 4 of those UARTs.  Let's
 * hope for a better port registration and dynamic device allocation scheme
 * with the serial core maintainer satisfaction to appear soon.
 */


#if defined(CONFIG_SERIAL_PXA_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/pxa-regs.h>
#include <asm/dma.h>
#ifdef CONFIG_DVFM
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>
#endif
#ifdef CONFIG_IPM
#include <asm/arch/ipmc.h>
#endif

#if defined(CONFIG_SERIAL_PXA_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#ifdef	CONFIG_PXA27x
#include <asm/arch/mainstone.h>
#endif

#include "mdp.h"

#define		UART_IER_DMA		(1 << 7)
#define		UART_LSR_FIFOE		(1 << 7)
#define		UART_FCR_PXA_BUS32	(1 << 5)
#define		UART_FCR_PXA_TRAIL	(1 << 4)
#define		DMA_BLOCK		UART_XMIT_SIZE

#define		POWER_RUN		(0)
#define		POWER_PRE_SUSPEND	(1)
#define		POWER_SUSPEND		(2)
#define		POWER_PRE_RESUME	(3)


struct uart_pxa_port {
	struct uart_port        port;
	unsigned                ier;
	unsigned char           lcr;
	unsigned char           mcr;
	unsigned int            lsr_break_flag;
	unsigned int		cken;
#ifdef CONFIG_DVFM
	struct pxa3xx_fv_notifier dvfm_notifier;
	unsigned int		baud;
	unsigned int		inuse;
	wait_queue_head_t	delay_wait;
#endif
	char			*name;

	int			txdma;
	int			rxdma;
	void 			*txdma_addr;
	void			*rxdma_addr;
	dma_addr_t		txdma_addr_phys;
	dma_addr_t		rxdma_addr_phys;
	int			tx_stop;
	int			rx_stop;
	int			data_len;
	volatile long 		*txdrcmr;
	volatile long		*rxdrcmr;

#ifdef	CONFIG_PM
	/* We needn't save rx dma register because we
	 * just restart the dma totallly after resume
	 */
	void			*buf_save;
	unsigned long		dcsr_tx;
	unsigned long		dsadr_tx;
	unsigned long		dtadr_tx;
	unsigned long		dcmd_tx;
	unsigned int		power_mode;
#endif
	struct	tasklet_struct	tklet;

};

static int uart_dma;
#ifdef CONFIG_SERIAL_PXA_DMA
static void pxa_uart_transmit_dma(int channel, void *data);
static void pxa_uart_receive_dma(int channel, void *data);
static void pxa_uart_receive_dma_err(struct uart_pxa_port *up,int *status);
static void pxa_uart_transmit_dma_start(struct uart_pxa_port *up, int count);
static void pxa_uart_receive_dma_start(struct uart_pxa_port *up);
#endif

#ifdef CONFIG_DVFM
static int uart_dvfm_notifier(unsigned int cmd, void* client_data, void *info);
#endif
static int is_switch_clk(void);

static inline unsigned int serial_in(struct uart_pxa_port *up, int offset)
{
	offset <<= 2;
	return readl(up->port.membase + offset);
}

static inline void serial_out(struct uart_pxa_port *up, int offset, int value)
{
	offset <<= 2;
	writel(value, up->port.membase + offset);
}

static void serial_pxa_enable_ms(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;

	if (uart_dma)
		return;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void serial_pxa_stop_tx(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	if (uart_dma) {
		if (up->ier & UART_IER_DMA) {
			while (!(DCSR(up->txdma) & DCSR_STOPSTATE))
				rmb();
				
		}
	} else {
		if (up->ier & UART_IER_THRI) {
			up->ier &= ~UART_IER_THRI;
			serial_out(up, UART_IER, up->ier);
		}
 	}
	up->tx_stop = 1;
}

static void serial_pxa_stop_rx(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;

	if (uart_dma) {
		if (up->ier & UART_IER_DMA) {
			DCSR(up->rxdma) &= ~DCSR_RUN;
			while (!(DCSR(up->rxdma) & DCSR_STOPSTATE))
				rmb();
		}
	} else {
		up->ier &= ~UART_IER_RLSI;
		up->port.read_status_mask &= ~UART_LSR_DR;
		serial_out(up, UART_IER, up->ier);
	}
	up->rx_stop = 1;
}

static inline void pio_receive_chars(struct uart_pxa_port *up, int *status)
{
	struct tty_struct *tty = up->port.info->tty;
	unsigned int ch, flag;
	int max_count = 256;

	do {
		ch = serial_in(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_PXA_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);

	ignore_char:
		*status = serial_in(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
}

#ifndef CONFIG_SERIAL_PXA_DMA
static void pio_transmit_chars(struct uart_pxa_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_pxa_stop_tx(&up->port);
		return;
	}

	count = up->port.fifosize / 2;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);


	if (uart_circ_empty(xmit))
		serial_pxa_stop_tx(&up->port);
}
#endif

#ifdef CONFIG_SERIAL_PXA_DMA
static inline void
dma_receive_chars(struct uart_pxa_port *up, int *status)
{
	struct tty_struct *tty = up->port.info->tty;
	unsigned char ch;
	int max_count = 256;
	int count = 0;
	unsigned char *tmp;
	unsigned int flag = TTY_NORMAL;

	DCSR(up->rxdma) &= ~DCSR_RUN;
	count = DTADR(up->rxdma) - up->rxdma_addr_phys;
	tmp = up->rxdma_addr;

	while (count > 0) {
		if (!uart_handle_sysrq_char(&up->port, *tmp))
			uart_insert_char(&up->port, *status, 0, *tmp, flag);
		tmp++;
		count--;
	}

	do {
		ch = serial_in(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
					UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char2;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_PXA_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (!uart_handle_sysrq_char(&up->port, ch))
			uart_insert_char(&up->port, *status, UART_LSR_OE,
					 ch, flag);
	ignore_char2:
		*status = serial_in(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));

	tty_schedule_flip(tty);
	if (up->rx_stop)
		return;
	pxa_uart_receive_dma_start(up);
}
#endif

static void serial_pxa_start_tx(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;

	if (uart_dma) {
		tasklet_schedule(&up->tklet);
	} else {
		if (!(up->ier & UART_IER_THRI)) {
			up->ier |= UART_IER_THRI;
			serial_out(up, UART_IER, up->ier);
		}
	}
	up->tx_stop=0;
}

static inline void check_modem_status(struct uart_pxa_port *up)
{
	int status;

	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

	wake_up_interruptible(&up->port.info->delta_msr_wait);
}

#ifdef	CONFIG_PXA27x_E20
static inline void disable_timeout_int(struct uart_pxa_port *up)
{
	up->ier &= ~UART_IER_RTOIE;
	serial_out(up, UART_IER, up->ier);
}

static inline void enable_timeout_int(struct uart_pxa_port *up)
{
	up->ier |= UART_IER_RTOIE;
	serial_out(up, UART_IER, up->ier);
}
#else
static inline void disable_timeout_int(struct uart_pxa_port *up) {}
static inline void enable_timeout_int(struct uart_pxa_port *up) {}
#endif

#ifdef	CONFIG_PXA27x_E36
static void pxa_uart_e36fix(struct uart_pxa_port *up)
{	
	unsigned int lsr;
	lsr = serial_in(up, UART_LSR);
	pxa_uart_receive_dma_err(up, &lsr);
}
#else
static void pxa_uart_e36fix(struct uart_pxa_port *up) {}
#endif

/*
 * This handles the interrupt from one port.
 */
static inline irqreturn_t serial_pxa_irq(int irq, void *dev_id)
{
	struct uart_pxa_port *up = dev_id;
	unsigned int iir, lsr, lcr;

	iir = serial_in(up, UART_IIR);
	if (iir & UART_IIR_NO_INT) {
		pxa_uart_e36fix(up);
		/* FIXME, should return IRQ_NONE normally, but it would report
		 * unknown irq in PIO mode */
		return IRQ_HANDLED;
	}
	lsr = serial_in(up, UART_LSR);
#ifdef CONFIG_SERIAL_PXA_DMA
		if (UART_LSR_FIFOE & lsr) 
			pxa_uart_receive_dma_err(up, &lsr);

		if (iir & UART_IIR_TOD) {
			dma_receive_chars(up, &lsr);
#ifdef CONFIG_IPM
			ipm_event_notify(IPM_EVENT_DEVICE, IPM_EVENT_DEVICE_OUTD0CS, NULL, 0);
#endif
		}	
#else
		if (lsr & UART_LSR_DR) {
			/*FIXME: PXA27x E20 */
			disable_timeout_int(up);
			pio_receive_chars(up, &lsr);
			enable_timeout_int(up);
#ifdef CONFIG_IPM
			ipm_event_notify(IPM_EVENT_DEVICE, IPM_EVENT_DEVICE_OUTD0CS, NULL, 0);
#endif
		}
		check_modem_status(up);
		if (lsr & UART_LSR_THRE)
			pio_transmit_chars(up);
#endif
	lcr = serial_in(up, UART_LCR);
	return IRQ_HANDLED;
}

static unsigned int serial_pxa_tx_empty(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	if (uart_dma) {
		if (up->ier & UART_IER_DMA) {
			if (DCSR(up->txdma) & DCSR_RUN) {
				spin_unlock_irqrestore(&up->port.lock, flags);
				return 0;
			}
		}
	}
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serial_pxa_get_mctrl(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned char status;
	unsigned int ret;

	status = serial_in(up, UART_MSR);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_pxa_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS) {
		if (uart_dma) {
			mcr |= UART_MCR_RTS | UART_MCR_AFE;
		} else {
			mcr |= UART_MCR_RTS;
		}
	}	
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	if (uart_dma) {
		if (mctrl & TIOCM_CTS) {
			mcr |= UART_MCR_AFE;
		}
	}	

	mcr |= up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serial_pxa_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

#ifdef CONFIG_SERIAL_PXA_DMA
static void pxa_uart_transmit_dma_start(struct uart_pxa_port *up, int count)
{
	if (!(DCSR(up->txdma) & DCSR_STOPSTATE))
		return;

	DCSR(up->txdma)  = DCSR_NODESC;
	DSADR(up->txdma) = up->txdma_addr_phys;
	DTADR(up->txdma) = up->port.mapbase;
	DCMD(up->txdma) = DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_ENDIRQEN | DCMD_WIDTH1 | DCMD_BURST16 | count;
	DCSR(up->txdma) |= DCSR_RUN;
}

static void pxa_uart_receive_dma_start(struct uart_pxa_port *up)
{
	DCSR(up->rxdma)  = DCSR_NODESC;
	DSADR(up->rxdma) = up->port.mapbase;
	DTADR(up->rxdma) = up->rxdma_addr_phys;
	DCMD(up->rxdma) = DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_ENDIRQEN | DCMD_WIDTH1 | DCMD_BURST16 | DMA_BLOCK;
	DCSR(up->rxdma) |= DCSR_RUN;
}

static void pxa_uart_receive_dma_err(struct uart_pxa_port *up,int *status)
{
	unsigned char ch;
	struct tty_struct *tty = up->port.info->tty;
	unsigned char *tmp;
	int count;
	unsigned int flag = 0;

	DCSR(up->rxdma) &= ~DCSR_RUN;

	/* if have DMA reqeust, wait. */
	while (!(DCSR(up->rxdma) & DCSR_STOPSTATE))
		rmb();

	count = DTADR(up->rxdma) - up->rxdma_addr_phys;
	tmp = up->rxdma_addr;

	while (count > 0) {
		if (count > TTY_FLIPBUF_SIZE) {
			tty_insert_flip_string(tty, tmp, TTY_FLIPBUF_SIZE);
			up->port.icount.rx += TTY_FLIPBUF_SIZE;
			tmp += TTY_FLIPBUF_SIZE;
			count -= TTY_FLIPBUF_SIZE;
		} else {
			tty_insert_flip_string(tty, tmp, count);
			up->port.icount.rx += count;
			tmp += count;
			count -= count;
		}
	}

	do {
		ch = serial_in(up, UART_RX);
		up->port.icount.rx++;

		/*
		 * For statistics only
		 */
		if (*status & UART_LSR_BI) {
			*status &= ~(UART_LSR_FE | UART_LSR_PE);
			
			up->port.icount.brk++;
			/*
			 * We do the SysRQ and SAK checking
			 * here because otherwise the break
			 * may get masked by ignore_status_mask
			 * or read_status_mask.
			 */
			if (uart_handle_break(&up->port))
				goto ignore_char;
			flag = TTY_BREAK;
		} else if (*status & UART_LSR_PE) {
			up->port.icount.parity++;
			flag = TTY_PARITY;
		} else if (*status & UART_LSR_FE) {
			up->port.icount.frame++;
			flag = TTY_FRAME;
		}

		if (*status & UART_LSR_OE){
			up->port.icount.overrun++;
		}

		/*
		 * Mask off conditions which should be ignored.
		 */
		*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_PXA_CONSOLE
		if (up->port.line == up->port.cons->index) {
			/* Recover the break flag from console xmit */
			*status |= up->lsr_break_flag;
			up->lsr_break_flag = 0;
		}
#endif

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);

	ignore_char:
		*status = serial_in(up, UART_LSR);
	} while (*status & UART_LSR_DR);

	tty_flip_buffer_push(tty);
	if (up->rx_stop)
		return;
	pxa_uart_receive_dma_start(up);
}

static void pxa_uart_receive_dma(int channel, void *data)
{
	volatile unsigned long dcsr;
	struct uart_pxa_port *up = (struct uart_pxa_port *)data;
	struct tty_struct *tty = up->port.info->tty;
	unsigned int count;
	unsigned char *tmp = up->rxdma_addr;

	DCSR(channel) &= ~DCSR_RUN;
	dcsr = DCSR(channel);

	if ((dcsr & DCSR_ENDINTR) || (dcsr & DCSR_STOPSTATE)) {
		if (dcsr & DCSR_ENDINTR)
			DCSR(channel) |= DCSR_ENDINTR;
		if (dcsr & DCSR_STOPSTATE)
			DCSR(channel) &= ~DCSR_STOPSTATE;
	
		count = DTADR(channel) - up->rxdma_addr_phys;
		while (count > 0) {
			if (count > TTY_FLIPBUF_SIZE) {
				tty_insert_flip_string(tty, tmp, TTY_FLIPBUF_SIZE);
				up->port.icount.rx += TTY_FLIPBUF_SIZE;
				tmp += TTY_FLIPBUF_SIZE;
				count -= TTY_FLIPBUF_SIZE;
			} else {
				tty_insert_flip_string(tty, tmp, count);
				up->port.icount.rx += count;
				tmp += count;
				count -= count;
			}
		}
		tty_flip_buffer_push(tty);
		if (up->rx_stop)
			return;
		pxa_uart_receive_dma_start(up);
	}	
	return;
}

static void pxa_uart_transmit_dma(int channel, void *data)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)data;
	struct circ_buf *xmit = &up->port.info->xmit;
	volatile unsigned long dcsr;

	DCSR(channel) &= ~DCSR_RUN;
	dcsr = DCSR(channel);

	if (dcsr & DCSR_BUSERR) {
		DCSR(channel) |= DCSR_BUSERR;
		printk(KERN_ALERT "%s(): DMA channel bus error\n", __func__);
	}

	if ((dcsr & DCSR_ENDINTR) || (dcsr & DCSR_STOPSTATE)) {
		if (dcsr & DCSR_STOPSTATE) {
			DCSR(channel) &= ~DCSR_STOPSTATE;
		}

		if (dcsr & DCSR_ENDINTR) {
			DCSR(channel) |= DCSR_ENDINTR;
		}
		
		/* if tx stop, stop transmit DMA and return */
		if (up->tx_stop) {
			return;
		}

		if (up->port.x_char) {
			serial_out(up, UART_TX,up->port.x_char);
			up->port.icount.tx++;
			up->port.x_char = 0;
		}

		if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&up->port);

		if (!uart_circ_empty(xmit))
			tasklet_schedule(&up->tklet);
	}
	return;
}

extern void *dma_alloc_coherent(struct device *dev, size_t size,
				dma_addr_t *handle, gfp_t gfp);
extern void dma_free_coherent(struct device *dev, size_t size,
			      void *cpu_addr, dma_addr_t handle);
static void uart_pxa_dma_init(struct uart_pxa_port *up)
{
#ifdef CONFIG_PM
	pr_debug("enter, with power_mode = %d\n", up->power_mode);
	/* Resume. Needn't alloc dma channel and buffer just
	 * set the DRCMR register
	 */
	if (POWER_PRE_RESUME == up->power_mode) {
		writel(up->rxdma | DRCMR_MAPVLD, up->rxdrcmr);
		writel(up->txdma | DRCMR_MAPVLD, up->txdrcmr);
		return;
	}
#endif

	if (0 == up->rxdma) {
		up->rxdma =
			pxa_request_dma(up->name, DMA_PRIO_LOW, pxa_uart_receive_dma, up);
		if (up->rxdma < 0)
			goto out;
	}	

	if (0 == up->txdma) {
		up->txdma =
			pxa_request_dma(up->name, DMA_PRIO_LOW, pxa_uart_transmit_dma, up);
		if (up->txdma < 0)
			goto err_txdma;
	}	

	if (NULL == up->txdma_addr) {
		up->txdma_addr = dma_alloc_coherent(NULL, DMA_BLOCK, &up->txdma_addr_phys, GFP_KERNEL);
		if (!up->txdma_addr)
			goto txdma_err_alloc;
	}	

	if (NULL == up->rxdma_addr) {
		up->rxdma_addr = dma_alloc_coherent(NULL, DMA_BLOCK, &up->rxdma_addr_phys, GFP_KERNEL);
		if (!up->rxdma_addr)
			goto rxdma_err_alloc;
	}	

#ifdef CONFIG_PM
	up->buf_save = kmalloc(DMA_BLOCK, GFP_KERNEL);
	if (!up->buf_save) {
		goto buf_err_alloc;
	}
#endif

	writel(up->rxdma | DRCMR_MAPVLD, up->rxdrcmr);
	writel(up->txdma | DRCMR_MAPVLD, up->txdrcmr);

	return;

#ifdef CONFIG_PM
buf_err_alloc:
	dma_free_coherent(NULL, DMA_BLOCK, up->rxdma_addr, up->rxdma_addr_phys);
	up->rxdma_addr = NULL;
#endif
rxdma_err_alloc:
	dma_free_coherent(NULL, DMA_BLOCK, up->txdma_addr, up->txdma_addr_phys);
	up->txdma_addr = NULL;
txdma_err_alloc:
	pxa_free_dma(up->txdma);
	up->txdma = 0;
err_txdma:
	pxa_free_dma(up->rxdma);
	up->rxdma = 0;
out:
	return;
}

static void uart_pxa_dma_uninit(struct uart_pxa_port *up)
{
#ifdef CONFIG_PM
	if (POWER_PRE_SUSPEND == up->power_mode)
		return;
#endif

	if ( DCSR(up->rxdma) & DCSR_RUN)
		DCSR(up->rxdma) &= ~DCSR_RUN;

	if ( DCSR(up->txdma) & DCSR_RUN) 
		DCSR(up->txdma) &= ~DCSR_RUN;

	if (up->txdma_addr != NULL) {
		dma_free_coherent(NULL, DMA_BLOCK, up->txdma_addr, up->txdma_addr_phys);
		up->txdma_addr = NULL;
	}
	if (up->txdma != 0) {
		pxa_free_dma(up->txdma);
		writel(0, up->txdrcmr);
		up->txdma = 0;
	}	
	
	if (up->rxdma_addr != NULL) {
		dma_free_coherent(NULL, DMA_BLOCK, up->rxdma_addr, up->rxdma_addr_phys);
		up->rxdma_addr = NULL;
	}

	if (up->rxdma != 0) {
		pxa_free_dma(up->rxdma);
		writel(0, up->rxdrcmr);
		up->rxdma = 0;
	}	
		
	return;
}

static void uart_task_action(unsigned long data)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)data;
	struct circ_buf *xmit = &up->port.info->xmit;
	unsigned char *tmp = up->txdma_addr;
	unsigned long flags;
	int count = 0,c;

	/* if the tx is stop, just return.*/
	if (up->tx_stop)
		return;

	if ((DCSR(up->txdma) & DCSR_RUN))
		return;

	spin_lock_irqsave(&up->port.lock, flags);
	while (1) {
		c = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		if (c <= 0)
			break;
		
		memcpy(tmp, xmit->buf + xmit->tail, c);		
		xmit->tail = (xmit->tail + c) & (UART_XMIT_SIZE -1);
		tmp += c;
		count += c;
		up->port.icount.tx += c;
	}
	spin_unlock_irqrestore(&up->port.lock, flags);
	
	tmp = up->txdma_addr;
	up->tx_stop = 0;

	pr_debug("count =%d", count);

	pxa_uart_transmit_dma_start(up,count);
}
#endif

static int serial_pxa_startup(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned long flags;
	int retval;

	if (port->line == 3) /* HWUART */
		up->mcr |= UART_MCR_AFE;
	else
		up->mcr = 0;

#if defined(CONFIG_PXA3xx) && defined(CONFIG_DVFM)
	/* if the CPU is in D0CS mode, BT UART could not run normally */
        if ((IRQ_BTUART == up->port.irq) && (ACSR & 0x04000000)) {
#ifdef CONFIG_IPM
		ipm_event_notify(IPM_EVENT_DEVICE, IPM_EVENT_DEVICE_BUSY,
		       		NULL, 0);
#endif
		retval = wait_event_interruptible_timeout(up->delay_wait, 
				(!(ACSR & 0x04000000)), HZ/100);
		if((retval <= 0) && (ACSR & 0x04000000)){ 
			pr_debug("In D0CS mode, please try BTUART again\n");
			return -EAGAIN;
		}
	}
#endif

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_pxa_irq, 0, up->name, up);
	if (retval)
		return retval;

#ifdef	CONFIG_SERIAL_PXA_DMA
	uart_dma = 1;
#else
	uart_dma = 0;
#endif

	/* Enable the pins/clock for UART on PXA3xx
	 * Even one UART will be initialized on bootloader, we still add
	 * UART pins/clock initialization code here. Because we don't know
	 * which UART is used for serial console.
	 */
        if (IRQ_BTUART == up->port.irq) {
#ifdef CONFIG_PXA3xx
#ifndef CONFIG_MACH_WOODSTOCK
		extern void pxa3xx_enable_btuart_pins(void);
#endif
		extern void pxa3xx_enable_pxa_mwb_bt(void);

#ifndef CONFIG_MACH_WOODSTOCK
		pxa3xx_enable_btuart_pins();
#endif
#ifdef CONFIG_PXA_MWB_12
		pxa3xx_enable_pxa_mwb_bt();
#endif
#else
                MST_MSCWR1 |= (0x03<<7);
                CKEN |= CKEN7_BTUART;
                pxa_gpio_mode(GPIO42_BTRXD_MD);
                pxa_gpio_mode(GPIO43_BTTXD_MD);
                pxa_gpio_mode(GPIO44_BTCTS_MD);
                pxa_gpio_mode(GPIO45_BTRTS_MD);
#endif
        } else if (IRQ_STUART == up->port.irq) {
#ifdef CONFIG_PXA3xx
#ifndef CONFIG_MACH_WOODSTOCK
		extern void pxa3xx_enable_stuart_pins(void);
		pxa3xx_enable_stuart_pins();
#endif
#endif
	} else if (IRQ_FFUART == up->port.irq) {
#ifdef CONFIG_PXA3xx
#ifndef CONFIG_MACH_WOODSTOCK
		extern void pxa3xx_enable_ffuart_pins(void);
		pxa3xx_enable_ffuart_pins();
#endif
#endif
	}
	pxa_set_cken(up->cken, 1);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl |= TIOCM_OUT2;
	serial_pxa_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
#ifdef CONFIG_SERIAL_PXA_DMA
		uart_pxa_dma_init(up);
		up->rx_stop = 0;
		pxa_uart_receive_dma_start(up);
		up->ier = UART_IER_DMA | UART_IER_UUE | UART_IER_RTOIE;
		tasklet_init(&up->tklet, uart_task_action, (unsigned long)up);
#else
		up->ier = UART_IER_RLSI | UART_IER_RDI | UART_IER_RTOIE | UART_IER_UUE;
#endif

	serial_out(up, UART_IER, up->ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);
#ifdef CONFIG_PM
	up->power_mode = POWER_RUN;
#endif
#ifdef CONFIG_DVFM
	up->inuse = 1;
#endif

	return 0;
}

static void serial_pxa_shutdown(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned long flags;

#ifdef CONFIG_DVFM
	up->inuse = 0;
#endif
	free_irq(up->port.irq, up);
#ifdef CONFIG_SERIAL_PXA_DMA
		tasklet_kill(&up->tklet);
		uart_pxa_dma_uninit(up);
#endif

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;
	up->port.mctrl &= ~TIOCM_RTS;
	up->port.mctrl &= ~TIOCM_CTS;
	serial_pxa_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
				  UART_FCR_CLEAR_RCVR |
				  UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);
}

static void
serial_pxa_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

#ifdef CONFIG_PXA3xx
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	if (is_switch_clk() && (ACSR & 0x04000000)) {
		/* if the CPU is in D0CS mode */
		port->uartclk = 729600*16;
	}else{
		port->uartclk = 921600*16;
	}
#endif

#if defined (CONFIG_CPU_PXA300) || defined (CONFIG_CPU_PXA310)
	baud = uart_get_baud_rate(port, termios, old, 0, 921600*16*4/16);
	if((baud > 921600) && !(ACSR & 0x04000000)){
		port->uartclk = 921600*16*4; /* 58.9823MHz as the clk src */
		up->ier |= IER_HSE;
		if(B1500000 == (termios->c_cflag & B1500000))
			quot = 2;
		if(B3500000 == (termios->c_cflag & B3500000))
			quot = 1;
	} else {
		quot = uart_get_divisor(port, baud);
		port->uartclk = 921600*16;   /* 14.7456MHz as the clk src */
		up->ier &= ~IER_HSE;
	}
#else	
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);
#endif

#ifdef CONFIG_DVFM
	up->baud = baud; /* save for DVFM scale callback */
#endif
	
	if (uart_dma) {
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_PXAR32;
		fcr &= ~UART_FCR_PXA_BUS32;
	} else {
		if ((up->port.uartclk / quot) < (2400 * 16))
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_PXAR1;
		else if ((up->port.uartclk / quot) < (230400 * 16))
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_PXAR8;
		else
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_PXAR32;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Ensure the port will be enabled.
	 * This is required especially for serial console.
	 */
	up->ier |= IER_UUE;

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */

	if (uart_dma) {
		if (termios->c_cflag & CRTSCTS) {
			up->port.mctrl |= TIOCM_CTS;
			up->port.mctrl |= TIOCM_RTS;
		}	
	} else {
		up->ier &= ~UART_IER_MSI;
		if (UART_ENABLE_MS(&up->port, termios->c_cflag))
			up->ier |= UART_IER_MSI;
	}
	
	serial_out(up, UART_IER, up->ier);

	serial_out(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_out(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	serial_pxa_set_mctrl(&up->port, up->port.mctrl);
	serial_out(up, UART_FCR, fcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
serial_pxa_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	pxa_set_cken(up->cken, !state);
	if (!state)
		udelay(1);
}

static void serial_pxa_release_port(struct uart_port *port)
{
}

static int serial_pxa_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_pxa_config_port(struct uart_port *port, int flags)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	up->port.type = PORT_PXA;
}

static int
serial_pxa_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static const char *
serial_pxa_type(struct uart_port *port)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;
	return up->name;
}

#ifdef CONFIG_SERIAL_PXA_CONSOLE

static struct uart_pxa_port serial_pxa_ports[];
static struct uart_driver serial_pxa_reg;

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_pxa_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
		       ((serial_in(up, UART_MSR) & UART_MSR_CTS) == 0))
			udelay(1);
	}
}

static void serial_pxa_console_putchar(struct uart_port *port, int ch)
{
	struct uart_pxa_port *up = (struct uart_pxa_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_pxa_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_pxa_port *up = &serial_pxa_ports[co->index];
	unsigned int ier;

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, UART_IER_UUE);

	uart_console_write(&up->port, s, count, serial_pxa_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
}

static int __init
serial_pxa_console_setup(struct console *co, char *options)
{
	struct uart_pxa_port *up;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index == -1 || co->index >= serial_pxa_reg.nr)
		co->index = 0;
       	up = &serial_pxa_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_pxa_console = {
	.name		= "ttyS",
	.write		= serial_pxa_console_write,
	.device		= uart_console_device,
	.setup		= serial_pxa_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_pxa_reg,
};

static int __init
serial_pxa_console_init(void)
{
#ifndef CONFIG_SONOS_DIAGS
	if ((sys_mdp.mdp_flags & MDP_FLAG_CONSOLE_ENABLE) == 0)
		return 0;
#endif

	register_console(&serial_pxa_console);
	return 0;
}

console_initcall(serial_pxa_console_init);

#define PXA_CONSOLE	&serial_pxa_console
#else
#define PXA_CONSOLE	NULL
#endif

struct uart_ops serial_pxa_pops = {
	.tx_empty	= serial_pxa_tx_empty,
	.set_mctrl	= serial_pxa_set_mctrl,
	.get_mctrl	= serial_pxa_get_mctrl,
	.stop_tx	= serial_pxa_stop_tx,
	.start_tx	= serial_pxa_start_tx,
	.stop_rx	= serial_pxa_stop_rx,
	.enable_ms	= serial_pxa_enable_ms,
	.break_ctl	= serial_pxa_break_ctl,
	.startup	= serial_pxa_startup,
	.shutdown	= serial_pxa_shutdown,
	.set_termios	= serial_pxa_set_termios,
	.pm		= serial_pxa_pm,
	.type		= serial_pxa_type,
	.release_port	= serial_pxa_release_port,
	.request_port	= serial_pxa_request_port,
	.config_port	= serial_pxa_config_port,
	.verify_port	= serial_pxa_verify_port,
};

static struct uart_pxa_port serial_pxa_ports[] = {
     {	/* FFUART */
	.name	= "FFUART",
#ifdef CONFIG_PXA3xx
	.cken   = CKEN_FFUART,
#else
	.cken	= CKEN6_FFUART,
#endif
	.rxdrcmr = (long *)&DRCMRRXFFRBR,
	.txdrcmr = (long *)&DRCMRTXFFTHR,
	.txdma = 0,
	.rxdma = 0,
	.txdma_addr = NULL,
	.rxdma_addr = NULL,
	.port	= {
		.type		= PORT_PXA,
		.iotype		= UPIO_MEM,
		.membase	= (void *)&FFUART,
		.mapbase	= __PREG(FFUART),
		.irq		= IRQ_FFUART,
		.uartclk	= 921600 * 16,
		.fifosize	= 64,
		.ops		= &serial_pxa_pops,
		.line		= 0,
	},
#ifdef CONFIG_DVFM
    .dvfm_notifier = {
	.name = "pxa2xx-uart",
	.priority = 0,
	.notifier_call = uart_dvfm_notifier,
	},
#endif
  },
#ifndef CONFIG_MACH_WOODSTOCK
  {	/* BTUART */
	.name	= "BTUART",
#ifdef CONFIG_PXA3xx
	.cken   = CKEN_BTUART,
#else
	.cken	= CKEN7_BTUART,
#endif
	.rxdrcmr = (long *)&DRCMRRXBTRBR,
	.txdrcmr = (long *)&DRCMRTXBTTHR,
	.txdma = 0,
	.rxdma = 0,
	.txdma_addr = NULL,
	.rxdma_addr = NULL,
	.port	= {
		.type		= PORT_PXA,
		.iotype		= UPIO_MEM,
		.membase	= (void *)&BTUART,
		.mapbase	= __PREG(BTUART),
		.irq		= IRQ_BTUART,
		.uartclk	= 921600 * 16,
		.fifosize	= 64,
		.ops		= &serial_pxa_pops,
		.line		= 1,
	},
#ifdef CONFIG_DVFM
    .dvfm_notifier = {
	.name = "pxa2xx-uart",
	.priority = 0,
	.notifier_call = uart_dvfm_notifier,
	},
#endif
  }, {	/* STUART */
	.name	= "STUART",
#ifdef CONFIG_PXA3xx
	.cken   = CKEN_STUART,
#else
	.cken	= CKEN5_STUART,
#endif
	.rxdrcmr = (long *)&DRCMRRXSTRBR,
	.txdrcmr = (long *)&DRCMRTXSTTHR,
	.txdma = 0,
	.rxdma = 0,
	.txdma_addr = NULL,
	.rxdma_addr = NULL,
	.port	= {
		.type		= PORT_PXA,
		.iotype		= UPIO_MEM,
		.membase	= (void *)&STUART,
		.mapbase	= __PREG(STUART),
		.irq		= IRQ_STUART,
		.uartclk	= 921600 * 16,
		.fifosize	= 64,
		.ops		= &serial_pxa_pops,
		.line		= 2,
	},
#ifdef CONFIG_DVFM
    .dvfm_notifier = {
	.name = "pxa2xx-uart",
	.priority = 0,
	.notifier_call = uart_dvfm_notifier,
	},
#endif
  },
#ifndef CONFIG_PXA3xx 
  {  /* HWUART */
	.name	= "HWUART",
	.cken	= CKEN4_HWUART,
	.port = {
		.type		= PORT_PXA,
		.iotype		= UPIO_MEM,
		.membase	= (void *)&HWUART,
		.mapbase	= __PREG(HWUART),
		.irq		= IRQ_HWUART,
		.uartclk	= 921600 * 16,
		.fifosize	= 64,
		.ops		= &serial_pxa_pops,
		.line		= 3,
	},
  }
#endif
#endif /*0*/
};

static struct uart_driver serial_pxa_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "PXA serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= ARRAY_SIZE(serial_pxa_ports),
	.cons		= PXA_CONSOLE,
};

#ifdef CONFIG_PM
static int serial_pxa_suspend(struct platform_device *dev, pm_message_t state)
{
        struct uart_pxa_port *sport = platform_get_drvdata(dev);
	
	// printk(KERN_INFO "%s: enter\n", __FUNCTION__);
#ifdef CONFIG_SERIAL_PXA_DMA
	if (sport && (sport->ier & UART_IER_DMA)) {
		int length = 0,sent = 0;
		unsigned long flags;

		local_irq_save(flags);
		sport->tx_stop = 1;
		sport->rx_stop = 1;
		sport->data_len = 0;
		if (DCSR(sport->txdma) & DCSR_RUN) {
			DCSR(sport->txdma) &= ~DCSR_RUN;
			length = DCMD(sport->txdma) & 0x1FFF;
			sent = DSADR(sport->txdma) - 
				sport->txdma_addr_phys;
			memcpy(sport->buf_save, sport->txdma_addr
				 + sent, length);
			sport->data_len = length;

		}

		if (DCSR(sport->rxdma) & DCSR_RUN)
			DCSR(sport->rxdma) &= ~DCSR_RUN;
		pxa_uart_receive_dma(sport->rxdma, sport);

		local_irq_restore(flags);
	}
#endif
	if (sport) {
		sport->power_mode = POWER_PRE_SUSPEND;
               	uart_suspend_port(&serial_pxa_reg, &sport->port);
		sport->power_mode = POWER_SUSPEND;
	}
        return 0;
}

static int serial_pxa_resume(struct platform_device *dev)
{
        struct uart_pxa_port *sport = platform_get_drvdata(dev);
	unsigned int quot;
	unsigned long flags;

        if (sport) {

#ifdef CONFIG_PXA_MWB_12
		extern void pxa3xx_enable_pxa_mwb_bt(void);
        	if ( IRQ_BTUART == sport->port.irq) 
			pxa3xx_enable_pxa_mwb_bt();
#endif

		sport->power_mode = POWER_PRE_RESUME;
                uart_resume_port(&serial_pxa_reg, &sport->port);
		sport->power_mode = POWER_RUN;

#ifdef CONFIG_DVFM
		if (is_switch_clk()) {
			if (ACSR & 0x04000000) {
				/* if the CPU is in D0CS mode */
				sport->port.uartclk = 729600*16;
                        } else
				sport->port.uartclk = 921600*16;
			/* adjust uart baudrate */
			if (sport->baud) {
				quot = uart_get_divisor(&sport->port, sport->baud);
				serial_out(sport, UART_LCR, sport->lcr | UART_LCR_DLAB);
				serial_out(sport, UART_DLL, quot & 0xff);
				serial_out(sport, UART_DLM, quot >> 8);
				serial_out(sport, UART_LCR, sport->lcr);
			}
		}
#endif

		local_irq_save(flags);
		sport->tx_stop = 0;
		sport->rx_stop = 0;

#ifdef CONFIG_SERIAL_PXA_DMA
		if (sport->ier & UART_IER_DMA) {
			if (sport->data_len > 0) {
				memcpy( sport->txdma_addr, sport->buf_save,
					sport->data_len);
				pxa_uart_transmit_dma_start(sport, 
					sport->data_len);
			} else
				tasklet_schedule(&sport->tklet);

			pxa_uart_receive_dma_start(sport);
		}
#endif
		local_irq_restore(flags);
	}

	// printk(KERN_INFO "%s: resume\n", __FUNCTION__);
        return 0;
}
#else
#define serial_pxa_suspend	NULL
#define serial_pxa_resume	NULL
#endif

/* In old stepping of Monahans-P/L, UART clock should be changed when
 * CPU enters D0CS mode.
 */
static int is_switch_clk(void)
{
	unsigned int	cpuid;
	/* read CPU ID */
	__asm__ (
		"mrc p15, 0, %0, c0, c0, 0\n"
		: "=r" (cpuid)
	);
	/* It's not xscale chip. */
	if ((cpuid & 0xFFFF0000) != 0x69050000) 
		return 0;
	/* It's MH-P Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006420) 
		return 1;
	/* It's MH-P Bx */
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		/* MH-P B0/B1 */
		if ((cpuid & 0x0F) <= 5)
			return 1;
		else
			return 0;
	}
	/* It's MH-L Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		/* MH-L A0 */
		if ((cpuid & 0x0F) == 0)
			return 1;
		else
			return 0;
	}
	return 0;
}

#ifdef CONFIG_DVFM
static int
uart_dvfm_notifier(unsigned int cmd, void* client_data, void *info)
{
	struct pxa3xx_fv_notifier_info *notifier_info = (struct pxa3xx_fv_notifier_info*)info;
	struct uart_pxa_port *up = (struct uart_pxa_port*)client_data;
	unsigned int quot;
	unsigned int flag = 0;
	
	switch (cmd) {
	case FV_NOTIFIER_QUERY_SET :
		/* entry/exit D0CS mode */
		if (notifier_info->cur.d0cs != notifier_info->next.d0cs) {
			if ((serial_pxa_tx_empty((struct uart_port *)up)
					!= TIOCSER_TEMT) || (up->inuse
					&& (up->baud >= 57600))) {
				if (is_switch_clk())
					return -EAGAIN;
				else if (up->baud >= 921600)
					return -EAGAIN;
			}
		}
		break;

	case FV_NOTIFIER_PRE_SET :
		if (!is_switch_clk())
			break;
		if (notifier_info->cur.d0cs == 0 && notifier_info->next.d0cs == 1){
			up->port.uartclk = 729600*16;
			flag = 1;
		}else if (notifier_info->cur.d0cs == 1 && notifier_info->next.d0cs == 0){
			up->port.uartclk = 921600*16;
			flag = 1;
		}else
			flag = 0;
		
		if (flag){
			if (up->baud == 0)
				break;
			
			/* adjust uart baudrate */
			quot = uart_get_divisor(&up->port, up->baud);
			serial_out(up, UART_LCR, up->lcr | UART_LCR_DLAB);/* set DLAB */
			serial_out(up, UART_DLL, quot & 0xff);	  	/* LS of divisor */
			serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */
			serial_out(up, UART_LCR, up->lcr);		/* reset DLAB */
		}
		break;

	case FV_NOTIFIER_POST_SET :
		if (!is_switch_clk())
			break;
		if (notifier_info->cur.d0cs == 1 && notifier_info->next.d0cs == 0){
			wake_up(&up->delay_wait);
		}
		break;
	}

	return 0;
}

#endif

static int serial_pxa_probe(struct platform_device *dev)
{
	serial_pxa_ports[dev->id].port.dev = &dev->dev;
	uart_add_one_port(&serial_pxa_reg, &serial_pxa_ports[dev->id].port);
	platform_set_drvdata(dev, &serial_pxa_ports[dev->id]);
#ifdef CONFIG_PM
	serial_pxa_ports[dev->id].power_mode = POWER_RUN;
#endif

#ifdef CONFIG_DVFM
	serial_pxa_ports[dev->id].dvfm_notifier.client_data = &serial_pxa_ports[dev->id];
	pxa3xx_fv_register_notifier(&serial_pxa_ports[dev->id].dvfm_notifier);
	init_waitqueue_head(&serial_pxa_ports[dev->id].delay_wait);
#endif

	return 0;
}

static int serial_pxa_remove(struct platform_device *dev)
{
	struct uart_pxa_port *sport = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (sport)
		uart_remove_one_port(&serial_pxa_reg, &sport->port);
	
#ifdef CONFIG_DVFM
	pxa3xx_fv_unregister_notifier(&sport->dvfm_notifier);
#endif

	return 0;
}

static struct platform_driver serial_pxa_driver = {
        .probe          = serial_pxa_probe,
        .remove         = serial_pxa_remove,

	.suspend	= serial_pxa_suspend,
	.resume		= serial_pxa_resume,
	.driver		= {
	        .name	= "pxa2xx-uart",
	},
};

int __init serial_pxa_init(void)
{
	int ret;

#ifndef CONFIG_SONOS_DIAGS
	if ((sys_mdp.mdp_flags & MDP_FLAG_CONSOLE_ENABLE) == 0)
		return 0;
#endif

	ret = uart_register_driver(&serial_pxa_reg);
	if (ret != 0)
		return ret;

	ret = platform_driver_register(&serial_pxa_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_pxa_reg);

	return ret;
}

void __exit serial_pxa_exit(void)
{
	platform_driver_unregister(&serial_pxa_driver);
	uart_unregister_driver(&serial_pxa_reg);
}

module_init(serial_pxa_init);
module_exit(serial_pxa_exit);

MODULE_LICENSE("GPL");

