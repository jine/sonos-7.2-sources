/* $Id: sh-sci.c,v 1.11 2006/05/22 01:21:11 tober Exp $
 *
 *  linux/drivers/char/sh-sci.c
 *
 *  SuperH on-chip serial module support.  (SCI with no FIFO / with FIFO)
 *  Copyright (C) 1999, 2000  Niibe Yutaka
 *  Copyright (C) 2000  Sugioka Toshinobu
 *  Modified to support multiple serial ports. Stuart Menefy (May 2000).
 *
 * TTY code is based on sx.c (Specialix SX driver) by:
 *
 *   (C) 1998 R.E.Wolff@BitWizard.nl
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_SH_KGDB_CONSOLE)
#include <linux/console.h>
#endif

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <linux/generic_serial.h>

#include "mdp.h"
extern struct manufacturing_data_page sys_mdp;

#if 0
#include <asm/sh_bios.h>
#endif

#include "sh-sci.h"

#ifdef CONFIG_SH_KGDB
#include <asm/kgdb.h>

int kgdb_sci_setup(void);
static int kgdb_get_char(struct sci_port *port);
static void kgdb_put_char(struct sci_port *port, char c);
static void kgdb_handle_error(struct sci_port *port);
static struct sci_port *kgdb_sci_port;

#ifdef CONFIG_SH_KGDB_CONSOLE
static struct console kgdbcons;
void __init kgdb_console_init(void);
#endif /* CONFIG_SH_KGDB_CONSOLE */

#endif /* CONFIG_SH_KGDB */

#ifdef CONFIG_SERIAL_CONSOLE
static struct console sercons;
static struct sci_port* sercons_port=0;
static int sercons_baud;
#ifdef CONFIG_MAGIC_SYSRQ
#include <linux/sysrq.h>
static int break_pressed;
#endif /* CONFIG_MAGIC_SYSRQ */
#endif /* CONFIG_SERIAL_CONSOLE */

/* Function prototypes */
static void sci_init_pins_sci(struct sci_port* port, unsigned int cflag);
#ifndef SCI_ONLY
static void sci_init_pins_scif(struct sci_port* port, unsigned int cflag);
#if defined(__sh3__)
static void sci_init_pins_irda(struct sci_port* port, unsigned int cflag);
#endif
#endif
static void sci_disable_tx_interrupts(void *ptr);
static void sci_enable_tx_interrupts(void *ptr);
static void sci_disable_rx_interrupts(void *ptr);
static void sci_enable_rx_interrupts(void *ptr);
static int  sci_get_CD(void *ptr);
static void sci_shutdown_port(void *ptr);
static int sci_set_real_termios(void *ptr);
static void sci_hungup(void *ptr);
static void sci_close(void *ptr);
static int sci_chars_in_buffer(void *ptr);
static int sci_request_irq(struct sci_port *port);
static void sci_free_irq(struct sci_port *port);
static int sci_init_drivers(void);

static struct tty_driver sci_driver, sci_callout_driver;

static struct sci_port sci_ports[SCI_NPORTS] = SCI_INIT;
static struct tty_struct *sci_table[SCI_NPORTS] = { NULL, };
static struct termios *sci_termios[SCI_NPORTS];
static struct termios *sci_termios_locked[SCI_NPORTS];

static int sci_refcount;
static int sci_debug = 0;

#ifdef MODULE
MODULE_PARM(sci_debug, "i");
#endif

#define dprintk(x...) do { if (sci_debug) printk(x); } while(0)

#ifdef CONFIG_SERIAL_CONSOLE
static void put_char(struct sci_port *port, char c)
{
	unsigned long flags;
	unsigned short status;

	save_and_cli(flags);

	do
		status = sci_in(port, SCxSR);
	while (!(status & SCxSR_TDxE(port)));
	
	sci_out(port, SCxTDR, c);
	sci_in(port, SCxSR);            /* Dummy read */
	sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));

	restore_flags(flags);
}
#endif

#if 0

static void handle_error(struct sci_port *port)
{				/* Clear error flags */
	sci_out(port, SCxSR, SCxSR_ERROR_CLEAR(port));
}

static int get_char(struct sci_port *port)
{
	unsigned long flags;
	unsigned short status;
	int c;

	save_and_cli(flags);
        do {
		status = sci_in(port, SCxSR);
		if (status & SCxSR_ERRORS(port)) {
			handle_error(port);
			continue;
		}
	} while (!(status & SCxSR_RDxF(port)));
	c = sci_in(port, SCxRDR);
	sci_in(port, SCxSR);            /* Dummy read */
	sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
	restore_flags(flags);

	return c;
}

/* Taken from sh-stub.c of GDB 4.18 */
static const char hexchars[] = "0123456789abcdef";

static __inline__ char highhex(int  x)
{
	return hexchars[(x >> 4) & 0xf];
}

static __inline__ char lowhex(int  x)
{
	return hexchars[x & 0xf];
}

#endif /* CONFIG_SH_STANDARD_BIOS || CONFIG_SH_KGDB */

/*
 * Send the packet in buffer.  The host gets one chance to read it.
 * This routine does not wait for a positive acknowledge.
 */

#ifdef CONFIG_SERIAL_CONSOLE
static void put_string(struct sci_port *port, const char *buffer, int count)
{
	int i;
	const unsigned char *p = buffer;

#if 0
	int checksum;
	int usegdb=0;

#ifdef CONFIG_SH_STANDARD_BIOS
    	/* This call only does a trap the first time it is
	 * called, and so is safe to do here unconditionally
	 */
	usegdb |= sh_bios_in_gdb_mode();
#endif
#ifdef CONFIG_SH_KGDB
	usegdb |= (kgdb_in_gdb_mode && (port == kgdb_sci_port));
#endif

	if (usegdb) {
	    /*  $<packet info>#<checksum>. */
	    do {
		unsigned char c;
		put_char(port, '$');
		put_char(port, 'O'); /* 'O'utput to console */
		checksum = 'O';

		for (i=0; i<count; i++) { /* Don't use run length encoding */
			int h, l;

			c = *p++;
			h = highhex(c);
			l = lowhex(c);
			put_char(port, h);
			put_char(port, l);
			checksum += h + l;
		}
		put_char(port, '#');
		put_char(port, highhex(checksum));
		put_char(port, lowhex(checksum));
	    } while  (get_char(port) != '+');
	} else
#endif /* CONFIG_SH_STANDARD_BIOS || CONFIG_SH_KGDB */
	for (i=0; i<count; i++) {
		if (*p == 10)
			put_char(port, '\r');
		put_char(port, *p++);
	}
}
#endif /* CONFIG_SERIAL_CONSOLE */


#ifdef CONFIG_SH_KGDB

/* Is the SCI ready, ie is there a char waiting? */
static int kgdb_is_char_ready(struct sci_port *port)
{
        unsigned short status = sci_in(port, SCxSR);

        if (status & (SCxSR_ERRORS(port) | SCxSR_BRK(port)))
                kgdb_handle_error(port);

        return (status & SCxSR_RDxF(port));
}

/* Write a char */
static void kgdb_put_char(struct sci_port *port, char c)
{
        unsigned short status;

        do
                status = sci_in(port, SCxSR);
        while (!(status & SCxSR_TDxE(port)));

        sci_out(port, SCxTDR, c);
        sci_in(port, SCxSR);    /* Dummy read */
        sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));
}

/* Get a char if there is one, else ret -1 */
static int kgdb_get_char(struct sci_port *port)
{
        int c;

        if (kgdb_is_char_ready(port) == 0)
                c = -1;
        else {
                c = sci_in(port, SCxRDR);
                sci_in(port, SCxSR);    /* Dummy read */
                sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
        }

        return c;
}

/* Called from kgdbstub.c to get a character, i.e. is blocking */
static int kgdb_sci_getchar(void)
{
        volatile int c;

        /* Keep trying to read a character, this could be neater */
        while ((c = kgdb_get_char(kgdb_sci_port)) < 0);

        return c;
}

/* Called from kgdbstub.c to put a character, just a wrapper */
static void kgdb_sci_putchar(int c)
{

        kgdb_put_char(kgdb_sci_port, c);
}

/* Clear any errors on the SCI */
static void kgdb_handle_error(struct sci_port *port)
{
        sci_out(port, SCxSR, SCxSR_ERROR_CLEAR(port));  /* Clear error flags */
}

/* Breakpoint if there's a break sent on the serial port */
static void kgdb_break_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
        struct sci_port *port = ptr;
        unsigned short status = sci_in(port, SCxSR);

        if (status & SCxSR_BRK(port)) {

                /* Break into the debugger if a break is detected */
                BREAKPOINT();

                /* Clear */
                sci_out(port, SCxSR, SCxSR_BREAK_CLEAR(port));
                return;
        }
}

#endif /* CONFIG_SH_KGDB */

static struct real_driver sci_real_driver = {
	sci_disable_tx_interrupts,
	sci_enable_tx_interrupts,
	sci_disable_rx_interrupts,
	sci_enable_rx_interrupts,
	sci_get_CD,
	sci_shutdown_port,
	sci_set_real_termios,
	sci_chars_in_buffer,
	sci_close,
	sci_hungup,
	NULL
};

#if defined(SCI_ONLY) || defined(SCI_AND_SCIF)
static void sci_init_pins_sci(struct sci_port* port, unsigned int cflag)
{
}
#endif

#if defined(SCIF_ONLY) || defined(SCI_AND_SCIF)
#if defined(__sh3__)
/* For SH7707, SH7709, SH7709A, SH7729 */
static void sci_init_pins_scif(struct sci_port* port, unsigned int cflag)
{
	unsigned int fcr_val = 0;

	{
		unsigned short data;

		/* We need to set SCPCR to enable RTS/CTS */
		data = ctrl_inw(SCPCR);
		/* Clear out SCP7MD1,0, SCP6MD1,0, SCP4MD1,0*/
		ctrl_outw(data&0x0fcf, SCPCR);
	}
	if (cflag & CRTSCTS)
		fcr_val |= SCFCR_MCE;
	else {
		unsigned short data;

		/* We need to set SCPCR to enable RTS/CTS */
		data = ctrl_inw(SCPCR);
		/* Clear out SCP7MD1,0, SCP4MD1,0,
		   Set SCP6MD1,0 = {01} (output)  */
		ctrl_outw((data&0x0fcf)|0x1000, SCPCR);

		data = ctrl_inb(SCPDR);
		/* Set /RTS2 (bit6) = 0 */
		ctrl_outb(data&0xbf, SCPDR);
	}
	sci_out(port, SCFCR, fcr_val);
}

static void sci_init_pins_irda(struct sci_port* port, unsigned int cflag)
{
	unsigned int fcr_val = 0;

	if (cflag & CRTSCTS)
		fcr_val |= SCFCR_MCE;

	sci_out(port, SCFCR, fcr_val);
}

#else

/* For SH7750 */
static void sci_init_pins_scif(struct sci_port* port, unsigned int cflag)
{
	unsigned int fcr_val = 0;

	if (cflag & CRTSCTS) {
		fcr_val |= SCFCR_MCE;
	} else {
		ctrl_outw(0x0080, SCSPTR2); /* Set RTS = 1 */
	}
	sci_out(port, SCFCR, fcr_val);
}

#endif
#endif /* SCIF_ONLY || SCI_AND_SCIF */

static void sci_setsignals(struct sci_port *port, int dtr, int rts)
{
	/* This routine is used for seting signals of: DTR, DCD, CTS/RTS */
	/* We use SCIF's hardware for CTS/RTS, so don't need any for that. */
	/* If you have signals for DTR and DCD, please implement here. */
	;
}

static int sci_getsignals(struct sci_port *port)
{
	/* This routine is used for geting signals of: DTR, DCD, DSR, RI,
	   and CTS/RTS */

	return TIOCM_DTR|TIOCM_RTS|TIOCM_DSR;
/*
	(((o_stat & OP_DTR)?TIOCM_DTR:0) |
	 ((o_stat & OP_RTS)?TIOCM_RTS:0) |
	 ((i_stat & IP_CTS)?TIOCM_CTS:0) |
	 ((i_stat & IP_DCD)?TIOCM_CAR:0) |
	 ((i_stat & IP_DSR)?TIOCM_DSR:0) |
	 ((i_stat & IP_RI) ?TIOCM_RNG:0)
*/
}

static void sci_set_baud(struct sci_port *port, int baud)
{
	int t;

	switch (baud) {
	case 0:
		t = -1;
		break;
	case 2400:
		t = BPS_2400;
		break;
	case 4800:
		t = BPS_4800;
		break;
	case 9600:
		t = BPS_9600;
		break;
	case 19200:
		t = BPS_19200;
		break;
	case 38400:
		t = BPS_38400;
		break;
	case 57600:
		t = BPS_57600;
		break;
	default:
		printk(KERN_INFO "sci: unsupported baud rate: %d, using 115200 instead.\n", baud);
	case 115200:
		t = BPS_115200;
		break;
	}

	if (t > 0) {
		sci_setsignals (port, 1, -1);
		if(t >= 256) {
			sci_out(port, SCSMR, (sci_in(port, SCSMR) & ~3) | 1);
			t >>= 2;
		} else {
			sci_out(port, SCSMR, sci_in(port, SCSMR) & ~3);
		}
		sci_out(port, SCBRR, t);
		udelay((1000000+(baud-1)) / baud); /* Wait one bit interval */
	} else {
		sci_setsignals (port, 0, -1);
	}
}

static void sci_set_termios_cflag(struct sci_port *port, int cflag, int baud)
{
	unsigned int status;
	unsigned int smr_val;

	do
		status = sci_in(port, SCxSR);
	while (!(status & SCxSR_TEND(port)));

	sci_out(port, SCSCR, 0x00);	/* TE=0, RE=0, CKE1=0 */

	if (port->type == PORT_SCIF) {
		sci_out(port, SCFCR, SCFCR_RFRST | SCFCR_TFRST);
	}

	smr_val = sci_in(port, SCSMR) & 3;
	if ((cflag & CSIZE) == CS7)
		smr_val |= 0x40;
	if (cflag & PARENB)
		smr_val |= 0x20;
	if (cflag & PARODD)
		smr_val |= 0x30;
	if (cflag & CSTOPB)
		smr_val |= 0x08;
	sci_out(port, SCSMR, smr_val);
	sci_set_baud(port, baud);

	port->init_pins(port, cflag);
	sci_out(port, SCSCR, SCSCR_INIT(port));
}

static int sci_active=0;

void sci_reset(void)
{
    if (!sci_active) return;
    sci_set_termios_cflag(sercons_port,sercons_port->old_cflag,sercons_baud);
    sci_enable_rx_interrupts(sercons_port);
    sci_enable_tx_interrupts(sercons_port);
}

static int sci_set_real_termios(void *ptr)
{
	struct sci_port *port = ptr;

	if (port->old_cflag != port->gs.tty->termios->c_cflag) {
		port->old_cflag = port->gs.tty->termios->c_cflag;
		sci_set_termios_cflag(port, port->old_cflag, port->gs.baud);
		sci_enable_rx_interrupts(port);
	}

	/* Tell line discipline whether we will do input cooking */
	if (I_OTHER(port->gs.tty))
		clear_bit(TTY_HW_COOK_IN, &port->gs.tty->flags);
	else
		set_bit(TTY_HW_COOK_IN, &port->gs.tty->flags);

/* Tell line discipline whether we will do output cooking.
 * If OPOST is set and no other output flags are set then we can do output
 * processing.  Even if only *one* other flag in the O_OTHER group is set
 * we do cooking in software.
 */
	if (O_OPOST(port->gs.tty) && !O_OTHER(port->gs.tty))
		set_bit(TTY_HW_COOK_OUT, &port->gs.tty->flags);
	else
		clear_bit(TTY_HW_COOK_OUT, &port->gs.tty->flags);

	return 0;
}

/* ********************************************************************** *
 *                   the interrupt related routines                       *
 * ********************************************************************** */

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */
static inline void sci_sched_event(struct sci_port *port, int event)
{
	port->event |= 1 << event;
	queue_task(&port->tqueue, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
}

static void sci_transmit_chars(struct sci_port *port)
{
	int count, i;
	int txroom;
	unsigned long flags;
	unsigned short status;
	unsigned short ctrl;
	unsigned char c;

	status = sci_in(port, SCxSR);
	if (!(status & SCxSR_TDxE(port))) {
		save_and_cli(flags);
		ctrl = sci_in(port, SCSCR);
		if (port->gs.xmit_cnt == 0) {
			ctrl &= ~SCI_CTRL_FLAGS_TIE;
			port->gs.flags &= ~GS_TX_INTEN;
		} else
			ctrl |= SCI_CTRL_FLAGS_TIE;
		sci_out(port, SCSCR, ctrl);
		restore_flags(flags);
		return;
	}

	while (1) {
		count = port->gs.xmit_cnt;
		if (port->type == PORT_SCIF) {
			txroom = 16 - (sci_in(port, SCFDR)>>8);
		} else {
			txroom = (sci_in(port, SCxSR) & SCI_TDRE)?1:0;
		}
		if (count > txroom)
			count = txroom;

		/* Don't copy past the end of the source buffer */
		if (count > SERIAL_XMIT_SIZE - port->gs.xmit_tail)
                	count = SERIAL_XMIT_SIZE - port->gs.xmit_tail;

		/* If for one reason or another, we can't copy more data, we're done! */
		if (count == 0)
			break;

		for (i=0; i<count; i++) {
			c = port->gs.xmit_buf[port->gs.xmit_tail + i];
			sci_out(port, SCxTDR, c);
		}
		sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));

		port->icount.tx += count;

		/* Update the kernel buffer end */
		port->gs.xmit_tail = (port->gs.xmit_tail + count) & (SERIAL_XMIT_SIZE-1);

		/* This one last. (this is essential)
		   It would allow others to start putting more data into the buffer! */
		port->gs.xmit_cnt -= count;
	}

	if (port->gs.xmit_cnt <= port->gs.wakeup_chars)
		sci_sched_event(port, SCI_EVENT_WRITE_WAKEUP);

	save_and_cli(flags);
	ctrl = sci_in(port, SCSCR);
	if (port->gs.xmit_cnt == 0) {
		ctrl &= ~SCI_CTRL_FLAGS_TIE;
		port->gs.flags &= ~GS_TX_INTEN;
	} else {
		if (port->type == PORT_SCIF) {
			sci_in(port, SCxSR); /* Dummy read */
			sci_out(port, SCxSR, SCxSR_TDxE_CLEAR(port));
		}
		ctrl |= SCI_CTRL_FLAGS_TIE;
	}
	sci_out(port, SCSCR, ctrl);
	restore_flags(flags);
}

/* On SH3, SCIF may read end-of-break as a space->mark char */
#define STEPFN(c)  ({int __c=(c); (((__c-1)|(__c)) == -1); })

static inline void sci_receive_chars(struct sci_port *port,
				     struct pt_regs *regs)
{
	int i, count;
	struct tty_struct *tty;
	int copied=0;
	unsigned short status;

	status = sci_in(port, SCxSR);
	if (!(status & SCxSR_RDxF(port)))
		return;

	tty = port->gs.tty;
	while (1) {
		if (port->type == PORT_SCIF) {
			count = sci_in(port, SCFDR)&0x001f;
		} else {
			count = (sci_in(port, SCxSR)&SCxSR_RDxF(port))?1:0;
		}

		/* Don't copy more bytes than there is room for in the buffer */
		if (tty->flip.count + count > TTY_FLIPBUF_SIZE)
			count = TTY_FLIPBUF_SIZE - tty->flip.count;

		/* If for any reason we can't copy more data, we're done! */
		if (count == 0)
			break;

		if (port->type == PORT_SCI) {
			tty->flip.char_buf_ptr[0] = sci_in(port, SCxRDR);
			tty->flip.flag_buf_ptr[0] = TTY_NORMAL;
		} else {
			for (i=0; i<count; i++) {
				char c = sci_in(port, SCxRDR);
				status = sci_in(port, SCxSR);
#if defined(__SH3__)
				/* Skip "chars" during break */
				if (port->break_flag) {
					if ((c == 0) &&
					    (status & SCxSR_FER(port))) {
						count--; i--;
						continue;
					}
					/* Nonzero => end-of-break */
					dprintk("scif: debounce<%02x>\n", c);
					port->break_flag = 0;
					if (STEPFN(c)) {
						count--; i--;
						continue;
					}
				}
#endif /* __SH3__ */
#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
				if (break_pressed && (port == sercons_port)) {
					if (c != 0 &&
					    time_before(jiffies,
							break_pressed + HZ*5)) {
						handle_sysrq(c, regs,
							     NULL, NULL);
						break_pressed = 0;
						count--; i--;
						continue;
					} else if (c != 0) {
						break_pressed = 0;
					}
				}
#endif /* CONFIG_SERIAL_CONSOLE && CONFIG_MAGIC_SYSRQ */

				/* Store data and status */
				tty->flip.char_buf_ptr[i] = c;
				if (status&SCxSR_FER(port)) {
					tty->flip.flag_buf_ptr[i] = TTY_FRAME;
					dprintk("sci: frame error\n");
				} else if (status&SCxSR_PER(port)) {
					tty->flip.flag_buf_ptr[i] = TTY_PARITY;
					dprintk("sci: parity error\n");
				} else {
					tty->flip.flag_buf_ptr[i] = TTY_NORMAL;
				}
			}
		}

		sci_in(port, SCxSR); /* dummy read */
		sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));

		/* Update the kernel buffer end */
		tty->flip.count += count;
		tty->flip.char_buf_ptr += count;
		tty->flip.flag_buf_ptr += count;

		copied += count;
		port->icount.rx += count;
	}

	if (copied)
		/* Tell the rest of the system the news. New characters! */
		tty_flip_buffer_push(tty);
	else {
		sci_in(port, SCxSR); /* dummy read */
		sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
	}
}

static inline int sci_handle_errors(struct sci_port *port)
{
	int copied = 0;
	unsigned short status = sci_in(port, SCxSR);
	struct tty_struct *tty = port->gs.tty;

	if (status&SCxSR_ORER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		/* overrun error */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
		dprintk("sci: overrun error\n");
	}

	if (status&SCxSR_FER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		if (sci_rxd_in(port) == 0) {
			/* Notify of BREAK */
			copied++;
			*tty->flip.flag_buf_ptr++ = TTY_BREAK;
			dprintk("sci: BREAK detected\n");
		}
		else {
			/* frame error */
			copied++;
			*tty->flip.flag_buf_ptr++ = TTY_FRAME;
			dprintk("sci: frame error\n");
		}
	}

	if (status&SCxSR_PER(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
		/* parity error */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_PARITY;
		dprintk("sci: parity error\n");
	}

	if (copied) {
		tty->flip.count += copied;
		tty_flip_buffer_push(tty);
	}

	return copied;
}

static inline int sci_handle_breaks(struct sci_port *port)
{
	int copied = 0;
	unsigned short status = sci_in(port, SCxSR);
	struct tty_struct *tty = port->gs.tty;

	if (status&SCxSR_BRK(port) && tty->flip.count<TTY_FLIPBUF_SIZE) {
#if defined(__SH3__)
		/* Debounce break */
		if (port->break_flag)
			goto break_continue;
		port->break_flag = 1;
#endif
#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
		if (port == sercons_port) {
			if (break_pressed == 0) {
				break_pressed = jiffies;
				dprintk("sci: implied sysrq\n");
				goto break_continue;
			}
			/* Double break implies a real break */
			break_pressed = 0;
		}
#endif
		/* Notify of BREAK */
		copied++;
		*tty->flip.flag_buf_ptr++ = TTY_BREAK;
		dprintk("sci: BREAK detected\n");
	}
 break_continue:

#if defined(CONFIG_CPU_SUBTYPE_SH7750) || defined(CONFIG_CPU_SUBTYPE_ST40STB1)
	/* XXX: Handle SCIF overrun error */
	if (port->type == PORT_SCIF && (sci_in(port, SCLSR) & SCIF_ORER) != 0) {
		sci_out(port, SCLSR, 0);
		if(tty->flip.count<TTY_FLIPBUF_SIZE) {
			copied++;
			*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
			dprintk("sci: overrun error\n");
		}
	}
#endif

	if (copied) {
		tty->flip.count += copied;
		tty_flip_buffer_push(tty);
	}

	return copied;
}

static void sci_rx_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct sci_port *port = ptr;

	if (port->gs.flags & GS_ACTIVE)
		if (!(port->gs.flags & SCI_RX_THROTTLE)) {
			sci_receive_chars(port, regs);
			return;
		}
	sci_disable_rx_interrupts(port);
}

static void sci_tx_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct sci_port *port = ptr;

	if (port->gs.flags & GS_ACTIVE)
		sci_transmit_chars(port);
	else {
		sci_disable_tx_interrupts(port);
	}
}

static void sci_er_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct sci_port *port = ptr;

	/* Handle errors */
	if (port->type == PORT_SCI) {
		if(sci_handle_errors(port)) {
			/* discard character in rx buffer */
			sci_in(port, SCxSR);
			sci_out(port, SCxSR, SCxSR_RDxF_CLEAR(port));
		}
	}
	else
		sci_rx_interrupt(irq, ptr, regs);
		
	sci_out(port, SCxSR, SCxSR_ERROR_CLEAR(port));

	/* Kick the transmission */
	sci_tx_interrupt(irq, ptr, regs);
}

static void sci_br_interrupt(int irq, void *ptr, struct pt_regs *regs)
{
	struct sci_port *port = ptr;

	/* Handle BREAKs */
	sci_handle_breaks(port);
	sci_out(port, SCxSR, SCxSR_BREAK_CLEAR(port));
}

static void do_softint(void *private_)
{
	struct sci_port *port = (struct sci_port *) private_;
	struct tty_struct	*tty;
	
	tty = port->gs.tty;
	if (!tty)
		return;

	if (test_and_clear_bit(SCI_EVENT_WRITE_WAKEUP, &port->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

/* ********************************************************************** *
 *                Here are the routines that actually                     *
 *              interface with the generic_serial driver                  *
 * ********************************************************************** */

static void sci_disable_tx_interrupts(void *ptr)
{
	struct sci_port *port = ptr;
	unsigned long flags;
	unsigned short ctrl;

	/* Clear TIE (Transmit Interrupt Enable) bit in SCSCR */
	save_and_cli(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl &= ~SCI_CTRL_FLAGS_TIE;
	sci_out(port, SCSCR, ctrl);
	restore_flags(flags);
}

static void sci_enable_tx_interrupts(void *ptr)
{
	struct sci_port *port = ptr; 

	disable_irq(port->irqs[SCIx_TXI_IRQ]);
	sci_transmit_chars(port);
	enable_irq(port->irqs[SCIx_TXI_IRQ]);
}

static void sci_disable_rx_interrupts(void * ptr)
{
	struct sci_port *port = ptr;
	unsigned long flags;
	unsigned short ctrl;

	/* Clear RIE (Receive Interrupt Enable) bit in SCSCR */
	save_and_cli(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl &= ~SCI_CTRL_FLAGS_RIE;
	sci_out(port, SCSCR, ctrl);
	restore_flags(flags);
}

static void sci_enable_rx_interrupts(void * ptr)
{
	struct sci_port *port = ptr;
	unsigned long flags;
	unsigned short ctrl;

	/* Set RIE (Receive Interrupt Enable) bit in SCSCR */
	save_and_cli(flags);
	ctrl = sci_in(port, SCSCR);
	ctrl |= SCI_CTRL_FLAGS_RIE;
	sci_out(port, SCSCR, ctrl);
	restore_flags(flags);
}

static int sci_get_CD(void * ptr)
{
	/* If you have signal for CD (Carrier Detect), please change here. */
	return 1;
}

static int sci_chars_in_buffer(void * ptr)
{
	struct sci_port *port = ptr;

	if (port->type == PORT_SCIF) {
		return (sci_in(port, SCFDR) >> 8) + ((sci_in(port, SCxSR) & SCxSR_TEND(port))? 0: 1);
	} else {
		return (sci_in(port, SCxSR) & SCxSR_TEND(port))? 0: 1;
	}
}

static void sci_shutdown_port(void * ptr)
{
	struct sci_port *port = ptr; 

	port->gs.flags &= ~ GS_ACTIVE;
	if (port->gs.tty && port->gs.tty->termios->c_cflag & HUPCL)
		sci_setsignals(port, 0, 0);
	sci_free_irq(port);
}

/* ********************************************************************** *
 *                Here are the routines that actually                     *
 *               interface with the rest of the system                    *
 * ********************************************************************** */

static int sci_open(struct tty_struct * tty, struct file * filp)
{
	struct sci_port *port;
	int retval, line;

	line = MINOR(tty->device) - SCI_MINOR_START;

	if ((line < 0) || (line >= SCI_NPORTS))
		return -ENODEV;

	port = &sci_ports[line];

	tty->driver_data = port;
	port->gs.tty = tty;
	port->gs.count++;

	port->event = 0;
	port->tqueue.routine = do_softint;
	port->tqueue.data = port;
	port->break_flag = 0;

	if (port->gs.count == 1) {
		MOD_INC_USE_COUNT;

		retval = sci_request_irq(port);
		if (retval) {
			goto failed_1;
		}
	}

	/*
	 * Start up serial port
	 */
	retval = gs_init_port(&port->gs);
	if (retval) {
		goto failed_2;
	}

	port->gs.flags |= GS_ACTIVE;
	sci_setsignals(port, 1,1);

	retval = gs_block_til_ready(port, filp);

	if (retval) {
		goto failed_2;
	}

	if ((port->gs.count == 1) && (port->gs.flags & ASYNC_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
			*tty->termios = port->gs.normal_termios;
		else 
			*tty->termios = port->gs.callout_termios;
		sci_set_real_termios(port);
	}

#ifdef CONFIG_SERIAL_CONSOLE
	if (sercons.cflag && sercons.index == line) {
		tty->termios->c_cflag = sercons.cflag;
		port->gs.baud = sercons_baud;
		sercons.cflag = 0;
		sci_set_real_termios(port);
	}
#endif

#ifdef CONFIG_SH_KGDB_CONSOLE
        if (kgdbcons.cflag && kgdbcons.index == line) {
                tty->termios->c_cflag = kgdbcons.cflag;
                port->gs.baud = kgdb_baud;
                sercons.cflag = 0;
                sci_set_real_termios(port);
        }
#elif CONFIG_SH_KGDB
	/* Even for non-console, may defer to kgdb */
	if (port == kgdb_sci_port && kgdb_in_gdb_mode) {
		tty->termios->c_cflag = kgdb_cflag;
		port->gs.baud = kgdb_baud;
		sercons.cflag = 0;
		sci_set_real_termios(port);
	}
#endif /* CONFIG_SH_KGDB */

	sci_enable_rx_interrupts(port);

	port->gs.session = current->session;
	port->gs.pgrp = current->pgrp;

	return 0;

failed_2:
	sci_free_irq(port);
failed_1:
	MOD_DEC_USE_COUNT;
	port->gs.count--;
	return retval;
}

static void sci_hungup(void *ptr)
{
	MOD_DEC_USE_COUNT;
}

static void sci_close(void *ptr)
{
	MOD_DEC_USE_COUNT;
}

static int sci_ioctl(struct tty_struct * tty, struct file * filp, 
                     unsigned int cmd, unsigned long arg)
{
	int rc;
	struct sci_port *port = tty->driver_data;
	int ival;

	rc = 0;
	switch (cmd) {
	case TIOCGSOFTCAR:
		rc = put_user(((tty->termios->c_cflag & CLOCAL) ? 1 : 0),
		              (unsigned int *) arg);
		break;
	case TIOCSSOFTCAR:
		if ((rc = get_user(ival, (unsigned int *) arg)) == 0)
			tty->termios->c_cflag =
				(tty->termios->c_cflag & ~CLOCAL) |
				(ival ? CLOCAL : 0);
		break;
	case TIOCGSERIAL:
		if ((rc = verify_area(VERIFY_WRITE, (void *) arg,
		                      sizeof(struct serial_struct))) == 0)
			rc = gs_getserial(&port->gs, (struct serial_struct *) arg);
		break;
	case TIOCSSERIAL:
		if ((rc = verify_area(VERIFY_READ, (void *) arg,
		                      sizeof(struct serial_struct))) == 0)
			rc = gs_setserial(&port->gs,
					  (struct serial_struct *) arg);
		break;
	case TIOCMGET:
		ival = sci_getsignals(port);
		rc = put_user(ival, (unsigned int *) arg);
		break;
	case TIOCMBIS:
		if ((rc = get_user(ival, (unsigned int *) arg)) == 0)
			sci_setsignals(port, ((ival & TIOCM_DTR) ? 1 : -1),
			                     ((ival & TIOCM_RTS) ? 1 : -1));
		break;
	case TIOCMBIC:
		if ((rc = get_user(ival, (unsigned int *) arg)) == 0)
			sci_setsignals(port, ((ival & TIOCM_DTR) ? 0 : -1),
			                     ((ival & TIOCM_RTS) ? 0 : -1));
		break;
	case TIOCMSET:
		if ((rc = get_user(ival, (unsigned int *)arg)) == 0)
			sci_setsignals(port, ((ival & TIOCM_DTR) ? 1 : 0),
			                     ((ival & TIOCM_RTS) ? 1 : 0));
		break;

	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static void sci_throttle(struct tty_struct * tty)
{
	struct sci_port *port = (struct sci_port *)tty->driver_data;

	/* If the port is using any type of input flow
	 * control then throttle the port.
	 */
	if ((tty->termios->c_cflag & CRTSCTS) || (I_IXOFF(tty)) )
		port->gs.flags |= SCI_RX_THROTTLE;
}

static void sci_unthrottle(struct tty_struct * tty)
{
	struct sci_port *port = (struct sci_port *)tty->driver_data;

	/* Always unthrottle even if flow control is not enabled on
	 * this port in case we disabled flow control while the port
	 * was throttled
	 */
	port->gs.flags &= ~SCI_RX_THROTTLE;
	sci_enable_rx_interrupts(port);
	return;
}

#ifdef CONFIG_PROC_FS
static int sci_read_proc(char *page, char **start, off_t off, int count,
			 int *eof, void *data)
{
	int i;
	struct sci_port *port;
	int len = 0;
	
        len += sprintf(page, "sciinfo:0.1\n");
	for (i = 0; i < SCI_NPORTS && len < 4000; i++) {
		port = &sci_ports[i];
		len += sprintf(page+len, "%d: uart:%s address: %08x", i,
			       (port->type == PORT_SCI) ? "SCI" : "SCIF",
			       port->base);
		len += sprintf(page+len, " baud:%d", port->gs.baud);
		len += sprintf(page+len, " tx:%d rx:%d",
			       port->icount.tx, port->icount.rx);

		if (port->icount.frame)
			len += sprintf(page+len, " fe:%d", port->icount.frame);
		if (port->icount.parity)
			len += sprintf(page+len, " pe:%d", port->icount.parity);
		if (port->icount.brk)
			len += sprintf(page+len, " brk:%d", port->icount.brk);
		if (port->icount.overrun)
			len += sprintf(page+len, " oe:%d", port->icount.overrun);
		len += sprintf(page+len, "\n");
	}
	return len;
}
#endif

/* ********************************************************************** *
 *                    Here are the initialization routines.               *
 * ********************************************************************** */

static int sci_init_drivers(void)
{
	int error;
	struct sci_port *port;

	memset(&sci_driver, 0, sizeof(sci_driver));
	sci_driver.magic = TTY_DRIVER_MAGIC;
	sci_driver.driver_name = "sci";
#ifdef CONFIG_DEVFS_FS
	sci_driver.name = "tts/%d";
#else
	sci_driver.name = "ttySC";
#endif
	sci_driver.major = SCI_MAJOR;
	sci_driver.minor_start = SCI_MINOR_START;
	sci_driver.num = SCI_NPORTS;
	sci_driver.type = TTY_DRIVER_TYPE_SERIAL;
	sci_driver.subtype = SERIAL_TYPE_NORMAL;
	sci_driver.init_termios = tty_std_termios;
	sci_driver.init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL | CRTSCTS;
	sci_driver.flags = TTY_DRIVER_REAL_RAW;
	sci_driver.refcount = &sci_refcount;
	sci_driver.table = sci_table;
	sci_driver.termios = sci_termios;
	sci_driver.termios_locked = sci_termios_locked;

	sci_driver.open	= sci_open;
	sci_driver.close = gs_close;
	sci_driver.write = gs_write;
	sci_driver.put_char = gs_put_char;
	sci_driver.flush_chars = gs_flush_chars;
	sci_driver.write_room = gs_write_room;
	sci_driver.chars_in_buffer = gs_chars_in_buffer;
	sci_driver.flush_buffer = gs_flush_buffer;
	sci_driver.ioctl = sci_ioctl;
	sci_driver.throttle = sci_throttle;
	sci_driver.unthrottle = sci_unthrottle;
	sci_driver.set_termios = gs_set_termios;
	sci_driver.stop = gs_stop;
	sci_driver.start = gs_start;
	sci_driver.hangup = gs_hangup;
#ifdef CONFIG_PROC_FS
	sci_driver.read_proc = sci_read_proc;
#endif

	sci_callout_driver = sci_driver;
#ifdef CONFIG_DEVFS_FS
	sci_callout_driver.name = "cusc/%d";
#else
	sci_callout_driver.name = "cusc";
#endif
	sci_callout_driver.major = SCI_MAJOR+1;
	sci_callout_driver.subtype = SERIAL_TYPE_CALLOUT;
	sci_callout_driver.read_proc = NULL;

	if ((error = tty_register_driver(&sci_driver))) {
		printk(KERN_ERR "sci: Couldn't register SCI driver, error = %d\n",
		       error);
		return 1;
	}
	if ((error = tty_register_driver(&sci_callout_driver))) {
		tty_unregister_driver(&sci_driver);
		printk(KERN_ERR "sci: Couldn't register SCI callout driver, error = %d\n",
		       error);
		return 1;
	}

	for (port = &sci_ports[0]; port < &sci_ports[SCI_NPORTS]; port++) {
		port->gs.callout_termios = sci_callout_driver.init_termios;
		port->gs.normal_termios	= sci_driver.init_termios;
		port->gs.magic = SCI_MAGIC;
		port->gs.close_delay = HZ/2;
		port->gs.closing_wait = 30 * HZ;
		port->gs.rd = &sci_real_driver;
		init_waitqueue_head(&port->gs.open_wait);
		init_waitqueue_head(&port->gs.close_wait);
		port->old_cflag = 0;
		port->icount.cts = port->icount.dsr = 
			port->icount.rng = port->icount.dcd = 0;
		port->icount.rx = port->icount.tx = 0;
		port->icount.frame = port->icount.parity = 0;
		port->icount.overrun = port->icount.brk = 0;
	}

	return 0;
}

static int sci_request_irq(struct sci_port *port)
{
	int i;
	void (*handlers[4])(int irq, void *ptr, struct pt_regs *regs) = {
		sci_er_interrupt, sci_rx_interrupt, sci_tx_interrupt,
		sci_br_interrupt,
	};

	for (i=0; i<4; i++) {
		if (!port->irqs[i]) continue;
		if (request_irq(port->irqs[i], handlers[i], SA_INTERRUPT,
				"sci", port)) {
			printk(KERN_ERR "sci: Cannot allocate irq.\n");
			return -ENODEV;
		}
	}
	return 0;
}

static void sci_free_irq(struct sci_port *port)
{
	int i;

	for (i=0; i<4; i++) {
		if (!port->irqs[i]) continue;
		free_irq(port->irqs[i], port);
	}
}

static char banner[] __initdata =
	KERN_INFO "SuperH SCI(F) driver initialized\n";

int __init sci_init(void)
{
	struct sci_port *port;
	int j;

#ifndef CONFIG_SONOS_DIAGS
	if ((sys_mdp.mdp_flags&2)==0) return 0;
#endif

	sci_active=1;

	printk("%s", banner);

	for (j=0; j<SCI_NPORTS; j++) {
		port = &sci_ports[j];
		printk(KERN_INFO "ttySC%d at 0x%08x is a %s\n", j, port->base,
		       (port->type == PORT_SCI) ? "SCI" : "SCIF");
	}

	sci_init_drivers();

#if 0
	sh_bios_gdb_detach();
#endif
	return 0;		/* Return -EIO when not detected */
}

module_init(sci_init);

#ifdef MODULE
#undef func_enter
#undef func_exit

void cleanup_module(void)
{
	tty_unregister_driver(&sci_driver);
	tty_unregister_driver(&sci_callout_driver);
}

#include "generic_serial.c"
#endif

#ifdef CONFIG_SERIAL_CONSOLE
/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 */
static void serial_console_write(struct console *co, const char *s,
				 unsigned count)
{
	put_string(sercons_port, s, count);
}

static kdev_t serial_console_device(struct console *c)
{
	return MKDEV(SCI_MAJOR, SCI_MINOR_START + c->index);
}

/*
 *	Setup initial baud/bits/parity. We do two things here:
 *	- construct a cflag setting for the first rs_open()
 *	- initialize the serial port
 *	Return non-zero if we didn't find a serial port.
 */
static int __init serial_console_setup(struct console *co, char *options)
{
	int	baud = 9600;
	int	bits = 8;
	int	parity = 'n';
	int	cflag = CREAD | HUPCL | CLOCAL;
	char	*s;

	sercons_port = &sci_ports[co->index];

	if (options) {
		baud = simple_strtoul(options, NULL, 10);
		s = options;
		while(*s >= '0' && *s <= '9')
			s++;
		if (*s) parity = *s++;
		if (*s) bits   = *s - '0';
	}

	/*
	 *	Now construct a cflag setting.
	 */
	switch (baud) {
		case 19200:
			cflag |= B19200;
			break;
		case 38400:
			cflag |= B38400;
			break;
		case 57600:
			cflag |= B57600;
			break;
		case 115200:
			cflag |= B115200;
			break;
		case 9600:
		default:
			cflag |= B9600;
			baud = 9600;
			break;
	}
	switch (bits) {
		case 7:
			cflag |= CS7;
			break;
		default:
		case 8:
			cflag |= CS8;
			break;
	}
	switch (parity) {
		case 'o': case 'O':
			cflag |= PARODD;
			break;
		case 'e': case 'E':
			cflag |= PARENB;
			break;
	}

#ifdef CONFIG_SH_KGDB
	if (kgdb_in_gdb_mode && sercons_port == kgdb_sci_port) {
		co->cflag = kgdb_cflag;
		sercons_baud = kgdb_baud;
		sercons_port->old_cflag = cflag;
	}
	else
#endif /* CONFIG_SH_KGDB */
	{
		co->cflag = cflag;
		sercons_baud = baud;

		sci_set_termios_cflag(sercons_port, cflag, baud);
		sercons_port->old_cflag = cflag;
	}

	return 0;
}

static struct console sercons = {
	name:		"ttySC",
	write:		serial_console_write,
	device:		serial_console_device,
	setup:		serial_console_setup,
	flags:		CON_PRINTBUFFER,
	index:		-1,
};

/*
 *	Register console.
 */

#ifdef CONFIG_SH_EARLY_PRINTK
extern void sh_console_unregister (void);
#endif

void __init sci_console_init(void)
{
#ifndef CONFIG_SONOS_DIAGS
	if ((sys_mdp.mdp_flags&2)==0) return;
#endif
	register_console(&sercons);
#ifdef CONFIG_SH_EARLY_PRINTK
	/* Now that the real console is available, unregister the one we
	 * used while first booting.
	 */
	sh_console_unregister();
#endif
}
#endif /* CONFIG_SERIAL_CONSOLE */


#ifdef CONFIG_SH_KGDB

/* Initialise the KGDB serial port */
int kgdb_sci_setup(void)
{
	int cflag = CREAD | HUPCL | CLOCAL;

	if ((kgdb_portnum < 0) || (kgdb_portnum >= SCI_NPORTS))
		return -1;

        kgdb_sci_port = &sci_ports[kgdb_portnum];

	switch (kgdb_baud) {
        case 115200:
                cflag |= B115200;
                break;
	case 57600:
                cflag |= B57600;
                break;
        case 38400:
                cflag |= B38400;
                break;
        case 19200:
                cflag |= B19200;
                break;
        case 9600:
        default:
                cflag |= B9600;
                kgdb_baud = 9600;
                break;
        }

	switch (kgdb_bits) {
        case '7':
                cflag |= CS7;
                break;
        default:
        case '8':
                cflag |= CS8;
                break;
        }

        switch (kgdb_parity) {
        case 'O':
                cflag |= PARODD;
                break;
        case 'E':
                cflag |= PARENB;
                break;
        }

        kgdb_cflag = cflag;
        sci_set_termios_cflag(kgdb_sci_port, kgdb_cflag, kgdb_baud);

        /* Set up the interrupt for BREAK from GDB */
	/* Commented out for now since it may not be possible yet...
	   request_irq(kgdb_sci_port->irqs[0], kgdb_break_interrupt,
	               SA_INTERRUPT, "sci", kgdb_sci_port);
	   sci_enable_rx_interrupts(kgdb_sci_port);
	*/

	/* Setup complete: initialize function pointers */
	kgdb_getchar = kgdb_sci_getchar;
	kgdb_putchar = kgdb_sci_putchar;

        return 0;
}

#ifdef CONFIG_SH_KGDB_CONSOLE

/* Create a console device */
static kdev_t kgdb_console_device(struct console *c)
{
        return MKDEV(SCI_MAJOR, SCI_MINOR_START + c->index);
}

/* Set up the KGDB console */
static int __init kgdb_console_setup(struct console *co, char *options)
{
        /* NB we ignore 'options' because we've already done the setup */
        co->cflag = kgdb_cflag;

        return 0;
}

/* Register the KGDB console so we get messages (d'oh!) */
void __init kgdb_console_init(void)
{
        register_console(&kgdbcons);
}

/* The console structure for KGDB */
static struct console kgdbcons = {
        name:"ttySC",
        write:kgdb_console_write,
        device:kgdb_console_device,
        /*wait_key:serial_console_wait_key,*/
        setup:kgdb_console_setup,
        flags:CON_PRINTBUFFER | CON_ENABLED,
        index:-1,
};

#endif /* CONFIG_SH_KGDB_CONSOLE */

#endif /* CONFIG_SH_KGDB */
