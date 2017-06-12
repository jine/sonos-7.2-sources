#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/immap_8260.h>
#include <asm/mpc8260.h>
#include <asm/pci-bridge.h>
#include "ppc826xpci_pic.h"

/* The MPC826x PCI chips have PCI bridge without PCI interrupt controller.
 * So, the user has to design the PCI interrupt controller for itself. The
 * different hardware implementation causes the different programing model.
 * Here is one for MPC8266ADS board. 
 */

static volatile int *pciir;	/* PCI Interrut Register */
static volatile int *pciimr;	/* PCI Interrupt Mask Register */
#if 0
static	u_char	irq_to_pcireg[] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0
};

static	u_char	irq_to_pcibit[] = {
	31, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 25, 26, 27, 28, 29, 30,
	29, 30, 16, 17, 18, 19, 20, 21,
	22, 23, 24, 25, 26, 27, 28, 31,
	 0,  1,  2,  3,  4,  5,  6,  7,
	 8,  9, 10, 11, 12, 13, 14, 15,
	15, 14, 13, 12, 11, 10,  9,  8,
	 7,  6,  5,  4,  3,  2,  1,  0
};
#endif
#ifdef CONFIG_PCI /* For PCI Interrupt Controller */
#if defined CONFIG_MPC8272ADS || defined CONFIG_ADS8266
#define PCI_SLOTS_NUM	3 /* the number for PCI slots */
static char pci_irq_table[][4] =
/*
 *	PCI IDSEL/INTPIN->INTLINE
 *	 A   B   C   D
 */
{
	{64, 65, 66, 67},	/* IDSEL 0 - PCI slot 0 */
	{67, 64, 65, 66},	/* IDSEL 1 - PCI slot 1 */
	{66, 67, 64, 65},	/* IDSEL 2 - PCI slot 2 */
};
#endif

void m826xpci_pic_init() {
#ifdef CONFIG_ADS8266
	pciir = (int *)0xff000000;
	pciimr = (int *)0xff000004;

	*pciir = 0; /* Clear all PCI interrupts. */
	*pciimr = 0xfff00000; /* Mask all PCI interrupts. */
#elif defined CONFIG_MPC8272ADS
	pciir = (int *)0xf8200000;
	pciimr = (int *)0xf8200004;

	*pciir = 0; /* Clear all PCI interrupts. */
	*pciimr = 0xfff00000; /* Mask all PCI interrupts. */
#else
	return;	
#endif
}

static void m826xpci_mask_irq(unsigned int irq_nr) {
	int i, j;

	for (i = 0; i < PCI_SLOTS_NUM; i++) {
		j = 4 * i + (irq_nr - NR_SIU_INTS + i) % 4;
		*pciimr |= (1 << (31 - j));
#if 0 /* zhx */
		eieio();
#endif
	}
}

static void m826xpci_unmask_irq(unsigned int irq_nr) {
	int i, j;

	for (i = 0; i < PCI_SLOTS_NUM; i++) {
		j = 4 * i + (irq_nr - NR_SIU_INTS + i) % 4;
		*pciimr &= ~(1 << (31 - j));
#if 0 /* zhx */
		eieio();
#endif
	}
}

static void m826xpci_mask_and_ack(unsigned int irq_nr) {
	int i, j;

	for (i = 0; i < PCI_SLOTS_NUM; i++) {
		j = 4 * i + (irq_nr - NR_SIU_INTS + i) % 4;
		*pciir &= ~(1 << (31 - j));
		*pciimr |= (1 << (31 - j));
#if 0 /* zhx */
		eieio();
#endif
	}
}

static void m826xpci_end_irq(unsigned int irq_nr) {
	int i, j;

	if (!(irq_desc[irq_nr - NR_SIU_INTS].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		for (i = 0; i < PCI_SLOTS_NUM; i++) {
			j = 4 * i + (irq_nr - NR_SIU_INTS + i) % 4;
			*pciimr &= ~(1 << (31 - j));
#if 0 /* zhx */
			eieio();
#endif
		}
}

struct hw_interrupt_type ppc826xpci_pic = {
	" 826x PCI ",
	NULL,
	NULL,
	m826xpci_unmask_irq,
	m826xpci_mask_irq,
	m826xpci_mask_and_ack,
	m826xpci_end_irq,
	0
};

int m826xpci_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin) {
#if defined CONFIG_MPC8272ADS || defined CONFIG_ADS8266
        const long min_idsel = 0x16, max_idsel = 0x18, irqs_per_slot = 4;
#endif
        return PCI_IRQ_TABLE_LOOKUP;
}

int m826xpci_get_irq(struct pt_regs *regs) {
	int i, bit;
	int irq;
	char *t = (char *)pci_irq_table;

	for (i = 0; i < 12; i++) {
		bit = 1 << (31 - i);
		if (bit & *pciir)
			break;
	}
	if (i == 12)
		return -1;

	irq = (int)t[i];

	return irq;
}

/* MPC826x PCI interrupt controller interrupt. */
void mpc826xpci_intr(int irq, void * dev, struct pt_regs * regs) {
	int i, bit;
	//int irq;
	char *t = (char *)pci_irq_table;
	int match = 0;
#if 0 /* zhx */
	unsigned long flags;

	save_flags(flags);
	cli();
#endif
	for (;;) {
		for (i = 0; i < 12; i++) {
			bit = 1 << (31 - i);
			match = (bit & *pciir) && (bit & ~(*pciimr));
#if 0 /* zhx */
			eieio();
			sync(); /* zhx */
#endif
			if (match)
				break;
		}
		if (i == 12)
			irq = -1;
		else
			irq = (int)t[i];

#if 0 /* zhx */
printk("irq = %d\n", irq);
#endif
		if (irq >= 0)
			ppc_irq_dispatch_handler(regs, irq);
		else
			break;

#if 0 /* Has been done in ack_irq().  -- zhx */
		for (i = 0; i < PCI_SLOTS_NUM; i++) {
			j = 4 * i + (irq - NR_SIU_INTS + i) % 4;
			*pciir &= ~(1 << (31 - j));
		}
#endif
	}
#if 0 /* zhx */
	restore_flags(flags); /* zhx */
#endif
}

#endif /* For PCI Interrupt Controller */

