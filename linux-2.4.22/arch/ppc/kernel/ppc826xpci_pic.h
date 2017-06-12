#ifndef _PPC_KERNEL_PPC826XPCI_H
#define _PPC_KERNEL_PPC826XPCI_H

#include <linux/irq.h>

extern struct hw_interrupt_type ppc826xpci_pic;

void m826xpci_pic_init(void);
void m826xpci_do_IRQ(struct pt_regs *regs, int cpu);
int m826xpci_get_irq(struct pt_regs *regs);

#endif /* _PPC_KERNEL_PPC826XPCI_H */
