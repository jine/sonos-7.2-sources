/* This is the single file included by all MPC8260 build options.
 * Since there are many different boards and no standard configuration,
 * we have a unique include file for each.  Rather than change every
 * file that has to include MPC8260 configuration, they all include
 * this one and the configuration switching is done here.
 */
#ifdef __KERNEL__
#ifndef __CONFIG_8260_DEFS
#define __CONFIG_8260_DEFS

#include <linux/config.h>

#ifdef CONFIG_8260

#ifdef CONFIG_EST8260
#include <platforms/est8260.h>
#endif

#ifdef CONFIG_MPC8272ADS
#include <platforms/mpc8272ads.h>
#endif

#ifdef CONFIG_PPC_SONOS
#include <platforms/wembley.h>
#endif

/* I don't yet have the ISA or PCI stuff done....no 8260 with
 * such thing.....
 */
#ifdef CONFIG_PCI
#if defined CONFIG_8272 || defined CONFIG_ADS8266
extern unsigned long isa_io_base; 
extern unsigned long isa_mem_base; 
extern unsigned long pci_dram_offset; 
#define _IO_BASE        isa_io_base
#define _ISA_MEM_BASE   isa_mem_base
#define PCI_DRAM_OFFSET pci_dram_offset
#else
#define _IO_BASE        0
#define _ISA_MEM_BASE   0
#define PCI_DRAM_OFFSET 0
#endif /* 8266 and 8272 */
#else /*XXX BT no PCI */
#define _IO_BASE	0
#define _ISA_MEM_BASE	0
#define PCI_DRAM_OFFSET 0
#endif /* pci */

#ifndef __ASSEMBLY__
/* The "residual" data board information structure the boot loader
 * hands to us.
 */
extern unsigned char __res[];
#endif /* __ASSEMBLY__ */

#include <asm/ptrace.h> //??

#ifdef CONFIG_PCI
#if defined CONFIG_8272 || defined CONFIG_ADS8266
extern int request_8xxirq(unsigned int irq,
		       void (*handler)(int, void *, struct pt_regs *),
		       unsigned long flags, 
		       const char *device,
		       void *dev_id);


#endif /*8266 and 8272 */
#endif /* pci */

#define request_8xxirq request_irq

#endif /* CONFIG_8260 */
#endif /* !__CONFIG_8260_DEFS */
#endif /* __KERNEL__ */
