/*
 * arch/ppc/kernel/m826x_pci.c
 * 
 * Author: Xu Zhang
 *                  xuzhang@motorola.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/mpc10x.h>
#include <asm/machdep.h>

#include <asm/immap_8260.h>
#include <asm/cpm_8260.h>

#include "pci.h"

#ifdef CONFIG_SONOS
#include <asm/system.h>
#include <asm/irq.h>
int wembley_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin) {
    if (idsel==16) return SIU_INT_IRQ1; else return -1;
}
#endif /*SONOS*/

void m826x_pcibios_after_init(void)
{
    volatile immap_t *immap=(immap_t *)IMAP_ADDR;
    immap->im_pci.pci_esr = cpu_to_le32(0x00001FFF);
    mb();
    immap->im_pci.pci_emr = cpu_to_le32(0x00000FFF);
    printk("Did post-init PCI EMR/ESR reset\n");
}

void __init m826x_find_bridges() {
	struct pci_controller *hose;

#ifdef CONFIG_SONOS
	unsigned short s;
        volatile immap_t *immap=(immap_t *)IMAP_ADDR;
        /* BT the following is sort of taken from the U-Boot PCI init code.
         * I moved it here because I was having trouble fitting it with
         * everything else in our 64k OTP
         */
        immap->im_siu_conf.sc_ppc_alrh = 0x61207893;
        immap->im_siu_conf.sc_ppc_acr = 0x6;
        immap->im_memctl.memc_pcimsk0 = 0xC0000000;
        immap->im_memctl.memc_pcibr0 = 0x80000001;

	immap->im_pci.pci_emr = cpu_to_le32(0x00000FF7);
	immap->im_pci.pci_esr = cpu_to_le32(0x00001FFF);
        immap->im_pci.pci_gcr = cpu_to_le32(0x1);
	mdelay(100);
        immap->im_pci.pci_potar0 = cpu_to_le32(0x80000000 >> 12);
        immap->im_pci.pci_pobar0 = cpu_to_le32(0x80000000 >> 12);
        immap->im_pci.pci_pocmr0 = cpu_to_le32(0xA00E0000);

        immap->im_pci.pci_potar1 = cpu_to_le32(0xA0000000 >> 12);
        immap->im_pci.pci_pobar1 = cpu_to_le32(0xA0000000 >> 12);
        immap->im_pci.pci_pocmr1 = cpu_to_le32(0x800E0000);

	immap->im_pci.pci_potar2 = cpu_to_le32(0xF4000000 >> 12);
        immap->im_pci.pci_pobar2 = cpu_to_le32(0xF4000000 >> 12);
        immap->im_pci.pci_pocmr2 = cpu_to_le32(0xC00FC000);

        immap->im_pci.pci_pitar0 = 0;
        immap->im_pci.pci_pibar0 = 0;
        immap->im_pci.pci_picmr0 = cpu_to_le32(0xA00E0000);

        immap->im_siu_conf.sc_ppc_alrh = 0x01236745;
        immap->im_siu_conf.sc_ppc_acr = 0x3;
	
	immap->im_pci.pci_cfg_addr = cpu_to_le32(0x80000008);
	mb();
	((unsigned char *)(&(immap->im_pci.pci_cfg_data)))[3]= 0x06;
	mb();
	immap->im_pci.pci_cfg_addr = cpu_to_le32(0x80000004);
	mb();
	s=((unsigned short *)(&(immap->im_pci.pci_cfg_data)))[0];
	mb();
	s|=cpu_to_le16(0x06);
	immap->im_pci.pci_cfg_addr = cpu_to_le32(0x80000004);
	mb();
	((unsigned short *)(&(immap->im_pci.pci_cfg_data)))[0]= s;
	mb();
	immap->im_pci.pci_cfg_addr = cpu_to_le32(0x8000000C);
	mb();
	((unsigned char *)(&(immap->im_pci.pci_cfg_data)))[0]=0x08;
	mb();
	immap->im_pci.pci_cfg_addr = cpu_to_le32(0x8000000C);
	mb();
	((unsigned char *)(&(immap->im_pci.pci_cfg_data)))[1]=0xF8;
	mb();
#endif /*CONFIG_SONOS*/

	hose = pcibios_alloc_controller(); /* the controller has been attached to the hose_head link */
	if (!hose)
		return;
	
	hose->first_busno = 0;
	hose->last_busno = 0xff;
	/* MPC826x PCI bridge has its own address mapping, and has not EUMB. */
	if (mpc10x_bridge_init(hose, MPC10X_MEM_MAP_826x, MPC10X_MEM_MAP_826x, 0) == 0) {

                /* Do early winbond init, then scan PCI bus */
                hose->last_busno = pciauto_bus_scan(hose, hose->first_busno);

                ppc_md.pcibios_fixup = NULL;
                ppc_md.pcibios_fixup_bus = NULL;
                ppc_md.pci_swizzle = common_swizzle;
                ppc_md.pcibios_after_init = m826x_pcibios_after_init;
#ifdef CONFIG_PPC_SONOS
                ppc_md.pci_map_irq = wembley_map_irq;
#else
                ppc_md.pci_map_irq = m826xpci_map_irq;
#endif
        } else {
                if (ppc_md.progress)
                        ppc_md.progress("Bridge init failed", 0x100);
                printk("Host bridge init failed\n");
        }

        return;
}
static spinlock_t idma_lock;

void pci_idma1_init(void) {
	volatile immap_t *immap;
	volatile sdma8260_t *sdmap;
	volatile idma_t *ip;
	uint dp_addr;

	immap = (immap_t *)IMAP_ADDR;
	sdmap = (sdma8260_t *)&immap->im_sdma;

	spin_lock_init(&idma_lock);

	dp_addr = m8260_cpm_dpalloc(sizeof(idma_t), 64); /* the parameter ram */
	*((ushort *)&immap->im_dprambase[PROFF_IDMA1_BASE]) = dp_addr;

	ip = (idma_t *)&immap->im_dprambase[dp_addr];
	memset((void *)ip, 0, sizeof(idma_t));

	dp_addr = m8260_cpm_dpalloc(64, 64); /* the buffer */
	ip->ibase = dp_addr;
	dp_addr = m8260_cpm_dpalloc(sizeof(idma_bd_t), 16); /* the BD */
	ip->dpr_buf = dp_addr;
	ip->ss_max = 32;
	ip->dts = 32;

	sdmap->sdma_idmr1 = 0;
	sdmap->sdma_idsr1 = 0xff;
}

void pci_idma1_read(u8 *dst, u8 *src, int len, int unit, int sinc) {
	unsigned long flags;
	volatile immap_t *immap;
	volatile sdma8260_t *sdmap;
	volatile idma_t *ip;
	volatile idma_bd_t *bdp;

	immap = (immap_t *)IMAP_ADDR;
	sdmap = (sdma8260_t *)&immap->im_sdma;
	ip = (idma_t *)&immap->im_dprambase[*((ushort *)&immap->im_dprambase[PROFF_IDMA1_BASE])];
	bdp = (idma_bd_t *)&immap->im_dprambase[ip->ibase];

	spin_lock(&idma_lock);

	if (sinc)
		ip->dcm = IDMA_DCM_DMA_WRAP_64 | IDMA_DCM_SINC | IDMA_DCM_DINC | IDMA_DCM_SD_MEM2MEM;
	else
		ip->dcm = IDMA_DCM_DMA_WRAP_64 |                 IDMA_DCM_DINC | IDMA_DCM_SD_MEM2MEM;
	ip->ibdptr = ip->ibase;
	ip->sts = unit;
	ip->istate = 0;

	bdp->dst = (uint)virt_to_phys(dst);
	bdp->src = (uint)src;
	bdp->datlen = len;
	bdp->sc = IDMA_BD_VALID | IDMA_BD_WRAP | IDMA_BD_LAST | IDMA_BD_DGBL | IDMA_BD_DBO_BE | IDMA_BD_SBO_BE | IDMA_BD_SDTB;

	save_flags(flags);
	cli();

	while(immap->im_cpm.cp_cpcr & CPM_CR_FLG);
	immap->im_cpm.cp_cpcr = mk_cr_cmd(CPM_CR_IDMA1_PAGE, CPM_CR_IDMA1_SBLOCK, 0, CPM_CR_START_IDMA) | CPM_CR_FLG;
	while (immap->im_cpm.cp_cpcr & CPM_CR_FLG);
	while(bdp->sc & IDMA_BD_VALID);
#if 0 /* zhx */
	eieio();
#endif

	restore_flags(flags);

	spin_unlock(&idma_lock);

	return;
}

#if 0 /* zhx */
int pci_idma1_readb(volatile unsigned char *addr) {
	u8 val;
	unsigned long pa = iopa((unsigned long)addr);

	pci_idma1_read((u8 *)&val, (u8 *)pa, sizeof(val), sizeof(val), 0);
	return val;
}

int pci_idma1_readw(volatile unsigned short *addr) {
	u16 val;
	unsigned long pa = iopa((unsigned long)addr);

	pci_idma1_read((u8 *)&val, (u8 *)pa, sizeof(val), sizeof(val), 0);
	return swab16(val);
}

unsigned pci_idma1_readl(volatile unsigned *addr) {
	u32 val;
	unsigned long pa = iopa((unsigned long)addr);

	pci_idma1_read((u8 *)&val, (u8 *)pa, sizeof(val), sizeof(val), 0);
	return swab32(val);
}

void *pci_idma1_memcpy_fromio(unsigned dst, unsigned src, size_t count) {
	pci_idma1_read((u8 *)dst, (u8 *)src, count, 32, 1);
	return (void *)dst;
}
#endif

