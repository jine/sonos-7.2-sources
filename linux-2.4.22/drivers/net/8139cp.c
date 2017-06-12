/* 8139cp.c: A Linux PCI Ethernet driver for the RealTek 8139C+ chips. */
/*
	Copyright 2001,2002 Jeff Garzik <jgarzik@mandrakesoft.com>

	Copyright (C) 2001, 2002 David S. Miller (davem@redhat.com) [tg3.c]
	Copyright (C) 2000, 2001 David S. Miller (davem@redhat.com) [sungem.c]
	Copyright 2001 Manfred Spraul				    [natsemi.c]
	Copyright 1999-2001 by Donald Becker.			    [natsemi.c]
       	Written 1997-2001 by Donald Becker.			    [8139too.c]
	Copyright 1998-2001 by Jes Sorensen, <jes@trained-monkey.org>. [acenic.c]

	This software may be used and distributed according to the terms of
	the GNU General Public License (GPL), incorporated herein by reference.
	Drivers based on or derived from this code fall under the GPL and must
	retain the authorship, copyright and license notice.  This file is not
	a complete program and may only be used when the entire operating
	system is licensed under the GPL.

	See the file COPYING in this distribution for more information.

	Contributors:
	
		Wake-on-LAN support - Felipe Damasio <felipewd@terra.com.br>
		PCI suspend/resume  - Felipe Damasio <felipewd@terra.com.br>
		LinkChg interrupt   - Felipe Damasio <felipewd@terra.com.br>
			
	TODO, in rough priority order:
	* Test Tx checksumming thoroughly
	* dev->tx_timeout
	* Constants (module parms?) for Rx work limit
	* Complete reset on PciErr
	* Consider Rx interrupt mitigation using TimerIntr
	* Implement 8139C+ statistics dump; maybe not...
	  h/w stats can be reset only by software reset
	* Handle netif_rx return value
	* Investigate using skb->priority with h/w VLAN priority
	* Investigate using High Priority Tx Queue with skb->priority
	* Adjust Rx FIFO threshold and Max Rx DMA burst on Rx FIFO error
	* Adjust Tx FIFO threshold and Max Tx DMA burst on Tx FIFO error
	* Implement Tx software interrupt mitigation via
	  Tx descriptor bit
	* The real minimum of CP_MIN_MTU is 4 bytes.  However,
	  for this to be supported, one must(?) turn on packet padding.
	* Support 8169 GMII
	* Support external MII transceivers

 */

#define DRV_NAME		"8139cp"
#define DRV_VERSION		"0.3.0"
#define DRV_RELDATE		"Sep 29, 2002"


#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/rtnetlink.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#include "mdp.h"
extern struct manufacturing_data_page sys_mdp;

#define REDROCKS (sys_mdp.mdp_submodel==MDP_SUBMODEL_REDROCKS)

/* experimental TX checksumming feature enable/disable */
#undef CP_TX_CHECKSUM

/* VLAN tagging feature enable/disable */
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define CP_VLAN_TAG_USED 1
#define CP_VLAN_TX_TAG(tx_desc,vlan_tag_value) \
	do { (tx_desc)->opts2 = (vlan_tag_value); } while (0)
#else
#define CP_VLAN_TAG_USED 0
#define CP_VLAN_TX_TAG(tx_desc,vlan_tag_value) \
	do { (tx_desc)->opts2 = 0; } while (0)
#endif

#define MII_EXTERNAL_PHY		1
#undef  EMULATE_EEPROM
#define RTL8139_PROC_DEBUG      1
#define AUTOPROGRAM_EEPROM		1

#define CP_WATCHDOG_TIMEOUT 	(jiffies + (2 * HZ))

/* These identify the driver base version and may not be removed. */
static char version[] __devinitdata =
KERN_INFO DRV_NAME ": 10/100 PCI Ethernet driver v" DRV_VERSION " (" DRV_RELDATE ")\n";

MODULE_AUTHOR("Jeff Garzik <jgarzik@mandrakesoft.com>");
MODULE_DESCRIPTION("RealTek RTL-8139C+ series 10/100 PCI Ethernet driver");
MODULE_LICENSE("GPL");

static int debug = -1;
MODULE_PARM (debug, "i");
MODULE_PARM_DESC (debug, "8139cp: bitmapped message enable number");

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC.  */
static int multicast_filter_limit = 32;
MODULE_PARM (multicast_filter_limit, "i");
MODULE_PARM_DESC (multicast_filter_limit, "8139cp: maximum number of filtered multicast addresses");

#define PFX			DRV_NAME ": "

#ifndef TRUE
#define FALSE 0
#define TRUE (!FALSE)
#endif

#define CP_DEF_MSG_ENABLE	(NETIF_MSG_DRV		| \
				 NETIF_MSG_PROBE 	| \
				 NETIF_MSG_LINK)
#define CP_NUM_STATS		14	/* struct cp_dma_stats, plus one */
#define CP_STATS_SIZE		64	/* size in bytes of DMA stats block */
#define CP_REGS_SIZE		(0xff + 1)
#define CP_REGS_VER		1		/* version 1 */
#define CP_RX_RING_SIZE		64
#define CP_TX_RING_SIZE		64
#define CP_RING_BYTES		\
		((sizeof(struct cp_desc) * CP_RX_RING_SIZE) +	\
		 (sizeof(struct cp_desc) * CP_TX_RING_SIZE) +	\
		 CP_STATS_SIZE)
#define NEXT_TX(N)		(((N) + 1) & (CP_TX_RING_SIZE - 1))
#define NEXT_RX(N)		(((N) + 1) & (CP_RX_RING_SIZE - 1))
#define TX_BUFFS_AVAIL(CP)					\
	(((CP)->tx_tail <= (CP)->tx_head) ?			\
	  (CP)->tx_tail + (CP_TX_RING_SIZE - 1) - (CP)->tx_head :	\
	  (CP)->tx_tail - (CP)->tx_head - 1)

#define PKT_BUF_SZ		1536	/* Size of each temporary Rx buffer.*/
#define RX_OFFSET		2
#define CP_INTERNAL_PHY		32

/* The following settings are log_2(bytes)-4:  0 == 16 bytes .. 6==1024, 7==end of packet. */
#define RX_FIFO_THRESH		5	/* Rx buffer level before first PCI xfer.  */
#define RX_DMA_BURST		4	/* Maximum PCI burst, '4' is 256 */
#define TX_DMA_BURST		6	/* Maximum PCI burst, '6' is 1024 */
#define TX_EARLY_THRESH		256	/* Early Tx threshold, in bytes */

/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT		(6*HZ)

/* hardware minimum and maximum for a single frame's data payload */
#define CP_MIN_MTU		60	/* TODO: allow lower, but pad */
#define CP_MAX_MTU		4096

enum {
	/* NIC register offsets */
	MAC0		= 0x00,	/* Ethernet hardware address. */
	MAR0		= 0x08,	/* Multicast filter. */
	StatsAddr	= 0x10,	/* 64-bit start addr of 64-byte DMA stats blk */
	TxRingAddr	= 0x20, /* 64-bit start addr of Tx ring */
	HiTxRingAddr	= 0x28, /* 64-bit start addr of high priority Tx ring */
	Cmd		= 0x37, /* Command register */
	IntrMask	= 0x3C, /* Interrupt mask */
	IntrStatus	= 0x3E, /* Interrupt status */
	TxConfig	= 0x40, /* Tx configuration */
	ChipVersion	= 0x43, /* 8-bit chip version, inside TxConfig */
	RxConfig	= 0x44, /* Rx configuration */
	Cfg9346		= 0x50, /* EEPROM select/control; Cfg reg [un]lock */
	Config0		= 0x51,	/* Config0 */
	Config1		= 0x52, /* Config1 */
	MediaStatus	= 0x58, /* Media Status */
	Config3		= 0x59, /* Config3 */
	Config4		= 0x5A, /* Config4 */
	MultiIntr	= 0x5C, /* Multiple interrupt select */
	BasicModeCtrl	= 0x62,	/* MII BMCR */
	BasicModeStatus	= 0x64, /* MII BMSR */
	NWayAdvert	= 0x66, /* MII ADVERTISE */
	NWayLPAR	= 0x68, /* MII LPA */
	NWayExpansion	= 0x6A, /* MII Expansion */
	CSCR		= 0x74,	/* CSCR */
	PARA78		= 0x78, /* PARA78 */
	PARA7c		= 0x7c, /* para7c */
	PHY2_Parm	= 0x82,	/* Phy2_parm_u */
	Config5		= 0xD8,	/* Config5 */
	TxPoll		= 0xD9,	/* Tell chip to check Tx descriptors for work */
	RxMaxSize	= 0xDA, /* Max size of an Rx packet (8169 only) */
	CpCmd		= 0xE0, /* C+ Command register (C+ mode only) */
	IntrMitigate	= 0xE2,	/* rx/tx interrupt mitigation control */
	RxRingAddr	= 0xE4, /* 64-bit start addr of Rx ring */
	TxThresh	= 0xEC, /* Early Tx threshold */
	MIIReg		= 0xFC, /* Function Force Event/MII Register */
	OldRxBufAddr	= 0x30, /* DMA address of Rx ring buffer (C mode) */
	OldTSD0		= 0x10, /* DMA address of first Tx desc (C mode) */

	/* Tx and Rx status descriptors */
	DescOwn		= (1 << 31), /* Descriptor is owned by NIC */
	RingEnd		= (1 << 30), /* End of descriptor ring */
	FirstFrag	= (1 << 29), /* First segment of a packet */
	LastFrag	= (1 << 28), /* Final segment of a packet */
	TxError		= (1 << 23), /* Tx error summary */
	RxError		= (1 << 20), /* Rx error summary */
	IPCS		= (1 << 18), /* Calculate IP checksum */
	UDPCS		= (1 << 17), /* Calculate UDP/IP checksum */
	TCPCS		= (1 << 16), /* Calculate TCP/IP checksum */
	TxVlanTag	= (1 << 17), /* Add VLAN tag */
	RxVlanTagged	= (1 << 16), /* Rx VLAN tag available */
	IPFail		= (1 << 15), /* IP checksum failed */
	UDPFail		= (1 << 14), /* UDP/IP checksum failed */
	TCPFail		= (1 << 13), /* TCP/IP checksum failed */
	NormalTxPoll	= (1 << 6),  /* One or more normal Tx packets to send */
	PID1		= (1 << 17), /* 2 protocol id bits:  0==non-IP, */
	PID0		= (1 << 16), /* 1==UDP/IP, 2==TCP/IP, 3==IP */
	RxProtoTCP	= 1,
	RxProtoUDP	= 2,
	RxProtoIP	= 3,
	TxFIFOUnder	= (1 << 25), /* Tx FIFO underrun */
	TxOWC		= (1 << 22), /* Tx Out-of-window collision */
	TxLinkFail	= (1 << 21), /* Link failed during Tx of packet */
	TxMaxCol	= (1 << 20), /* Tx aborted due to excessive collisions */
	TxColCntShift	= 16,	     /* Shift, to get 4-bit Tx collision cnt */
	TxColCntMask	= 0x01 | 0x02 | 0x04 | 0x08, /* 4-bit collision count */
	RxErrFrame	= (1 << 27), /* Rx frame alignment error */
	RxMcast		= (1 << 26), /* Rx multicast packet rcv'd */
	RxErrCRC	= (1 << 18), /* Rx CRC error */
	RxErrRunt	= (1 << 19), /* Rx error, packet < 64 bytes */
	RxErrLong	= (1 << 21), /* Rx error, packet > 4096 bytes */
	RxErrFIFO	= (1 << 22), /* Rx error, FIFO overflowed, pkt bad */

	/* StatsAddr register */
	DumpStats	= (1 << 3),  /* Begin stats dump */

	/* RxConfig register */
	RxCfgFIFOShift	= 13,	     /* Shift, to get Rx FIFO thresh value */
	RxCfgDMAShift	= 8,	     /* Shift, to get Rx Max DMA value */
	AcceptErr	= 0x20,	     /* Accept packets with CRC errors */
	AcceptRunt	= 0x10,	     /* Accept runt (<64 bytes) packets */
	AcceptBroadcast	= 0x08,	     /* Accept broadcast packets */
	AcceptMulticast	= 0x04,	     /* Accept multicast packets */
	AcceptMyPhys	= 0x02,	     /* Accept pkts with our MAC as dest */
	AcceptAllPhys	= 0x01,	     /* Accept all pkts w/ physical dest */

	/* IntrMask / IntrStatus registers */
	PciErr		= (1 << 15), /* System error on the PCI bus */
	TimerIntr	= (1 << 14), /* Asserted when TCTR reaches TimerInt value */
	LenChg		= (1 << 13), /* Cable length change */
	SWInt		= (1 << 8),  /* Software-requested interrupt */
	TxEmpty		= (1 << 7),  /* No Tx descriptors available */
	RxFIFOOvr	= (1 << 6),  /* Rx FIFO Overflow */
	LinkChg		= (1 << 5),  /* Packet underrun, or link change */
	RxEmpty		= (1 << 4),  /* No Rx descriptors available */
	TxErr		= (1 << 3),  /* Tx error */
	TxOK		= (1 << 2),  /* Tx packet sent */
	RxErr		= (1 << 1),  /* Rx error */
	RxOK		= (1 << 0),  /* Rx packet received */
	IntrResvd	= (1 << 10), /* reserved, according to RealTek engineers,
					but hardware likes to raise it */

	IntrAll		= PciErr | TimerIntr | LenChg | SWInt | TxEmpty |
			  RxFIFOOvr | LinkChg | RxEmpty | TxErr | TxOK |
			  RxErr | RxOK | IntrResvd,

	/* C mode command register */
	CmdReset	= (1 << 4),  /* Enable to reset; self-clearing */
	RxOn		= (1 << 3),  /* Rx mode enable */
	TxOn		= (1 << 2),  /* Tx mode enable */

	/* C+ mode command register */
	RxVlanOn	= (1 << 6),  /* Rx VLAN de-tagging enable */
	RxChkSum	= (1 << 5),  /* Rx checksum offload enable */
	PCIDAC		= (1 << 4),  /* PCI Dual Address Cycle (64-bit PCI) */
	PCIMulRW	= (1 << 3),  /* Enable PCI read/write multiple */
	CpRxOn		= (1 << 1),  /* Rx mode enable */
	CpTxOn		= (1 << 0),  /* Tx mode enable */

	/* Cfg9436 EEPROM control register */
	Cfg9346_Lock	= 0x00,	     /* Lock ConfigX/MII register access */
	Cfg9346_Unlock	= 0xC0,	     /* Unlock ConfigX/MII register access */

	/* TxConfig register */
	IFG		= (1 << 25) | (1 << 24), /* standard IEEE interframe gap */
	TxDMAShift	= 8,	     /* DMA burst value (0-7) is shift this many bits */

	/* Early Tx Threshold register */
	TxThreshMask	= 0x3f,	     /* Mask bits 5-0 */
	TxThreshMax	= 2048,	     /* Max early Tx threshold */

	/* Config1 register */
	DriverLoaded	= (1 << 5),  /* Software marker, driver is loaded */
	LWACT           = (1 << 4),  /* LWAKE active mode */
	PMEnable	= (1 << 0),  /* Enable various PM features of chip */

	/* Config3 register */
	PARMEnable	= (1 << 6),  /* Enable auto-loading of PHY parms */
	MagicPacket     = (1 << 5),  /* Wake up when receives a Magic Packet */
	LinkUp          = (1 << 4),  /* Wake up when the cable connection is re-established */

	/* Config4 register */
	LWPTN           = (1 << 1),  /* LWAKE Pattern */
	LWPME           = (1 << 4),  /* LANWAKE vs PMEB */

	/* Config5 register */
	BWF             = (1 << 6),  /* Accept Broadcast wakeup frame */
	MWF             = (1 << 5),  /* Accept Multicast wakeup frame */
	UWF             = (1 << 4),  /* Accept Unicast wakeup frame */
	LANWake         = (1 << 1),  /* Enable LANWake signal */
	PMEStatus	= (1 << 0),  /* PME status can be reset by PCI RST# */
};

// Switch control registers
#define SWITCH_GLOBAL_CTRL_DEV		0x1f
#define SWITCH_GLOBAL_CTRL_REG		0x04
#define CTR_MODE_ALL				0x0000
#define CTR_MODE_ERR				0x0100

enum {
	/* Switch statistics counter offsets*/
	RxCtr		= 0x10,
	TxCtr		= 0x11,
};

static spinlock_t statistics_lock = SPIN_LOCK_UNLOCKED;

static const unsigned int cp_intr_mask =
	PciErr | LinkChg |
	RxOK | RxErr | RxEmpty | RxFIFOOvr |
	TxOK | TxErr | TxEmpty;

static const unsigned int cp_rx_config =
	  (RX_FIFO_THRESH << RxCfgFIFOShift) |
	  (RX_DMA_BURST << RxCfgDMAShift);

struct cp_desc {
	u32		opts1;
	u32		opts2;
	u64		addr;
};

struct ring_info {
	struct sk_buff		*skb;
	dma_addr_t		mapping;
	unsigned		frag;
};

struct cp_dma_stats {
	u64			tx_ok;
	u64			rx_ok;
	u64			tx_err;
	u32			rx_err;
	u16			rx_fifo;
	u16			frame_align;
	u32			tx_ok_1col;
	u32			tx_ok_mcol;
	u64			rx_ok_phys;
	u64			rx_ok_bcast;
	u32			rx_ok_mcast;
	u16			tx_abort;
	u16			tx_underrun;
} __attribute__((packed));

struct cp_extra_stats {
	unsigned long		rx_frags;
};

struct cp_private {
	unsigned		tx_head;
	unsigned		tx_tail;
	unsigned		rx_tail;

	void			*regs;
	struct net_device	*dev;
	spinlock_t		lock;

	struct cp_desc		*rx_ring;
	struct cp_desc		*tx_ring;
	struct ring_info	tx_skb[CP_TX_RING_SIZE];
	struct ring_info	rx_skb[CP_RX_RING_SIZE];
	unsigned		rx_buf_sz;
	dma_addr_t		ring_dma;

#if CP_VLAN_TAG_USED
	struct vlan_group	*vlgrp;
#endif

	u32			msg_enable;

	struct net_device_stats net_stats;
	struct cp_extra_stats	cp_stats;
	struct cp_dma_stats	*nic_stats;
	dma_addr_t		nic_stats_dma;

	struct pci_dev		*pdev;
	u32			rx_config;

	struct sk_buff		*frag_skb;
	unsigned		dropping_frag : 1;
	unsigned		pci_using_dac : 1;
	unsigned int		board_type;

	unsigned int		wol_enabled : 1; /* Is Wake-on-LAN enabled? */
	u32			power_state[16];

	u16					mii;
	struct mii_if_info	mii_if;
	struct timer_list	watchdog_timer;
	int			min_phy,max_phy;
};

#define cpr8(reg)	readb(cp->regs + (reg))
#define cpr16(reg)	readw(cp->regs + (reg))
#define cpr32(reg)	readl(cp->regs + (reg))
#define cpw8(reg,val)	writeb((val), cp->regs + (reg))
#define cpw16(reg,val)	writew((val), cp->regs + (reg))
#define cpw32(reg,val)	writel((val), cp->regs + (reg))
#define cpw8_f(reg,val) do {			\
	writeb((val), cp->regs + (reg));	\
	readb(cp->regs + (reg));		\
	} while (0)
#define cpw16_f(reg,val) do {			\
	writew((val), cp->regs + (reg));	\
	readw(cp->regs + (reg));		\
	} while (0)
#define cpw32_f(reg,val) do {			\
	writel((val), cp->regs + (reg));	\
	readl(cp->regs + (reg));		\
	} while (0)


static void __cp_set_rx_mode (struct net_device *dev);
static void cp_tx (struct cp_private *cp);
static void cp_clean_rings (struct cp_private *cp);
static void cp_watchdog( struct net_device *dev );

enum board_type {
	RTL8139Cp,
	RTL8169,
};

static struct cp_board_info {
	const char *name;
} cp_board_tbl[] __devinitdata = {
	/* RTL8139Cp */
	{ "RTL-8139C+" },

	/* RTL8169 */
	{ "RTL-8169" },
};

static struct pci_device_id cp_pci_tbl[] __devinitdata = {
	{ PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8139,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139Cp },
#if defined(EMULATE_EEPROM)||defined(AUTOPROGRAM_EEPROM)
	{ PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8129,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139Cp },
#endif
#if 0
	{ PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8169,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8169 },
#endif
	{ },
};
MODULE_DEVICE_TABLE(pci, cp_pci_tbl);

static struct {
	const char str[ETH_GSTRING_LEN];
} ethtool_stats_keys[] = {
	{ "tx_ok" },
	{ "rx_ok" },
	{ "tx_err" },
	{ "rx_err" },
	{ "rx_fifo" },
	{ "frame_align" },
	{ "tx_ok_1col" },
	{ "tx_ok_mcol" },
	{ "rx_ok_phys" },
	{ "rx_ok_bcast" },
	{ "rx_ok_mcast" },
	{ "tx_abort" },
	{ "tx_underrun" },
	{ "rx_frags" },
};


static inline void cp_set_rxbufsize (struct cp_private *cp)
{
	unsigned int mtu = cp->dev->mtu;
	
	if (mtu > ETH_DATA_LEN)
		/* MTU + ethernet header + FCS + optional VLAN tag */
		cp->rx_buf_sz = mtu + ETH_HLEN + 8;
	else
		cp->rx_buf_sz = PKT_BUF_SZ;
}

static inline void cp_rx_skb (struct cp_private *cp, struct sk_buff *skb,
			      struct cp_desc *desc)
{
	skb->protocol = eth_type_trans (skb, cp->dev);

	cp->net_stats.rx_packets++;
	cp->net_stats.rx_bytes += skb->len;
	cp->dev->last_rx = jiffies;

#if CP_VLAN_TAG_USED
	if (cp->vlgrp && (desc->opts2 & RxVlanTagged)) {
		vlan_hwaccel_rx(skb, cp->vlgrp, desc->opts2 & 0xffff);
	} else
#endif
		netif_rx(skb);
}

static void cp_rx_err_acct (struct cp_private *cp, unsigned rx_tail,
			    u32 status, u32 len)
{
	if (netif_msg_rx_err (cp))
		printk (KERN_DEBUG
			"%s: rx err, slot %d status 0x%x len %d\n",
			cp->dev->name, rx_tail, status, len);
	cp->net_stats.rx_errors++;
	if (status & RxErrFrame)
		cp->net_stats.rx_frame_errors++;
	if (status & RxErrCRC)
		cp->net_stats.rx_crc_errors++;
	if (status & RxErrRunt)
		cp->net_stats.rx_length_errors++;
	if (status & RxErrLong)
		cp->net_stats.rx_length_errors++;
	if (status & RxErrFIFO)
		cp->net_stats.rx_fifo_errors++;
}

static void cp_rx_frag (struct cp_private *cp, unsigned rx_tail,
			struct sk_buff *skb, u32 status, u32 len)
{
	struct sk_buff *copy_skb, *frag_skb = cp->frag_skb;
	unsigned orig_len = frag_skb ? frag_skb->len : 0;
	unsigned target_len = orig_len + len;
	unsigned first_frag = status & FirstFrag;
	unsigned last_frag = status & LastFrag;

	if (netif_msg_rx_status (cp))
		printk (KERN_DEBUG "%s: rx %s%sfrag, slot %d status 0x%x len %d\n",
			cp->dev->name,
			cp->dropping_frag ? "dropping " : "",
			first_frag ? "first " :
			last_frag ? "last " : "",
			rx_tail, status, len);

	cp->cp_stats.rx_frags++;

	if (!frag_skb && !first_frag)
		cp->dropping_frag = 1;
	if (cp->dropping_frag)
		goto drop_frag;

	copy_skb = dev_alloc_skb (target_len + RX_OFFSET);
	if (!copy_skb) {
		printk(KERN_WARNING "%s: rx slot %d alloc failed\n",
		       cp->dev->name, rx_tail);

		cp->dropping_frag = 1;
drop_frag:
		if (frag_skb) {
			dev_kfree_skb_irq(frag_skb);
			cp->frag_skb = NULL;
		}
		if (last_frag) {
			cp->net_stats.rx_dropped++;
			cp->dropping_frag = 0;
		}
		return;
	}

	copy_skb->dev = cp->dev;
	skb_reserve(copy_skb, RX_OFFSET);
	skb_put(copy_skb, target_len);
	if (frag_skb) {
		memcpy(copy_skb->data, frag_skb->data, orig_len);
		dev_kfree_skb_irq(frag_skb);
	}
	pci_dma_sync_single(cp->pdev, cp->rx_skb[rx_tail].mapping,
			    len, PCI_DMA_FROMDEVICE);
	memcpy(copy_skb->data + orig_len, skb->data, len);

	copy_skb->ip_summed = CHECKSUM_NONE;

	if (last_frag) {
		if (status & (RxError | RxErrFIFO)) {
			cp_rx_err_acct(cp, rx_tail, status, len);
			dev_kfree_skb_irq(copy_skb);
		} else
			cp_rx_skb(cp, copy_skb, &cp->rx_ring[rx_tail]);
		cp->frag_skb = NULL;
	} else {
		cp->frag_skb = copy_skb;
	}
}

static inline unsigned int cp_rx_csum_ok (u32 status)
{
	unsigned int protocol = (status >> 16) & 0x3;
	
	if (likely((protocol == RxProtoTCP) && (!(status & TCPFail))))
		return 1;
	else if ((protocol == RxProtoUDP) && (!(status & UDPFail)))
		return 1;
	else if ((protocol == RxProtoIP) && (!(status & IPFail)))
		return 1;
	return 0;
}

static void cp_rx (struct cp_private *cp)
{
	unsigned rx_tail = cp->rx_tail;
	unsigned rx_work = 100;

	while (rx_work--) {
		u32 status, len;
		dma_addr_t mapping;
		struct sk_buff *skb, *new_skb;
		struct cp_desc *desc;
		unsigned buflen;

		skb = cp->rx_skb[rx_tail].skb;
		if (!skb)
			BUG();

		desc = &cp->rx_ring[rx_tail];
		status = le32_to_cpu(desc->opts1);
		if (status & DescOwn)
			break;

		len = (status & 0x1fff) - 4;
		mapping = cp->rx_skb[rx_tail].mapping;

		if ((status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag)) {
			cp_rx_frag(cp, rx_tail, skb, status, len);
			goto rx_next;
		}

		if (status & (RxError | RxErrFIFO)) {
			cp_rx_err_acct(cp, rx_tail, status, len);
			goto rx_next;
		}

		if (netif_msg_rx_status(cp))
			printk(KERN_DEBUG "%s: rx slot %d status 0x%x len %d\n",
			       cp->dev->name, rx_tail, status, len);

		buflen = cp->rx_buf_sz + RX_OFFSET;
		new_skb = dev_alloc_skb (buflen);
		if (!new_skb) {
			cp->net_stats.rx_dropped++;
			goto rx_next;
		}

		skb_reserve(new_skb, RX_OFFSET);
		new_skb->dev = cp->dev;

		pci_unmap_single(cp->pdev, mapping,
				 buflen, PCI_DMA_FROMDEVICE);

		/* Handle checksum offloading for incoming packets. */
		if (cp_rx_csum_ok(status))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb->ip_summed = CHECKSUM_NONE;

		skb_put(skb, len);

		mapping =
		cp->rx_skb[rx_tail].mapping =
			pci_map_single(cp->pdev, new_skb->tail,
				       buflen, PCI_DMA_FROMDEVICE);
		cp->rx_skb[rx_tail].skb = new_skb;

		cp_rx_skb(cp, skb, desc);

rx_next:
		cp->rx_ring[rx_tail].opts2 = 0;
		cp->rx_ring[rx_tail].addr = cpu_to_le64(mapping);
		if (rx_tail == (CP_RX_RING_SIZE - 1))
			desc->opts1 = cpu_to_le32(DescOwn | RingEnd |
						  cp->rx_buf_sz);
		else
			desc->opts1 = cpu_to_le32(DescOwn | cp->rx_buf_sz);
		rx_tail = NEXT_RX(rx_tail);
	}

	if (!rx_work)
		printk(KERN_WARNING "%s: rx work limit reached\n", cp->dev->name);

	cp->rx_tail = rx_tail;
}

static void cp_interrupt (int irq, void *dev_instance, struct pt_regs *regs)
{
	struct net_device *dev = dev_instance;
	struct cp_private *cp = dev->priv;
	u16 status;

	status = cpr16(IntrStatus);
	if (!status || (status == 0xFFFF))
		return;

	if (netif_msg_intr(cp))
		printk(KERN_DEBUG "%s: intr, status %04x cmd %02x cpcmd %04x\n",
		        dev->name, status, cpr8(Cmd), cpr16(CpCmd));

	cpw16_f(IntrStatus, status);

	spin_lock(&cp->lock);

	if (status & (RxOK | RxErr | RxEmpty | RxFIFOOvr))
		cp_rx(cp);
	if (status & (TxOK | TxErr | TxEmpty | SWInt))
		cp_tx(cp);
	if (status & LinkChg)
		mii_check_media(&cp->mii_if, netif_msg_link(cp), FALSE);

	if (status & PciErr) {
		u16 pci_status;

		pci_read_config_word(cp->pdev, PCI_STATUS, &pci_status);
		pci_write_config_word(cp->pdev, PCI_STATUS, pci_status);
		printk(KERN_ERR "%s: PCI bus error, status=%04x, PCI status=%04x\n",
		       dev->name, status, pci_status);
	}

	spin_unlock(&cp->lock);
}

static void cp_tx (struct cp_private *cp)
{
	unsigned tx_head = cp->tx_head;
	unsigned tx_tail = cp->tx_tail;

	while (tx_tail != tx_head) {
		struct sk_buff *skb;
		u32 status;

		rmb();
		status = le32_to_cpu(cp->tx_ring[tx_tail].opts1);
		if (status & DescOwn)
			break;

		skb = cp->tx_skb[tx_tail].skb;
		if (!skb)
			BUG();

		pci_unmap_single(cp->pdev, cp->tx_skb[tx_tail].mapping,
					skb->len, PCI_DMA_TODEVICE);

		if (status & LastFrag) {
			if (status & (TxError | TxFIFOUnder)) {
				if (netif_msg_tx_err(cp))
					printk(KERN_DEBUG "%s: tx err, status 0x%x\n",
					       cp->dev->name, status);
				cp->net_stats.tx_errors++;
				if (status & TxOWC)
					cp->net_stats.tx_window_errors++;
				if (status & TxMaxCol)
					cp->net_stats.tx_aborted_errors++;
				if (status & TxLinkFail)
					cp->net_stats.tx_carrier_errors++;
				if (status & TxFIFOUnder)
					cp->net_stats.tx_fifo_errors++;
			} else {
				cp->net_stats.collisions +=
					((status >> TxColCntShift) & TxColCntMask);
				cp->net_stats.tx_packets++;
				cp->net_stats.tx_bytes += skb->len;
				if (netif_msg_tx_done(cp))
					printk(KERN_DEBUG "%s: tx done, slot %d\n", cp->dev->name, tx_tail);
			}
			dev_kfree_skb_irq(skb);
		}

		cp->tx_skb[tx_tail].skb = NULL;

		tx_tail = NEXT_TX(tx_tail);
	}

	cp->tx_tail = tx_tail;

	if (netif_queue_stopped(cp->dev) && (TX_BUFFS_AVAIL(cp) > (MAX_SKB_FRAGS + 1)))
		netif_wake_queue(cp->dev);
}

static int cp_start_xmit (struct sk_buff *skb, struct net_device *dev)
{
	struct cp_private *cp = dev->priv;
	unsigned entry;
	u32 eor;
#if CP_VLAN_TAG_USED
	u32 vlan_tag = 0;
#endif

	spin_lock_irq(&cp->lock);

	/* This is a hard error, log it. */
	if (TX_BUFFS_AVAIL(cp) <= (skb_shinfo(skb)->nr_frags + 1)) {
		netif_stop_queue(dev);
		spin_unlock_irq(&cp->lock);
		printk(KERN_ERR PFX "%s: BUG! Tx Ring full when queue awake!\n",
		       dev->name);
		return 1;
	}

#if CP_VLAN_TAG_USED
	if (cp->vlgrp && vlan_tx_tag_present(skb))
		vlan_tag = TxVlanTag | vlan_tx_tag_get(skb);
#endif

	entry = cp->tx_head;
	eor = (entry == (CP_TX_RING_SIZE - 1)) ? RingEnd : 0;
	if (skb_shinfo(skb)->nr_frags == 0) {
		struct cp_desc *txd = &cp->tx_ring[entry];
		u32 len;
		dma_addr_t mapping;

		len = skb->len;
		mapping = pci_map_single(cp->pdev, skb->data, len, PCI_DMA_TODEVICE);
		CP_VLAN_TX_TAG(txd, vlan_tag);
		txd->addr = cpu_to_le64(mapping);
		wmb();

#ifdef CP_TX_CHECKSUM
		if (skb->ip_summed == CHECKSUM_HW) {
			const struct iphdr *ip = skb->nh.iph;
			if (ip->protocol == IPPROTO_TCP)
				txd->opts1 = cpu_to_le32(eor | len | DescOwn |
							 FirstFrag | LastFrag |
							 IPCS | TCPCS);
			else if (ip->protocol == IPPROTO_UDP)
				txd->opts1 = cpu_to_le32(eor | len | DescOwn |
							 FirstFrag | LastFrag |
							 IPCS | UDPCS);
			else
				BUG();
		} else
#endif
			txd->opts1 = cpu_to_le32(eor | len | DescOwn |
						 FirstFrag | LastFrag);
		wmb();

		cp->tx_skb[entry].skb = skb;
		cp->tx_skb[entry].mapping = mapping;
		cp->tx_skb[entry].frag = 0;
		entry = NEXT_TX(entry);
	} else {
		struct cp_desc *txd;
		u32 first_len, first_eor;
		dma_addr_t first_mapping;
		int frag, first_entry = entry;
#ifdef CP_TX_CHECKSUM
		const struct iphdr *ip = skb->nh.iph;
#endif

		/* We must give this initial chunk to the device last.
		 * Otherwise we could race with the device.
		 */
		first_eor = eor;
		first_len = skb->len - skb->data_len;
		first_mapping = pci_map_single(cp->pdev, skb->data,
					       first_len, PCI_DMA_TODEVICE);
		cp->tx_skb[entry].skb = skb;
		cp->tx_skb[entry].mapping = first_mapping;
		cp->tx_skb[entry].frag = 1;
		entry = NEXT_TX(entry);

		for (frag = 0; frag < skb_shinfo(skb)->nr_frags; frag++) {
			skb_frag_t *this_frag = &skb_shinfo(skb)->frags[frag];
			u32 len;
			u32 ctrl;
			dma_addr_t mapping;

			len = this_frag->size;
			mapping = pci_map_single(cp->pdev,
						 ((void *) page_address(this_frag->page) +
						  this_frag->page_offset),
						 len, PCI_DMA_TODEVICE);
			eor = (entry == (CP_TX_RING_SIZE - 1)) ? RingEnd : 0;
#ifdef CP_TX_CHECKSUM
			if (skb->ip_summed == CHECKSUM_HW) {
				ctrl = eor | len | DescOwn | IPCS;
				if (ip->protocol == IPPROTO_TCP)
					ctrl |= TCPCS;
				else if (ip->protocol == IPPROTO_UDP)
					ctrl |= UDPCS;
				else
					BUG();
			} else
#endif
				ctrl = eor | len | DescOwn;

			if (frag == skb_shinfo(skb)->nr_frags - 1)
				ctrl |= LastFrag;

			txd = &cp->tx_ring[entry];
			CP_VLAN_TX_TAG(txd, vlan_tag);
			txd->addr = cpu_to_le64(mapping);
			wmb();

			txd->opts1 = cpu_to_le32(ctrl);
			wmb();

			cp->tx_skb[entry].skb = skb;
			cp->tx_skb[entry].mapping = mapping;
			cp->tx_skb[entry].frag = frag + 2;
			entry = NEXT_TX(entry);
		}

		txd = &cp->tx_ring[first_entry];
		CP_VLAN_TX_TAG(txd, vlan_tag);
		txd->addr = cpu_to_le64(first_mapping);
		wmb();

#ifdef CP_TX_CHECKSUM
		if (skb->ip_summed == CHECKSUM_HW) {
			if (ip->protocol == IPPROTO_TCP)
				txd->opts1 = cpu_to_le32(first_eor | first_len |
							 FirstFrag | DescOwn |
							 IPCS | TCPCS);
			else if (ip->protocol == IPPROTO_UDP)
				txd->opts1 = cpu_to_le32(first_eor | first_len |
							 FirstFrag | DescOwn |
							 IPCS | UDPCS);
			else
				BUG();
		} else
#endif
			txd->opts1 = cpu_to_le32(first_eor | first_len |
						 FirstFrag | DescOwn);
		wmb();
	}
	cp->tx_head = entry;
	if (netif_msg_tx_queued(cp))
		printk(KERN_DEBUG "%s: tx queued, slot %d, skblen %d\n",
		       dev->name, entry, skb->len);
	if (TX_BUFFS_AVAIL(cp) <= (MAX_SKB_FRAGS + 1))
		netif_stop_queue(dev);

	spin_unlock_irq(&cp->lock);

	cpw8(TxPoll, NormalTxPoll);
	dev->trans_start = jiffies;

	return 0;
}

/* Set or clear the multicast filter for this adaptor.
   This routine is not state sensitive and need not be SMP locked. */

static void __cp_set_rx_mode (struct net_device *dev)
{
	struct cp_private *cp = dev->priv;
	u32 mc_filter[2];	/* Multicast hash filter */
	int i, rx_mode;
	u32 tmp;

	/* Note: do not reorder, GCC is clever about common statements. */
	if (dev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		printk (KERN_NOTICE "%s: Promiscuous mode enabled.\n",
			dev->name);
		rx_mode =
		    AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
		    AcceptAllPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else if ((dev->mc_count > multicast_filter_limit)
		   || (dev->flags & IFF_ALLMULTI)) {
		/* Too many to filter perfectly -- accept all multicasts. */
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
		struct dev_mc_list *mclist;
		rx_mode = AcceptBroadcast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0;
		for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
		     i++, mclist = mclist->next) {
			int bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;

			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
			rx_mode |= AcceptMulticast;
		}
	}

	/* We can safely update without stopping the chip. */
	tmp = cp_rx_config | rx_mode;
	if (cp->rx_config != tmp) {
		cpw32_f (RxConfig, tmp);
		cp->rx_config = tmp;
	}
	cpw32_f (MAR0 + 0, mc_filter[0]);
	cpw32_f (MAR0 + 4, mc_filter[1]);
}

static void cp_set_rx_mode (struct net_device *dev)
{
	unsigned long flags;
	struct cp_private *cp = dev->priv;

	spin_lock_irqsave (&cp->lock, flags);
	__cp_set_rx_mode(dev);
	spin_unlock_irqrestore (&cp->lock, flags);
}

static void __cp_get_stats(struct cp_private *cp)
{
	/* XXX implement */
}

static struct net_device_stats *cp_get_stats(struct net_device *dev)
{
	struct cp_private *cp = dev->priv;

	/* The chip only need report frame silently dropped. */
	spin_lock_irq(&cp->lock);
 	if (netif_running(dev) && netif_device_present(dev))
 		__cp_get_stats(cp);
	spin_unlock_irq(&cp->lock);

	return &cp->net_stats;
}

static void cp_stop_hw (struct cp_private *cp)
{
	cpw16(IntrMask, 0);
	cpr16(IntrMask);
	cpw8(Cmd, 0);
	cpw16(CpCmd, 0);
	cpr16(CpCmd);
	cpw16(IntrStatus, ~(cpr16(IntrStatus)));
	synchronize_irq();
	udelay(10);

	cp->rx_tail = 0;
	cp->tx_head = cp->tx_tail = 0;
}

static void cp_reset_hw (struct cp_private *cp)
{
	unsigned work = 1000;

	cpw8(Cmd, CmdReset);

	while (work--) {
		if (!(cpr8(Cmd) & CmdReset))
			return;

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(10);
	}

	printk(KERN_ERR "%s: hardware reset timeout\n", cp->dev->name);
}

static inline void cp_start_hw (struct cp_private *cp)
{
	u16 pci_dac = cp->pci_using_dac ? PCIDAC : 0;
	if (cp->board_type == RTL8169)
		cpw16(CpCmd, pci_dac | PCIMulRW | RxChkSum);
	else
		cpw16(CpCmd, pci_dac | PCIMulRW | RxChkSum | CpRxOn | CpTxOn);
	cpw8(Cmd, RxOn | TxOn);
}

static void cp_init_hw (struct cp_private *cp)
{
	struct net_device *dev = cp->dev;

	cp_reset_hw(cp);

	cpw8_f (Cfg9346, Cfg9346_Unlock);

	/* Restore our idea of the MAC address. */
	cpw32_f (MAC0 + 0, cpu_to_le32 (*(u32 *) (dev->dev_addr + 0)));
	cpw32_f (MAC0 + 4, cpu_to_le32 (*(u32 *) (dev->dev_addr + 4)));

	cp_start_hw(cp);
	cpw8(TxThresh, 0x06); /* XXX convert magic num to a constant */

	__cp_set_rx_mode(dev);
	cpw32_f (TxConfig, IFG | (TX_DMA_BURST << TxDMAShift));

	cpw8(Config1, cpr8(Config1) | DriverLoaded | PMEnable);
	/* Disable Wake-on-LAN. Can be turned on with ETHTOOL_SWOL */
	if (cp->board_type == RTL8139Cp) {
		cpw8(Config3, PARMEnable);
		cp->wol_enabled = 0;
	}
	cpw8(Config5, cpr8(Config5) & PMEStatus); 
	if (cp->board_type == RTL8169)
		cpw16(RxMaxSize, cp->rx_buf_sz);

	cpw32_f(HiTxRingAddr, 0);
	cpw32_f(HiTxRingAddr + 4, 0);

	cpw32_f(RxRingAddr, cp->ring_dma);
	cpw32_f(RxRingAddr + 4, 0);		/* FIXME: 64-bit PCI */
	cpw32_f(TxRingAddr, cp->ring_dma + (sizeof(struct cp_desc) * CP_RX_RING_SIZE));
	cpw32_f(TxRingAddr + 4, 0);		/* FIXME: 64-bit PCI */

	cpw16(MultiIntr, 0);

	cpw16_f(IntrMask, cp_intr_mask);

	cpw8_f(Cfg9346, Cfg9346_Lock);
}

static int cp_refill_rx (struct cp_private *cp)
{
	unsigned i;

	for (i = 0; i < CP_RX_RING_SIZE; i++) {
		struct sk_buff *skb;

		skb = dev_alloc_skb(cp->rx_buf_sz + RX_OFFSET);
		if (!skb)
			goto err_out;

		skb->dev = cp->dev;
		skb_reserve(skb, RX_OFFSET);

		cp->rx_skb[i].mapping = pci_map_single(cp->pdev,
			skb->tail, cp->rx_buf_sz, PCI_DMA_FROMDEVICE);
		cp->rx_skb[i].skb = skb;
		cp->rx_skb[i].frag = 0;

		cp->rx_ring[i].opts2 = 0;
		cp->rx_ring[i].addr = cpu_to_le64(cp->rx_skb[i].mapping);
		if (i == (CP_RX_RING_SIZE - 1))
			cp->rx_ring[i].opts1 =
				cpu_to_le32(DescOwn | RingEnd | cp->rx_buf_sz);
		else
			cp->rx_ring[i].opts1 =
				cpu_to_le32(DescOwn | cp->rx_buf_sz);
	}

	return 0;

err_out:
	cp_clean_rings(cp);
	return -ENOMEM;
}

static int cp_init_rings (struct cp_private *cp)
{
	memset(cp->tx_ring, 0, sizeof(struct cp_desc) * CP_TX_RING_SIZE);
	cp->tx_ring[CP_TX_RING_SIZE - 1].opts1 = cpu_to_le32(RingEnd);

	cp->rx_tail = 0;
	cp->tx_head = cp->tx_tail = 0;

	return cp_refill_rx (cp);
}

static int cp_alloc_rings (struct cp_private *cp)
{
	void *mem;

	mem = pci_alloc_consistent(cp->pdev, CP_RING_BYTES, &cp->ring_dma);
	if (!mem)
		return -ENOMEM;

	cp->rx_ring = mem;
	cp->tx_ring = &cp->rx_ring[CP_RX_RING_SIZE];

	mem += (CP_RING_BYTES - CP_STATS_SIZE);
	cp->nic_stats = mem;
	cp->nic_stats_dma = cp->ring_dma + (CP_RING_BYTES - CP_STATS_SIZE);

	return cp_init_rings(cp);
}

static void cp_clean_rings (struct cp_private *cp)
{
	unsigned i;

	memset(cp->rx_ring, 0, sizeof(struct cp_desc) * CP_RX_RING_SIZE);
	memset(cp->tx_ring, 0, sizeof(struct cp_desc) * CP_TX_RING_SIZE);

	for (i = 0; i < CP_RX_RING_SIZE; i++) {
		if (cp->rx_skb[i].skb) {
			pci_unmap_single(cp->pdev, cp->rx_skb[i].mapping,
					 cp->rx_buf_sz, PCI_DMA_FROMDEVICE);
			dev_kfree_skb(cp->rx_skb[i].skb);
		}
	}

	for (i = 0; i < CP_TX_RING_SIZE; i++) {
		if (cp->tx_skb[i].skb) {
			struct sk_buff *skb = cp->tx_skb[i].skb;
			pci_unmap_single(cp->pdev, cp->tx_skb[i].mapping,
					 skb->len, PCI_DMA_TODEVICE);
			dev_kfree_skb(skb);
			cp->net_stats.tx_dropped++;
		}
	}

	memset(&cp->rx_skb, 0, sizeof(struct ring_info) * CP_RX_RING_SIZE);
	memset(&cp->tx_skb, 0, sizeof(struct ring_info) * CP_TX_RING_SIZE);
}

static void cp_free_rings (struct cp_private *cp)
{
	cp_clean_rings(cp);
	pci_free_consistent(cp->pdev, CP_RING_BYTES, cp->rx_ring, cp->ring_dma);
	cp->rx_ring = NULL;
	cp->tx_ring = NULL;
	cp->nic_stats = NULL;
}

static int cp_open (struct net_device *dev)
{
	struct cp_private *cp = dev->priv;
	int rc;

	if (netif_msg_ifup(cp))
		printk(KERN_DEBUG "%s: enabling interface\n", dev->name);

	rc = cp_alloc_rings(cp);
	if (rc)
		return rc;

	cp_init_hw(cp);

	rc = request_irq(dev->irq, cp_interrupt, SA_SHIRQ, dev->name, dev);
	if (rc)
		goto err_out_hw;

	netif_carrier_off(dev);
	mii_check_media(&cp->mii_if, netif_msg_link(cp), TRUE);
	netif_start_queue(dev);

	return 0;

err_out_hw:
	cp_stop_hw(cp);
	cp_free_rings(cp);
	return rc;
}

static int cp_close (struct net_device *dev)
{
	struct cp_private *cp = dev->priv;

	if (netif_msg_ifdown(cp))
		printk(KERN_DEBUG "%s: disabling interface\n", dev->name);

	if( cp->mii )
		del_timer_sync(&cp->watchdog_timer);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	spin_lock_irq(&cp->lock);
	cp_stop_hw(cp);
	spin_unlock_irq(&cp->lock);

	free_irq(dev->irq, dev);
	cp_free_rings(cp);
	return 0;
}

#ifdef BROKEN
static int cp_change_mtu(struct net_device *dev, int new_mtu)
{
	struct cp_private *cp = dev->priv;
	int rc;

	/* check for invalid MTU, according to hardware limits */
	if (new_mtu < CP_MIN_MTU || new_mtu > CP_MAX_MTU)
		return -EINVAL;

	/* if network interface not up, no need for complexity */
	if (!netif_running(dev)) {
		dev->mtu = new_mtu;
		cp_set_rxbufsize(cp);	/* set new rx buf size */
		return 0;
	}

	spin_lock_irq(&cp->lock);

	cp_stop_hw(cp);			/* stop h/w and free rings */
	cp_clean_rings(cp);

	dev->mtu = new_mtu;
	cp_set_rxbufsize(cp);		/* set new rx buf size */
	if (cp->board_type == RTL8169)
		cpw16(RxMaxSize, cp->rx_buf_sz);

	rc = cp_init_rings(cp);		/* realloc and restart h/w */
	cp_start_hw(cp);

	spin_unlock_irq(&cp->lock);

	return rc;
}
#endif /* BROKEN */

static char mii_2_8139_map[8] = {
	BasicModeCtrl,
	BasicModeStatus,
	0,
	0,
	NWayAdvert,
	NWayLPAR,
	NWayExpansion,
	0
};

#ifdef MII_EXTERNAL_PHY

/* Read and write the MII management registers using software-generated
   serial MDIO protocol.
   The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues. */

#define MDIO_DIR			0x08000000
#define MDIO_DATA_OUT		0x04000000
#define MDIO_DATA_IN		0x02000000
#define MDIO_CLK			0x01000000
#define MDIO_WRITE0			(MDIO_DIR)
#define MDIO_WRITE1			(MDIO_DIR | MDIO_DATA_OUT)
#define mdio_delay(mdio_addr)	readl(mdio_addr)

/* Syncronize the MII management interface by shifting 32 one bits out. */
static void mdio_sync(long mdio_addr)
{
	int i;

	for (i = 32; i >= 0; i--) {
		writel(MDIO_WRITE1, mdio_addr);
		mdio_delay(mdio_addr);
		writel(MDIO_WRITE1 | MDIO_CLK, mdio_addr);
		mdio_delay(mdio_addr);
	}
	return;
}

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	long mdio_addr;
	int mii_cmd = (0xf6 << 10) | (phy_id << 5) | location;
	int retval = 0;
	int i;

	mdio_addr = dev->base_addr + 0xFC;
	if( location > 31 )
	{
		printk("Invalid MII MDIO_READ location: %d\n",location );
		return( 0 );
	}

	if (phy_id > 31) {	/* Really an internal register request. */
        	return location < 8 && mii_2_8139_map[location] ?
               		readw(dev->base_addr + mii_2_8139_map[location]) : 0;
	}
	mdio_sync(mdio_addr);
	/* Shift the read command bits out. */
	for (i = 15; i >= 0; i--) {
		int dataval = (mii_cmd & (1 << i)) ? MDIO_DATA_OUT : 0;

		writel(MDIO_DIR | dataval, mdio_addr);
		mdio_delay(mdio_addr);
		writel(MDIO_DIR | dataval | MDIO_CLK, mdio_addr);
		mdio_delay(mdio_addr);
	}

	/* Read the two transition, 16 data, and wire-idle bits. */
	for (i = 19; i > 0; i--) {
		writel(0, mdio_addr);
		mdio_delay(mdio_addr);
		retval = (retval << 1) | ((readl(mdio_addr) & MDIO_DATA_IN) ? 1 : 0);
		writel(MDIO_CLK, mdio_addr);
		mdio_delay(mdio_addr);
	}
	return (retval>>1) & 0xffff;
}

static void mdio_write(struct net_device *dev, int phy_id, int location,
					   int value)
{
	long mdio_addr;
	int mii_cmd = (0x5002 << 16) | (phy_id << 23) | (location<<18) | value;
	int i;

	mdio_addr = dev->base_addr + 0xFC;
	if( location > 31 )
	{
		printk("Invalid MII MDIO_READ location: %d\n",location );
		return;
	}

	if (phy_id > 31) {	/* Really a request for 8139 internal register. */
		struct cp_private *cp = dev->priv;
        	if (location == 0) {
                	cpw8(Cfg9346, Cfg9346_Unlock);
                	cpw16(BasicModeCtrl, value);
                	cpw8(Cfg9346, Cfg9346_Lock);
        	} else if (location < 8 && mii_2_8139_map[location])
                	cpw16(mii_2_8139_map[location], value);
		return;
	}
	mdio_sync(mdio_addr);

	/* Shift the command bits out. */
	for (i = 31; i >= 0; i--) {
		int dataval = (mii_cmd & (1 << i)) ? MDIO_WRITE1 : MDIO_WRITE0;
		writel(dataval, mdio_addr);
		mdio_delay(mdio_addr);
		writel(dataval | MDIO_CLK, mdio_addr);
		mdio_delay(mdio_addr);
	}
	/* Clear out extra bits. */
	for (i = 2; i > 0; i--) {
		writel(0, mdio_addr);
		mdio_delay(mdio_addr);
		writel(MDIO_CLK, mdio_addr);
		mdio_delay(mdio_addr);
	}
}

#else

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct cp_private *cp = dev->priv;

	return location < 8 && mii_2_8139_map[location] ?
	       readw(cp->regs + mii_2_8139_map[location]) : 0;
}


static void mdio_write(struct net_device *dev, int phy_id, int location,
		       int value)
{
	struct cp_private *cp = dev->priv;

	if (location == 0) {
		cpw8(Cfg9346, Cfg9346_Unlock);
		cpw16(BasicModeCtrl, value);
		cpw8(Cfg9346, Cfg9346_Lock);
	} else if (location < 8 && mii_2_8139_map[location])
		cpw16(mii_2_8139_map[location], value);
}

#endif

/* Set the ethtool Wake-on-LAN settings */
static void netdev_set_wol (struct cp_private *cp,
                     const struct ethtool_wolinfo *wol)
{
	u8 options;

	options = cpr8 (Config3) & ~(LinkUp | MagicPacket);
	/* If WOL is being disabled, no need for complexity */
	if (wol->wolopts) {
		if (wol->wolopts & WAKE_PHY)	options |= LinkUp;
		if (wol->wolopts & WAKE_MAGIC)	options |= MagicPacket;
	}

	cpw8 (Cfg9346, Cfg9346_Unlock);
	cpw8 (Config3, options);
	cpw8 (Cfg9346, Cfg9346_Lock);

	options = 0; /* Paranoia setting */
	options = cpr8 (Config5) & ~(UWF | MWF | BWF);
	/* If WOL is being disabled, no need for complexity */
	if (wol->wolopts) {
		if (wol->wolopts & WAKE_UCAST)  options |= UWF;
		if (wol->wolopts & WAKE_BCAST)	options |= BWF;
		if (wol->wolopts & WAKE_MCAST)	options |= MWF;
	}

	cpw8 (Config5, options);

	cp->wol_enabled = (wol->wolopts) ? 1 : 0;
}

/* Get the ethtool Wake-on-LAN settings */
static void netdev_get_wol (struct cp_private *cp,
	             struct ethtool_wolinfo *wol)
{
	u8 options;

	wol->wolopts   = 0; /* Start from scratch */
	wol->supported = WAKE_PHY   | WAKE_BCAST | WAKE_MAGIC |
		         WAKE_MCAST | WAKE_UCAST;
	/* We don't need to go on if WOL is disabled */
	if (!cp->wol_enabled) return;
	
	options        = cpr8 (Config3);
	if (options & LinkUp)        wol->wolopts |= WAKE_PHY;
	if (options & MagicPacket)   wol->wolopts |= WAKE_MAGIC;

	options        = 0; /* Paranoia setting */
	options        = cpr8 (Config5);
	if (options & UWF)           wol->wolopts |= WAKE_UCAST;
	if (options & BWF)           wol->wolopts |= WAKE_BCAST;
	if (options & MWF)           wol->wolopts |= WAKE_MCAST;
}

int mv88e6060_get_stats(struct net_device *dev, int port, struct net_device_stats *phy_stats) {
	unsigned long flags;

	memset(phy_stats, 0x00, sizeof(struct net_device_stats));

	spin_lock_irqsave(&statistics_lock, flags);
	phy_stats->rx_crc_errors	= (unsigned long)mdio_read(dev, port, RxCtr);
	phy_stats->collisions		= (unsigned long)mdio_read(dev, port, TxCtr);
	spin_unlock_irqrestore(&statistics_lock, flags);

	return 0;
}

static int cp_ethtool_ioctl (struct cp_private *cp, void *useraddr)
{
	u32 ethcmd;

	/* dev_ioctl() in ../../net/core/dev.c has already checked
	   capable(CAP_NET_ADMIN), so don't bother with that here.  */

	if (get_user(ethcmd, (u32 *)useraddr))
		return -EFAULT;

	switch (ethcmd) {

	case ETHTOOL_GDRVINFO: {
		struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
		strcpy (info.driver, DRV_NAME);
		strcpy (info.version, DRV_VERSION);
		strcpy (info.bus_info, cp->pdev->slot_name);
		info.regdump_len = CP_REGS_SIZE;
		info.n_stats = CP_NUM_STATS;
		if (copy_to_user (useraddr, &info, sizeof (info)))
			return -EFAULT;
		return 0;
	}

	/* get settings */
	case ETHTOOL_GSET: {
		struct ethtool_cmd ecmd = { ETHTOOL_GSET };
		spin_lock_irq(&cp->lock);
		mii_ethtool_gset(&cp->mii_if, &ecmd);
		spin_unlock_irq(&cp->lock);
		if (copy_to_user(useraddr, &ecmd, sizeof(ecmd)))
			return -EFAULT;
		return 0;
	}
	/* set settings */
	case ETHTOOL_SSET: {
		int r;
		struct ethtool_cmd ecmd;
		if (copy_from_user(&ecmd, useraddr, sizeof(ecmd)))
			return -EFAULT;
		spin_lock_irq(&cp->lock);
		r = mii_ethtool_sset(&cp->mii_if, &ecmd);
		spin_unlock_irq(&cp->lock);
		return r;
	}
	/* restart autonegotiation */
	case ETHTOOL_NWAY_RST: {
		return mii_nway_restart(&cp->mii_if);
	}
	/* get link status */
	case ETHTOOL_GLINK: {
		struct ethtool_value edata = {ETHTOOL_GLINK};
		edata.data = mii_link_ok(&cp->mii_if);
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}

	/* get message-level */
	case ETHTOOL_GMSGLVL: {
		struct ethtool_value edata = {ETHTOOL_GMSGLVL};
		edata.data = cp->msg_enable;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	/* set message-level */
	case ETHTOOL_SMSGLVL: {
		struct ethtool_value edata;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		cp->msg_enable = edata.data;
		return 0;
	}

	/* NIC register dump */
	case ETHTOOL_GREGS: {
                struct ethtool_regs regs;
                u8 *regbuf = kmalloc(CP_REGS_SIZE, GFP_KERNEL);
                int rc;

		if (!regbuf)
			return -ENOMEM;
		memset(regbuf, 0, CP_REGS_SIZE);

                rc = copy_from_user(&regs, useraddr, sizeof(regs));
		if (rc) {
			rc = -EFAULT;
			goto err_out_gregs;
		}
                
                if (regs.len > CP_REGS_SIZE)
                        regs.len = CP_REGS_SIZE;
                if (regs.len < CP_REGS_SIZE) {
			rc = -EINVAL;
			goto err_out_gregs;
		}

                regs.version = CP_REGS_VER;
                rc = copy_to_user(useraddr, &regs, sizeof(regs));
		if (rc) {
			rc = -EFAULT;
			goto err_out_gregs;
		}

                useraddr += offsetof(struct ethtool_regs, data);

                spin_lock_irq(&cp->lock);
                memcpy_fromio(regbuf, (int)cp->regs, CP_REGS_SIZE);
                spin_unlock_irq(&cp->lock);

                if (copy_to_user(useraddr, regbuf, regs.len))
                        rc = -EFAULT;

err_out_gregs:
		kfree(regbuf);
		return rc;
	}

	/* get/set RX checksumming */
	case ETHTOOL_GRXCSUM: {
		struct ethtool_value edata = { ETHTOOL_GRXCSUM };
		u16 cmd = cpr16(CpCmd) & RxChkSum;

		edata.data = cmd ? 1 : 0;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	case ETHTOOL_SRXCSUM: {
		struct ethtool_value edata;
		u16 cmd = cpr16(CpCmd), newcmd;

		newcmd = cmd;

		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;

		if (edata.data)
			newcmd |= RxChkSum;
		else
			newcmd &= ~RxChkSum;

		if (newcmd == cmd)
			return 0;

		spin_lock_irq(&cp->lock);
		cpw16_f(CpCmd, newcmd);
		spin_unlock_irq(&cp->lock);
	}

	/* get/set TX checksumming */
	case ETHTOOL_GTXCSUM: {
		struct ethtool_value edata = { ETHTOOL_GTXCSUM };

		edata.data = (cp->dev->features & NETIF_F_IP_CSUM) != 0;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	case ETHTOOL_STXCSUM: {
		struct ethtool_value edata;

		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;

		if (edata.data)
			cp->dev->features |= NETIF_F_IP_CSUM;
		else
			cp->dev->features &= ~NETIF_F_IP_CSUM;

		return 0;
	}

	/* get/set scatter-gather */
	case ETHTOOL_GSG: {
		struct ethtool_value edata = { ETHTOOL_GSG };

		edata.data = (cp->dev->features & NETIF_F_SG) != 0;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	case ETHTOOL_SSG: {
		struct ethtool_value edata;

		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;

		if (edata.data)
			cp->dev->features |= NETIF_F_SG;
		else
			cp->dev->features &= ~NETIF_F_SG;

		return 0;
	}

	/* get string list(s) */
	case ETHTOOL_GSTRINGS: {
		struct ethtool_gstrings estr = { ETHTOOL_GSTRINGS };

		if (copy_from_user(&estr, useraddr, sizeof(estr)))
			return -EFAULT;
		if (estr.string_set != ETH_SS_STATS)
			return -EINVAL;

		estr.len = CP_NUM_STATS;
		if (copy_to_user(useraddr, &estr, sizeof(estr)))
			return -EFAULT;
		if (copy_to_user(useraddr + sizeof(estr),
				 &ethtool_stats_keys,
				 sizeof(ethtool_stats_keys)))
			return -EFAULT;
		return 0;
	}

	/* get NIC-specific statistics */
	case ETHTOOL_GSTATS: {
		struct ethtool_stats estats = { ETHTOOL_GSTATS };
		u64 *tmp_stats;
		unsigned int work = 100;
		const unsigned int sz = sizeof(u64) * CP_NUM_STATS;
		int i;

		/* begin NIC statistics dump */
		cpw32(StatsAddr + 4, 0); /* FIXME: 64-bit PCI */
		cpw32(StatsAddr, cp->nic_stats_dma | DumpStats);
		cpr32(StatsAddr);

		estats.n_stats = CP_NUM_STATS;
		if (copy_to_user(useraddr, &estats, sizeof(estats)))
			return -EFAULT;

		while (work-- > 0) {
			if ((cpr32(StatsAddr) & DumpStats) == 0)
				break;
			cpu_relax();
		}

		if (cpr32(StatsAddr) & DumpStats)
			return -EIO;

		tmp_stats = kmalloc(sz, GFP_KERNEL);
		if (!tmp_stats)
			return -ENOMEM;
		memset(tmp_stats, 0, sz);

		i = 0;
		tmp_stats[i++] = le64_to_cpu(cp->nic_stats->tx_ok);
		tmp_stats[i++] = le64_to_cpu(cp->nic_stats->rx_ok);
		tmp_stats[i++] = le64_to_cpu(cp->nic_stats->tx_err);
		tmp_stats[i++] = le32_to_cpu(cp->nic_stats->rx_err);
		tmp_stats[i++] = le16_to_cpu(cp->nic_stats->rx_fifo);
		tmp_stats[i++] = le16_to_cpu(cp->nic_stats->frame_align);
		tmp_stats[i++] = le32_to_cpu(cp->nic_stats->tx_ok_1col);
		tmp_stats[i++] = le32_to_cpu(cp->nic_stats->tx_ok_mcol);
		tmp_stats[i++] = le64_to_cpu(cp->nic_stats->rx_ok_phys);
		tmp_stats[i++] = le64_to_cpu(cp->nic_stats->rx_ok_bcast);
		tmp_stats[i++] = le32_to_cpu(cp->nic_stats->rx_ok_mcast);
		tmp_stats[i++] = le16_to_cpu(cp->nic_stats->tx_abort);
		tmp_stats[i++] = le16_to_cpu(cp->nic_stats->tx_underrun);
		tmp_stats[i++] = cp->cp_stats.rx_frags;
		if (i != CP_NUM_STATS)
			BUG();

		i = copy_to_user(useraddr + sizeof(estats),
				 tmp_stats, sz);
		kfree(tmp_stats);

		if (i)
			return -EFAULT;
		return 0;
	}

	/* get/set Wake-on-LAN settings */
	case ETHTOOL_GWOL: {
		struct ethtool_wolinfo wol = { ETHTOOL_GWOL };
		
		spin_lock_irq (&cp->lock);
		netdev_get_wol (cp, &wol);
		spin_unlock_irq (&cp->lock);
		return ((copy_to_user (useraddr, &wol, sizeof (wol)))? -EFAULT : 0);
	}
	
	case ETHTOOL_SWOL: {
		struct ethtool_wolinfo wol;

		if (copy_from_user (&wol, useraddr, sizeof (wol)))
			return -EFAULT;
		spin_lock_irq (&cp->lock);
		netdev_set_wol (cp, &wol);
		spin_unlock_irq (&cp->lock);
		return 0;
	}

	default:
		break;
	}

	return -EOPNOTSUPP;
}


static int cp_ioctl (struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct cp_private *cp = dev->priv;
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *) &rq->ifr_data;
	int rc, phy_port;

	if (!netif_running(dev))
		return -EINVAL;

	if (cmd == SIOCETHTOOL) {
#ifdef	__SONOS_LINUX_SH4__
		if (REDROCKS) {
			if (strcmp("eth0", rq->ifr_name) == 0)
				cp->mii_if.phy_id=0x14;
			else if (strcmp("eth1", rq->ifr_name) == 0)
				cp->mii_if.phy_id=0x10;
			else 
				return -ENODEV;
		} else {
			if (strcmp("eth0", rq->ifr_name) == 0)
				cp->mii_if.phy_id=1;
			else if (strcmp("eth1", rq->ifr_name) == 0)
				cp->mii_if.phy_id=2;
			else if (strcmp("eth2", rq->ifr_name) == 0)
				cp->mii_if.phy_id=3;
			else if (strcmp("eth3", rq->ifr_name) == 0)
				cp->mii_if.phy_id=4;
			else 
				return -ENODEV;
		}
		rq->ifr_name[3] = '0';
#endif	// __SONOS_LINUX_SH4__
		return cp_ethtool_ioctl(cp, (void *) rq->ifr_data);
	}
	else if (cmd == SIOCGIFSTATS) {
#ifdef __SONOS_LINUX_SH4__
		struct net_device_stats stats;

		if (REDROCKS) {
			if (strcmp("eth0", rq->ifr_name) == 0) {
				phy_port = 0x1C;
			}
			else if (strcmp("eth1", rq->ifr_name) == 0) {
				phy_port = 0x18;
			} else {
				return -ENODEV;
			}
		} else {
			return -ENOSYS;
		}
		mv88e6060_get_stats(cp->mii_if.dev, phy_port, &stats);
		if (copy_to_user(rq->ifr_data, (void *)&stats, sizeof(struct net_device_stats)))
			return -EINVAL;
		else return 0;
#endif
	}
	spin_lock_irq(&cp->lock);
	rc = generic_mii_ioctl(&cp->mii_if, mii, cmd, NULL);
	spin_unlock_irq(&cp->lock);
	return rc;
}

#if CP_VLAN_TAG_USED
static void cp_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	struct cp_private *cp = dev->priv;

	spin_lock_irq(&cp->lock);
	cp->vlgrp = grp;
	cpw16(CpCmd, cpr16(CpCmd) | RxVlanOn);
	spin_unlock_irq(&cp->lock);
}

static void cp_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid)
{
	struct cp_private *cp = dev->priv;

	spin_lock_irq(&cp->lock);
	cpw16(CpCmd, cpr16(CpCmd) & ~RxVlanOn);
	if (cp->vlgrp)
		cp->vlgrp->vlan_devices[vid] = NULL;
	spin_unlock_irq(&cp->lock);
}
#endif

/* Emulate Serial EEPROM section. */

#if defined(EMULATE_EEPROM)||defined(AUTOPROGRAM_EEPROM)

typedef struct  {
	int 	se_addr;
	int	se_len;
	int	se_data;
} SOFT_EEPROM;

SOFT_EEPROM soft_eeprom [] = {
{	0,	2,	0x8129	},
{	1,	2,	0x10EC	},
{	2,	2,	0x8139  },
{	3,	2,	0x10EC	},
{	4,	2,	0x8139	},
{	5,	2,	0x4020	},
{	6,	2,	0xC121	},
{	7,	2,	0xE000	},
{	8,	2,	0x004C	},
{	9,	2,	0x0F00	},
{	10,	2,	0x8D10	},
{	11,	2,	0xF7C2	},
{	12,	2,	0xA801	},
{	13,	2,	0x4369	},
{	14,	2,	0xA0F2	},
{	15,	2,	0x071A	},
{	16,	2,	0xDF43	},
{	17,	2,	0x8A36 	},
{	18,	2,	0xDF43	},
{	19,	2,	0x8A36	},
{	20,	2,	0x43B9	},
{	21,	2,	0xA0F2	},
{	22,	2,	0x1111	},
{	23,	2,	0x1111	},
{	24,	2,	0x0000	},
{	25,	2,	0x57AC	},
{	26,	2,	0x0000	},
{	27,	2,	0x0000	},
{	28,	2,	0x0000	},
{	29,	2,	0x0000	},
{	30,	2,	0x0000	},
{	31,	2,	0x6000	},
{	0,	0,	0	}
};
#endif
#ifdef EMULATE_EEPROM
static int __devinit read_eeprom (void *ioaddr, int location, int addr_len)
{
	unsigned retval = 0;
	if( location >= 0 && location < 0x20 )
		retval = soft_eeprom[location].se_data;
	return( retval );
}

#define MSR_REG_MASK		0xC0
#define BMCR_REG_MASK		0x3F
#define BMCR_REG_SHIFT		0x08

// this emulates the eeprom copy process loading configuration memory, io memory
// and core registers
static void	emulate_eeprom_init( struct net_device *dev, struct cp_private *cp )
{
	unsigned int c;
	unsigned int bmcr;
	struct pci_dev *pdev = cp->pdev;
	unsigned short r;
	int x;

	pci_write_config_word(pdev, 0x00, soft_eeprom[1].se_data );
	pci_write_config_word(pdev, 0x02, soft_eeprom[2].se_data );
	pci_write_config_word(pdev, 0x2C, soft_eeprom[3].se_data );
	pci_write_config_word(pdev, 0x2E, soft_eeprom[4].se_data );
	pci_write_config_byte(pdev, 0x3E, soft_eeprom[5].se_data&0xFF );
	pci_write_config_byte(pdev, 0x3F, (soft_eeprom[5].se_data>>8)&0xFF );

	// make sure we can write Config 0, 1, 3, 4, and BMCR bits 13, 12, 8
	cpw8_f (Cfg9346, Cfg9346_Unlock);
	r=cpr16(BasicModeCtrl);
	cpw16(BasicModeCtrl,0x8000);
	for (x=0;x<1000;x++) {
		r=cpr16(BasicModeCtrl);
		if ((r&0x8000)==0) {
			break;
		}
	}
	if (r&0x8000) printk("8139cp: In emulate_eeprom_init, BMCR reset didn't self-clear\n");
	c = soft_eeprom[6].se_data;
	cpw8( MediaStatus,c&MSR_REG_MASK );	
	bmcr = ((c&BMCR_REG_MASK)<<BMCR_REG_SHIFT);
	cpw16( BasicModeCtrl, bmcr );
	cpw8( Config3,c>>8 );

	// set the ethernet address 
	cpw32_f (MAC0 + 0, *(u32 *) (&soft_eeprom[7].se_data) );
	cpw32_f (MAC0 + 4, *(u32 *) (&soft_eeprom[9].se_data) );

	cpw8( Config0,soft_eeprom[10].se_data&0xFF );
	cpw8( Config1,(soft_eeprom[10].se_data)>>8 );

	pci_write_config_word( pdev, 0x52, soft_eeprom[11].se_data );

	cpw8( Config4,(soft_eeprom[12].se_data>>8)&0xFF );

	c = soft_eeprom[13].se_data << 16 | soft_eeprom[14].se_data;
	cpw32( PARA78, c );
	cpw8( PHY2_Parm,soft_eeprom[15].se_data&0xFF );
	cpw8( Config5,((soft_eeprom[15].se_data>>8)&0xFF) );
	
	c = soft_eeprom[16].se_data << 16 | soft_eeprom[17].se_data;
	cpw32( PARA7c,c );

	c = soft_eeprom[18].se_data << 16 | soft_eeprom[19].se_data;
	cpw32( PARA7c,c );

	c = soft_eeprom[20].se_data << 16 | soft_eeprom[21].se_data;
	cpw16( PARA78, c );
	cpw8( PHY2_Parm,soft_eeprom[22].se_data&0xFF );

	// lock configuration register again
	cpw8_f(Cfg9346, Cfg9346_Lock);
}

#else
/*  EEPROM_Ctrl bits. */
#define EE_SHIFT_CLK	0x04	/* EEPROM shift clock. */
#define EE_CS			0x08	/* EEPROM chip select. */
#define EE_DATA_WRITE	0x02	/* EEPROM chip data in. */
#define EE_WRITE_0		0x00
#define EE_WRITE_1		0x02
#define EE_DATA_READ	0x01	/* EEPROM chip data out. */
#define EE_ENB			(0x80 | EE_CS)
#define EE_ENB_ONLY		0x80

/* Delay between EEPROM clock transitions.
   No extra delay is needed with 33Mhz PCI, but 66Mhz may change this.
 */

#define eeprom_delay()	do { readl(ee_addr); udelay(1); } while(0)

/* The EEPROM commands include the alway-set leading bit. */
#define EE_WRITE_CMD	(5)
#define EE_READ_CMD		(6)
#define EE_ERASE_CMD	(7)

static int __devinit read_eeprom (void *ioaddr, int location, int addr_len)
{
	int i;
	unsigned retval = 0;
	void *ee_addr = ioaddr + Cfg9346;
	int read_cmd = location | (EE_READ_CMD << addr_len);

	writeb (EE_ENB & ~EE_CS, ee_addr);
	writeb (EE_ENB, ee_addr);
	eeprom_delay ();

	/* Shift the read command bits out. */
	for (i = 4 + addr_len; i >= 0; i--) {
		int dataval = (read_cmd & (1 << i)) ? EE_DATA_WRITE : 0;
		writeb (EE_ENB | dataval, ee_addr);
		eeprom_delay ();
		writeb (EE_ENB | dataval | EE_SHIFT_CLK, ee_addr);
		eeprom_delay ();
	}
	writeb (EE_ENB, ee_addr);
	eeprom_delay ();

	for (i = 16; i > 0; i--) {
		writeb (EE_ENB | EE_SHIFT_CLK, ee_addr);
		eeprom_delay ();
		retval =
		    (retval << 1) | ((readb (ee_addr) & EE_DATA_READ) ? 1 :
				     0);
		writeb (EE_ENB, ee_addr);
		eeprom_delay ();
	}

	/* Terminate the EEPROM access. */
	writeb (~EE_CS, ee_addr);
	eeprom_delay ();

	return retval;
}

#endif		/* EMULATE_EEPROM */

#ifdef AUTOPROGRAM_EEPROM
static int eeprom_write_word (void *ioaddr,int addr,unsigned short data)
{
	void *ee_addr = ioaddr + Cfg9346;
	unsigned int d;
	int x;
	d=0x130; /*EWEN, MSB first*/
	
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	for (x=8;x>=0;x--) {
		writeb(EE_ENB_ONLY|EE_CS| ((d&(1<<x))?EE_DATA_WRITE:0) , ee_addr);
		eeprom_delay();
		writeb(EE_ENB_ONLY|EE_CS|EE_SHIFT_CLK| ((d&(1<<x))?EE_DATA_WRITE:0), ee_addr);
		eeprom_delay();
	}
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	d=0x1400000|addr<<16|data;
	for (x=24;x>=0;x--) {
		writeb(EE_ENB_ONLY|EE_CS| ((d&(1<<x))?EE_DATA_WRITE:0) , ee_addr);
		eeprom_delay();
		writeb(EE_ENB_ONLY|EE_CS|EE_SHIFT_CLK| ((d&(1<<x))?EE_DATA_WRITE:0), ee_addr);
		eeprom_delay();
	}
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	while (((readb(ee_addr))&EE_DATA_READ)==0) ; /*wait for ready*/
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	d=0x100; /*EWDS*/
	for (x=8;x>=0;x--) {
                writeb(EE_ENB_ONLY|EE_CS| ((d&(1<<x))?EE_DATA_WRITE:0) , ee_addr);
                eeprom_delay();
                writeb(EE_ENB_ONLY|EE_CS|EE_SHIFT_CLK| ((d&(1<<x))?EE_DATA_WRITE:0), ee_addr);
                eeprom_delay();
        }
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	d=0x180|addr; /*READ*/
	for (x=8;x>=0;x--) {
		writeb(EE_ENB_ONLY|EE_CS| ((d&(1<<x))?EE_DATA_WRITE:0) , ee_addr);
		eeprom_delay();
		writeb(EE_ENB_ONLY|EE_CS|EE_SHIFT_CLK| ((d&(1<<x))?EE_DATA_WRITE:0), ee_addr);
		eeprom_delay();
	}
	d=0;
	writeb(EE_ENB_ONLY|EE_CS,ee_addr);
	eeprom_delay();
	for (x=0;x<16;x++) {
		writeb(EE_ENB_ONLY|EE_CS|EE_SHIFT_CLK,ee_addr);
		eeprom_delay();
		d=(d<<1)|(((readb(ee_addr))&EE_DATA_READ)?1:0);
		writeb(EE_ENB_ONLY|EE_CS,ee_addr);
		eeprom_delay();
	}
	writeb(EE_ENB_ONLY,ee_addr);
	eeprom_delay();
	writeb(0,ee_addr);
	if (d==data) return 0;
	return -1;
}
	
static int eeprom_program(void *ioaddr)
{
	void *ee_addr=ioaddr+Cfg9346;
	int x;
	for (x=0;x<32;x++) {
		if ((eeprom_write_word(ioaddr,x,soft_eeprom[x].se_data))!=0) {
			printk("Failed to program EEPROM (location %d)\n",x);
			return -1;
		}
	}
	printk("Resetting 8139C+ with new EEPROM values\n");
	writeb(0x40,ee_addr);
	while (((readb(ee_addr))&0xc0)!=0) ;
	printk("8139C+ has been reset with new EEPROM values\n");
	return 0;
}
#endif /*AUTOPROGRAM_EEPROM*/

/* Put the board into D3cold state and wait for WakeUp signal */
static void cp_set_d3_state (struct cp_private *cp)
{
	pci_enable_wake (cp->pdev, 0, 1); /* Enable PME# generation */
	pci_set_power_state (cp->pdev, 3);
}

#ifdef MII_EXTERNAL_PHY

#define	MII_EXTERNAL_ENABLE		0x80
static void mii_external_mode( struct net_device *dev )
{
	struct cp_private *cp = dev->priv;
	cpw16( BasicModeStatus,MII_EXTERNAL_ENABLE );
}

#define MII_EXTERNAL_PHY_ID		5
#define MII_MAX_PHY_ADDRS   	5
#define MII_STAT_REG            0x01
#define MII_STAT_LINK_UP        0x04
#define MII_CTL_REG             0x00
#define MII_CTL_SPEED_100M      0x2000
#define MII_CTL_FD              0x1000
#define MII_ADV_REG				0x04

/* notify user mode of a new MII link */
static int rtmsg_add_mii_info(struct net_device *dev, 
                              struct sk_buff *skb)
{
    struct nlmsghdr *nlh;
    unsigned char *b = skb->tail;

    nlh = NLMSG_PUT(skb, 0, 0, RWM_MII, 0);
    RTA_PUT(skb, RWA_DEV_NAME,   IFNAMSIZ,           dev->name);
    nlh->nlmsg_len = skb->tail - b;
    return skb->len;

nlmsg_failure:
rtattr_failure:
    skb_trim(skb, b - skb->data);
    return -1;    
}

static void rtmsg_on_mii_new_link(struct net_device *dev)
{
    struct sk_buff *skb;
    int size = NLMSG_SPACE(1024);

    skb = alloc_skb(size, GFP_ATOMIC);
    if (!skb) {
        netlink_set_err(rtnl, 0, RTMGRP_Rincon, ENOBUFS);
        return;
    }
    if (rtmsg_add_mii_info(dev, skb) < 0) {
        kfree_skb(skb);
        netlink_set_err(rtnl, 0, RTMGRP_Rincon, EINVAL);
        return;
    }
    NETLINK_CB(skb).dst_groups = RTMGRP_Rincon;
    netlink_broadcast(rtnl, skb, 0, RTMGRP_Rincon, GFP_ATOMIC);    
}

#define MAX_PHYS 5
#define PHYSTATE_INITIALIZER {-1,-1,-1,-1,-1}
void mii_setup( struct net_device *dev )
{
    struct cp_private *cp = dev->priv;
    if (REDROCKS) {
      cp->min_phy=0x10;
      cp->max_phy=0x14;
    } else {
      cp->min_phy=1;
      cp->max_phy=4;
    }
    cp->mii_if.phy_id=cp->min_phy;
}
static int mii_port_scan( struct net_device *dev )
{
    int val = 0;
    int i, stat, ctl;
	int sav_val = 0, sav_indx = -1;

	static int last_indx = -1;
    static int phystate[MAX_PHYS]=PHYSTATE_INITIALIZER;
    int schg=0;
    struct cp_private *cp = dev->priv;

    for( i = cp->min_phy; i <= cp->max_phy; i++ ) {

        stat = mdio_read( dev,i,MII_STAT_REG );
		if( stat == 0xffff || stat == 0x0000 ) 
			continue;

        if( (stat&MII_STAT_LINK_UP) == 0 ) {
			if (phystate[i-cp->min_phy]!= -1) schg=1;
			phystate[i-cp->min_phy]= -1;
			continue;
		}

		val = 0;
		ctl = mdio_read( dev,i,MII_CTL_REG );
		if( ctl&MII_CTL_SPEED_100M )
			val |= 2;
		if( ctl&MII_CTL_FD )
			val |= 1;

		if (val!=phystate[i-cp->min_phy]) schg=1;
		phystate[i-cp->min_phy]=val;
		/* remember fastest, full duplex */
		if( val > sav_val ) {
			sav_val = val;
			sav_indx = i;	
		}
    }

	if (schg) {
		if (sav_indx>=0) printk( KERN_INFO "%s: RTL8139CP - MII_PORT_SCAN: new link: %d speed %s duplex: %s\n",
                      dev->name, sav_indx-cp->min_phy, sav_val&2 ? "100Mbps" : "10Mbps",
                      sav_val&1 ? "full duplex" : "half duplex" ); else
                      printk( KERN_INFO "%s: RTL8139CP - all links down\n",dev->name);
		rtmsg_on_mii_new_link(dev);
	}
	if( sav_indx >= 0 ) {
		cp->mii_if.phy_id = sav_indx;
		last_indx = sav_indx;
	} else {
		if( last_indx != -2 ) {
			last_indx = -2;
			cp->mii_if.phy_id = cp->min_phy;
			printk(KERN_INFO "%s: RTL8139CP - No MII transceivers active\n", dev->name);
		}
	}

    return( sav_indx );
}

static void cp_watchdog( struct net_device *dev )
{
    struct cp_private *cp = dev->priv;

    /* Print the link status if it has changed */
    if (cp->mii) {
		mii_port_scan( dev );
		mii_check_media (&cp->mii_if, 1, 0);
    }
	mod_timer (&(cp->watchdog_timer), CP_WATCHDOG_TIMEOUT);
}


#endif

#ifdef RTL8139_PROC_DEBUG
//==========================================================================================
struct proc_dir_entry *rtl8139_Proc_root;
struct proc_dir_entry *rtl8139_Proc;

static int rtl8139_proc_status(char *buffer,
		  	char **buffer_location,
		  	off_t offset,
		  	int buffer_length,
		  	int *eof,
		  	void *data)
{
	char *buf = buffer;
	struct net_device *dev = (struct net_device *)data;
	struct cp_private *cp = dev->priv;

	buf += sprintf( buf, " ************ Current driver status ************\n\n\n" );
	buf += sprintf( buf, " MAC address = %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
				dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2], dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5] );

	buf += sprintf( buf, " ioaddr = 0x%8.8lx\n",  (unsigned long)dev->base_addr );
	buf += sprintf( buf, " irq = %d\n",  dev->irq );
	buf += sprintf( buf, " IntrStatus: 0x%x TxConfig: 0x%x\n",cpr16(IntrStatus),(unsigned int)cpr32(TxConfig) );
	buf += sprintf( buf, " CommandReg: 0x%x Config0: 0x%x Config1: 0x%x\n",cpr8(Cfg9346),cpr8(Config0), cpr8(Config1) );
	buf += sprintf( buf, " MediaStat: 0x%x Config3: 0x%x Config4: 0x%x\n",cpr8(MediaStatus),cpr8(Config3), cpr8(Config4) );
	buf += sprintf( buf, " BMCR: 0x%x BMSR: 0x%x\n",cpr16(BasicModeCtrl),cpr16(BasicModeStatus) );
	buf += sprintf( buf, " ADV: 0x%x LPAR: 0x%x ADV-Exp: 0x%x\n",cpr16(NWayAdvert),cpr16(NWayLPAR), cpr16(NWayExpansion) );
	buf += sprintf( buf, " CSCR: 0x%x PARA78: 0x%x PARA7C: 0x%x\n",(unsigned int)cpr32(CSCR),(unsigned int)cpr32(PARA78), cpr8(PARA7c) );
	buf += sprintf( buf, " Config5: 0x%x MII: 0x%x\n",cpr8(Config5),(unsigned int)cpr32(MIIReg) );
	buf += sprintf( buf, " IntrMask: 0x%x\n",cpr16(IntrMask));

	return (buf - buffer);
}


static void rtl8139_init_proc(struct net_device *dev)
{
	if(rtl8139_Proc_root == NULL)
		rtl8139_Proc_root = proc_mkdir("rtl8139cp", NULL);
	if(rtl8139_Proc_root != NULL){
		rtl8139_Proc = create_proc_read_entry(
				"status",0644, rtl8139_Proc_root, rtl8139_proc_status, (void *)dev);
	}
}

static void rtl8139_remove_proc(struct net_device *dev)
{
	if(rtl8139_Proc_root != NULL){
		remove_proc_entry("status", rtl8139_Proc_root );
		remove_proc_entry("rtl8139cp", NULL );
	}
}
//==========================================================================================
#endif	//#ifdef RTL8139_PROC_DEBUG


static int __devinit cp_init_one (struct pci_dev *pdev,
				  const struct pci_device_id *ent)
{
	struct net_device *dev;
	struct cp_private *cp;
	int rc;
	void *regs;
	long pciaddr;
	unsigned int addr_len, i;
	u8 pci_rev, cache_size;
	u16 pci_command;
	unsigned int board_type = (unsigned int) ent->driver_data;
	int switch_global_ctrl_reg_val;

#ifndef MODULE
	static int version_printed;
	if (version_printed++ == 0)
		printk("%s", version);
#endif

	pci_read_config_byte(pdev, PCI_REVISION_ID, &pci_rev);
	if (pdev->vendor == PCI_VENDOR_ID_REALTEK &&
	    pdev->device == PCI_DEVICE_ID_REALTEK_8139 && pci_rev < 0x20) {
		printk(KERN_ERR PFX "pci dev %s (id %04x:%04x rev %02x) is not an 8139C+ compatible chip\n",
		       pdev->slot_name, pdev->vendor, pdev->device, pci_rev);
		printk(KERN_ERR PFX "Try the \"8139too\" driver instead.\n");
		return -ENODEV;
	}

	dev = alloc_etherdev(sizeof(struct cp_private));
	if (!dev)
		return -ENOMEM;
	SET_MODULE_OWNER(dev);
	cp = dev->priv;
	cp->pdev = pdev;
	cp->board_type = board_type;
	cp->dev = dev;
	cp->msg_enable = (debug < 0 ? CP_DEF_MSG_ENABLE : debug);
	spin_lock_init (&cp->lock);
	cp->mii_if.dev = dev;
	cp->mii_if.mdio_read = mdio_read;
	cp->mii_if.mdio_write = mdio_write;
	cp->mii_if.phy_id = CP_INTERNAL_PHY;
	cp->mii_if.phy_id_mask = 0x1f;
	cp->mii_if.reg_num_mask = 0x1f;
	cp_set_rxbufsize(cp);

	rc = pci_enable_device(pdev);
	if (rc)
		goto err_out_free;

	rc = pci_request_regions(pdev, DRV_NAME);
	if (rc)
		goto err_out_disable;

	if (pdev->irq < 2) {
		rc = -EIO;
		printk(KERN_ERR PFX "invalid irq (%d) for pci dev %s\n",
		       pdev->irq, pdev->slot_name);
		goto err_out_res;
	}
	pciaddr = pci_resource_start(pdev, 1);
	if (!pciaddr) {
		rc = -EIO;
		printk(KERN_ERR PFX "no MMIO resource for pci dev %s\n",
		       pdev->slot_name);
		goto err_out_res;
	}
	if (pci_resource_len(pdev, 1) < CP_REGS_SIZE) {
		rc = -EIO;
		printk(KERN_ERR PFX "MMIO resource (%lx) too small on pci dev %s\n",
		       pci_resource_len(pdev, 1), pdev->slot_name);
		goto err_out_res;
	}

	/* Configure DMA attributes. */
	if (!pci_set_dma_mask(pdev, (u64) 0xffffffffffffffff)) {
		cp->pci_using_dac = 1;
	} else {
		rc = pci_set_dma_mask(pdev, (u64) 0xffffffff);
		if (rc) {
			printk(KERN_ERR PFX "No usable DMA configuration, "
			       "aborting.\n");
			goto err_out_res;
		}
		cp->pci_using_dac = 0;
	}

	regs = ioremap_nocache(pciaddr, CP_REGS_SIZE);
	if (!regs) {
		rc = -EIO;
		printk(KERN_ERR PFX "Cannot map PCI MMIO (%lx@%lx) on pci dev %s\n",
		       pci_resource_len(pdev, 1), pciaddr, pdev->slot_name);
		goto err_out_res;
	}
	dev->base_addr = (unsigned long) regs;
	cp->regs = regs;

#ifdef AUTOPROGRAM_EEPROM
	if ((pdev->vendor==PCI_VENDOR_ID_REALTEK)&&(pdev->device==0x8129)) {
		printk("Looks like the 8139C+ EEPROM isn't initialized, I'll initialize it\n");
		if ((eeprom_program(regs))!=0) {
			printk("EEPROM programming failed\n");
		}
	}
#endif /*AUTOPROGRAM_EEPROM*/

	cp_stop_hw(cp);

#ifdef EMULATE_EEPROM
	emulate_eeprom_init( dev,cp );
#endif
	/* read MAC address from EEPROM */
	addr_len = read_eeprom (regs, 0, 8) == 0x8129 ? 8 : 6;
	for (i = 0; i < 3; i++)
		((u16 *) (dev->dev_addr))[i] =
		    le16_to_cpu (read_eeprom (regs, i + 7, addr_len));

	dev->open = cp_open;
	dev->stop = cp_close;
	dev->set_multicast_list = cp_set_rx_mode;
	dev->hard_start_xmit = cp_start_xmit;
	dev->get_stats = cp_get_stats;
	dev->do_ioctl = cp_ioctl;
#ifdef BROKEN
	dev->change_mtu = cp_change_mtu;
#endif
#if 0
	dev->tx_timeout = cp_tx_timeout;
	dev->watchdog_timeo = TX_TIMEOUT;
#endif
#ifdef CP_TX_CHECKSUM
	dev->features |= NETIF_F_SG | NETIF_F_IP_CSUM;
#endif
#if CP_VLAN_TAG_USED
	dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
	dev->vlan_rx_register = cp_vlan_rx_register;
	dev->vlan_rx_kill_vid = cp_vlan_rx_kill_vid;
#endif

	dev->irq = pdev->irq;

	rc = register_netdev(dev);
	if (rc)
		goto err_out_iomap;

	printk (KERN_INFO "%s: %s at 0x%lx, "
		"%02x:%02x:%02x:%02x:%02x:%02x, "
		"IRQ %d\n",
		dev->name,
		cp_board_tbl[board_type].name,
		dev->base_addr,
		dev->dev_addr[0], dev->dev_addr[1],
		dev->dev_addr[2], dev->dev_addr[3],
		dev->dev_addr[4], dev->dev_addr[5],
		dev->irq);

	pci_set_drvdata(pdev, dev);

	/*
	 * Looks like this is necessary to deal with on all architectures,
	 * even this %$#%$# N440BX Intel based thing doesn't get it right.
	 * Ie. having two NICs in the machine, one will have the cache
	 * line set at boot time, the other will not.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cache_size);
	cache_size <<= 2;
	if (cache_size != SMP_CACHE_BYTES) {
		printk(KERN_INFO "%s: PCI cache line size set incorrectly "
		       "(%i bytes) by BIOS/FW, ", dev->name, cache_size);
		if (cache_size > SMP_CACHE_BYTES)
			printk("expecting %i\n", SMP_CACHE_BYTES);
		else {
			printk("correcting to %i\n", SMP_CACHE_BYTES);
			pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE,
					      SMP_CACHE_BYTES >> 2);
		}
	}

	/* enable busmastering and memory-write-invalidate */
	pci_read_config_word(pdev, PCI_COMMAND, &pci_command);
	if (!(pci_command & PCI_COMMAND_INVALIDATE)) {
		pci_command |= PCI_COMMAND_INVALIDATE;
		pci_write_config_word(pdev, PCI_COMMAND, pci_command);
	}
	pci_set_master(pdev);

#ifdef MII_EXTERNAL_PHY
	cp->mii = 1;
	mii_external_mode( dev );
    mii_setup(dev);
    init_timer ( &cp->watchdog_timer );
    cp->watchdog_timer.data = (unsigned long) dev;
    cp->watchdog_timer.function = (void *) &cp_watchdog;
	cp_watchdog( dev );
#endif

#ifdef RTL8139_PROC_DEBUG
	//-----------------------------------------------------------------------------
	// Add proc file system
	//-----------------------------------------------------------------------------
	rtl8139_init_proc(dev);
#endif	//#ifdef RTL8139_PROC_DEBUG

	if (cp->wol_enabled) cp_set_d3_state (cp);
	
	// Set the counter mode to get useful statistics instead of just frames sent/recieved.
	switch_global_ctrl_reg_val = mdio_read(dev, SWITCH_GLOBAL_CTRL_DEV, SWITCH_GLOBAL_CTRL_REG);
	mdio_write(dev, SWITCH_GLOBAL_CTRL_DEV, SWITCH_GLOBAL_CTRL_REG, switch_global_ctrl_reg_val | CTR_MODE_ERR);

	return 0;

err_out_iomap:
	iounmap(regs);
err_out_res:
	pci_release_regions(pdev);
err_out_disable:
	pci_disable_device(pdev);
err_out_free:
	kfree(dev);
	return rc;
}

static void __devexit cp_remove_one (struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct cp_private *cp = dev->priv;

	if (!dev)
		BUG();

#ifdef RTL8139_PROC_DEBUG
	//-----------------------------------------------------------------------------
	// Remove proc file system
	//-----------------------------------------------------------------------------
	rtl8139_remove_proc(dev);
#endif	//#ifdef RTL8139_PROC_DEBUG

	unregister_netdev(dev);
	iounmap(cp->regs);
	if (cp->wol_enabled) pci_set_power_state (pdev, 0);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(dev);
}

#ifdef CONFIG_PM
static int cp_suspend (struct pci_dev *pdev, u32 state)
{
	struct net_device *dev;
	struct cp_private *cp;
	unsigned long flags;

	dev = pci_get_drvdata (pdev);
	cp  = dev->priv;

	if (!dev || !netif_running (dev)) return 0;

	netif_device_detach (dev);
	netif_stop_queue (dev);

	spin_lock_irqsave (&cp->lock, flags);

	/* Disable Rx and Tx */
	cpw16 (IntrMask, 0);
	cpw8  (Cmd, cpr8 (Cmd) & (~RxOn | ~TxOn));

	spin_unlock_irqrestore (&cp->lock, flags);

	if (cp->pdev && cp->wol_enabled) {
		pci_save_state (cp->pdev, cp->power_state);
		cp_set_d3_state (cp);
	}

	return 0;
}

static int cp_resume (struct pci_dev *pdev)
{
	struct net_device *dev;
	struct cp_private *cp;

	dev = pci_get_drvdata (pdev);
	cp  = dev->priv;

	netif_device_attach (dev);
	
	if (cp->pdev && cp->wol_enabled) {
		pci_set_power_state (cp->pdev, 0);
		pci_restore_state (cp->pdev, cp->power_state);
	}
	
	cp_init_hw (cp);
	netif_start_queue (dev);
	
	return 0;
}
#endif /* CONFIG_PM */

static struct pci_driver cp_driver = {
	.name         = DRV_NAME,
	.id_table     = cp_pci_tbl,
	.probe        =	cp_init_one,
	.remove       = __devexit_p(cp_remove_one),
#ifdef CONFIG_PM
	.resume       = cp_resume,
	.suspend      = cp_suspend,
#endif
};

static int __init cp_init (void)
{
#ifdef MODULE
	printk("%s", version);
#endif
	return pci_module_init (&cp_driver);
}

static void __exit cp_exit (void)
{
	pci_unregister_driver (&cp_driver);
}

module_init(cp_init);
module_exit(cp_exit);
