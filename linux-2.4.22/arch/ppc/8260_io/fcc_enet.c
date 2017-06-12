/*
 * Fast Ethernet Controller (FCC) driver for Motorola MPC8260.
 * Copyright (c) 2000 MontaVista Software, Inc.   Dan Malek (dmalek@jlc.net)
 *
 * This version of the driver is a combination of the 8xx fec and
 * 8260 SCC Ethernet drivers.  People seem to be choosing common I/O
 * configurations, so this driver will work on the EST8260 boards and
 * others yet to be announced.
 *
 * Right now, I am very watseful with the buffers.  I allocate memory
 * pages and then divide them into 2K frame buffers.  This way I know I
 * have buffers large enough to hold one frame within one buffer descriptor.
 * Once I get this working, I will use 64 or 128 byte CPM buffers, which
 * will be much more memory efficient and will easily handle lots of
 * small packets.
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/rtnetlink.h>
#include <linux/ethtool.h>
#include <linux/proc_fs.h>

#include <asm/immap_8260.h>
#include <asm/pgtable.h>
#include <asm/mpc8260.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/cpm_8260.h>
#include <asm/system.h>

/* The transmitter timeout
 */
#define TX_TIMEOUT	(2*HZ)

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it is best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#define FCC_ENET_RX_PAGES	16
#define FCC_ENET_RX_FRSIZE	2048
#define RX_RING_SIZE		32
#define RX_RING_MOD_MASK        (RX_RING_SIZE-1)
#define TX_RING_SIZE		16	/* Must be power of two */
#define TX_RING_MOD_MASK	15	/*   for this to work */

/* The FCC stores dest/src/type, data, and checksum for receive packets.
 */
#define PKT_MAXBUF_SIZE		1518
#define PKT_MINBUF_SIZE		64

/* Maximum input DMA size.  Must be a should(?) be a multiple of 4.
*/
#define PKT_MAXDMA_SIZE		1520

/* Maximum input buffer size.  Must be a multiple of 32.
*/
#define PKT_MAXBLR_SIZE		1536

static int fcc_enet_open(struct net_device *dev);
static int fcc_enet_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int fcc_enet_rx(struct net_device *dev);
static void fcc_enet_mii(struct net_device *dev);
static	void fcc_enet_interrupt(int irq, void * dev_id, struct pt_regs * regs);
static int fcc_enet_close(struct net_device *dev);
static struct net_device_stats *fcc_enet_get_stats(struct net_device *dev);
static void set_multicast_list(struct net_device *dev);
static void restart_fcc(struct net_device *dev);
static int fcc_enet_set_mac_address(struct net_device *dev, void *p);

/* Just enough ethtool to be dangerous... */
static int fcc_enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
static int netdev_ethtool_ioctl (struct net_device *dev, void *useraddr);

/*
 * RX Error collection
 */
#define BD_ENET_RX_ERR  (BD_ENET_RX_LG|BD_ENET_RX_SH|BD_ENET_RX_NO| \
                         BD_ENET_RX_CR|BD_ENET_RX_OV|BD_ENET_RX_CL)
#define BD_RX_CPYBRK    200

/* These will be configurable for the FCC choice.
 * Multiple ports can be configured.  There is little choice among the
 * I/O pins to the PHY, except the clocks.  We will need some board
 * dependent clock selection.
 * Why in the hell did I put these inside #ifdef's?  I dunno, maybe to
 * help show what pins are used for each device.
 */

#ifdef CONFIG_PPC_SONOS
#undef CONFIG_RMII
#endif

#ifdef CONFIG_8272
/* I/O Pin assignment for FCC1.  I don't yet know the best way to do this,
 * but there is little variation among the choices.
 */
#define PA1_COL		((uint)0x00000001)
#define PA1_CRS		((uint)0x00000002)
#define PA1_TXER	((uint)0x00000004)
#define PA1_TXEN	((uint)0x00000008)
#define PA1_RXDV	((uint)0x00000010)
#define PA1_RXER	((uint)0x00000020)
#define PA1_TXDAT	((uint)0x00003c00)
#define PA1_RMII_TXDAT	((uint)0x00003000)
#define PA1_RXDAT	((uint)0x0003c000)
#define PA1_RMII_RXDAT	((uint)0x0000c000)
#ifdef CONFIG_RMII
#define PA1_PSORA0	(PA1_RMII_RXDAT | PA1_RMII_TXDAT)
#define PA1_PSORA1	(PA1_TXEN | PA1_RXDV | PA1_RXER)
#define PA1_DIRA0	(PA1_RMII_RXDAT | PA1_RXDV | PA1_RXER)
#define PA1_DIRA1	(PA1_RMII_TXDAT | PA1_TXEN)
#else
#define PA1_PSORA0	(PA1_RXDAT | PA1_TXDAT)
#define PA1_PSORA1	(PA1_COL | PA1_CRS | PA1_TXER | PA1_TXEN | \
				PA1_RXDV | PA1_RXER)
#define PA1_DIRA0	(PA1_RXDAT | PA1_CRS | PA1_COL | PA1_RXER | PA1_RXDV)
#define PA1_DIRA1	(PA1_TXDAT | PA1_TXEN | PA1_TXER)
#endif

/* CLK11 is receive, CLK10 is transmit.  These are board specific.
*/
#ifdef CONFIG_PPC_SONOS
#define PC_F1RXCLK	((uint)0x00000100)
#define PC_F1TXCLK	((uint)0x00000200)
#define CMX1_CLK_ROUTE	((uint)0x25000000)
#define F1PHYADDR	1
#endif
#ifdef CONFIG_MPC8272ADS
#define PC_F1RXCLK	((uint)0x00000400)
#define PC_F1TXCLK	((uint)0x00000200)
#define CMX1_CLK_ROUTE	((uint)0x35000000)
#endif
#define CMX1_CLK_MASK	((uint)0xff000000)

/* I/O Pin assignment for FCC2.  I don't yet know the best way to do this,
 * but there is little variation among the choices.
 */
#define PB2_TXER	((uint)0x00000001)
#define PB2_RXDV	((uint)0x00000002)
#define PB2_TXEN	((uint)0x00000004)
#define PB2_RXER	((uint)0x00000008)
#define PB2_COL		((uint)0x00000010)
#define PB2_CRS		((uint)0x00000020)
#define PB2_TXDAT	((uint)0x000003c0)
#define PB2_RMII_TXDAT	((uint)0x00000300)
#define PB2_RXDAT	((uint)0x00003c00)
#define PB2_RMII_RXDAT	((uint)0x00000c00)
#ifdef CONFIG_RMII
#define PB2_PSORB0	(PB2_RMII_RXDAT | PB2_RMII_TXDAT | PB2_RXER | PB2_RXDV)
#define PB2_PSORB1	(PB2_TXEN)
#define PB2_DIRB0	(PB2_RMII_RXDAT | PB2_RXER | PB2_RXDV)
#define PB2_DIRB1	(PB2_RMII_TXDAT | PB2_TXEN)
#else
#define PB2_PSORB0	(PB2_RXDAT | PB2_TXDAT | PB2_CRS | PB2_COL | \
				PB2_RXER | PB2_RXDV | PB2_TXER)
#define PB2_PSORB1	(PB2_TXEN)
#define PB2_DIRB0	(PB2_RXDAT | PB2_CRS | PB2_COL | PB2_RXER | PB2_RXDV)
#define PB2_DIRB1	(PB2_TXDAT | PB2_TXEN | PB2_TXER)
#endif

/* CLK15 is receive, CLK16 is transmit.  These are board dependent.
*/
#ifdef CONFIG_PPC_SONOS
#define PC_F2RXCLK	((uint)0x00001000)
#define PC_F2TXCLK	((uint)0x00002000)
#define CMX2_CLK_ROUTE	((uint)0x00250000)
#define F2PHYADDR	1
#endif
#ifdef CONFIG_MPC8272ADS
#define PC_F2RXCLK	((uint)0x00004000)
#define PC_F2TXCLK	((uint)0x00008000)
#define CMX2_CLK_ROUTE	((uint)0x00370000)
#endif
#define CMX2_CLK_MASK	((uint)0x00ff0000)

/* MII status/control serial interface.
*/
#ifdef CONFIG_PPC_SONOS
/*these are really on port D - that distinction is handled elsewhere*/
#define PC_MDIO		((uint)0x00000080)
#define PC_MDCK		((uint)0x00000100)
#define PC_MDCK2	((uint)0x00000040)
#endif
#ifdef CONFIG_MPC8272ADS
#define PC_MDIO		((uint)0x00002000)
#define PC_MDCK		((uint)0x00001000)
#endif

#else /*!8272*/

/* I/O Pin assignment for FCC1.  I don't yet know the best way to do this,
 * but there is little variation among the choices.
 */
#define PA1_COL		((uint)0x00000001)
#define PA1_CRS		((uint)0x00000002)
#define PA1_TXER	((uint)0x00000004)
#define PA1_TXEN	((uint)0x00000008)
#define PA1_RXDV	((uint)0x00000010)
#define PA1_RXER	((uint)0x00000020)
#define PA1_TXDAT	((uint)0x00003c00)
#define PA1_RXDAT	((uint)0x0003c000)
#define PA1_PSORA0	(PA1_RXDAT | PA1_TXDAT)
#define PA1_PSORA1	(PA1_COL | PA1_CRS | PA1_TXER | PA1_TXEN | \
				PA1_RXDV | PA1_RXER)
#define PA1_DIRA0	(PA1_RXDAT | PA1_CRS | PA1_COL | PA1_RXER | PA1_RXDV)
#define PA1_DIRA1	(PA1_TXDAT | PA1_TXEN | PA1_TXER)

/* CLK12 is receive, CLK11 is transmit.  These are board specific.
*/
#define PC_F1RXCLK	((uint)0x00000800)
#define PC_F1TXCLK	((uint)0x00000400)
#define CMX1_CLK_ROUTE	((uint)0x3e000000)
#define CMX1_CLK_MASK	((uint)0xff000000)

/* I/O Pin assignment for FCC2.  I don't yet know the best way to do this,
 * but there is little variation among the choices.
 */
#define PB2_TXER	((uint)0x00000001)
#define PB2_RXDV	((uint)0x00000002)
#define PB2_TXEN	((uint)0x00000004)
#define PB2_RXER	((uint)0x00000008)
#define PB2_COL		((uint)0x00000010)
#define PB2_CRS		((uint)0x00000020)
#define PB2_TXDAT	((uint)0x000003c0)
#define PB2_RXDAT	((uint)0x00003c00)
#define PB2_PSORB0	(PB2_RXDAT | PB2_TXDAT | PB2_CRS | PB2_COL | \
				PB2_RXER | PB2_RXDV | PB2_TXER)
#define PB2_PSORB1	(PB2_TXEN)
#define PB2_DIRB0	(PB2_RXDAT | PB2_CRS | PB2_COL | PB2_RXER | PB2_RXDV)
#define PB2_DIRB1	(PB2_TXDAT | PB2_TXEN | PB2_TXER)

/* CLK13 is receive, CLK14 is transmit.  These are board dependent.
*/
#define PC_F2RXCLK	((uint)0x00001000)
#define PC_F2TXCLK	((uint)0x00002000)
#define CMX2_CLK_ROUTE	((uint)0x00250000)
#define CMX2_CLK_MASK	((uint)0x00ff0000)

/* I/O Pin assignment for FCC3.  I don't yet know the best way to do this,
 * but there is little variation among the choices.
 */
#define PB3_RXDV	((uint)0x00004000)
#define PB3_RXER	((uint)0x00008000)
#define PB3_TXER	((uint)0x00010000)
#define PB3_TXEN	((uint)0x00020000)
#define PB3_COL		((uint)0x00040000)
#define PB3_CRS		((uint)0x00080000)
#define PB3_TXDAT	((uint)0x0f000000)
#define PB3_RXDAT	((uint)0x00f00000)
#define PB3_PSORB0	(PB3_RXDAT | PB3_TXDAT | PB3_CRS | PB3_COL | \
				PB3_RXER | PB3_RXDV | PB3_TXER | PB3_TXEN)
#define PB3_PSORB1	(0)
#define PB3_DIRB0	(PB3_RXDAT | PB3_CRS | PB3_COL | PB3_RXER | PB3_RXDV)
#define PB3_DIRB1	(PB3_TXDAT | PB3_TXEN | PB3_TXER)

/* CLK15 is receive, CLK16 is transmit.  These are board dependent.
*/
#define PC_F3RXCLK	((uint)0x00004000)
#define PC_F3TXCLK	((uint)0x00008000)
#define CMX3_CLK_ROUTE	((uint)0x00003700)
#define CMX3_CLK_MASK	((uint)0x0000ff00)

/* MII status/control serial interface.
*/
#define PC_MDIO		((uint)0x00400000)
#define PC_MDCK		((uint)0x00200000)

#endif /* CONFIG_MPC8272ADS */

/* A table of information for supporting FCCs.  This does two things.
 * First, we know how many FCCs we have and they are always externally
 * numbered from zero.  Second, it holds control register and I/O
 * information that could be different among board designs.
 */
typedef struct fcc_info {
	uint	fc_fccnum;
	uint	fc_cpmblock;
	uint	fc_cpmpage;
	uint	fc_proff;
	uint	fc_interrupt;
	uint	fc_trxclocks;
	uint	fc_clockroute;
	uint	fc_clockmask;
	uint	fc_mdio;
	uint	fc_mdck;
	int	phyaddr;
	volatile uint *mdio_pdir;
	volatile uint *mdio_ppar;
	volatile uint *mdio_psor;
	volatile uint *mdio_podr;
	volatile uint *mdio_pdat;
} fcc_info_t;

static fcc_info_t fcc_ports[] = {
#ifdef CONFIG_FCC1_ENET
	{ 0, CPM_CR_FCC1_SBLOCK, CPM_CR_FCC1_PAGE, PROFF_FCC1, SIU_INT_FCC1,
		(PC_F1RXCLK | PC_F1TXCLK), CMX1_CLK_ROUTE, CMX1_CLK_MASK,
		PC_MDIO, PC_MDCK, F1PHYADDR },
#endif
#ifdef CONFIG_FCC2_ENET
	{ 1, CPM_CR_FCC2_SBLOCK, CPM_CR_FCC2_PAGE, PROFF_FCC2, SIU_INT_FCC2,
		(PC_F2RXCLK | PC_F2TXCLK), CMX2_CLK_ROUTE, CMX2_CLK_MASK,
#ifdef CONFIG_PPC_SONOS
		PC_MDIO, PC_MDCK2, F2PHYADDR },
#else
		PC_MDIO, PC_MDCK, F2PHYADDR },
#endif
#endif
#ifdef CONFIG_FCC3_ENET
	{ 2, CPM_CR_FCC3_SBLOCK, CPM_CR_FCC3_PAGE, PROFF_FCC3, SIU_INT_FCC3,
		(PC_F3RXCLK | PC_F3TXCLK), CMX3_CLK_ROUTE, CMX3_CLK_MASK,
		PC_MDIO, PC_MDCK, F3PHYADDR },
#endif
};

/* The FCC buffer descriptors track the ring buffers.  The rx_bd_base and
 * tx_bd_base always point to the base of the buffer descriptors.  The
 * cur_rx and cur_tx point to the currently available buffer.
 * The dirty_tx tracks the current buffer that is being sent by the
 * controller.  The cur_tx and dirty_tx are equal under both completely
 * empty and completely full conditions.  The empty/ready indicator in
 * the buffer descriptor determines the actual condition.
 */
struct fcc_enet_private {
	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	struct	sk_buff* tx_skbuff[TX_RING_SIZE];
	struct	sk_buff* rx_skbuff[RX_RING_SIZE];
	ushort	skb_cur;
	ushort	skb_dirty;

	atomic_t n_pkts;	/* Number of packets in tx ring */

	/* CPM dual port RAM relative addresses.
	*/
	cbd_t	*rx_bd_base;		/* Address of Rx and Tx buffers. */
	cbd_t	*tx_bd_base;
	cbd_t	*cur_rx, *cur_tx;		/* The next free ring entry */
	cbd_t	*dirty_tx;	/* The ring entries to be free()ed. */
	volatile fcc_t	*fccp;
	volatile fcc_enet_t	*ep;
	struct	net_device_stats stats;
	uint	tx_full;
	spinlock_t lock;
	uint	phy_address;
	uint	phy_type;
	uint	phy_duplex;
	fcc_info_t	*fip;
	volatile fcce_t *fccep;
	int linkstate;
	struct timer_list phytimer;
};

#define LINKSTATE_NOCARRIER 0
#define LINKSTATE_10HD 1
#define LINKSTATE_10FD 2
#define LINKSTATE_100HD 3
#define LINKSTATE_100FD 4
#define LINKSTATE_INITIAL 5

static void init_fcc_shutdown(fcc_info_t *fip, struct fcc_enet_private *cep,
	volatile immap_t *immap);
static void init_fcc_startup(fcc_info_t *fip, struct net_device *dev);
static void init_fcc_ioports(fcc_info_t *fip, volatile iop8260_t *io,
	volatile immap_t *immap);
static void init_fcc_param(fcc_info_t *fip, struct net_device *dev,
	volatile immap_t *immap);

/* MII processing.  We keep this as simple as possible.  Requests are
 * placed on the list (if there is room).  When the request is finished
 * by the MII, an optional function may be called.
 */
typedef struct mii_list {
	uint	mii_regval;
	void	(*mii_func)(uint val, struct net_device *dev);
	struct	mii_list *mii_next;
} mii_list_t;

#define		NMII	20
mii_list_t	mii_cmds[NMII];
mii_list_t	*mii_free;
mii_list_t	*mii_head;
mii_list_t	*mii_tail;

static	int	phyaddr;
static	uint	phytype;

static int	mii_queue(int request, void (*func)(uint, struct net_device *));
static void	mii_startup_cmds(void);
static uint	mii_send_receive(fcc_info_t *fip, uint cmd);

/* Make MII read/write commands for the FCC.
*/

#define mk_mii_phyaddr(ADDR)	(0x60020000 | ((ADDR) << 23) | (2 << 18))

#define mk_mii_read(REG)	(0x60020000 | ((phyaddr << 23) | \
						(REG & 0x1f) << 18))

#define mk_mii_write(REG, VAL)	(0x50020000 | ((phyaddr << 23) | \
						(REG & 0x1f) << 18) | \
						(VAL & 0xffff))


static int
fcc_enet_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;					/* Always succeed */
}

static int
fcc_enet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fcc_enet_private *cep = (struct fcc_enet_private *)dev->priv;
	volatile cbd_t	*bdp;
	int idx;


	/* Fill in a Tx ring entry */
	bdp = cep->cur_tx;

	/* Clear all of the status flags.
	 */
	bdp->cbd_sc &= ~BD_ENET_TX_STATS;

	/* If the frame is short, tell CPM to pad it.
	*/
	if (skb->len <= ETH_ZLEN)
		bdp->cbd_sc |= BD_ENET_TX_PAD;
	else
		bdp->cbd_sc &= ~BD_ENET_TX_PAD;

	/* Set buffer length and buffer pointer.
	*/
	bdp->cbd_datlen = skb->len;
	bdp->cbd_bufaddr = __pa(skb->data);

	spin_lock_irq(&cep->lock);

	/* Save skb pointer.
	*/
	idx = cep->skb_cur & TX_RING_MOD_MASK;
	if (cep->tx_skbuff[idx]) {
		/* This should never happen (any more).
		    Leave the sanity check in for now... */
		printk(KERN_ERR "EEP. cep->tx_skbuff[%d] is %p not NULL in %s\n", 
		idx, cep->tx_skbuff[idx], __func__);
		printk(KERN_ERR "Expect to lose %d bytes of sock space", 
			cep->tx_skbuff[idx]->truesize);
	}
	cep->tx_skbuff[idx] = skb;

	cep->stats.tx_bytes += skb->len;
	cep->skb_cur++;

	atomic_inc(&cep->n_pkts);

	/* Send it on its way.  Tell CPM its ready, interrupt when done,
	 * its the last BD of the frame, and to put the CRC on the end.
	 */
	bdp->cbd_sc |= (BD_ENET_TX_READY | BD_ENET_TX_INTR | BD_ENET_TX_LAST | BD_ENET_TX_TC);

#if 0
	/* Errata says don't do this.
	*/
	cep->fccp->fcc_ftodr = 0x8000;
#endif
	dev->trans_start = jiffies;

	/* If this was the last BD in the ring, start at the beginning again.
	*/
	if (bdp->cbd_sc & BD_ENET_TX_WRAP)
		bdp = cep->tx_bd_base;
	else
		bdp++;

	/* If the tx_ring is full, stop the queue */
	if (atomic_read(&cep->n_pkts) >= (TX_RING_SIZE-1)) {
	  if (!netif_queue_stopped(dev)) {
		netif_stop_queue(dev);    
		cep->tx_full = 1;
	  }
	}

	cep->cur_tx = (cbd_t *)bdp;

	spin_unlock_irq(&cep->lock);

	return 0;
}


static void
fcc_enet_timeout(struct net_device *dev)
{
	struct fcc_enet_private *cep = (struct fcc_enet_private *)dev->priv;

	printk("%s: transmit timed out.\n", dev->name);
	cep->stats.tx_errors++;
#ifndef final_version
	{
		int	i;
		cbd_t	*bdp;
		printk(" Ring data dump: cur_tx %p%s cur_rx %p.\n",
		       cep->cur_tx, cep->tx_full ? " (full)" : "",
		       cep->cur_rx);
		bdp = cep->tx_bd_base;
		printk(" Tx @base %p :\n", bdp);
		for (i = 0 ; i < TX_RING_SIZE; i++, bdp++)
			printk("%04x %04x %08x\n",
			       bdp->cbd_sc,
			       bdp->cbd_datlen,
			       bdp->cbd_bufaddr);
		bdp = cep->rx_bd_base;
		printk(" Rx @base %p :\n", bdp);
		for (i = 0 ; i < RX_RING_SIZE; i++, bdp++)
			printk("%04x %04x %08x\n",
			       bdp->cbd_sc,
			       bdp->cbd_datlen,
			       bdp->cbd_bufaddr);
	}
#endif
	if (!cep->tx_full)
		netif_wake_queue(dev);
}

/* The interrupt handler.
 */
static void
fcc_enet_interrupt(int irq, void * dev_id, struct pt_regs * regs)
{
	struct	net_device *dev = dev_id;
	volatile struct	fcc_enet_private *cep;
	volatile cbd_t	*bdp;
	ushort	int_events;
	int	must_restart;
	int idx;

	cep = (struct fcc_enet_private *)dev->priv;

	/* Get the interrupt events that caused us to be here.
	*/
	int_events = cep->fccp->fcc_fcce;
	cep->fccp->fcc_fcce = int_events;
	must_restart = 0;

	if (int_events & FCC_ENET_GRA) {
            netif_carrier_off(dev);
            printk("%s: Transmitter graceful stop complete\n",dev->name);
        }
       /* Handle receive event in its own function.
	*/
	if (int_events & FCC_ENET_RXF)
		fcc_enet_rx(dev_id);

	/* Check for a transmit error.  The manual is a little unclear
	 * about this, so the debug code until I get it figured out.  It
	 * appears that if TXE is set, then TXB is not set.  However,
	 * if carrier sense is lost during frame transmission, the TXE
	 * bit is set, "and continues the buffer transmission normally."
	 * I don't know if "normally" implies TXB is set when the buffer
	 * descriptor is closed.....trial and error :-).
	 */

	/* Transmit OK, or non-fatal error.  Update the buffer descriptors.
	*/
	if (int_events & (FCC_ENET_TXE | FCC_ENET_TXB)) {
	    spin_lock(&cep->lock);
	    bdp = cep->dirty_tx;
	    while ((bdp->cbd_sc&BD_ENET_TX_READY)==0) {
		if ((bdp==cep->cur_tx) && (cep->tx_full == 0))
		    break;

		if (bdp->cbd_sc & BD_ENET_TX_HB)	/* No heartbeat */
			cep->stats.tx_heartbeat_errors++;
		if (bdp->cbd_sc & BD_ENET_TX_LC)	/* Late collision */
			cep->stats.tx_window_errors++;
		if (bdp->cbd_sc & BD_ENET_TX_RL)	/* Retrans limit */
			cep->stats.tx_aborted_errors++;
		if (bdp->cbd_sc & BD_ENET_TX_UN)	/* Underrun */
			cep->stats.tx_fifo_errors++;
		if (bdp->cbd_sc & BD_ENET_TX_CSL)	/* Carrier lost */
			cep->stats.tx_carrier_errors++;


		/* No heartbeat or Lost carrier are not really bad errors.
		 * The others require a restart transmit command.
		 */
		if (bdp->cbd_sc &
		    (BD_ENET_TX_LC | BD_ENET_TX_RL | BD_ENET_TX_UN)) {
			must_restart = 1;
			cep->stats.tx_errors++;
		}

		cep->stats.tx_packets++;

		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (bdp->cbd_sc & BD_ENET_TX_DEF)
			cep->stats.collisions++;

		/* Free the sk buffer associated with this last transmit. */
		idx = cep->skb_dirty & TX_RING_MOD_MASK;
		dev_kfree_skb_irq(cep->tx_skbuff[idx]);
		cep->tx_skbuff[idx] = NULL;
		cep->skb_dirty++;

		atomic_dec((atomic_t *)&cep->n_pkts);

		/* Update pointer to next buffer descriptor to be transmitted.
		*/
		if (bdp->cbd_sc & BD_ENET_TX_WRAP)
			bdp = cep->tx_bd_base;
		else
			bdp++;

		/* I don't know if we can be held off from processing these
		 * interrupts for more than one frame time.  I really hope
		 * not.  In such a case, we would now want to check the
		 * currently available BD (cur_tx) and determine if any
		 * buffers between the dirty_tx and cur_tx have also been
		 * sent.  We would want to process anything in between that
		 * does not have BD_ENET_TX_READY set.
		 */

		/* Since we have freed up a buffer, the ring is no longer
		 * full.
		 */
		if (cep->tx_full) {
			cep->tx_full = 0;
			if (netif_queue_stopped(dev)) {
				netif_wake_queue(dev);
			}
		}

		cep->dirty_tx = (cbd_t *)bdp;
	    }

	    if ((netif_carrier_ok(dev))&&(must_restart)) {
		volatile cpm8260_t *cp;

		/* Some transmit errors cause the transmitter to shut
		 * down.  We now issue a restart transmit.  Since the
		 * errors close the BD and update the pointers, the restart
		 * _should_ pick up without having to reset any of our
		 * pointers either.
		 */

		cp = cpmp;
		cp->cp_cpcr =
		    mk_cr_cmd(cep->fip->fc_cpmpage, cep->fip->fc_cpmblock,
		    		0x0c, CPM_CR_RESTART_TX) | CPM_CR_FLG;
		while (cp->cp_cpcr & CPM_CR_FLG);
	    }
	    spin_unlock(&cep->lock);
	}

	/* Check for receive busy, i.e. packets coming but no place to
	 * put them.
	 */
	if (int_events & FCC_ENET_BSY) {
		cep->stats.rx_dropped++;
	}
	return;
}

/* During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static int
fcc_enet_rx(struct net_device *dev)
{
    struct  fcc_enet_private *cep = 
                    (struct fcc_enet_private *)dev->priv;
    volatile cbd_t  *bdp;
    struct sk_buff  *skb;
    struct sk_buff  *newskb;

    ushort           indx;
    ushort           pkt_cnt = 0;
    ushort           pktlen;
    ushort           bdSc;


    /*
     * First, grab all of the stats for the incoming packet.
     * These get messed up if we get called due to a busy condition.
     */
    bdp = cep->cur_rx;

    while( ((bdSc = bdp->cbd_sc) & BD_ENET_RX_EMPTY) == 0 ) {

        if( ++pkt_cnt >= 100 )  /* no inf loops */
            break;

        indx = bdp - cep->rx_bd_base;

        if ((dev->flags&IFF_UP)==0) {
            goto drop;
        }

        if( bdSc&BD_ENET_RX_ERR ) {

	    /* Frame too long or too short */
            if (bdSc & (BD_ENET_RX_LG | BD_ENET_RX_SH))
		cep->stats.rx_length_errors++;
            if (bdSc & BD_ENET_RX_NO)	/* Frame alignment */
		cep->stats.rx_frame_errors++;
            if (bdSc & BD_ENET_RX_CR)	/* CRC Error */
		cep->stats.rx_crc_errors++;
            if (bdSc & BD_ENET_RX_OV)	/* FIFO overrun */
		cep->stats.rx_crc_errors++;

	    /* Report late collisions as a frame error.
	     * On this error, the BD is closed, but we don't know what we
	     * have in the buffer.  So, just drop this frame on the floor.
	     */
	    if (bdSc & BD_ENET_RX_CL) 
		cep->stats.rx_frame_errors++;

        } else {

            /* Process the incoming frame */
            cep->stats.rx_packets++;
            pktlen = bdp->cbd_datlen - 4;      // dont include FCS
            cep->stats.rx_bytes += pktlen;

            skb = cep->rx_skbuff[ indx ];
            newskb = dev_alloc_skb( pktlen > BD_RX_CPYBRK ?
                                    PKT_MAXBLR_SIZE : pktlen);

            if (skb != NULL && newskb != NULL) {

                if( pktlen > BD_RX_CPYBRK ) {
                    __skb_put( skb,pktlen );
                    cep->rx_skbuff[indx] = newskb;
                } else {
                    memcpy( __skb_put(newskb,pktlen),skb->tail,pktlen );
                    skb = newskb;
                }

                skb->dev = dev;
                skb->protocol = eth_type_trans( skb,dev );
                netif_rx( skb );

            } else {
                /* Here, certainly at least one of skb or newskb is NULL -
                 * skb is null should be impossible, that means a null wound
                 * up on the ring somehow.  newskb is null means that the
                 * dev_alloc_skb above failed, which is certainly possible -
                 * in which case we just drop the incoming frame and go on
                 * with life
                 */
                if (skb == NULL) {
                    printk("%s: Internal error - null pointer on receive ring\n", dev->name);
                    /* If newskb is not null here, in theory should free
                     * but at this point we're very hosed anyhow so I will
                     * not bother.
                     */
                }
                if (newskb == NULL) {
                    printk("%s: Memory squeeze, dropping packet.\n", dev->name);
                    cep->stats.rx_dropped++;
                }
            }

	}

drop:
	/* Clear the status flags for this buffer.  */
	bdp->cbd_sc &= ~BD_ENET_RX_STATS;
        /* Absolutely always reset cbd_bufaddr and cbd_datlen before
         * handing descriptor back to CPM
         */
        bdp->cbd_bufaddr = __pa(cep->rx_skbuff[indx]->tail);
        bdp->cbd_datlen = 0;

	/* assure cbd_bufaddr update completes before setting BD_ENET_RX_EMPTY*/
	wmb();
	/* Mark the buffer empty.  */
	bdp->cbd_sc |= BD_ENET_RX_EMPTY;

	/* Update BD pointer to next entry.  */
        if(bdSc&BD_ENET_RX_WRAP) bdp = cep->rx_bd_base; else ++bdp;
   }
    cep->cur_rx = (cbd_t *)bdp;
    return 0;
}

static int
fcc_enet_close(struct net_device *dev)
{
	/* Don't know what to do yet.
	*/
	netif_stop_queue(dev);

	return 0;
}

static struct net_device_stats *fcc_enet_get_stats(struct net_device *dev)
{
	struct fcc_enet_private *cep = (struct fcc_enet_private *)dev->priv;

	return &cep->stats;
}

#if 0

/* The MII is simulated from the 8xx FEC implementation.  The FCC
 * is not responsible for the MII control/status interface.
 */
static void
fcc_enet_mii(struct net_device *dev)
{
	struct	fcc_enet_private *fep;
	mii_list_t	*mip;
	uint		mii_reg;

	fep = (struct fcc_enet_private *)dev->priv;
#if 0
	ep = &(((immap_t *)IMAP_ADDR)->im_cpm.cp_fec);
	mii_reg = ep->fec_mii_data;
#endif

	if ((mip = mii_head) == NULL) {
		printk("MII and no head!\n");
		return;
	}

	if (mip->mii_func != NULL)
		(*(mip->mii_func))(mii_reg, dev);

	mii_head = mip->mii_next;
	mip->mii_next = mii_free;
	mii_free = mip;

#if 0
	if ((mip = mii_head) != NULL)
		ep->fec_mii_data = mip->mii_regval;
#endif
}

static int
mii_queue(int regval, void (*func)(uint, struct net_device *))
{
	unsigned long	flags;
	mii_list_t	*mip;
	int		retval;

	retval = 0;

	save_flags(flags);
	cli();

	if ((mip = mii_free) != NULL) {
		mii_free = mip->mii_next;
		mip->mii_regval = regval;
		mip->mii_func = func;
		mip->mii_next = NULL;
		if (mii_head) {
			mii_tail->mii_next = mip;
			mii_tail = mip;
		}
		else {
			mii_head = mii_tail = mip;
#if 0
			(&(((immap_t *)IMAP_ADDR)->im_cpm.cp_fec))->fec_mii_data = regval;
#endif
		}
	}
	else {
		retval = 1;
	}

	restore_flags(flags);

	return(retval);
}

static	volatile uint	full_duplex;

static void
mii_status(uint mii_reg, struct net_device *dev)
{
	volatile uint	prev_duplex;

	if (((mii_reg >> 18) & 0x1f) == 1) {
		/* status register.
		*/
		printk("fec: ");
		if (mii_reg & 0x0004)
			printk("link pu");
		else
			printk("link down");

		if (mii_reg & 0x0010)
			printk(",remote fault");
		if (mii_reg & 0x0020)
			printk(",auto complete");
		printk("\n");
	}
	if (((mii_reg >> 18) & 0x1f) == 0x14) {
		/* Extended chip status register.
		*/
		prev_duplex = full_duplex;
		printk("fec: ");
		if (mii_reg & 0x0800)
			printk("100 Mbps");
		else
			printk("10 Mbps");

		if (mii_reg & 0x1000) {
			printk(", Full-Duplex\n");
			full_duplex = 1;
		}
		else {
			printk(", Half-Duplex\n");
			full_duplex = 0;
		}
#if 0
		if (prev_duplex != full_duplex)
			restart_fec(dev);
#endif
	}
	if (((mii_reg >> 18) & 0x1f) == 31) {
		/* QS6612 PHY Control/Status.
		 * OK, now we have it all, so figure out what is going on.
		 */
		prev_duplex = full_duplex;
		printk("fec: ");

		mii_reg = (mii_reg >> 2) & 7;

		if (mii_reg & 1)
			printk("10 Mbps");
		else
			printk("100 Mbps");

		if (mii_reg > 4) {
			printk(", Full-Duplex\n");
			full_duplex = 1;
		}
		else {
			printk(", Half-Duplex\n");
			full_duplex = 0;
		}

#if 0
		if (prev_duplex != full_duplex)
			restart_fec(dev);
#endif
	}
}

static	uint	phyno;

static void
mii_discover_phy3(uint mii_reg, struct net_device *dev)
{
	phytype <<= 16;
	phytype |= (mii_reg & 0xffff);
	printk("fec: Phy @ 0x%x, type 0x%08x\n", phyno, phytype);
	mii_startup_cmds();
}

static void
mii_discover_phy(uint mii_reg, struct net_device *dev)
{
	if (phyno < 32) {
		if ((phytype = (mii_reg & 0xffff)) != 0xffff) {
			phyaddr = phyno;
			mii_queue(mk_mii_read(3), mii_discover_phy3);
		}
		else {
			phyno++;
			mii_queue(mk_mii_phyaddr(phyno), mii_discover_phy);
		}
	}
	else {
		printk("FEC: No PHY device found.\n");
	}
}

static void
mii_discover_phy_poll(fcc_info_t *fip)
{
	uint	rv;
	int	i;

	for (i=0; i<32; i++) {
		rv = mii_send_receive(fip, mk_mii_phyaddr(i));
		if ((phytype = (rv & 0xffff)) != 0xffff) {
			phyaddr = i;
			rv = mii_send_receive(fip, mk_mii_read(3));
			phytype <<= 16;
			phytype |= (rv & 0xffff);
			printk("fec: Phy @ 0x%x, type 0x%08x\n", phyaddr, phytype);
		}
	}
}

static	void
mii_startup_cmds(void)
{

#if 1
	/* Level One PHY.
	*/

	/* Read status registers to clear any pending interrupt.
	*/
	mii_queue(mk_mii_read(1), mii_status);
	mii_queue(mk_mii_read(18), mii_status);

	/* Read extended chip status register.
	*/
	mii_queue(mk_mii_read(0x14), mii_status);

	/* Set default operation of 100-TX....for some reason
	 * some of these bits are set on power up, which is wrong.
	 */
	mii_queue(mk_mii_write(0x13, 0), NULL);

	/* Enable Link status change interrupts.
	*/
	mii_queue(mk_mii_write(0x11, 0x0002), NULL);

	/* Don't advertize Full duplex.
	mii_queue(mk_mii_write(0x04, 0x0021), NULL);
	*/
#endif

}

/* This supports the mii_link interrupt below.
 * We should get called three times.  Once for register 1, once for
 * register 18, and once for register 20.
 */
static	uint mii_saved_reg1;

static void
mii_relink(uint mii_reg, struct net_device *dev)
{
	volatile uint	prev_duplex;
	unsigned long	flags;

	if (((mii_reg >> 18) & 0x1f) == 1) {
		/* Just save the status register and get out.
		*/
		mii_saved_reg1 = mii_reg;
		return;
	}
	if (((mii_reg >> 18) & 0x1f) == 18) {
		/* Not much here, but has to be read to clear the
		 * interrupt condition.
		 */
		if ((mii_reg & 0x8000) == 0)
			printk("fec: re-link and no IRQ?\n");
		if ((mii_reg & 0x4000) == 0)
			printk("fec: no PHY power?\n");
	}
	if (((mii_reg >> 18) & 0x1f) == 20) {
		/* Extended chip status register.
		 * OK, now we have it all, so figure out what is going on.
		 */
		prev_duplex = full_duplex;
		printk("fec: ");
		if (mii_saved_reg1 & 0x0004)
			printk("link up");
		else
			printk("link down");

		if (mii_saved_reg1 & 0x0010)
			printk(", remote fault");
		if (mii_saved_reg1 & 0x0020)
			printk(", auto complete");

		if (mii_reg & 0x0800)
			printk(", 100 Mbps");
		else
			printk(", 10 Mbps");

		if (mii_reg & 0x1000) {
			printk(", Full-Duplex\n");
			full_duplex = 1;
		}
		else {
			printk(", Half-Duplex\n");
			full_duplex = 0;
		}
		if (prev_duplex != full_duplex) {
			save_flags(flags);
			cli();
#if 0
			restart_fec(dev);
#endif
			restore_flags(flags);
		}
	}
	if (((mii_reg >> 18) & 0x1f) == 31) {
		/* QS6612 PHY Control/Status.
		 * OK, now we have it all, so figure out what is going on.
		 */
		prev_duplex = full_duplex;
		printk("fec: ");
		if (mii_saved_reg1 & 0x0004)
			printk("link up");
		else
			printk("link down");

		if (mii_saved_reg1 & 0x0010)
			printk(", remote fault");
		if (mii_saved_reg1 & 0x0020)
			printk(", auto complete");

		mii_reg = (mii_reg >> 2) & 7;

		if (mii_reg & 1)
			printk(", 10 Mbps");
		else
			printk(", 100 Mbps");

		if (mii_reg > 4) {
			printk(", Full-Duplex\n");
			full_duplex = 1;
		}
		else {
			printk(", Half-Duplex\n");
			full_duplex = 0;
		}

#if 0
		if (prev_duplex != full_duplex) {
			save_flags(flags);
			cli();
			restart_fec(dev);
			restore_flags(flags);
		}
#endif
	}
}
#endif /*0*/

/* Set or clear the multicast filter for this adaptor.
 * Skeleton taken from sunlance driver.
 * The CPM Ethernet implementation allows Multicast as well as individual
 * MAC address filtering.  Some of the drivers check to make sure it is
 * a group multicast address, and discard those that are not.  I guess I
 * will do the same for now, but just remove the test if you want
 * individual filtering as well (do the upper net layers want or support
 * this kind of feature?).
 */
static void
set_multicast_list(struct net_device *dev)
{
	struct	fcc_enet_private *cep;
	struct	dev_mc_list *dmi;
	u_char	*mcptr, *tdptr;
	volatile fcc_enet_t *ep;
	int	i, j;

	cep = (struct fcc_enet_private *)dev->priv;
	spin_lock_irq(&cep->lock);

	/* Get pointer to FCC area in parameter RAM.
	*/
	ep = (fcc_enet_t *)dev->base_addr;

	cep->fccp->fcc_gfmr &= (~FCC_GFMR_ENR);

	if (dev->flags&IFF_PROMISC) {

		/* Log any net taps. */
		printk("%s: Promiscuous mode enabled.\n", dev->name);
		cep->fccp->fcc_fpsmr |= FCC_PSMR_PRO;
	} else {

		cep->fccp->fcc_fpsmr &= ~FCC_PSMR_PRO;

		if (dev->flags & IFF_ALLMULTI) {
			/* Catch all multicast addresses, so set the
			 * filter to all 1's.
			 */
			ep->fen_gaddrh = 0xffffffff;
			ep->fen_gaddrl = 0xffffffff;
		}
		else {
			/* Clear filter and add the addresses in the list.
			*/
			ep->fen_gaddrh = 0;
			ep->fen_gaddrl = 0;

			dmi = dev->mc_list;

			for (i=0; i<dev->mc_count; i++, dmi = dmi->next) {

				/* Only support group multicast for now.
				*/
				if (!(dmi->dmi_addr[0] & 1))
					continue;

				/* The address in dmi_addr is LSB first,
				 * and taddr is MSB first.  We have to
				 * copy bytes MSB first from dmi_addr.
				 */
				mcptr = (u_char *)dmi->dmi_addr + 5;
				tdptr = (u_char *)&ep->fen_taddrh;
				for (j=0; j<6; j++)
					*tdptr++ = *mcptr--;

				/* Ask CPM to run CRC and set bit in
				 * filter mask.
				 */
				cpmp->cp_cpcr = mk_cr_cmd(cep->fip->fc_cpmpage,
						cep->fip->fc_cpmblock, 0x0c,
						CPM_CR_SET_GADDR) | CPM_CR_FLG;
				udelay(10);
				while (cpmp->cp_cpcr & CPM_CR_FLG);
			}
		}
	}
	cpmp->cp_cpcr = mk_cr_cmd(cep->fip->fc_cpmpage,
                                  cep->fip->fc_cpmblock, 0x0c,
                                  CPM_CR_HUNT_MODE) | CPM_CR_FLG;
	udelay(10);
	while (cpmp->cp_cpcr & CPM_CR_FLG) ;
	cep->fccp->fcc_gfmr |= FCC_GFMR_ENR;
	spin_unlock_irq(&cep->lock);
}

static int fcc_enet_set_mac_address(struct net_device *dev, void *p)
{
        struct sockaddr *addr= (struct sockaddr *) p;
        struct fcc_enet_private *cep;
        volatile fcc_enet_t *ep;
        unsigned char *eap;
        int i;

        cep = (struct fcc_enet_private *)(dev->priv);
        ep = cep->ep;

        if (netif_running(dev))
                return -EBUSY;

        memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

        eap = (unsigned char *) &(ep->fen_paddrh);
        for (i=5; i>=0; i--)
                *eap++ = addr->sa_data[i];

        return 0;
}

static void mii_media_eval(struct net_device *dp);
static void mii_io(fcc_info_t *fip,int read,int reg,int *pval);

void fcc_phytimer(unsigned long x)
{
	struct net_device *dp=(struct net_device *)x;
	struct fcc_enet_private *cep=(struct fcc_enet_private *)dp->priv;
	mii_media_eval(dp);
	mod_timer(&cep->phytimer,jiffies+HZ);
}

static int
teridian_proc_read( char *page, char **start, off_t off, int count, int*eof, void *data)
{
	fcc_info_t *fip = (fcc_info_t *)data;
	int len = 0, reg;
	*eof = 1;
	if (off != 0) {
		return 0;
	}

	for (reg = 0; reg <= 24; reg++) {
		int val;
		mii_io(fip, 1, reg, &val);
		len += sprintf(page+len, "%04x ", val);
		if ((reg % 16) == 15) {
			len += sprintf(page+len, "\n");
		}
		else if ((reg % 8) == 7) {
			len += sprintf(page+len, "  ");
		}
	}
	len += sprintf(page+len, "\n");
	
	return len;
}

static int
teridian_proc_write(struct file *file, const char * buffer,
					unsigned long count, void *data)
{
	fcc_info_t *fip = (fcc_info_t *)data;
	char buf[20];
	char *peq;
	int result;
	long longtemp;
	int  regnum;
	int  val;

	/* The following command writes register RR with the hex value VV. The register number RR 
	   must be a 2-digit hex number and the value VVVV a 4-digit hex value. */
	/* echo regRR=VVVV > /proc/driver/phyN */

	if (count >= 20) {
		result = -EIO;
	}
	else {
		memcpy(buf, buffer, count);
		buf[count] = '\0';
		peq = strchr(buf, '=');
		if (peq != NULL) {
			*peq = 0;
			longtemp = simple_strtol(peq+1, NULL, 16);
			val = longtemp;
			if (strncmp(buf, "reg", 3) == 0) {
				longtemp = simple_strtol(peq-2, NULL, 16);
				regnum = longtemp;
				if (regnum <= 24) {
					mii_io(fip, 0, regnum, &val);
				}
				else {
					printk("%s: register num (0x%02x) too large\n", __func__, regnum);
				}
			}
		}

		result = count;
	}

	return result;
}

void fcc_init_proc_file(int port, fcc_info_t *fip)
{
	struct proc_dir_entry *proc;
	char name[40];

	sprintf(name, "driver/phy%d", port);
    proc = create_proc_read_entry(name, (S_IRUSR | S_IWUSR), 0, teridian_proc_read, fip);
    if (proc == NULL) {
		printk("Unable to create %s proc file\n", name);
        return;
    }
    proc->write_proc = (write_proc_t *) teridian_proc_write;
}	

/* Initialize the CPM Ethernet on FCC.
 */
int __init fec_enet_init(void)
{
	struct net_device *dev;
	struct fcc_enet_private *cep;
	fcc_info_t	*fip;
	int	i, np, port;
	volatile	immap_t		*immap;
	volatile	iop8260_t	*io;
	int v1,v2;

	immap = (immap_t *)IMAP_ADDR;	/* and to internal registers */
	io = &immap->im_ioport;

	np = sizeof(fcc_ports) / sizeof(fcc_info_t);
	fip = fcc_ports;

	port = 0;
	while (np-- > 0) {

#ifdef CONFIG_PPC_SONOS
		fip->mdio_pdir= &(io->iop_pdird);
		fip->mdio_ppar= &(io->iop_ppard);
		fip->mdio_psor= &(io->iop_psord);
		fip->mdio_podr= &(io->iop_podrd);
		fip->mdio_pdat= &(io->iop_pdatd);
#else
                fip->mdio_pdir= &(io->iop_pdirc);
                fip->mdio_ppar= &(io->iop_pparc);
                fip->mdio_psor= &(io->iop_psorc);
                fip->mdio_podr= &(io->iop_podrc);
                fip->mdio_pdat= &(io->iop_pdatc);
#endif
		/* Allocate some private information.
		*/
		cep = (struct fcc_enet_private *)
					kmalloc(sizeof(*cep), GFP_KERNEL);
		if (cep == NULL)
			return -ENOMEM;

		__clear_user(cep,sizeof(*cep));
		spin_lock_init(&cep->lock);
		init_timer(&cep->phytimer);
		cep->phytimer.function=fcc_phytimer;
		cep->linkstate=LINKSTATE_INITIAL;
		cep->fip = fip;

		/* Create an Ethernet device instance.
		*/
		dev = init_etherdev(0, 0);
		dev->priv = cep;
		cep->phytimer.data=(unsigned long)dev;

		init_fcc_shutdown(fip, cep, immap);
		init_fcc_ioports(fip, io, immap);
		init_fcc_param(fip, dev, immap);

		dev->base_addr = (unsigned long)(cep->ep);

		/* The CPM Ethernet specific entries in the device
		 * structure.
		 */
		dev->open = fcc_enet_open;
		dev->hard_start_xmit = fcc_enet_start_xmit;
		dev->tx_timeout = fcc_enet_timeout;
		dev->watchdog_timeo = TX_TIMEOUT;
		dev->stop = fcc_enet_close;
		dev->get_stats = fcc_enet_get_stats;
		dev->set_multicast_list = set_multicast_list;
		dev->set_mac_address = fcc_enet_set_mac_address;
                dev->do_ioctl = fcc_enet_ioctl;                
		netif_carrier_off(dev);

		init_fcc_startup(fip, dev);

		printk("%s: FCC ENET Version 0.2, ", dev->name);
		for (i=0; i<5; i++)
			printk("%02x:", dev->dev_addr[i]);
		printk("%02x\n", dev->dev_addr[5]);

		/* This is just a hack for now that works only on the EST
		 * board, or anything else that has MDIO/CK configured.
		 * It is mainly to test the MII software clocking.
		 */
		/*mii_discover_phy_poll(fip);*/
		mii_io(fip,1,2,&v1);
		mii_io(fip,1,3,&v2);
		printk("%s: PHY ID is %04x%04x\n",dev->name,v1,v2);
		mii_io(fip,1,0,&v1);
		mii_io(fip,1,1,&v2);
		printk("%s: PHY control %04x status %04x\n",dev->name,v1,v2);
		fcc_phytimer((unsigned long)dev);

		fcc_init_proc_file(port, fip);
		port++;

		fip++;
	}

	return 0;
}

/* Make sure the device is shut down during initialization.
*/
static void __init
init_fcc_shutdown(fcc_info_t *fip, struct fcc_enet_private *cep,
						volatile immap_t *immap)
{
	volatile	fcc_enet_t	*ep;
	volatile	fcc_t		*fccp;

	/* Get pointer to FCC area in parameter RAM.
	*/
	ep = (fcc_enet_t *)(&immap->im_dprambase[fip->fc_proff]);

	/* And another to the FCC register area.
	*/
	fccp = (volatile fcc_t *)(&immap->im_fcc[fip->fc_fccnum]);
	cep->fccp = fccp;		/* Keep the pointers handy */
	cep->ep = ep;
	cep->fccep = (volatile fcce_t *)(&immap->im_fcce[fip->fc_fccnum]);

	/* Disable receive and transmit in case someone left it running.
	*/
	fccp->fcc_gfmr &= ~(FCC_GFMR_ENR | FCC_GFMR_ENT);
}

/* Initialize the I/O pins for the FCC Ethernet.
*/
static void __init
init_fcc_ioports(fcc_info_t *fip, volatile iop8260_t *io,
						volatile immap_t *immap)
{

	/* FCC1 pins are on port A/C.  FCC2/3 are port B/C.
	*/
	if (fip->fc_proff == PROFF_FCC1) {
		/* Configure port A and C pins for FCC1 Ethernet.
		 */
		io->iop_pdira &= ~PA1_DIRA0;
		io->iop_pdira |= PA1_DIRA1;
		io->iop_psora &= ~PA1_PSORA0;
		io->iop_psora |= PA1_PSORA1;
		io->iop_ppara |= (PA1_DIRA0 | PA1_DIRA1);
	}
	if (fip->fc_proff == PROFF_FCC2) {
		/* Configure port B and C pins for FCC Ethernet.
		 */
		io->iop_pdirb &= ~PB2_DIRB0;
		io->iop_pdirb |= PB2_DIRB1;
		io->iop_psorb &= ~PB2_PSORB0;
		io->iop_psorb |= PB2_PSORB1;
		io->iop_pparb |= (PB2_DIRB0 | PB2_DIRB1);
	}
#ifndef CONFIG_8272
	if (fip->fc_proff == PROFF_FCC3) {
		/* Configure port B and C pins for FCC Ethernet.
		 */
		io->iop_pdirb &= ~PB3_DIRB0;
		io->iop_pdirb |= PB3_DIRB1;
		io->iop_psorb &= ~PB3_PSORB0;
		io->iop_psorb |= PB3_PSORB1;
		io->iop_pparb |= (PB3_DIRB0 | PB3_DIRB1);
	}
#endif

	/* Port C has clocks......
	*/
	io->iop_psorc &= ~(fip->fc_trxclocks);
	io->iop_pdirc &= ~(fip->fc_trxclocks);
	io->iop_pparc |= fip->fc_trxclocks;

	/* ....and the MII serial clock/data.
	*/
	*(fip->mdio_pdat) |= (fip->fc_mdio | fip->fc_mdck);
	*(fip->mdio_pdir) |= (fip->fc_mdio | fip->fc_mdck);
	*(fip->mdio_ppar) &= ~(fip->fc_mdio | fip->fc_mdck);
#ifndef CONFIG_PPC_SONOS
	*(fip->mdio_podr) |= fip->fc_mdio;
#endif

	/* Configure Serial Interface clock routing.
	 * First, clear all FCC bits to zero,
	 * then set the ones we want.
	 */
	immap->im_cpmux.cmx_fcr &= ~(fip->fc_clockmask);
	immap->im_cpmux.cmx_fcr |= fip->fc_clockroute;
}

static void __init
init_fcc_param(fcc_info_t *fip, struct net_device *dev,
						volatile immap_t *immap)
{
	unsigned char	*eap;
	unsigned long	mem_addr;
	bd_t		*bd;
	int		i;
        struct          sk_buff *skb;
	struct		fcc_enet_private *cep;
	volatile	fcc_enet_t	*ep;
	volatile	cbd_t		*bdp;
	volatile	cpm8260_t	*cp;

	cep = (struct fcc_enet_private *)(dev->priv);
	ep = cep->ep;
	cp = cpmp;

	bd = (bd_t *)__res;

	/* Zero the whole thing.....I must have missed some individually.
	 * It works when I do this.
	 */
	memset((char *)ep, 0, sizeof(fcc_enet_t));

	/* Allocate space for the buffer descriptors in the DP ram.
	 * These are relative offsets in the DP ram address space.
	 * Initialize base addresses for the buffer descriptors.
	 */
#if 0
	/* I really want to do this, but for some reason it doesn't
	 * work with the data cache enabled, so I allocate from the
	 * main memory instead.
	 */
	i = m8260_cpm_dpalloc(sizeof(cbd_t) * RX_RING_SIZE, 8);
	ep->fen_genfcc.fcc_rbase = (uint)&immap->im_dprambase[i];
	cep->rx_bd_base = (cbd_t *)&immap->im_dprambase[i];

	i = m8260_cpm_dpalloc(sizeof(cbd_t) * TX_RING_SIZE, 8);
	ep->fen_genfcc.fcc_tbase = (uint)&immap->im_dprambase[i];
	cep->tx_bd_base = (cbd_t *)&immap->im_dprambase[i];
#else
	cep->rx_bd_base = (cbd_t *)m8260_cpm_hostalloc(sizeof(cbd_t) * RX_RING_SIZE, 8);
	ep->fen_genfcc.fcc_rbase = __pa(cep->rx_bd_base);
	cep->tx_bd_base = (cbd_t *)m8260_cpm_hostalloc(sizeof(cbd_t) * TX_RING_SIZE, 8);
	ep->fen_genfcc.fcc_tbase = __pa(cep->tx_bd_base);
#endif

	cep->dirty_tx = cep->cur_tx = cep->tx_bd_base;
	cep->cur_rx = cep->rx_bd_base;

	ep->fen_genfcc.fcc_rstate = (CPMFCR_GBL | CPMFCR_EB) << 24;
	ep->fen_genfcc.fcc_tstate = (CPMFCR_GBL | CPMFCR_EB) << 24;

	/* Set maximum bytes per receive buffer.
	 * It must be a multiple of 32.
	 */
	ep->fen_genfcc.fcc_mrblr = PKT_MAXBLR_SIZE;

	/* Allocate space in the reserved FCC area of DPRAM for the
	 * internal buffers.  No one uses this space (yet), so we
	 * can do this.  Later, we will add resource management for
	 * this area.
	 */
	mem_addr = CPM_FCC_SPECIAL_BASE + (fip->fc_fccnum * 128);
	ep->fen_genfcc.fcc_riptr = mem_addr;
	ep->fen_genfcc.fcc_tiptr = mem_addr+32;
	ep->fen_padptr = mem_addr+64;
	memset((char *)(&(immap->im_dprambase[(mem_addr+64)])), 0x88, 32);

	ep->fen_genfcc.fcc_rbptr = 0;
	ep->fen_genfcc.fcc_tbptr = 0;
	ep->fen_genfcc.fcc_rcrc = 0;
	ep->fen_genfcc.fcc_tcrc = 0;
	ep->fen_genfcc.fcc_res1 = 0;
	ep->fen_genfcc.fcc_res2 = 0;

	ep->fen_camptr = 0;	/* CAM isn't used in this driver */

	/* Set CRC preset and mask.
	*/
	ep->fen_cmask = 0xdebb20e3;
	ep->fen_cpres = 0xffffffff;

	ep->fen_crcec = 0;	/* CRC Error counter */
	ep->fen_alec = 0;	/* alignment error counter */
	ep->fen_disfc = 0;	/* discard frame counter */
	ep->fen_retlim = 15;	/* Retry limit threshold */
	ep->fen_pper = 0;	/* Normal persistence */

	/* Clear hash filter tables.
	*/
	ep->fen_gaddrh = 0;
	ep->fen_gaddrl = 0;
	ep->fen_iaddrh = 0;
	ep->fen_iaddrl = 0;

	/* Clear the Out-of-sequence TxBD.
	*/
	ep->fen_tfcstat = 0;
	ep->fen_tfclen = 0;
	ep->fen_tfcptr = 0;

	ep->fen_mflr = PKT_MAXBUF_SIZE;   /* maximum frame length register */
	ep->fen_minflr = PKT_MINBUF_SIZE;  /* minimum frame length register */

	/* Set Ethernet station address.
	 *
	 * This is supplied in the board information structure, so we
	 * copy that into the controller.
	 * So, far we have only been given one Ethernet address. We make
	 * it unique by setting a few bits in the upper byte of the
	 * non-static part of the address.
	 */
	eap = (unsigned char *)&(ep->fen_paddrh);
	for (i=5; i>=0; i--) {
		if (i == 3) {
			dev->dev_addr[i] = bd->bi_enetaddr[i];
			dev->dev_addr[i] |= (1 << (7 - fip->fc_fccnum));
			*eap++ = dev->dev_addr[i];
		}
		else {
			*eap++ = dev->dev_addr[i] = bd->bi_enetaddr[i];
		}
	}

	ep->fen_taddrh = 0;
	ep->fen_taddrm = 0;
	ep->fen_taddrl = 0;

	ep->fen_maxd1 = PKT_MAXDMA_SIZE;	/* maximum DMA1 length */
	ep->fen_maxd2 = PKT_MAXDMA_SIZE;	/* maximum DMA2 length */

	/* Clear stat counters, in case we ever enable RMON.
	*/
	ep->fen_octc = 0;
	ep->fen_colc = 0;
	ep->fen_broc = 0;
	ep->fen_mulc = 0;
	ep->fen_uspc = 0;
	ep->fen_frgc = 0;
	ep->fen_ospc = 0;
	ep->fen_jbrc = 0;
	ep->fen_p64c = 0;
	ep->fen_p65c = 0;
	ep->fen_p128c = 0;
	ep->fen_p256c = 0;
	ep->fen_p512c = 0;
	ep->fen_p1024c = 0;

	ep->fen_rfthr = 0;	/* Suggested by manual */
	ep->fen_rfcnt = 0;
	ep->fen_cftype = 0;

	/* Now allocate the host memory pages and initialize the
	 * buffer descriptors.
	 */
	bdp = cep->tx_bd_base;
	for (i=0; i<TX_RING_SIZE; i++) {

		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_sc = 0;
		bdp->cbd_datlen = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	bdp = cep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; ++i,++bdp) {

            skb = dev_alloc_skb( PKT_MAXBLR_SIZE );
            if( skb == NULL ) {
                printk( "Couldnt allocate Rx bfr - no mem\n" );
                break;
            }
            cep->rx_skbuff[i] = skb;

            bdp->cbd_sc = BD_ENET_RX_EMPTY | BD_ENET_RX_INTR;
            bdp->cbd_datlen = 0;
            bdp->cbd_bufaddr = __pa(skb->tail);
	}
	wmb(); /*Perhaps not necessary, certainly doesn't hurt*/

	/* Set the last buffer to wrap */
        if( --bdp >= cep->rx_bd_base )
	    bdp->cbd_sc |= BD_SC_WRAP;

	/* Let's re-initialize the channel now.  We have to do it later
	 * than the manual describes because we have just now finished
	 * the BD initialization.
	 */
	cp->cp_cpcr = mk_cr_cmd(fip->fc_cpmpage, fip->fc_cpmblock, 0x0c,
			CPM_CR_INIT_TRX) | CPM_CR_FLG;
	while (cp->cp_cpcr & CPM_CR_FLG);

	cep->skb_cur = cep->skb_dirty = 0;
	atomic_set(&cep->n_pkts, 0);
}

/* Let 'er rip.
*/
static void __init
init_fcc_startup(fcc_info_t *fip, struct net_device *dev)
{
	volatile fcc_t	*fccp;
	struct fcc_enet_private *cep;

	cep = (struct fcc_enet_private *)(dev->priv);
	fccp = cep->fccp;

	fccp->fcc_fcce = 0xffff;	/* Clear any pending events */

	/* Enable interrupts for transmit error, complete frame
	 * received, and any transmit buffer we have also set the
	 * interrupt flag.
	 */
	fccp->fcc_fccm = (FCC_ENET_TXE | FCC_ENET_RXF | FCC_ENET_TXB | FCC_ENET_GRA );

	/* Install our interrupt handler.
	*/
	if (request_irq(fip->fc_interrupt, fcc_enet_interrupt, 0, "fenet",
				dev) < 0)
		printk("Can't get FCC IRQ %d\n", fip->fc_interrupt);

	/* Set GFMR to enable Ethernet operating mode.
	 */
	fccp->fcc_gfmr = (FCC_GFMR_TCI | FCC_GFMR_MODE_ENET);

	/* Set sync/delimiters.
	*/
	fccp->fcc_fdsr = 0xd555;

	/* Set protocol specific processing mode for Ethernet.
	 * This has to be adjusted for Full Duplex operation after we can
	 * determine how to detect that.
	 */
#ifdef CONFIG_MPC8272ADS
	/*XXX BT need a real PHY driver but for now be consistent with UBoot
	 *and assume full duplex, not half
	 */
	fccp->fcc_fpsmr = FCC_PSMR_ENCRC | FCC_PSMR_FDE | FCC_PSMR_LPB;
#else
#ifdef CONFIG_PPC_SONOS
	fccp->fcc_fpsmr = FCC_PSMR_ENCRC | FCC_PSMR_FDE | FCC_PSMR_LPB;
#else
	fccp->fcc_fpsmr = FCC_PSMR_ENCRC;
#endif
#endif

	/* And last, enable the transmit and receive processing.
	*/
	/*fccp->fcc_gfmr |= (FCC_GFMR_ENR | FCC_GFMR_ENT);*/
	/* XXX BT don't actually enable here, the PHY management code
	 * will do that when appropriate
	 */
}

/* next two functions copied verbatim from 8139cp.c */

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

static void fcc_process_link_change(struct net_device *dp,int newstate)
{
	struct fcc_enet_private *cep=(struct fcc_enet_private *)dp->priv;
	volatile cpm8260_t *cp=cpmp;
	int initial=(cep->linkstate==LINKSTATE_INITIAL);
	if (cep->linkstate==newstate) return; /*nothing for this case*/
	if ((newstate==LINKSTATE_NOCARRIER)&&initial) return; /*or this one*/
	spin_lock_irq(&cep->lock);
	if ((cep->linkstate!=LINKSTATE_NOCARRIER)&&(!initial)) {
		/* need to stop */
		cp->cp_cpcr = mk_cr_cmd(cep->fip->fc_cpmpage,
                                  cep->fip->fc_cpmblock, 0x0c,
                                  CPM_CR_GRACEFUL_STOP_TX) | CPM_CR_FLG;
		eieio();
		while (cp->cp_cpcr&CPM_CR_FLG) eieio();
		/* synchronize with interrupt handler */
		spin_unlock_irq(&cep->lock);
		while (netif_carrier_ok(dp)) mb();
		spin_lock_irq(&cep->lock);
		/* transmitter now completely stopped */
		
		cep->fccp->fcc_gfmr &= (~(FCC_GFMR_ENR | FCC_GFMR_ENT));
		eieio();
	}
	cep->linkstate=newstate;
	switch (cep->linkstate) {
		case LINKSTATE_NOCARRIER:
			printk("%s: link down\n",dp->name);
			/*netif_carrier_off(dp);*/
			spin_unlock_irq(&cep->lock);
                        rtmsg_on_mii_new_link(dp); /*down case only*/
			return;
		case LINKSTATE_10HD:
			printk("%s: link up, 10Mbps half-duplex\n",dp->name);
			// cep->fccep->fcce_gfemr |= FCCE_GFEMR_CLK;
			cep->fccp->fcc_fpsmr &= (~(FCC_PSMR_FDE|FCC_PSMR_LPB));
			break;
		case LINKSTATE_10FD:
			printk("%s: link up, 10Mbps full-duplex\n",dp->name);
			// cep->fccep->fcce_gfemr |= FCCE_GFEMR_CLK;
			cep->fccp->fcc_fpsmr |= (FCC_PSMR_FDE|FCC_PSMR_LPB);
			break;
		case LINKSTATE_100HD:
			printk("%s: link up, 100Mbps half-duplex\n",dp->name);
			// cep->fccep->fcce_gfemr &= ~FCCE_GFEMR_CLK;
			cep->fccp->fcc_fpsmr &= (~(FCC_PSMR_FDE|FCC_PSMR_LPB));
			break;
		case LINKSTATE_100FD:
			printk("%s: link up, 100Mbps full-duplex\n",dp->name);
			// cep->fccep->fcce_gfemr &= ~FCCE_GFEMR_CLK;
			cep->fccp->fcc_fpsmr |= (FCC_PSMR_FDE|FCC_PSMR_LPB);
			break;
	}
	eieio();
	if (!initial) {
	cp->cp_cpcr = mk_cr_cmd(cep->fip->fc_cpmpage,
                                cep->fip->fc_cpmblock, 0x0c,
                                CPM_CR_RESTART_TX) | CPM_CR_FLG;
	eieio();
	while (cp->cp_cpcr&CPM_CR_FLG) eieio();
	cp->cp_cpcr = mk_cr_cmd(cep->fip->fc_cpmpage,
                                cep->fip->fc_cpmblock, 0x0c,
                                CPM_CR_HUNT_MODE) | CPM_CR_FLG;
	eieio();
	while (cp->cp_cpcr&CPM_CR_FLG) eieio();
	}
	cep->fccp->fcc_gfmr |= (FCC_GFMR_ENR | FCC_GFMR_ENT);
	eieio();
	netif_carrier_on(dp);
	spin_unlock_irq(&cep->lock);
        rtmsg_on_mii_new_link(dp); /*all the up cases*/
}
	
#define MDIO_IN do { *(fip->mdio_pdir) &= (~(fip->fc_mdio)); } while (0)
#define MDIO_OUT do { *(fip->mdio_pdir) |= (fip->fc_mdio); } while (0)
#define MDIO_HIGH do { *(fip->mdio_pdat) |= (fip->fc_mdio); } while (0)
#define MDIO_LOW do { *(fip->mdio_pdat) &= (~(fip->fc_mdio)); } while (0)
#define MDC_HIGH do { *(fip->mdio_pdat) |= (fip->fc_mdck); } while (0)
#define MDC_LOW do { *(fip->mdio_pdat) &= (~(fip->fc_mdck)); } while (0)
#define MDIO_VAL ((*(fip->mdio_pdat))&(fip->fc_mdio))

#define MII_PREAMBLE /*Not required for the ADM7001, saves considerable time*/
static void mii_io(fcc_info_t *fip,int read,int reg,int *pval)
{
	int x;
	MDIO_OUT;
#ifdef MII_PREAMBLE
	for (x=0;x<31;x++) {
		MDC_LOW;
		MDIO_HIGH;
		ndelay(50);
		MDC_HIGH;
		ndelay(50);
	}
#endif
	MDC_LOW;
	MDIO_HIGH;
	ndelay(50);
	MDC_HIGH; /*1 preamble bit*/
	ndelay(50);
	MDC_LOW;
	MDIO_LOW;
	ndelay(50);
	MDC_HIGH; /*0*/
	ndelay(50);
	MDC_LOW;
	MDIO_HIGH;
	ndelay(50);
	MDC_HIGH; /*1*/
	ndelay(50);
	if (read) {
		MDC_LOW;
		MDIO_HIGH;
		ndelay(50);
		MDC_HIGH; /*1*/
		ndelay(50);
		MDC_LOW;
		MDIO_LOW;
		ndelay(50);
		MDC_HIGH; /*0*/
		ndelay(50);
	} else {
		MDC_LOW;
		MDIO_LOW;
		ndelay(50);
		MDC_HIGH; /*0*/
		ndelay(50);
		MDC_LOW;
		MDIO_HIGH;
		ndelay(50);
		MDC_HIGH; /*1*/
		ndelay(50);
	}
	x=16;
	while (x!=0) {
		MDC_LOW;
		if ((fip->phyaddr)&x) MDIO_HIGH; else MDIO_LOW;
		ndelay(50);
		MDC_HIGH;
		ndelay(50);
		x>>=1;
	}
	x=16;
	while (x!=0) {
		MDC_LOW;
		if (reg&x) MDIO_HIGH; else MDIO_LOW;
		ndelay(50);
		MDC_HIGH;
		ndelay(50);
		x>>=1;
	}
	MDC_LOW;
	if (read) MDIO_IN; else MDIO_HIGH;
	ndelay(50);
	MDC_HIGH;
	ndelay(50);

	if (!read) {
		MDC_LOW;
		MDIO_LOW;
		ndelay(50);
		MDC_HIGH;
		ndelay(50);
	}
	
	if (read) {
		*pval=0;
		x=0x8000;
		while (x!=0) {
			MDC_LOW;
			ndelay(50);
			MDC_HIGH;
			ndelay(50);
			eieio();
			if (MDIO_VAL) (*pval)|=x;
			x>>=1;
		}
	} else {
		x=0x8000;
		while (x!=0) {
			MDC_LOW;
			if (x&(*pval)) MDIO_HIGH; else MDIO_LOW;
			ndelay(50);
			MDC_HIGH;
			ndelay(50);
			x>>=1;
		}
	}
	MDC_LOW;
	MDIO_IN;
	MDIO_LOW;
	ndelay(50);
	MDC_HIGH;
	ndelay(50);
	MDC_LOW;
}

static void mii_media_eval(struct net_device *dp)
{
	struct fcc_enet_private *cep=(struct fcc_enet_private *)dp->priv;
	int rv;
        // int rv2;
	/*XXX BT fcc_process_link_change does its own locking, I don't think
	 *locks are necessary here.  We would only need it if timer handlers
	 *could interrupt other timer handlers or if the management interface
	 *was used from inside the interrupt handler or possibly on MP.
	 */
	/*unsigned long flags;*/
	/*spin_lock_irqsave(&cep->lock,flags);*/
	mii_io(cep->fip,1,1,&rv);
	/* XXX BT for the Teridian PHY in the event that the link partner
	 * can't do capability exchange, the auto-negotiation complete bit
	 * of the status register never sets and the link partner ability
	 * register is never valid.  Because of this, we just check for
	 * link up in the status register and if link up then we check
	 * the diagnostic register (register 18) to see what mode we're
	 * actually operating in.
	 */
	if ((rv&0x04)!=0x04) {
		/* This is either link down or autoneg not complete or
		 * both
		 */
		fcc_process_link_change(dp,LINKSTATE_NOCARRIER);
		return;
	}
#if 0
	/*we're up and the autoneg state is meaningful*/
	mii_io(cep->fip,1,4,&rv);
	mii_io(cep->fip,1,5,&rv2);
	rv&=rv2;
	if (rv&0x100) {
		fcc_process_link_change(dp,LINKSTATE_100FD);
		return;
	}
	if (rv&0x80) {
		fcc_process_link_change(dp,LINKSTATE_100HD);
		return;
	}
	if (rv&0x40) {
		fcc_process_link_change(dp,LINKSTATE_10FD);
		return;
	}
	if (rv&0x20) {
		fcc_process_link_change(dp,LINKSTATE_10HD);
		return;
	}
#else
        mii_io(cep->fip,1,18,&rv);
        if ((rv&0xC00)==0xC00) {
                fcc_process_link_change(dp,LINKSTATE_100FD);
                return;
        }
        if ((rv&0xC00)==0x400) {
                fcc_process_link_change(dp,LINKSTATE_100HD);
                return;
        }
        if ((rv&0xC00)==0x800) {
                fcc_process_link_change(dp,LINKSTATE_10FD);
                return;
        }
        if ((rv&0xC00)==0x000) {
                fcc_process_link_change(dp,LINKSTATE_10HD);
                return;
        }
#endif
}
	

#if 0	
/* MII command/status interface.
 * I'm not going to describe all of the details.  You can find the
 * protocol definition in many other places, including the data sheet
 * of most PHY parts.
 * I wonder what "they" were thinking (maybe weren't) when they leave
 * the I2C in the CPM but I have to toggle these bits......
 */
static uint
mii_send_receive(fcc_info_t *fip, uint cmd)
{
	unsigned long	flags;
	uint		retval;
	int		read_op, i;
	/*volatile	immap_t		*immap;*/
	/*volatile	iop8260_t	*io;*/

	/*immap = (immap_t *)IMAP_ADDR;*/
	/*io = &immap->im_ioport;*/

	/* When we get here, both clock and data are high, outputs.
	 * Output is open drain.
	 * Data transitions on high->low clock, is valid on low->high clock.
	 * Spec says edge transitions no closer than 160 nSec, minimum clock
	 * cycle 400 nSec.  I could only manage about 500 nSec edges with
	 * an XOR loop, so I won't worry about delays yet.
	 * I disable interrupts during bit flipping to ensure atomic
	 * updates of the registers.  I do lots of interrupt disable/enable
	 * to ensure we don't hang out too long with interrupts disabled.
	 */

	/* First, crank out 32 1-bits as preamble.
	 * This is 64 transitions to clock the bits, with clock/data
	 * left high.
	 */
	save_flags(flags);
	cli();
	for (i=0; i<64; i++) {
		*(fip->mdio_pdat) ^= fip->fc_mdck;
		udelay(0);
	}
	restore_flags(flags);

	read_op = ((cmd & 0xf0000000) == 0x60000000);

	/* We return the command word on a write op, or the command portion
	 * plus the new data on a read op.  This is what the 8xx FEC does,
	 * and it allows the functions to simply look at the returned value
	 * and know the PHY/register as well.
	 */
	if (read_op)
		retval = cmd;
	else
		retval = (cmd >> 16);

	/* Clock out the first 16 MS bits of the command.
	*/
	save_flags(flags);
	cli();
	for (i=0; i<16; i++) {
		*(fip->mdio_pdat) &= ~(fip->fc_mdck);
		if (cmd & 0x80000000)
			*(fip->mdio_pdat) |= fip->fc_mdio;
		else
			*(fip->mdio_pdat) &= ~(fip->fc_mdio);
		cmd <<= 1;
		*(fip->mdio_pdat) |= fip->fc_mdck;
		udelay(0);
	}

	/* Do the turn-around.  If read op, we make the IO and input.
	 * If write op, do the 1/0 thing.
	 */
	*(fip->mdio_pdat) &= ~(fip->fc_mdck);
	if (read_op)
		*(fip->mdio_pdir) &= ~(fip->fc_mdio);
	else
		*(fip->mdio_pdat) |= fip->fc_mdio;
	*(fip->mdio_pdat) |= fip->fc_mdck;

	/* I do this mainly to get just a little delay.
	*/
	restore_flags(flags);
	save_flags(flags);
	cli();
	*(fip->mdio_pdat) &= ~(fip->fc_mdck);
	*(fip->mdio_pdir) &= ~(fip->fc_mdio);
	*(fip->mdio_pdat) |= fip->fc_mdck;

	restore_flags(flags);
	save_flags(flags);
	cli();

	/* For read, clock in 16 bits.  For write, clock out
	 * rest of command.
	 */
	if (read_op) {
		*(fip->mdio_pdat) &= ~(fip->fc_mdck);
		udelay(0);
		for (i=0; i<16; i++) {
			*(fip->mdio_pdat) |= fip->fc_mdck;
			udelay(0);
			retval <<= 1;
			if (*(fip->mdio_pdat) & fip->fc_mdio)
				retval |= 1;
			*(fip->mdio_pdat) &= ~(fip->fc_mdck);
			udelay(0);
		}
	}
	else {
		for (i=0; i<16; i++) {
			*(fip->mdio_pdat) &= ~(fip->fc_mdck);
			if (cmd & 0x80000000)
				*(fip->mdio_pdat) |= fip->fc_mdio;
			else
				*(fip->mdio_pdat) &= ~(fip->fc_mdio);
			cmd <<= 1;
			*(fip->mdio_pdat) |= fip->fc_mdck;
			udelay(0);
		}
		*(fip->mdio_pdat) &= ~(fip->fc_mdck);
	}
	restore_flags(flags);

	/* Some diagrams show two 1 bits for "idle".  I don't know if
	 * this is really necessary or if it was just to indicate nothing
	 * is going to happen for a while.
	 * Make the data pin an output, set the data high, and clock it.
	 */
	save_flags(flags);
	cli();
	*(fip->mdio_pdat) |= fip->fc_mdio;
	*(fip->mdio_pdir) |= fip->fc_mdio;
	for (i=0; i<3; i++)
		*(fip->mdio_pdat) ^= fip->fc_mdck;
	restore_flags(flags);

	/* We exit with the same conditions as entry.
	*/
	return(retval);
}
#endif /*0*/

static int fcc_enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	int retval = -EINVAL;

	switch(cmd)
	{
	case SIOCETHTOOL:
		retval = netdev_ethtool_ioctl(dev, (void*)rq->ifr_data);
		break;
		
	default:
		retval = -EOPNOTSUPP;
		break;
	}
	return retval;
}

static int netdev_ethtool_ioctl (struct net_device *dev, void *useraddr)
{
	struct fcc_enet_private *cep=(struct fcc_enet_private *)dev->priv;
	u32 ethcmd;

	/* dev_ioctl() in ../../net/core/dev.c has already checked
	   capable(CAP_NET_ADMIN), so don't bother with that here.  */

	if (copy_from_user (&ethcmd, useraddr, sizeof (ethcmd)))
		return -EFAULT;

	// printk("ethtool: cmd=0x%08x, state=%d\n", ethcmd, cep->linkstate);
	
	switch (ethcmd) {
		
	case ETHTOOL_GLINK:
	{
		struct ethtool_value val = { ETHTOOL_GLINK };
		
		switch (cep->linkstate) {
			
		case LINKSTATE_NOCARRIER:
		case LINKSTATE_INITIAL:
			val.data = 0;
			break;
		default:
			val.data = 1;
			break;
		}
		
		if (copy_to_user (useraddr, &val, sizeof (val)))
			return -EFAULT;
		
		return 0;
	}
	
	case ETHTOOL_GSET:
	{
		struct ethtool_cmd cmd = { ETHTOOL_GSET };
		
		switch (cep->linkstate) {
			
		case LINKSTATE_10HD:
			cmd.speed  = SPEED_10;
			cmd.duplex = DUPLEX_HALF;
			break;
			
		case LINKSTATE_10FD:
			cmd.speed  = SPEED_10;
			cmd.duplex = DUPLEX_FULL;
			break;

		case LINKSTATE_100HD:
			cmd.speed  = SPEED_100;
			cmd.duplex = DUPLEX_HALF;
			break;

		case LINKSTATE_100FD:
			cmd.speed  = SPEED_100;
			cmd.duplex = DUPLEX_FULL;
			break;
			
		default:
			break;
		}
		
		if (copy_to_user (useraddr, &cmd, sizeof (cmd)))
			return -EFAULT;
		
		return 0;		
	}
	break;
	
	default:
		break;
	}

	return -EOPNOTSUPP;
}

