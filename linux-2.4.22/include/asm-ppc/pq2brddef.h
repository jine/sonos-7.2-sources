/*******************************************************************
 * pq2brddef.h 2003/07/13 14:11:13
 *
 * Author: Igor Ternovsky
 * Creation Date: 2003/07/13 14:11:13
 *
 * Copyright (c) 2003 Arabella Software
 *
 * The author may be reached at igort@arabellasw.com.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR   IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *******************************************************************/

/*
 * Board-specific stuff
 */
#ifndef PQ2BRDDEF_H

#define PQ2BRDDEF_H

#include <linux/types.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <asm/pq2defs.h>

/*
 * The only section to be configured in this header file
 * is "configuration section"
 */

/*--------------------------------------------------------*/
/*  Peripherals Flags define what peripherals and RAM     */
/*  micocode extensions are to be used on specific        */
/*  PQUICCII chip. This information will be used in DPRAM */
/*  allocation and usage scheme.                          */
/*--------------------------------------------------------*/
#define PQ_SCC1        0x80000000
#define PQ_SCC2        0x40000000
#define PQ_SCC3        0x20000000
#define PQ_SCC4        0x10000000
#define PQ_FCC1        0x08000000
#define PQ_FCC2        0x04000000
#define PQ_FCC3        0x02000000
#define PQ_MCC1        0x01000000
#define PQ_MCC2        0x00800000
#define PQ_SMC1        0x00400000
#define PQ_SMC2        0x00200000
#define PQ_IDMA1       0x00100000
#define PQ_IDMA2       0x00080000
#define PQ_IDMA3       0x00040000
#define PQ_IDMA4       0x00020000
#define PQ_SPI         0x00010000
#define PQ_I2C         0x00008000
#define PQ_AAL2_PATCH  0x00004000 /* AAL2 SAR micocode RAM patch */
#define PQ_4744_PATCH  0x00002000 /*CPM41 4744 PM microcode patch*/

/*--------------------------------------------------------*/
/*  Parallel Port Flags define the pin configuration      */
/*  used on specific PQUICCII chip. This information is   */
/*  used by various drivers for proper port programming.  */
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
/* FCC2 UTOPIA TxD[0] could be either on PB27 or PB11     */
/* Set the following flag in Parallel Port configuration  */
/* word if PB27 pin is used - otherwise PB11 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_TXD0_ON_PB27  0x80000000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA TxD[1] could be either on PB26 or PB10     */
/* Set the following flag in Parallel Port configuration  */
/* word if PB26 pin is used - otherwise PB10 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_TXD1_ON_PB26  0x40000000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA TxD[2] could be either on PB9, PC25 or PC3 */
/* Set one of the following flags in Port configuration   */
/* word if PB9 or PC25 pin is used - otherwise - PC3 used */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_TXD2_ON_PB9   0x20000000
#define PQ_PP_FCC2_UTOPIA_TXD2_ON_PC25  0x10000000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA TxD[3] could be either on PB8,PC30,PC24,PC2*/
/* Set one of the following flags in Port configuration   */
/* word if PB8,PC30,PC24 pin is used - otherwise- PC2 used*/
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_TXD3_ON_PB8   0x08000000
#define PQ_PP_FCC2_UTOPIA_TXD3_ON_PC30  0x04000000
#define PQ_PP_FCC2_UTOPIA_TXD3_ON_PC24  0x02000000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA RxD[0] could be either on PD11 or PB4      */
/* Set the following flag in Parallel Port configuration  */
/* word if PD11 pin is used - otherwise PB4 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_RXD0_ON_PD11  0x01000000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA RxD[1] could be either on PD10 or PB5      */
/* Set the following flag in Parallel Port configuration  */
/* word if PD10 pin is used - otherwise PB5 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_RXD1_ON_PD10  0x00800000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA RxD[2] could be either on PC11 or PB6      */
/* Set the following flag in Parallel Port configuration  */
/* word if PC11 pin is used - otherwise PB6 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_RXD2_ON_PC11  0x00400000

/*--------------------------------------------------------*/
/* FCC2 UTOPIA RxD[3] could be either on PC10 or PB7      */
/* Set the following flag in Parallel Port configuration  */
/* word if PC10 pin is used - otherwise PB7 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_FCC2_UTOPIA_RXD3_ON_PC10  0x00200000

/*--------------------------------------------------------*/
/* Set the following flag in Parallel Port configuration  */
/* word if CAM interface is used for SCC1 in Ethernet mode*/
/*--------------------------------------------------------*/
#define PQ_PP_SCC1_ETHERNET_CAM         0x00100000

/*--------------------------------------------------------*/
/* Set the following flag in Parallel Port configuration  */
/* word if CAM interface is used for SCC2 in Ethernet mode*/
/*--------------------------------------------------------*/
#define PQ_PP_SCC2_ETHERNET_CAM         0x00080000

/*--------------------------------------------------------*/
/* SCC1 CTS signal could be either on PC15 or PC29        */
/* Set the following flag in Parallel Port configuration  */
/* word if PC29 pin is used - otherwise PC15 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC1_CTS_ON_PC29          0x00040000

/*--------------------------------------------------------*/
/* SCC2 CTS signal could be either on PC13 or PC28        */
/* Set the following flag in Parallel Port configuration  */
/* word if PC28 pin is used - otherwise PC13 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC2_CTS_ON_PC28          0x00020000

/*--------------------------------------------------------*/
/* SCC3 CTS signal could be either on PC11 or PC8         */
/* Set the following flag in Parallel Port configuration  */
/* word if PC8 pin is used - otherwise PC11 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_SCC3_CTS_ON_PC8           0x00010000

/*--------------------------------------------------------*/
/* SCC4 CTS signal could be either on PC9 or PC3          */
/* Set the following flag in Parallel Port configuration  */
/* word if PC3 pin is used - otherwise PC9 is used        */
/*--------------------------------------------------------*/
#define PQ_PP_SCC4_CTS_ON_PC3           0x00008000

/*--------------------------------------------------------*/
/* SCC1 TxD signal could be either on PB28 or PD30        */
/* Set the following flag in Parallel Port configuration  */
/* word if PD30 pin is used - otherwise PB28 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC1_TXD_ON_PD30          0x00004000

/*--------------------------------------------------------*/
/* SCC2 TxD signal could be either on PB12 or PD27        */
/* Set the following flag in Parallel Port configuration  */
/* word if PD27 pin is used - otherwise PB12 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC2_TXD_ON_PD27          0x00002000

/*--------------------------------------------------------*/
/* SCC3 TxD signal could be either on PB8 or PD24         */
/* Set the following flag in Parallel Port configuration  */
/* word if PD24 pin is used - otherwise PB8 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_SCC3_TXD_ON_PD24          0x00001000

/*--------------------------------------------------------*/
/* SCC2 RxD signal could be either on PB15 or PD28        */
/* Set the following flag in Parallel Port configuration  */
/* word if PD28 pin is used - otherwise PB15 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC2_RXD_ON_PD28          0x00000800

/*--------------------------------------------------------*/
/* SCC3 RxD signal could be either on PB14 or PD25        */
/* Set the following flag in Parallel Port configuration  */
/* word if PD25 pin is used - otherwise PB14 is used      */
/*--------------------------------------------------------*/
#define PQ_PP_SCC3_RXD_ON_PD25          0x00000400

/*--------------------------------------------------------*/
/* FCC3 TxD signal could be either on PB7 or PC27         */
/* Set the following flag in Parallel Port configuration  */
/* word if PC27 pin is used - otherwise PB7 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_FCC3_TXD_ON_PC27          0x00000200

/*--------------------------------------------------------*/
/* FCC3 RTS signal could be either on PB4 or PD4          */
/* Set the following flag in Parallel Port configuration  */
/* word if PD4 pin is used - otherwise PB4 is used        */
/*--------------------------------------------------------*/
#define PQ_PP_FCC3_RTS_ON_PD4           0x00000100

/*--------------------------------------------------------*/
/* TDMA2 L1TXD signal could be either on PB7 or PD22      */
/* Set the following flag in Parallel Port configuration  */
/* word if PD22 pin is used - otherwise PB7 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_TDMA2_L1TXD_ON_PD22       0x00000080

/*--------------------------------------------------------*/
/* TDMA2 L1RXD signal could be either on PB6 or PD21      */
/* Set the following flag in Parallel Port configuration  */
/* word if PD21 pin is used - otherwise PB6 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_TDMA2_L1RXD_ON_PD21       0x00000040

/*--------------------------------------------------------*/
/* TDMA2 L1TSYNC signal could be either on PB5 or PC9     */
/* Set the following flag in Parallel Port configuration  */
/* word if PC9 pin is used - otherwise PB6 is used        */
/*--------------------------------------------------------*/
#define PQ_PP_TDMA2_L1TSYNC_ON_PC9      0x00000020

/*--------------------------------------------------------*/
/* TDMA2 L1RSYNC signal could be either on PB4 or PD20    */
/* Set the following flag in Parallel Port configuration  */
/* word if PD20 pin is used - otherwise PB4 is used       */
/*--------------------------------------------------------*/
#define PQ_PP_TDMA2_L1RSYNC_ON_PD20     0x00000010

/*--------------------------------------------------------*/
/* TDMC1 signals could be either on Port B or Port D      */
/* Set the following flag in Parallel Port configuration  */
/* word if Port D is used - otherwise Port B is used      */
/*--------------------------------------------------------*/
#define PQ_PP_TDMC1_ON_PORTD            0x00000008

/*--------------------------------------------------------*/
/* TDMD1 signals could be either on Port B or Port D      */
/* Set the following flag in Parallel Port configuration  */
/* word if Port D is used - otherwise Port B is used      */
/*--------------------------------------------------------*/
#define PQ_PP_TDMD1_ON_PORTD            0x00000004


/*--------------------------------------------------------*/
/*  BRGO configuration flags define if BRGO signals should*/
/*  be routed or no and (if yes) what pins are used       */
/*--------------------------------------------------------*/
/*--------------------------------------------------------*/
/* BRGO1 could be either on PC31 or PD19 pin.             */
/* Set the following flag in BRGO configuration word if   */
/* PC31 or PD19 pins is used, otherwise BRGO1 is not used */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO1_ON_PC31  0x8000
#define PQ_PP_BRGO1_ON_PD19  0x4000

/*--------------------------------------------------------*/
/* BRGO2 could be either on PC29 or PD17 pin.             */
/* Set the following flag in BRGO configuration word if   */
/* PC29 or PD17 pins is used, otherwise BRGO2 is not used */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO2_ON_PC29  0x2000
#define PQ_PP_BRGO2_ON_PD17  0x1000

/*--------------------------------------------------------*/
/* BRGO3 could be either on PC27 or PD9 pin.              */
/* Set the following flag in BRGO configuration word if   */
/* PC27 or PD9 pins is used, otherwise BRGO3 is not used  */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO3_ON_PC27  0x0800
#define PQ_PP_BRGO3_ON_PD9   0x0400

/*--------------------------------------------------------*/
/* BRGO4 could be either on PC25 or PD10 pin.             */
/* Set the following flag in BRGO configuration word if   */
/* PC25 or PD10 pins is used, otherwise BRGO4 is not used */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO4_ON_PC25  0x0200
#define PQ_PP_BRGO4_ON_PD10  0x0100

/*--------------------------------------------------------*/
/* BRGO5 could be either on PC23 or PD8 pin.              */
/* Set the following flag in BRGO configuration word if   */
/* PC23 or PD8 pins is used, otherwise BRGO5 is not used  */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO5_ON_PC23  0x0080
#define PQ_PP_BRGO5_ON_PD8   0x0040

/*--------------------------------------------------------*/
/* BRGO6 could be either on PC21 or PC1 pin.              */
/* Set the following flag in BRGO configuration word if   */
/* PC21 or PC1 pins is used, otherwise BRGO6 is not used  */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO6_ON_PC21  0x0020
#define PQ_PP_BRGO6_ON_PC1   0x0010

/*--------------------------------------------------------*/
/* BRGO7 could be either on PC19 or PC0 pin.              */
/* Set the following flag in BRGO configuration word if   */
/* PC19 or PC0 pins is used, otherwise BRGO7 is not used  */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO7_ON_PC19  0x0008
#define PQ_PP_BRGO7_ON_PC0   0x0004

/*--------------------------------------------------------*/
/* BRGO8 could be either on PC17 or PD4 pin.              */
/* Set the following flag in BRGO configuration word if   */
/* PC17 or PD4 pins is used, otherwise BRGO8 is not used  */
/*--------------------------------------------------------*/
#define PQ_PP_BRGO8_ON_PC17  0x0002
#define PQ_PP_BRGO8_ON_PD4   0x0001


/*--------------------------------------------------------*/
/*              TSA configuration flags                   */
/*These flags should be ORed for TSA_CONFIG initialization*/
/*--------------------------------------------------------*/
/* First group of flags define the size of SIRAM used for */
/* particular TSA in 64 entry banks. Default = 0 means    */
/* that this TSA is not in use. Maximum possible size is  */
/* 3 banks.                                               */
/*--------------------------------------------------------*/
#define PQ_TSA_1_BANK       0x2000
#define PQ_TSA_2_BANK       0x4000
#define PQ_TSA_3_BANK       0x6000

/*--------------------------------------------------------*/
/* This group of flags define the RFSD value in SIxMR reg */
/* Default = 0 means no receive frame sync bit delay.     */
/*--------------------------------------------------------*/
#define PQ_TSA_RFSD_1       0x0100
#define PQ_TSA_RFSD_2       0x0200
#define PQ_TSA_RFSD_3       0x0300

/*--------------------------------------------------------*/
/* This group of flags define the TFSD value in SIxMR reg */
/* Default = 0 means no transmit frame sync bit delay.    */
/*--------------------------------------------------------*/
#define PQ_TSA_TFSD_1       0x0001
#define PQ_TSA_TFSD_2       0x0002
#define PQ_TSA_TFSD_3       0x0003

/*--------------------------------------------------------*/
/* This flag defines the CRT value in SIxMR reg           */
/* Default = 0 means separate Rx and Tx clocks            */
/*--------------------------------------------------------*/
#define PQ_TSA_CRT          0x0040

/*--------------------------------------------------------*/
/* This flag defines the SL value in SIxMR reg            */
/* Default = 0 means sync signal is active high           */
/*--------------------------------------------------------*/
#define PQ_TSA_SL           0x0020

/*--------------------------------------------------------*/
/* This flag defines the CE value in SIxMR reg            */
/* Default = 0 means the data is sent on the rising edge  */
/* of the clock and received on the falling edge          */
/*--------------------------------------------------------*/
#define PQ_TSA_CE           0x0010

/*--------------------------------------------------------*/
/* This flag defines the FE value in SIxMR reg            */
/* Default = 0 means frame sync edge is falling one.      */
/*--------------------------------------------------------*/
#define PQ_TSA_FE           0x0008

/*--------------------------------------------------------*/
/* This flag defines whether TSA is used in nibble mode - */
/* applicable only for TDMA1 and TDMA2 TSAs.              */
/* Default = 0 means serial (not nibble) mode.            */
/*--------------------------------------------------------*/
#define PQ_TSA_NIB          0x8000

/*--------------------------------------------------------*/
/* This flag defines whether TSA is used in dynamic mode -*/
/* applicable only for TSAs used with MCCs and if it's set*/
/* half SIRAM dedicated for this TSA is used as shared one*/
/* This mode is useful when a number of channels could be */
/* created on the same TSA.                               */
/* Default = 0 means static (not dynamic) mode.           */
/*--------------------------------------------------------*/
#define PQ_TSA_DYN          0x0004


/*--------------------------------------------------------*/
/*                MCC configuration                       */
/* Describes mapping of MCC channel groups to proper TSAs */
/* Following masks should be ORed to produce MCC_CONFIG   */
/* Default value (0) means this MCC channel group is not  */
/* in use and therefore corresponding channel memory could*/
/* be used for other purposes.                            */
/*--------------------------------------------------------*/
#define PQ_MCC1_GROUP1_FREE  0x00000000
#define PQ_MCC1_GROUP1_TDMA  0x10000000
#define PQ_MCC1_GROUP1_TDMB  0x20000000
#define PQ_MCC1_GROUP1_TDMC  0x40000000
#define PQ_MCC1_GROUP1_TDMD  0x80000000
#define PQ_MCC1_GROUP2_FREE  0x00000000
#define PQ_MCC1_GROUP2_TDMA  0x01000000
#define PQ_MCC1_GROUP2_TDMB  0x02000000
#define PQ_MCC1_GROUP2_TDMC  0x04000000
#define PQ_MCC1_GROUP2_TDMD  0x08000000
#define PQ_MCC1_GROUP3_FREE  0x00000000
#define PQ_MCC1_GROUP3_TDMA  0x00100000
#define PQ_MCC1_GROUP3_TDMB  0x00200000
#define PQ_MCC1_GROUP3_TDMC  0x00400000
#define PQ_MCC1_GROUP3_TDMD  0x00800000
#define PQ_MCC1_GROUP4_FREE  0x00000000
#define PQ_MCC1_GROUP4_TDMA  0x00010000
#define PQ_MCC1_GROUP4_TDMB  0x00020000
#define PQ_MCC1_GROUP4_TDMC  0x00040000
#define PQ_MCC1_GROUP4_TDMD  0x00080000
#define PQ_MCC2_GROUP1_FREE  0x00000000
#define PQ_MCC2_GROUP1_TDMA  0x00001000
#define PQ_MCC2_GROUP1_TDMB  0x00002000
#define PQ_MCC2_GROUP1_TDMC  0x00004000
#define PQ_MCC2_GROUP1_TDMD  0x00008000
#define PQ_MCC2_GROUP2_FREE  0x00000000
#define PQ_MCC2_GROUP2_TDMA  0x00000100
#define PQ_MCC2_GROUP2_TDMB  0x00000200
#define PQ_MCC2_GROUP2_TDMC  0x00000400
#define PQ_MCC2_GROUP2_TDMD  0x00000800
#define PQ_MCC2_GROUP3_FREE  0x00000000
#define PQ_MCC2_GROUP3_TDMA  0x00000010
#define PQ_MCC2_GROUP3_TDMB  0x00000020
#define PQ_MCC2_GROUP3_TDMC  0x00000040
#define PQ_MCC2_GROUP3_TDMD  0x00000080
#define PQ_MCC2_GROUP4_FREE  0x00000000
#define PQ_MCC2_GROUP4_TDMA  0x00000001
#define PQ_MCC2_GROUP4_TDMB  0x00000002
#define PQ_MCC2_GROUP4_TDMC  0x00000004
#define PQ_MCC2_GROUP4_TDMD  0x00000008

#define PQ_RESERVED_AREA_NUM    3

/*
 * Chip configuration
 */
typedef struct pq_board_config_s
{
      __u8   *dpram_base;            /* IMMR value for this PQUICCII              */
      __u16  brgo_config;            /* mask representing brgo signals config.    */
      __u32  peripherals;            /* mask representing all peripherals to be used */
      __u32  parallel_ports;         /* Parallel port configuration               */
      __u32  external_clk3;          /* external clock3 frequency in hz           */
      __u32  external_clk5;          /* external clock5 frequency in hz           */
      __u32  external_clk9;          /* external clock9 frequency in hz           */
      __u32  external_clk15;         /* external clock15 frequency in hz          */
      __u32  tsa_config[8];          /* tsa configuration masks array             */
      __u32  mcc_config;             /* mcc configuration- maps mcc groups to tsas*/

      /* Reserved DPRAM areas to avoid clashes with DPRAM-demanding
	 applications that do not use Arabella PQ resource management
      */
      __u32  reserved_dpram_start[PQ_RESERVED_AREA_NUM];
      __u32  reserved_dpram_size[PQ_RESERVED_AREA_NUM];
} pq_board_config;


/* Board-specific device configuration.
 */
typedef struct pq_board_dev_config_s
{
      int          pqid;
      pq_xcc_id    xcc;
      pq_clock_id  tx_clock;
      pq_clock_id  rx_clock;
} pq_board_dev_config;

/* Max number of PQ2 devices allowed in the system
 */
#define PQ_NUM_OF_PQII      1

/* Board configuration access functions
 */
int pqboard_num_of_pqii(void);
volatile pq_intram *pqboard_intram_base( int pqid );
pq_board_config *pqboard_get_config( int pqid );
pq_board_dev_config *pqboard_get_device_config( int pqid, pq_xcc_id xcc );
__u32 pqboard_get_port_config( int pqid );
__u32 pqboard_get_cpm_clock( int pqid );
__u32 pqboard_get_bus_clock( int pqid );
__u32 pqboard_get_clock3( int pqid );
__u32 pqboard_get_clock5( int pqid );
__u32 pqboard_get_clock9( int pqid );
__u32 pqboard_get_clock15( int pqid );
__u16 pqboard_get_brgo_config( int pqid );
/* The following function group allows to enable/disable
   external on-board devices given the device type and 0-based index.
   If enable request (enable=1) is successful it returns 0 and updates *dev_area
   with pointer to the device registers area.
   If disable request (enable=0) is successful, it returns 0. Pointer to the
   device configuration area returned by the corresponding enable request
   becomes invalid.
*/
int   pqboard_atm_iface(int pqid, int iface, int enable, __u8 **dev_area);

/* ethernet enable/disable interface and it's input/output flags */
int   pqboard_enet_iface(int pqid, int iface, int *flags, __u8 **dev_area);
#define PQ2_BRD_FCCENET_W_ENABLE         0x00000001
#define PQ2_BRD_FCCENET_R_RMII           0x00010000
#define PQ2_BRD_FCCENET_R_RXCLK_GET(m)   (((m & 0xF8000000)>>27)&0x1F)
#define PQ2_BRD_FCCENET_R_TXCLK_GET(m)   (((m & 0x07C00000)>>22)&0x1F)
#define PQ2_BRD_FCCENET_R_RXCLK_MASK(c)  ((c<<27)&0xF8000000)
#define PQ2_BRD_FCCENET_R_TXCLK_MASK(c)  ((c<<22)&0x07C00000)


typedef struct pq_board_uart_config_s
{
      int          pqid;
      pq_xcc_id    xcc;
      pq_clock_id  clock;
      int          baud_rate;         /* Board-specific default for baudrate. 0=use driver default */
      void         *reserved_dpram;   /* DPRAM reserved for this device */
      __u32        reserved_size;     /* Reserved area size */
      __u32        flags;
#define PQ_BRD_UART_HF_FLOW  0x00000001
} pq_board_uart_config;

int   pqboard_uart_iface(int pqid, int iface, int enable, pq_board_uart_config *uart_cfg);
int   pqboard_hdlc_iface(int pqid, int iface, int enable, __u8 **dev_area);
int   pqboard_usb_iface(int pqid, int iface, int enable, __u32 flags);
#define PQ2_BRD_USB_HOST          0x00000001
#define PQ2_BRD_USB_FUNCTION      0x00000002
#define PQ2_BRD_USB_LOW_SPEED     0x00000004
#define PQ2_BRD_USB_SUPPLY_VCC5V  0x00000008

/*
 * Board-specific PIC support
 */

/* Enable PCI interrupt source.
   pin is 1-4
   Returns 0 if OK.
 */
int  pqboard_enable_pci_irq(int pqid, int idsel, int pin);

/* Disable PCI IRQ source
   pin is 1-4
   Returns 0 if OK.
 */
int  pqboard_disable_pci_irq(int pqid, int idsel, int pin);


#endif /* #ifndef PQB2RDDEF_H */

