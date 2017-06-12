/*
 * Copyright (C) 2003 Arabella Software Ltd.
 * Yuli Barcohen <yuli@arabellasw.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _PQ2DEFS_H_
#define _PQ2DEFS_H_

#ifdef __GNUC__
#define _Packed
#define _PackedType __attribute__ ((packed))
#else
#define _Packed     __packed
#define _PackedType
#endif

typedef unsigned char  pq_byte;
typedef unsigned short pq_hword;
typedef unsigned int   pq_word;

typedef enum {pq_scc1, pq_scc2, pq_scc3, pq_scc4} pq_scc_id;
typedef enum {pq_fcc1, pq_fcc2, pq_fcc3, pq_num_fcc} pq_fcc_id;
typedef enum {pq_smc1, pq_smc2} pq_smc_id;
typedef enum {pq_mcc1, pq_mcc2} pq_mcc_id;
typedef enum {pq_idma1, pq_idma2, pq_idma3, pq_idma4} pq_idma_id;
typedef enum {pq_brg1, pq_brg2, pq_brg3, pq_brg4,
              pq_brg5, pq_brg6, pq_brg7, pq_brg8} pq_brg_id;
typedef enum {pq_xfcc1, pq_xfcc2, pq_xfcc3, pq_xsmc1,
	      pq_xsmc2, pq_xscc1, pq_xscc2, pq_xscc3,
	      pq_xscc4, pq_xmcc1, pq_xmcc2} pq_xcc_id;
typedef enum {pq_clock_default=0,
	      pq_xbrg1, pq_xbrg2, pq_xbrg3, pq_xbrg4,
	      pq_xbrg5, pq_xbrg6, pq_xbrg7, pq_xbrg8,
	      pq_xclk1, pq_xclk2, pq_xclk3, pq_xclk4,
	      pq_xclk5, pq_xclk6, pq_xclk7, pq_xclk8,
	      pq_xclk9, pq_xclk10,pq_xclk11,pq_xclk12,
	      pq_xclk13,pq_xclk14,pq_xclk15,pq_xclk16,
	      pq_xclk17,pq_xclk18,pq_xclk19,pq_xclk20,
	      pq_xtmr1, pq_xtmr2, pq_xtmr3, pq_xtmr4 } pq_clock_id;
typedef enum {pq_porta, pq_portb, pq_portc, pq_portd } pq_port_id;

/* fixme: add all protocols */
typedef enum {pq_enet, pq_hdlc, pq_atm, pq_uart, pq_usb_host, pq_usb_function } pq_protocol;
typedef enum {pq_none, pq_ether, pq_t1, pq_e1, pq_sphy, pq_mphy} pq_iface_type;
typedef enum {pq_mem_ncache, pq_mem_cache, pq_mem_local, pq_mem_internal } pq_mem;

/*
 * Parallel port pin mask
 */
#define PQ_PIN(n)  (1<<(31-(n)))

/* CPM Buffer Descriptor */

/* Common flags */
#define PQ_BD_EMPTY 0x8000
#define PQ_BD_WRAP  0x2000
#define PQ_BD_INT   0x1000
#define PQ_BD_READY 0x8000

#define PQ_BD_CLEAN 0x3000

/* UART status bits */
#define PQ_BD_CCHAR 0x0800
#define PQ_BD_ADDR  0x0400
#define PQ_BD_CONT  0x0200
#define PQ_BD_IDLE  0x0100
#define PQ_BD_AM    0x0080
#define PQ_BD_BR    0x0020
#define PQ_BD_FR    0x0010
#define PQ_BD_PR    0x0008
#define PQ_BD_OV    0x0002
#define PQ_BD_CD    0x0001

#define PQ_BD_CR    0x0800
#define PQ_BD_PREAM 0x0100
#define PQ_BD_NS    0x0080
#define PQ_BD_CT    0x0001

/* Ethernet status bits */
#define PQ_BD_PAD   0x4000
#define PQ_BD_LAST  0x0800
#define PQ_BD_TCRC  0x0400
#define PQ_BD_DEFER 0x0200
#define PQ_BD_HBEAT 0x0100
#define PQ_BD_LATE  0x0080
#define PQ_BD_RLIM  0x0040
#define PQ_BD_RCNT  0x003C
#define PQ_BD_URUN  0x0002
#define PQ_BD_CSL   0x0001

#define PQ_BD_FIRST 0x0400
#define PQ_BD_MISS  0x0100
#define PQ_BD_BCAST 0x0080
#define PQ_BD_MCAST 0x0040
#define PQ_BD_LONG  0x0020
#define PQ_BD_ALIGN 0x0010
#define PQ_BD_SHORT 0x0008
#define PQ_BD_RCRC  0x0004
#define PQ_BD_ORUN  0x0002
#define PQ_BD_CLSN  0x0001

/* ATM status flags */
#define PQ_BD_CRC32 0x0001
#define PQ_BD_LN    0x0002
#define PQ_BD_CPUU  0x0004
#define PQ_BD_AB    0x0008
#define PQ_BD_CNG   0x0010
#define PQ_BD_CLP   0x0020

/* USB rx status fields */
#define PQ_BD_RXPID_MASK   0x00c0
#define PQ_BD_RXPID_0      0x0000
#define PQ_BD_RXPID_1      0x0040
#define PQ_BD_RXPID_SETUP  0x0080
#define PQ_BD_NO    0x0010   /* Rx non-octet aligned packet */
#define PQ_BD_AB    0x0008   /* Frame aborted (bit stuff error) */
#define PQ_BD_CRC   0x0004   /* CRC error */
#define PQ_BD_OV    0x0002   /* Overrun */

/* USB tx status fields */
#define PQ_BD_TC    0x0400   /* Transmit CRC */
#define PQ_BD_CNF   0x0200   /* Expect for confirmation before sending the next packet */
#define PQ_BD_LSP   0x0100   /* Low-speed transaction */
#define PQ_BD_PID0  0x0080   /* Transmit DATA0 PID */
#define PQ_BD_PID1  0x00c0   /* Transmit DATA1 PID */
#define PQ_BD_NAK   0x0010   /* NAK received */
#define PQ_BD_STAL  0x0008   /* STALL received */
#define PQ_BD_TO    0x0004   /* Timeout */
#define PQ_BD_UN    0x0002   /* Underrun */
#define PQ_BD_USB_TXERR   (PQ_BD_NAK | PQ_BD_STAL | PQ_BD_TO | PQ_BD_UN)
#define PQ_BD_USB_RXERR   (PQ_BD_NO | PQ_BD_AB | PQ_BD_CRC | PQ_BD_OV)


typedef struct pq_cpm_bd_s {
      pq_hword     status;
      pq_hword     len;
      pq_word      data;
} pq_cpm_bd;
/*--------------------------------*/
/*  BD access macros              */
/*--------------------------------*/
#define PQ_BD_STATUS(_bd)                (((pq_cpm_bd *)(_bd))->status)
#define PQ_BD_STATUS_SET(_bd, _val)      (((pq_cpm_bd *)(_bd))->status = _val)
#define PQ_BD_LENGTH(_bd)                (((pq_cpm_bd *)(_bd))->len)
#define PQ_BD_LENGTH_SET(_bd, _val)      (((pq_cpm_bd *)(_bd))->len = _val)
#define PQ_BD_DATA_CLEAR(_bd)            (((pq_cpm_bd *)(_bd))->data = 0)
#define PQ_BD_IS_DATA(_bd)               (((pq_cpm_bd *)(_bd))->data)
#define PQ_BD_DATA(_bd)                  ((pq_byte *)(__va(((pq_cpm_bd *)(_bd))->data)))
#define PQ_BD_DATA_SET(_bd, _data)       (((pq_cpm_bd *)(_bd))->data = __pa(_data))
#define PQ_BD_ADVANCE(_bd,_status,_base) (((_status) & PQ_BD_WRAP) ? (_bd)=(_base) : ++((pq_cpm_bd *)(_bd)))

/* IDMA Buffer Descriptor */

typedef struct pq_idma_bd_s {
      pq_word      status;
      pq_word      len;
      pq_word      src;
      pq_word      dest;
} pq_idma_bd;


/* Parameter RAM Definitions */

/* RISC Timer Table Parameter RAM */

typedef struct pq_rtmr_parm_s {
      pq_hword     tm_base;
      pq_hword     tm_ptr;
      pq_hword     r_tmr;
      pq_hword     r_tmv;
      pq_word      tm_cmd;
      pq_word      tm_cnt;
} pq_rtmr_parms;


/* FCC Parameter RAM */

#define PQ_FCC_BASE  0x8400
#define PQ_FCC1_BASE 0x8400
#define PQ_FCC2_BASE 0x8500
#define PQ_FCC3_BASE 0x8600

typedef struct pq_fcc_parm_s {  /* FCC Parameter RAM Common to All Protocols */
      pq_hword     riptr;
      pq_hword     tiptr;
      pq_hword     reserved1;
      pq_hword     mrblr;
      pq_word      rstate;
      pq_word      rbase;
      pq_hword     rbdstat;
      pq_hword     rbdlen;
      pq_word      rdptr;
      pq_word      tstate;
      pq_word      tbase;
      pq_hword     tbdstat;
      pq_hword     tbdlen;
      pq_word      tdptr;
      pq_word      rbptr;
      pq_word      tbptr;
      pq_word      rcrc;
      pq_word      reserved2;
      pq_word      tcrc;
} pq_fcc_parms;

typedef struct pq_fcc_enet_s {  /* FCC Ethernet-Specific Parameter RAM */
      pq_fcc_parms gen;
      pq_word      stat_buf;
      pq_word      cam_ptr;
      pq_word      c_mask;
      pq_word      c_pres;
      pq_word      crcec;
      pq_word      alec;
      pq_word      disfc;
      pq_hword     ret_lim;
      pq_hword     ret_cnt;
      pq_hword     p_per;
      pq_hword     boff_cnt;
      pq_word      gaddr[2];
      pq_hword     tfcstat;
      pq_hword     tfclen;
      pq_word      tfcptr;
      pq_hword     mflr;
      pq_byte      paddr[6];
      pq_hword     ibd_cnt;
      pq_hword     ibd_start;
      pq_hword     ibd_end;
      pq_hword     tx_len;
      pq_byte      ibd_base[32];
      pq_word      iaddr[2];
      pq_hword     minflr;
      pq_byte      taddr[6];
      pq_hword     pad_ptr;
      pq_hword     reserved1;
      pq_hword     cf_range;
      pq_hword     max_b;
      pq_hword     maxd1;
      pq_hword     maxd2;
      pq_hword     maxd;
      pq_hword     dma_cnt;
      pq_word      octc;
      pq_word      colc;
      pq_word      broc;
      pq_word      mulc;
      pq_word      uspc;
      pq_word      frgc;
      pq_word      ospc;
      pq_word      jbrc;
      pq_word      p64c;
      pq_word      p65c;
      pq_word      p128c;
      pq_word      p256c;
      pq_word      p512c;
      pq_word      p1024c;
      pq_word      cam_buf;
      pq_word      reserved2;
} pq_fcc_enet;


/* SCC Parameter RAM */

#define PQ_SCC_BASE  0x8000
#define PQ_SCC1_BASE 0x8000
#define PQ_SCC2_BASE 0x8100
#define PQ_SCC3_BASE 0x8200
#define PQ_SCC4_BASE 0x8300

typedef struct pq_scc_parm_s {  /* SCC Parameter RAM Common to All Protocols */
      pq_hword rbase;
      pq_hword tbase;
      pq_byte  rfcr;
      pq_byte  tfcr;
      pq_hword mrblr;
      pq_word  rstate;
      pq_word  intern1;
      pq_hword rbptr;
      pq_hword intern2;
      pq_word  intern3;
      pq_word  tstate;
      pq_word  intern4;
      pq_hword tbptr;
      pq_hword intern5;
      pq_word  intern6;
      pq_word  rcrc;
      pq_word  tcrc;
} pq_scc_parms;

typedef struct pq_scc_enet_s {  /* SCC Ethernet-Specific Parameter RAM */
      pq_scc_parms gen;
      pq_word      c_pres;
      pq_word      c_mask;
      pq_word      crcec;
      pq_word      alec;
      pq_word      disfc;
      pq_hword     pads;
      pq_hword     ret_lim;
      pq_hword     ret_cnt;
      pq_hword     mflr;
      pq_hword     minflr;
      pq_hword     maxd1;
      pq_hword     maxd2;
      pq_hword     maxd;
      pq_hword     dma_cnt;
      pq_hword     max_b;
      pq_hword     gaddr[4];
      pq_word      tbuf0_data[2];
      pq_word      tbuf0_rba0;
      pq_word      tbuf0_crc;
      pq_hword     tbuf0_bcnt;
      pq_byte      paddr[6];
      pq_hword     p_per;
      pq_hword     rfbd_ptr;
      pq_hword     tfbd_ptr;
      pq_hword     tlbd_ptr;
      pq_word      tbuf1_data[2];
      pq_word      tbuf1_rba0;
      pq_word      tbuf1_crc;
      pq_hword     tbuf1_bcnt;
      pq_hword     tx_len;
      pq_hword     addr[4];
      pq_hword     boff_cnt;
      pq_byte      taddr[6];
} pq_scc_enet;

typedef struct pq_scc_hdlc_s {  /* SCC HDLC-Specific Parameter RAM */
      pq_scc_parms gen;
      pq_word      reserved1;
      pq_word      c_mask;
      pq_word      c_pres;
      pq_hword     disfc;
      pq_hword     crcec;
      pq_hword     abtsc;
      pq_hword     nmarc;
      pq_hword     retrc;
      pq_hword     mflr;
      pq_hword     max_cnt;
      pq_hword     rfthr;
      pq_hword     rfcnt;
      pq_hword     hmask;
      pq_hword     haddr[4];
      pq_hword     tmp;
      pq_hword     tmp_mb;
} pq_hdlc_enet;

typedef struct pq_scc_uart_s {  /* SCC UART-Specific Parameter RAM */
      pq_scc_parms gen;
      pq_word      reserved1;
      pq_word      reserved2;
      pq_hword     max_idl;
      pq_hword     idlc;
      pq_hword     brkcr;
      pq_hword     parec;
      pq_hword     frmec;
      pq_hword     nosec;
      pq_hword     brkec;
      pq_hword     brkln;
      pq_hword     uaddr[2];
      pq_hword     rtemp;
      pq_hword     toseq;
      pq_hword     cchar[8];
      pq_hword     rccm;
      pq_hword     rccr;
      pq_hword     rlbc;
} pq_scc_uart;


/* Part of SMC parameters are that looks similar to SCC
   parameters area. It makes easier code that
   operates on both SMC and SCC
   */
typedef struct pq_smc_parm_s
{
      pq_hword rbase;
      pq_hword tbase;
      pq_byte  rfcr;
      pq_byte  tfcr;
      pq_hword mrblr;
      pq_word  rstate;
      pq_word  intern1;
      pq_hword rbptr;
      pq_hword intern2;
      pq_word  intern3;
      pq_word  tstate;
      pq_word  intern4;
      pq_hword tbptr;
      pq_hword intern5;
} pq_smc_parm;

typedef struct pq_smc_uart_s {  /* SMC UART/Transparent mode Parameter RAM */
      pq_smc_parm  gen;
      pq_word      reserved1;
      pq_hword     max_idl;
      pq_hword     idlc;
      pq_hword     brkln;
      pq_hword     brkec;
      pq_hword     brkcr;
      pq_hword     r_mask;
      pq_word      reserved2;
} pq_smc_uart;

/* I2C Parameter RAM  */

#define PQ_I2C_BASE 0x8AFC

typedef struct pq_i2c_parm_s {
      pq_hword     rbase;
      pq_hword     tbase;
      pq_byte      rfcr;
      pq_byte      tfcr;
      pq_hword     mrblr;
      pq_word      rstate;
      pq_word      rptr;
      pq_hword     rbptr;
      pq_hword     rcount;
      pq_word      rtemp;
      pq_word      tstate;
      pq_word      tptr;
      pq_hword     tbptr;
      pq_hword     tcount;
      pq_word      ttemp;
} pq_i2c_parms;

/***************************************************************************************/
/*                      PowerQuiccII ATM Parameters RAM                                */
/***************************************************************************************/

/* Address Compression parameters table */
typedef struct pq_addr_compr_parms_s
{
      pq_word  vpt_base;  /* VP-level addressing table base address  */
      pq_word  vct_base;  /* VC-level addressing table base address  */
      pq_word  vpt1_base; /* VP1-level addressing table base address */
      pq_word  vct1_base; /* VC1-level addressing table base address */
      pq_hword vp_mask;   /* VP mask for address compression look-up */
} _PackedType pq_addr_compr_parms;


/* External CAM parameters table  */
typedef struct pq_ext_cam_parms_s
{
      pq_word  extcam_base;      /* base address of the external cam  */
      pq_byte  reserved00[4];
      pq_word  extcam1_base;     /* base address of the external cam1 */
      pq_byte  reserved01[6];
} _PackedType pq_ext_cam_parms;

/* ATM mode parameters table  */
typedef struct pq_atm_parms_s
{
   pq_word  reserved0[16];
   pq_hword rxcelltmpbase;  /* rx cell temporary base address            */
   pq_hword txcelltmpbase;  /* tx cell temporary base address            */
   pq_hword udctmpbase;     /* udc temp base address (in udc mode only)  */
   pq_hword intrctbase;     /* internal rtc base address                 */
   pq_hword inttctbase;     /* internal tct base address                 */
   pq_hword inttctebase;    /* internal act base address                 */
   pq_word  sssarrastimer;  /* sssar ras timer in microseconds           */
   pq_word  extrctbase;     /* extrnal rtc base address                  */
   pq_word  exttctbase;     /* extrnal tct base address                  */
   pq_word  exttctebase;    /* extrnal act base address                  */
   pq_hword ueadoffset;     /* the offset in half-wordunits of the       */
                                  /*  uead entry in the udc extra header.      */
                                  /*  should be even address.                  */
                                  /*  if little-endian format is used,         */
                                  /*  the ueadoffset is of the little-endian   */
                                  /*  format.                                  */
   pq_hword rxintbase;      /* aal2 sar specific internal rx data        */
   pq_hword pmtbase;        /* performance monitoring table base address */
   pq_hword apcparambase;   /* apc parameters table base address         */
   pq_hword fbpparambase;   /* free buffer pool parameters base  address */
   pq_hword intqparambase;  /* interrupt queue param table base address  */
   pq_hword reserved3;
   pq_hword unistattablebase;/* uni statistics table base                */
   pq_word  bdbaseext;      /* bd ring base address extension           */
   union
   {
      pq_addr_compr_parms addr_compr;
      pq_ext_cam_parms    ext_cam;
   } _PackedType addr_mapping;  /* Address look-up mechanism            */
   pq_hword vcifiltering;   /* vci filtering enable bits. if bit i  */
                                 /* is set, the cell with vci=i will be  */
                                 /* sent to the raw cell queue. the bits */
                                 /* 0-2 and 5 should be zero.            */
   pq_hword gmode;          /* global mode                          */
   pq_hword comminfo1;      /* the information field associated     */
   pq_word  comminfo2;      /* with the last host command           */
   pq_word  reserved4;      /* reserved                             */
   pq_word  crc32preset;    /* preset for crc32                     */
   pq_word  crc32mask;      /* constant mask for crc32              */
   pq_hword aal1snptablebase;/* aal1 sn protection table base     */
   pq_hword reserved5;      /* reserved                             */
   pq_word  srtsbase;       /* external srts logic base address.    */
                            /* for aal1 only - 16 bytes aligned !   */
   pq_hword idlebase;       /* idle cell base address               */
   pq_hword idlesize;       /* idle cell size: 52, 56, 60, 64       */
   pq_word  emptycellpayload;/* Empty cell payload (little-indian)*/
   /* ABR specific only */
   pq_word  trm;           /* the upper bound on the time between  */
                                 /*  f-rm cells for an active source     */
   pq_hword nrm;           /* controls the maximum data cells sent */
                                 /* for each f-rm cell.                  */
   pq_hword mrm;           /* controls the bandwidth between f-rm, */
                                 /* b-rm and user data cell              */
   pq_hword tcr;           /* tag cell rate                        */
   pq_hword abrrxtcte;     /* abr reserved area address-dblwrd alg */
   pq_word  rxdescrbaseext;/* aal2 sar - external rx descriptors   */
   pq_word  reserved6[7];
   pq_hword padtmpbase;    /* aal2 sar specific pad template addr  */
   pq_hword reserved7;
   pq_word  reserved8[2];
   pq_word  tcellextbase;  /* aal2 sar specific tx cell temp addr  */
   pq_word  reserved9[7];
}  _PackedType pq_atm_parms;


/* USB mode parameters table  */
typedef struct pq_usb_parms_s
{
   pq_hword ep0_ptr;       /* EP0 parameters are index */
   pq_hword ep1_ptr;       /* EP1 parameters are index */
   pq_hword ep2_ptr;       /* EP2 parameters are index */
   pq_hword ep3_ptr;       /* EP3 parameters are index */
   pq_word  rstate;
   pq_word  rptr;
   pq_hword frame_n;       /* Frame number */
   pq_hword rbcnt;         /* Receive internal byte count */
   pq_word  rtemp;
   pq_word  rxusb_data;
   pq_hword rxuptr;
   pq_hword reserved1;
}  _PackedType pq_usb_parms;

/* USB EP parameters block */
typedef struct pq_usb_ep_parms_s
{
   pq_hword rbase;         /* Rx BD ring base */
   pq_hword tbase;         /* Tx BD ring base */
   pq_byte  rfcr;
   pq_byte  tfcr;
   pq_hword mrblr;         /* Max RX packet length */
   pq_hword rbptr;         /* Rx BD pointer */
   pq_hword tbptr;         /* Tx BD pointer */
   pq_word  tstate;
   pq_word  tptr;
   pq_hword tcrc;
   pq_hword tbcnt;
   pq_word  ttemp;
   pq_hword tx_usbuptr;
   pq_hword reserved1;
}  _PackedType pq_usb_ep_parms;

/* RFCR/TFCR fields */
#define PQ2_USB_FCR_GBL      0x20   /* Enable bus snooping */
#define PQ2_USB_FCR_BO_LE    0x00   /* Little endian */
#define PQ2_USB_FCR_BO_PPC   0x08   /* PPC little endian (dword byte swapping) */
#define PQ2_USB_FCR_BO_BE    0x18   /* MOT mode: big endian */
#define PQ2_USB_FCR_TC2      0x04   /* Transfer mode */
#define PQ2_USB_FCR_DTB_LCL  0x02   /* 1=Local bus 0=60x bus */


/* Internal RAM Map */

typedef struct pq_siu_s {
   pq_word siumcr;        /* SIU module configuration register 4.3.2.6/4-31 */
   pq_word sypcr;         /* System protection control register 4.3.2.8/4-35 */
   pq_byte reserved1[6];
   pq_hword swsr;         /* Software service register 4.3.2.9/4-36 */
   pq_byte reserved2[20];
   pq_word bcr;           /* Bus configuration register 4.3.2.1/4-25 */
   pq_byte ppc_acr;       /* 60x bus arbiter configuration register 4.3.2.2/4-28 */
   pq_byte reserved3[3];
   pq_word ppc_alrh;      /* 60x bus arbitration-level register high 4.3.2.3/4-28 */
   pq_word ppc_alrl;      /* 60x bus arbitration-level register low 4.3.2.3/4-28 */
   pq_byte lcl_acr;       /* Local arbiter configuration register 4.3.2.4/4-29 */
   pq_byte reserved4[3];
   pq_word lcl_alrh;      /* Local arbitration-level register 4.3.2.5/4-30 */
   pq_word lcl_alrl;      /* Local arbitration-level register 4.3.2.3/4-28 */
   pq_word tescr1;        /* 60x bus transfer error status control register 1 4.3.2.10/4-36 */
   pq_word tescr2;        /* 60x bus transfer error status control register 2 4.3.2.11/4-37 */
   pq_word l_tescr1;      /* Local bus transfer error status control register 1 4.3.2.12/4-38 */
   pq_word l_tescr2;      /* Local bus transfer error status control register 2 4.3.2.13/4-39 */
   pq_word pdtea;         /* 60x bus DMA transfer error address 18.2.3/18-4 */
   pq_byte pdtem;         /* 60x bus DMA transfer error MSNUM 18.2.4/18-4 */
   pq_byte reserved5[3];
   pq_word ldtea;         /* Local bus DMA transfer error address 18.2.3/18-4 */
   pq_byte ldtem;         /* Local bus DMA transfer error MSNUM 18.2.4/18-4 */
   pq_byte reserved6[163];
} pq_siu;

/*
 * SIUMCR register encoding
 */
#define PQ_SIUMCR_BBD           0x80000000
#define PQ_SIUMCR_ESE           0x40000000  /* 1=external snoop enable */
#define PQ_SIUMCR_PBSE          0x20000000  /* 1=parity byte select enable */
#define PQ_SIUMCR_CDIS          0x10000000  /* 1=core disable */
#define PQ_SIUMCR_DPPC_MASK     0x0c000000  /* DPPC mask */
#define PQ_SIUMCR_DPPC_SHIFT    26          /* DPPC shift */
#define PQ_SIUMCR_L2CPC_MASK    0x03000000  /* L2CPC mask */
#define PQ_SIUMCR_L2CPC_SHIFT   24          /* L2CPC shift */
#define PQ_SIUMCR_LBPC_MASK     0x00c00000  /* LBPC mask */
#define PQ_SIUMCR_LBPC_SHIFT    22          /* LBPC shift */
#define PQ_SIUMCR_APPC_MASK     0x00300000  /* APPC mask */
#define PQ_SIUMCR_APPC_SHIFT    20          /* APPC shift */
#define PQ_SIUMCR_CS10PC_MASK   0x000c0000  /* CS10PC mask */
#define PQ_SIUMCR_CS10PC_SHIFT  18          /* CS10PC shift */
#define PQ_SIUMCR_BCTL_MASK     0x00030000  /* BCTL mask */
#define PQ_SIUMCR_BCTL_SHIFT    16          /* BCTL shift */
#define PQ_SIUMCR_MMR_MASK      0x0000c000  /* MMR mask */
#define PQ_SIUMCR_MMR_SHIFT     14          /* MMR shift */
#define PQ_SIUMCR_LPBSE         0x00002000  /* 1=LB parity enable */

#define PQ_SIUMCR_LBPC_LB       0x00000000  /* Local bus pins function as local bus */
#define PQ_SIUMCR_LBPC_PCI      0x00400000  /* Local bus pins function as local PCI */
#define PQ_SIUMCR_LBPC_CORE     0x00800000  /* Local bus pins function as core */
#define PQ_SIUMCR_CS10PC00	0x00000000  /* CS10 Pin Configuration	*/
#define PQ_SIUMCR_CS10PC01	0x00040000  /* - " -			*/
#define PQ_SIUMCR_CS10PC10	0x00080000  /* - " -			*/
#define PQ_SIUMCR_CS10PC11	0x000c0000  /* - " -			*/

typedef struct pq_chipsel_s {
      pq_word br;               /* Base register 10.3.1/10-14 */
      pq_word or;               /* Option register 10.3.2/10-16 */
} pq_chipsel;

typedef struct pq_memctl_s {
      pq_chipsel cs[12];        /* Chips selects 0-11 */
      pq_byte    reserved1[8];
      pq_word    mar;           /* Memory address register 10.3.7/10-29 */
      pq_byte    reserved2[4];
      pq_word    mamr;          /* Machine A mode register 10.3.5/10-26 */
      pq_word    mbmr;          /* Machine B mode register */
      pq_word    mcmr;          /* Machine C mode register */
      pq_byte    reserved3[8];
      pq_hword   mptpr;         /* Memory periodic timer prescaler 10.3.12/10-32 */
      pq_byte    reserved4[2];
      pq_word    mdr;           /* Memory data register 10.3.6/10-28 */
      pq_byte    reserved5[4];
      pq_word    psdmr;         /* 60x bus SDRAM mode register 10.3.3/10-21 */
      pq_word    lsdmr;         /* Local bus SDRAM mode register 10.3.4/10-24 */
      pq_byte    purt;          /* 60x bus-assigned UPM refresh timer 10.3.8/10-30 */
      pq_byte    reserved6[3];
      pq_byte    psrt;          /* 60x bus-assigned SDRAM refresh timer 10.3.10/10-31 */
      pq_byte    reserved7[3];
      pq_byte    lurt;          /* Local bus-assigned UPM refresh timer 10.3.9/10-30 */
      pq_byte    reserved8[3];
      pq_byte    lsrt;          /* Local bus-assigned SDRAM refresh timer 10.3.11/10-32 */
      pq_byte    reserved9[3];
      pq_word    immr;          /* Internal memory map register 4.3.2.7/4-34 */
      pq_word    pcibr0;
      pq_word    pcibr1;
      pq_byte    reserved10[16];
      pq_word    pcimsk0;
      pq_word    pcimsk1;
      pq_byte    reserved11[52];
} pq_memctl;

#define PQ_RTC_SEC        0x0080
#define PQ_RTC_ALR        0x0040
#define PQ_RTC_SIE        0x0008
#define PQ_RTC_ALE        0x0004
#define PQ_RTC_FREQ       0x0002
#define PQ_RTC_FREQ_32KHZ 0x0002
#define PQ_RTC_FREQ_4MHZ  0x0000
#define PQ_RTC_TCE        0x0001

typedef struct pq_sitmr_s {
      pq_byte  reserved1[32];
      pq_hword tmcntsc;         /* Time counter status and control register 4.3.2.14/4-40 */
      pq_byte  reserved2[2];
      pq_word  tmcnt;           /* Time counter register 4.3.2.15/4-41 */
      pq_byte  reserved3[4];
      pq_word  tmcntal;         /* Time counter alarm register 4.3.2.16/4-41 */
      pq_byte  reserved4[16];
      pq_hword piscr;           /* Periodic interrupt status and control register 4.3.3.1/4-42 */
      pq_byte  reserved5[2];
      pq_word  pitc;            /* Periodic interrupt count register 4.3.3.2/4-43 */
      pq_word  pitr;            /* Periodic interrupt timer register 4.3.3.3/4-44 */
      pq_byte  reserved6[94];
      pq_byte  reserved7[390];
   } pq_sitmr;

/* Interrupt vectors encoding */
#define PQ_VEC_I2C   1
#define PQ_VEC_SPI   2
#define PQ_VEC_RTMR  3
#define PQ_VEC_SMC   4
#define PQ_VEC_IDMA  6
#define PQ_VEC_SDMA  10
#define PQ_VEC_USB   11
#define PQ_VEC_GPT   12
#define PQ_VEC_TMCNT 16
#define PQ_VEC_PIT   17
#define PQ_VEC_IRQ   18
#define PQ_VEC_PCI   18
#define PQ_VEC_IRQ1  19
#define PQ_VEC_IRQ2  20
#define PQ_VEC_IRQ3  21
#define PQ_VEC_IRQ4  22
#define PQ_VEC_IRQ5  23
#define PQ_VEC_IRQ6  24
#define PQ_VEC_IRQ7  25
#define PQ_VEC_FCC   32
#define PQ_VEC_MCC   36
#define PQ_VEC_SCC   40
#define PQ_VEC_TC    44
#define PQ_VEC_PC    48

typedef struct pq_intctl_s {
      pq_hword sicr;            /* SIU interrupt configuration register 4.3.1.1/4-17 */
      pq_byte  reserved1[2];
      pq_word  sivec;           /* SIU interrupt vector register 4.3.1.6/4-23 */
      pq_word  sipnr[2];        /* SIU interrupt pending registers 4.3.1.4/4-21 */
      pq_word  siprr;           /* SIU interrupt priority register 4.3.1.2/4-18 */
      pq_word  scprr[2];        /* CPM interrupt priority registers 4.3.1.3/4-19 */
      pq_word  simr[2];         /* SIU interrupt mask registers 4.3.1.5/4-22 */
      pq_word  siexr;           /* SIU external interrupt control register 4.3.1.7/4-24 */
      pq_byte  reserved2[0x58];
} pq_intctl;

typedef struct pq_clocks_s {
      pq_word sccr;             /* System clock control register 9.8/9-8 */
      pq_byte reserved1[4];
      pq_word scmr;             /* System clock mode register 9.9/9-9 */
      pq_byte reserved2[4];
      pq_word rsr;              /* Reset status register 5.2/5-4 */
      pq_word rmr;              /* Reset mode register 5.3/5-5 */
      pq_byte reserved3[104];
} pq_clocks;

#define PQ_SCCR_PCI_MODE     0x00000100
#define PQ_SCCR_MODCK        0x00000080
#define PQ_SCCR_PCIDF_MASK   0x00000078
#define PQ_SCCR_CLPD         0x00000004
#define PQ_SCCR_DFBRG_MASK   0x00000003

/* SICR register fields (UM: 4.3.1.1) */
#define PQ_SICR_GSIU         0x0002
#define PQ_SICR_SPS          0x0001

typedef struct pq_port_s {
      pq_word dir;              /* Port data direction register 35.2.3/35-3 */
      pq_word par;              /* Port pin assignment register 35.2.4/35-4 */
      pq_word sor;              /* Port special options register 35.2.5/35-4 */
      pq_word odr;              /* Port open drain register 35.2.1/35-2 */
      pq_word dat;              /* Port data register 35.2.2/35-2 */
      pq_byte reserved1[12];
} pq_port;

typedef struct pq_timers_s {
      pq_byte  tgcr1;           /* Timers 1 and 2 global configuration register 17.2.2/17-4 */
      pq_byte  reserved1[3];
      pq_byte  tgcr2;           /* Timers 3 and 4 global configuration register 17.2.2/17-4 */
      pq_byte  reserved2[11];
      pq_hword tmr1;            /* Timer 1 mode register 17.2.3/17-6 */
      pq_hword tmr2;            /* Timer 2 mode register 17.2.3/17-6 */
      pq_hword trr1;            /* Timer 1 reference register 17.2.4/17-7 */
      pq_hword trr2;            /* Timer 2 reference register 17.2.4/17-7 */
      pq_hword tcr1;            /* Timer 1 capture register 17.2.5/17-8 */
      pq_hword tcr2;            /* Timer 2 capture register 17.2.5/17-8 */
      pq_hword tcn1;            /* Timer 1 counter 17.2.6/17-8 */
      pq_hword tcn2;            /* Timer 2 counter 17.2.6/17-8 */
      pq_hword tmr3;            /* Timer 3 mode register 17.2.3/17-6 */
      pq_hword tmr4;            /* Timer 4 mode register 17.2.3/17-6 */
      pq_hword trr3;            /* Timer 3 reference register 17.2.4/17-7 */
      pq_hword trr4;            /* Timer 4 reference register 17.2.4/17-7 */
      pq_hword tcr3;            /* Timer 3 capture register 17.2.5/17-8 */
      pq_hword tcr4;            /* Timer 4 capture register 17.2.5/17-8 */
      pq_hword tcn3;            /* Timer 3 counter 17.2.6/17-8 */
      pq_hword tcn4;            /* Timer 4 counter 17.2.6/17-8 */
      pq_hword ter1;            /* Timer 1 event register 17.2.7/17-8 */
      pq_hword ter2;            /* Timer 2 event register 17.2.7/17-8 */
      pq_hword ter3;            /* Timer 3 event register 17.2.7/17-8 */
      pq_hword ter4;            /* Timer 4 event register 17.2.7/17-8 */
      pq_byte reserved3[584];
} pq_timers;

typedef struct pq_sdma_s {
      pq_byte reserved1[24];
      pq_byte sdsr;             /* SDMA status register 18.2.1/18-3 */
      pq_byte reserved2[3];
      pq_byte sdmr;             /* SDMA mask register 18.2.2/18-4 */
      pq_byte reserved3[3];
} pq_sdma;

typedef struct pq_idma_s {
      pq_byte idsr;             /* IDMA event register 18.8.4/18-22 */
      pq_byte reserved1[3];
      pq_byte idmr;             /* IDMA mask register 18.8.4/18-22 */
      pq_byte reserved2[3];
} pq_idma;

#define PQ_GFMR_TCI   0x20000000
#define PQ_GFMR_ENR   0x00000020
#define PQ_GFMR_ENT   0x00000010
#define PQ_GFMR_HDLC  0x00000000
#define PQ_GFMR_ATM   0x0000000A
#define PQ_GFMR_ETHER 0x0000000C

#define PQ_FPSMR_LPB   0x10000000
#define PQ_FPSMR_FDE   0x04000000
#define PQ_FPSMR_MON   0x02000000
#define PQ_FPSMR_PRO   0x00400000
#define PQ_FPSMR_RSH   0x00100000
#define PQ_FPSMR_RMII  0x00020000
#define PQ_FPSMR_CRC32 0x00000080

#define PQ_FCCE_GRA 0x0080
#define PQ_FCCE_RXC 0x0040
#define PQ_FCCE_TXC 0x0020
#define PQ_FCCE_TXE 0x0010
#define PQ_FCCE_RXF 0x0008
#define PQ_FCCE_BSY 0x0004
#define PQ_FCCE_TXB 0x0002
#define PQ_FCCE_RXB 0x0001

typedef struct pq_fcc_regs_s {
      pq_word  gfmr;            /* FCC general mode register 28.2/28-3 */
      pq_word  fpsmr;           /* FCC protocol-specific mode register */
                                /* 29.13.2/29-85 (ATM) */
                                /* 30.18.1/30-20 (Ethernet) */
                                /* 31.6/31-7 (HDLC) */
      pq_hword ftodr;           /* FCC transmit on demand register 28.5/28-7 */
      pq_byte  reserved1[2];
      pq_hword fdsr;            /* FCC data synchronization register 28.4/28-7 */
      pq_byte  reserved2[2];
      pq_hword fcce;            /* FCC event register */
      pq_byte  reserved3[2];
      pq_hword fccm;            /* FCC mask register */
                                /* 29.13.3/29-87 (ATM) */
                                /* 30.18.2/30-21 (Ethernet) */
                                /* 31.9/31-14 (HDLC) */
      pq_byte  reserved4[2];
      pq_byte  fccs;            /* FCC status register 31.10/31-16 (HDLC) */
      pq_byte  reserved5[3];
      pq_byte  ftirr_phy[4];    /* FCC transmit internal rate registers for PHY0-3 */
                                /* 29.13.4/29-88 (ATM) */
} pq_fcc_regs;

#define PQ_BRG_RESET  0x20000
#define PQ_BRG_ENABLE 0x10000
#define PQ_BRG_CLOCK  0x0C000
#define PQ_BRG_BRGCLK 0x00000
#define PQ_BRG_CLK3   0x04000
#define PQ_BRG_CLK9   PQ_BRG_CLK3
#define PQ_BRG_CLK5   0x08000
#define PQ_BRG_CLK15  PQ_BRG_CLK5
#define PQ_BRG_ATB    0x02000
#define PQ_BRG_CLKDIV 0x01FFE
#define PQ_BRG_DIV16  0x00001
#define PQ_BRG_EXTC1  PQ_BRG_CLK3
#define PQ_BRG_EXTC2  PQ_BRG_CLK5

typedef pq_word pq_brgc;

typedef struct pq_i2c_regs_s {
      pq_byte i2mod;            /* I2C mode register 34.4.1/34-6 */
      pq_byte reserved1[3];
      pq_byte i2add;            /* I2C address register 34.4.2/34-7 */
      pq_byte reserved2[3];
      pq_byte i2brg;            /* I2C BRG register 34.4.3/34-7 */
      pq_byte reserved3[3];
      pq_byte i2com;            /* I2C command register 34.4.5/34-8 */
      pq_byte reserved4[3];
      pq_byte i2cer;            /* I2C event register 34.4.4/34-8 */
      pq_byte reserved5[3];
      pq_byte i2cmr;            /* I2C mask register 34.4.4/34-8 */
      pq_byte reserved6[0x14B];
} pq_i2c_regs;

#define PQ_CP_RST    0x80000000 /* CP reset command */
#define PQ_CP_FLG    0x00010000 /* CP busy flag */

#define PQ_CP_SCC    0x04
#define PQ_CP_SCC1   0x04
#define PQ_CP_SCC2   0x05
#define PQ_CP_SCC3   0x06
#define PQ_CP_SCC4   0x07
#define PQ_CP_SMC    0x08
#define PQ_CP_SMC1   0x08
#define PQ_CP_SMC2   0x09
#define PQ_CP_SPI    0x0A
#define PQ_CP_I2C    0x0B
#define PQ_CP_RAND   0x0E
#define PQ_CP_TMR    0x0F
#define PQ_CP_FCC    0x10
#define PQ_CP_FCC1   0x10
#define PQ_CP_FCC2   0x11
#define PQ_CP_FCC3   0x12
#define PQ_CP_IDMA   0x14
#define PQ_CP_IDMA1  0x14
#define PQ_CP_IDMA2  0x15
#define PQ_CP_IDMA3  0x16
#define PQ_CP_IDMA4  0x17
#define PQ_CP_MCC    0x1C
#define PQ_CP_MCC1   0x1C
#define PQ_CP_MCC2   0x1D
#define PQ_CP_ATM_FCC1   0x0E
#define PQ_CP_ATM_FCC2   0x2E

#define PQ_CP_CMD_INIT_RXTX  0
#define PQ_CP_CMD_INIT_RX    1
#define PQ_CP_CMD_INIT_TX    2
#define PQ_CP_CMD_HUNT_MODE  3
#define PQ_CP_CMD_STOP_TX    4
#define PQ_CP_CMD_GRCFL_STOP 5
#define PQ_CP_CMD_RESTART_TX 6
#define PQ_CP_CMD_CLOSE_RXBD 7
#define PQ_CP_CMD_GROUP_ADDR 8
#define PQ_CP_CMD_SET_TIMER  8
#define PQ_CP_CMD_GCI_TMOUT  9
#define PQ_CP_CMD_START_IDMA 9
#define PQ_CP_CMD_STOP_RX    9
#define PQ_CP_CMD_ATM_XMIT  10
#define PQ_CP_CMD_RESET_BCS 10
#define PQ_CP_CMD_GCI_ABORT 10
#define PQ_CP_CMD_STOP_IDMA 11
#define PQ_CP_CMD_RAND      12
#define PQ_CP_CMD_USB_STOP_TX    10
#define PQ_CP_CMD_USB_RESTART_TX 11

#define PQ_CP_PROTO_HDLC  0x00
#define PQ_CP_PROTO_ATM   0x0A
#define PQ_CP_PROTO_ETH   0x0C
#define PQ_CP_PROTO_TRANS 0x0F

typedef struct pq_commproc_s {
      pq_word  cpcr;            /* Communications processor command register 13.4.1/13-11 */
      pq_word  rccr;            /* CP configuration register 13.3.6/13-7 */
      pq_byte  reserved1[14];
      pq_hword rter;            /* CP timers event register 13.6.4/13-21 */
      pq_byte  reserved2[2];
      pq_hword rtmr;            /* CP timers mask register */
      pq_hword rtscr;           /* CP time-stamp timer control register 13.3.7/13-9 */
      pq_byte  reserved3[2];
      pq_word  rtsr;            /* CP time-stamp register 13.3.8/13-10 */
      pq_byte  reserved4[12];
} pq_commproc;

#define PQ_SCCE_GLR  0x1000
#define PQ_SCCE_GLT  0x0800
#define PQ_SCCE_AB   0x0200
#define PQ_SCCE_IDL  0x0100
#define PQ_SCCE_GRA  0x0080
#define PQ_SCCE_BRKE 0x0040
#define PQ_SCCE_BRKS 0x0020
#define PQ_SCCE_CCR  0x0008
#define PQ_SCCE_BSY  0x0004
#define PQ_SCCE_TX   0x0002
#define PQ_SCCE_RX   0x0001

typedef struct pq_scc_regs_s {
      pq_word  gsmr_l;          /* SCC general mode register (low) 19.1.1/19-3 */
      pq_word  gsmr_h;          /* SCC general mode register (high) */
      pq_hword psmr;            /* SCC protocol-specific mode register 19.1.2/19-9 */
                                /* 20.16/20-13 (UART) */
                                /* 21.8/21-7 (HDLC) */
                                /* 22.11/22-10 (BISYNC) */
                                /* 23.9/23-9 (Transparent) */
                                /* 24.17/24-15 (Ethernet) */
      pq_byte  reserved1[2];
      pq_hword todr;            /* SCC transmit-on-demand register 19.1.4/19-9 */
      pq_hword dsr;             /* SCC data synchronization register 19.1.3/19-9 */
      pq_hword scce;            /* SCC event register */
      pq_byte  reserved2[2];
      pq_hword sccm;            /* SCC mask register */
                                /* 20.19/20-19 (UART) */
                                /* 21.11/21-12 (HDLC) */
                                /* 22.14/22-15 (BISYNC) */
                                /* 23.12/23-12 (Transparent) */
                                /* 24.20/24-21 (Ethernet)3 */
      pq_byte  reserved3;
      pq_byte  sccs;            /* SCC2 status register */
                                /* 20.20/20-21 (UART) */
                                /* 21.12/21-14 (HDLC) */
                                /* 22.15/22-16 (BISYNC) */
                                /* 23.13/23-13 (Transparent) */
      pq_byte  reserved4[8];
} pq_scc_regs;

typedef struct pq_smc_regs_s {
      pq_byte  reserved1[2];
      pq_hword smcmr;           /* SMC mode register 26.2.1/26-3 */
      pq_byte  reserved2[2];
      pq_byte  smce;            /* SMC event register */
      pq_byte  reserved3[3];
      pq_byte  smcm;            /* SMC mask register */
                                /* 26.3.11/26-18 (UART) */
                                /* 26.4.10/26-28 (Transparent) */
                                /* 26.5.9/26-34 (GCI) */
      pq_byte  reserved4[5];
} pq_smc_regs;

typedef struct pq_spi_regs_s {
      pq_hword spmode;          /* SPI mode register 33.4.1/33-6 */
      pq_byte  reserved1[4];
      pq_byte  spie;            /* SPI event register 33.4.2/33-9 */
      pq_byte  reserved2[3];
      pq_byte  spim;            /* SPI mask register 33.4.2/33-9 */
      pq_byte  reserved3[2];
      pq_byte  spcom;           /* SPI command register 33.4.3/33-9 */
      pq_byte  reserved4[82];
} pq_spi_regs;

/* USB registers */
typedef struct pq_usb_regs_s {
      pq_byte  usmod;          /* USB mode register 7.5.7.1 */
      pq_byte  usadr;          /* USB address register 7.5.7.2 */
      pq_byte  uscom;          /* USB command register 7.5.7.4 */
      pq_byte  reserved1;
      pq_hword usep0;          /* USB endpoint 0 register 7.5.7.3 */
      pq_hword usep1;          /* USB endpoint 1 register */
      pq_hword usep2;          /* USB endpoint 2 register */
      pq_hword usep3;          /* USB endpoint 3 register */
      pq_word  reserved2;
      pq_hword usber;          /* USB event register 7.5.7.5 */
      pq_hword reserved3;
      pq_hword usbmr;          /* USB mask register 7.5.7.6 */
      pq_byte  reserved4;
      pq_byte  usbs;           /* USB status register 7.5.7.7 */
      pq_byte  reserved5[8];
} pq_usb_regs;

/* USB registers values */
#define PQ_USMOD_LSS       0x80    /* Low-speed operation */
#define PQ_USMOD_RESUME    0x40    /* Generate resume condition (function only) */
#define PQ_USMOD_TEST      0x04    /* Test (loopback) mode */
#define PQ_USMOD_HOST      0x02    /* USB host mode */
#define PQ_USMOD_ENABLE    0x01    /* USB enable */

#define PQ_USEP_NUM_MASK   0xf000  /* Endpoint number mask */
#define PQ_USEP_NUM_SHIFT  12
#define PQ_USEP_TM_CTRL    0x0000  /* Transfer mode: control */
#define PQ_USEP_TM_ISO     0x0300  /* Transfer mode: isochronous */
#define PQ_USEP_TM_BULK    0x0200  /* Transfer mode: bulk (function only) */
#define PQ_USEP_TM_INT     0x0100  /* Transfer mode: interrupt (function only) */
#define PQ_USEP_MF         0x0020  /* Multiframe enable */
#define PQ_USEP_RTE        0x0010  /* Retransmit enable (function only) */
#define PQ_USEP_THS_NORMAL 0x0000  /* Tx handshake: 0-normal, 1-ignore IN, 2-force NACK, 11-force STALL */
#define PQ_USEP_RHS_NORMAL 0x0000  /* Rx handshake: 0-normal, 1-ignore OUT, 2-force NACK, 11-force STALL */

#define PQ_USCOM_STR       0x80    /* Start FIFO fill */
#define PQ_USCOM_FLUSH     0x40    /* Flush FIFO */

#define PQ_USBE_RESET      0x0200  /* Reset condition detected */
#define PQ_USBE_IDLE       0x0100  /* Idle status changed */
#define PQ_USBE_TXE3       0x0080  /* Tx error: EP3 */
#define PQ_USBE_TXE2       0x0040  /* Tx error: EP2 */
#define PQ_USBE_TXE1       0x0020  /* Tx error: EP1 */
#define PQ_USBE_TXE0       0x0010  /* Tx error: EP0 */
#define PQ_USBE_SOF        0x0008  /* SOF received */
#define PQ_USBE_BSY        0x0004  /* Busy condition (no rx buffer) */
#define PQ_USBE_TXB        0x0002  /* A buffer has been transmitted */
#define PQ_USBE_RXB        0x0001  /* A buffer has been received */


#define PQ_CMX_BRG1  0
#define PQ_CMX_BRG2  1
#define PQ_CMX_BRG3  2
#define PQ_CMX_BRG4  3
#define PQ_CMX_CLK11 4
#define PQ_CMX_CLK12 5
#define PQ_CMX_CLK3  6
#define PQ_CMX_CLK4  7

#define PQ_CMX_BRG5  0
#define PQ_CMX_BRG6  1
#define PQ_CMX_BRG7  2
#define PQ_CMX_BRG8  3
#define PQ_CMX_CLK13 4
#define PQ_CMX_CLK14 5
#define PQ_CMX_CLK15 6
#define PQ_CMX_CLK16 7

#define PQ_CMX_SCC_GRANT  0x80
#define PQ_CMX_SCC_TSA    0x10
#define PQ_CMX_SCC_RCLK   0x38
#define PQ_CMX_SCC_RSHIFT 3
#define PQ_CMX_SCC_TCLK   0x07
#define PQ_CMX_SCC_TSHIFT 0

#define PQ_CMX_FCC_TSA    0x10
#define PQ_CMX_FCC_RCLK   0x38
#define PQ_CMX_FCC_RSHIFT 3
#define PQ_CMX_FCC_TCLK   0x07
#define PQ_CMX_FCC_TSHIFT 0


/*------------------*/
/*  CPM MUX         */
/*------------------*/
#define PQ_CMX_UAR_MAD3  0x0100
#define PQ_CMX_UAR_MAD4  0x0200
#define PQ_CMX_UAR_SAD4  0x0800
#define PQ_CMX_UAR_SAD3  0x1000
#define PQ_CMX_UAR_SAD2  0x2000
#define PQ_CMX_UAR_SAD1  0x4000
#define PQ_CMX_UAR_SAD0  0x8000

typedef struct pq_cpm_mux_s {
      pq_byte  si1cr;           /* CPM mux SI1 clock route register 15.4.2/15-10 */
      pq_byte  reserved1;
      pq_byte  si2cr;           /* CPM mux SI2 clock route register 15.4.3/15-11 */
      pq_byte  reserved2;
      pq_word  fcr;             /* CPM mux FCC clock route register 15.4.4/15-12 */
      pq_word  scr;             /* CPM mux SCC clock route register 15.4.5/15-14 */
      pq_byte  smr;             /* CPM mux SMC clock route register 15.4.6/15-17 */
      pq_byte  reserved3;
      pq_hword uar;             /* CPM mux UTOPIA address register 15.4.1/15-7 */
      pq_byte  reserved4[16];
} pq_cpm_mux;

typedef struct pq_si_regs_s {
      pq_hword amr;             /* SI TDMA mode register 14.5.2/14-17 */
      pq_hword bmr;             /* SI TDMB mode register */
      pq_hword cmr;             /* SI TDMC mode register */
      pq_hword dmr;             /* SI TDMD mode register */
      pq_byte  gmr;             /* SI global mode register 14.5.1/14-17 */
      pq_byte  reserved1;
      pq_byte  cmdr;            /* SI command register 14.5.4/14-24 */
      pq_byte  reserved2;
      pq_byte  str;             /* SI status register 14.5.5/14-25 */
      pq_byte  reserved3;
      pq_hword rsr;             /* SI RAM shadow address register 14.5.3/14-23 */
} pq_si_regs;

typedef struct pq_mcc_regs_s {
      pq_hword mcce;            /* MCC event register 27.10.1/27-18 */
      pq_byte  reserved1[2];
      pq_hword mccm;            /* MCC mask register */
      pq_byte  reserved2[2];
      pq_byte  mccf;            /* MCC configuration register 27.8/27-15 */
      pq_byte  reserved3[7];
} pq_mcc_regs;

typedef struct pq_si_ram_s {
      pq_hword tx[256];         /* SI transmit routing RAM 14.4.3/14-10 */
      pq_byte  reserved1[512];
      pq_hword rx[256];         /* SI receive routing RAM 14.4.3/14-10 */
      pq_byte  reserved2[512];
} pq_si_ram;


/* PCI controller parameters (MPC8250, MPC8255, MPC8265, MPC8270, MPC8280) */
typedef struct pq_pci_s
{
      pq_word  omisr;
      pq_word  omimr;
      pq_word  reserved1[2];
      pq_word  ifqpr;
      pq_word  ofqpr;
      pq_word  reserved2[2];
      pq_word  imr0;
      pq_word  imr1;
      pq_word  omr0;
      pq_word  omr1;
      pq_word  odr;
      pq_word  reserved3;
      pq_word  idr;
      pq_word  reserved4[5];
      pq_word  imisr;
      pq_word  imimr;
      pq_word  reserved5[6];
      pq_word  ifhpr;
      pq_word  reserved6;
      pq_word  iftpr;
      pq_word  reserved7;
      pq_word  iphpr;
      pq_word  reserved8;
      pq_word  iptpr;
      pq_word  reserved9;
      pq_word  ofhpr;
      pq_word  reserved10;
      pq_word  oftpr;
      pq_word  reserved11;
      pq_word  ophpr;
      pq_word  reserved12;
      pq_word  optpr;
      pq_word  reserved13[2];
      pq_word  mucr;
      pq_word  reserved14[2];
      pq_word  qbar;
      pq_word  reserved15[3];
      pq_word  dmamr0;
      pq_word  dmasr0;
      pq_word  dmacdar0;
      pq_word  reserved16;
      pq_word  dmasar0;
      pq_word  reserved17;
      pq_word  dmadar0;
      pq_word  reserved18;
      pq_word  dmabcr0;
      pq_word  dmandar0;
      pq_word  reserved19[22];
      pq_word  dmamr1;
      pq_word  dmasr1;
      pq_word  dmacdar1;
      pq_word  reserved20;
      pq_word  dmasar1;
      pq_word  reserved21;
      pq_word  dmadar1;
      pq_word  reserved22;
      pq_word  dmabcr1;
      pq_word  dmandar1;
      pq_word  reserved23[22];
      pq_word  dmamr2;
      pq_word  dmasr2;
      pq_word  dmacdar2;
      pq_word  reserved24;
      pq_word  dmasar2;
      pq_word  reserved25;
      pq_word  dmadar2;
      pq_word  reserved26;
      pq_word  dmabcr2;
      pq_word  dmandar2;
      pq_word  reserved27[22];
      pq_word  dmamr3;
      pq_word  dmasr3;
      pq_word  dmacdar3;
      pq_word  reserved28;
      pq_word  dmasar3;
      pq_word  reserved29;
      pq_word  dmadar3;
      pq_word  reserved30;
      pq_word  dmabcr3;
      pq_word  dmandar3;
      pq_word  reserved31[86];
      pq_word  potar0;
      pq_word  reserved32;
      pq_word  pobar0;
      pq_word  reserved33;
      pq_word  pocmr0;
      pq_word  reserved34;
      pq_word  potar1;
      pq_word  reserved35;
      pq_word  pobar1;
      pq_word  reserved36;
      pq_word  pocmr1;
      pq_word  reserved37;
      pq_word  potar2;
      pq_word  reserved38;
      pq_word  pobar2;
      pq_word  reserved39;
      pq_word  pocmr2;
      pq_word  reserved40[13];
      pq_word  ptcr;
      pq_word  gpcr;
      pq_word  pci_gcr;
      pq_word  esr;
      pq_word  emr;
      pq_word  ecr;
      pq_word  pci_eacr;
      pq_word  reserved41;
      pq_word  pci_edcr;
      pq_word  pci_eccr;
      pq_word  reserved42[12];
      pq_word  pitar1;
      pq_word  reserved43;
      pq_word  pibar1;
      pq_word  reserved44;
      pq_word  picmr1;
      pq_word  reserved45;
      pq_word  pitar0;
      pq_word  reserved46;
      pq_word  pibar0;
      pq_word  reserved47;
      pq_word  picmr0;
      pq_word  reserved48;
      pq_word  cfg_addr;
      pq_word  cfg_data;
      pq_word  intack;
      pq_word  reserved49[189];
} pq_pci;

/* POCMRx register encoding */
#define PQ_POCMR_EN           0x80000000
#define PQ_POCMR_IO           0x40000000
#define PQ_POCMR_PRE          0x20000000
#define PQ_POTA_ADDR_SHIFT    12          /* Address shift for all outbound
					     address translation registers */

/* PCIBRx register encoding */
#define PQ_PCIBR_ENABLE       0x00000001

/* PICMRx register encoding */
#define PQ_PICMR_EN           0x80000000
#define PQ_PICMR_NOSNOOP      0x40000000
#define PQ_PICMR_PREFETCH     0x20000000
#define PQ_PITA_ADDR_SHIFT    12          /* Address shift for all inbound
					     address translation registers */

typedef struct pq_intram_s {
      /* CPM Dual-Port RAM */
      pq_byte dpram1[16*1024];    /* Dual-port RAM 16 Kbytes 13.5/13-15 */
      pq_byte reserved1[16*1024]; /* 16 Kbytes */
      struct pq_pram_s {          /* Parameter RAM 4 Kbytes 13.5.2/13-17 */
            pq_byte       scc[4][256];
            pq_byte       fcc[3][256];
            pq_byte       mcc1[128];
            pq_byte       reserved1[124];
            pq_hword      smc1_base;
            pq_hword      idma1_base;
            pq_byte       mcc2[128];
            pq_byte       reserved2[124];
            pq_hword      smc2_base;
            pq_hword      idma2_base;
            pq_byte       reserved3[252];
            pq_hword      spi_base;
            pq_hword      idma3_base;
            pq_byte       reserved4[224];
            pq_rtmr_parms risc_tmr;
            pq_hword      rev_num;
            pq_byte       reserved5[2];
            pq_byte       reserved6[4];
            pq_word       rand;
            pq_hword      i2c_base;
            pq_hword      idma4_base;
	    pq_usb_parms  usb;
            pq_byte       reserved7[1248];
      } _PackedType pram;
      pq_byte reserved2[8*1024];  /* 8 Kbytes */
      pq_byte dpram3[4*1024];     /* Dual-port RAM 4 Kbytes 13.5/13-15 */
      pq_byte reserved3[16*1024]; /* 16 Kbytes */

      /* Memory-mapped registers */
      pq_siu      siu;            /* General SIU */
      pq_memctl   memc;           /* Memory controller */
      pq_sitmr    sit;            /* System Integration Timers */
      pq_pci      pci;            /* PCI controller */
      pq_intctl   pic;            /* Interrupt Controller */
      pq_clocks   clock;          /* Clocks and Reset */
      pq_port     gpio[4];        /* Input/Output Ports A-D */
      pq_timers   gpt;            /* CPM Timers */
      pq_sdma     sdma;           /* SDMA-General */
      pq_idma     idma[4];        /* IDMA */
      pq_byte     reserved4[0x2C0];
      pq_fcc_regs fcc[3];         /* FCCs 1-3 */
      pq_byte     reserved5[0x290];
      pq_brgc     brgc58[4];      /* BRGs 5-8 */
      pq_byte     reserved6[0x260];
      pq_i2c_regs i2c;            /* I2C */
      pq_commproc cp;             /* Communications Processor */
      pq_brgc     brgc14[4];      /* BRGs 1-4 */
      pq_scc_regs scc[4];         /* SCCs 1-4 */
      pq_smc_regs smc[2];         /* SMCs 1-2 */
      pq_spi_regs spi;            /* SPI */
      pq_cpm_mux  cmx;            /* CPM Mux */
      pq_si_regs  si1;            /* SI1 */
      pq_mcc_regs mcc1;           /* MCC1 */
      pq_si_regs  si2;            /* SI2 */
      pq_mcc_regs mcc2;           /* MCC2 */
      pq_usb_regs usb;
      pq_byte     reserved7[0x480];
      pq_si_ram   si_ram[2];      /* SI RAMs 1-2 */
      pq_byte     reserved8[0x800];
      pq_byte     reserved9[0x800];
} _PackedType pq_intram;

extern volatile pq_intram *pq_immr;

int  pq_cp_cmd(int command, int resource, int mcc_channel);
void pq_cp_reset(void);

void pq_gpio_init(pq_port_id port_id, pq_word par, pq_word dir,
                  pq_word sor, pq_word odr);
void pq_gpio_set(pq_port_id port_id, pq_word par, pq_word dir,
                 pq_word sor, pq_word odr);
void pq_gpio_clear(pq_port_id port_id, pq_word par, pq_word dir,
                   pq_word sor, pq_word odr);
pq_word pq_gpio_out(pq_port_id port_id, pq_word dat_clr, pq_word dat_set);
pq_word pq_gpio_in(pq_port_id port_id);

/*
 * Id conversion functions
 */
static __inline pq_scc_id pq_xcc2scc(pq_xcc_id xccid)
{
   return pq_scc1 + (xccid-pq_xscc1);
}
static __inline pq_fcc_id pq_xcc2fcc(pq_xcc_id xccid)
{
   return pq_fcc1 + (xccid-pq_xfcc1);
}
static __inline pq_mcc_id pq_xcc2mcc(pq_xcc_id xccid)
{
   return pq_mcc1 + (xccid-pq_xmcc1);
}
static __inline pq_smc_id pq_xcc2smc(pq_xcc_id xccid)
{
   return pq_smc1 + (xccid-pq_xsmc1);
}

static __inline pq_xcc_id pq_scc2xcc(pq_scc_id id)
{
   return pq_xscc1 + (id-pq_scc1);
}
static __inline pq_xcc_id pq_fcc2xcc(pq_fcc_id id)
{
   return pq_xfcc1 + (id-pq_fcc1);
}
static __inline pq_xcc_id pq_mcc2xcc(pq_mcc_id id)
{
   return pq_xmcc1 + (id-pq_mcc1);
}
static __inline pq_xcc_id pq_smc2xcc(pq_smc_id id)
{
   return pq_xsmc1 + (id-pq_smc1);
}

#endif /* _PQ2DEFS_H_ */
