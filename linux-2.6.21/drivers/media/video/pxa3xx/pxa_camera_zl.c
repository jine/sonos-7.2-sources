/*
 * pxa_camera_zl - main file for camera driver
 *
 * Copyright (C) 2005, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa3xx_pmic.h>

#ifdef CONFIG_DVFM
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>
#endif

#ifdef CONFIG_IPM
#include <asm/arch/ipmc.h>
#endif


#include <linux/mm.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>

#include <linux/pxa_camera_zl.h>
#include "ov2620.h"
#include "ov2630.h"
#include "ov7660.h"
#include "ov5623.h"
#include "ov7673.h"
#include "camera.h"
#include "ci.h"

int ov7660_detected = 0;
int ov2620_detected = 0;
int ov2630_detected = 0;
int ov5623_detected = 0;
int ov7673_detected = 0;

/* #define HW_IP_OV7660 */

#define assert(expr) ((void) ((expr) ? 0 : ( \
				printk(KERN_ALERT		\
				"%s: assert failed at line %d",	\
				 __func__, __LINE__))))

#define	CAMERA_DEBUG
#undef CAMERA_DEBUG

#ifdef  CAMERA_DEBUG
#define CAMERA_DEBUG_IRQ
#define	dbg(fmt, arg...)    printk(KERN_INFO "%s(line %d): " fmt "\n", 	\
		__FUNCTION__, __LINE__, ##arg)
#define PRINTFUNC printk(KERN_INFO "%s\n", __FUNCTION__)
#define TESTPOINT do {							\
	printk(KERN_INFO "\n");						\
	printk(KERN_INFO "%s(%s):%d\n", __FUNCTION__, __FILE__, __LINE__);\
	camera_print_context();						\
} while(0)
#else
#define	dbg(fmt, arg...)	do {} while (0)
#define	PRINTFUNC 		do {} while (0)
#define	TESTPOINT		do {} while (0)
#endif

#define        PXA_CAMERA_VERSION    KERNEL_VERSION(0,0,1)

/*
 * main camera driver macros and data
 */

/* This mask just enable EOFX interrupt */
#define    INT_MASK    CAMERA_INTMASK_FIFO_OVERRUN |	\
	CAMERA_INTMASK_END_OF_FRAME |			\
	CAMERA_INTMASK_START_OF_FRAME |			\
	CAMERA_INTMASK_CI_DISABLE_DONE |		\
	CAMERA_INTMASK_CI_QUICK_DISABLE |		\
	CAMERA_INTMASK_PARITY_ERROR |			\
	CAMERA_INTMASK_END_OF_LINE |			\
	CAMERA_INTMASK_FIFO_EMPTY  |			\
	CAMERA_INTMASK_TIME_OUT  |			\
	CAMERA_INTMASK_FIFO3_UNDERRUN |			\
	CAMERA_INTMASK_BRANCH_STATUS |			\
	/*CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER |*/	\
	CAMERA_INTMASK_DMA_CHANNEL0_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL1_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL2_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL3_STOP

#define DMA_DESCRIPTOR_SIZE (sizeof(CI_DMAC_DESCRIPTOR_T))

#define SENSOR_NUMBER  4

#ifdef	CONFIG_MACH_ZYLONITE
#define SET_OV2620_SENSOR(cam_ctx)  do {                	\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_2620;    	\
	cam_ctx->camera_functions->init = ov2620_init;     	\
	cam_ctx->camera_functions->deinit = ov2620_deinit;     	\
	cam_ctx->camera_functions->set_capture_format = 	\
			ov2620_set_capture_format; 		\
	cam_ctx->camera_functions->start_capture = 		\
			ov2620_start_capture; 			\
	cam_ctx->camera_functions->stop_capture =		\
			ov2620_stop_capture;	 		\
	cam_ctx->camera_functions->sleep = ov2620_sleep;	\
	cam_ctx->camera_functions->wakeup = ov2620_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov2620_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov2620_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode = 		\
			ov2620_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov2620_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov2620_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov2620_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while(0)

#define SET_OV2630_SENSOR(cam_ctx)  do {                	\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_2630;	\
	cam_ctx->camera_functions->init = ov2630_init;     	\
	cam_ctx->camera_functions->deinit = ov2630_deinit;     	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov2630_set_capture_format; 		\
	cam_ctx->camera_functions->start_capture =		\
			ov2630_start_capture;	 		\
	cam_ctx->camera_functions->stop_capture =		\
			ov2630_stop_capture; 			\
	cam_ctx->camera_functions->sleep = ov2630_sleep;	\
	cam_ctx->camera_functions->wakeup = ov2630_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov2630_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov2630_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov2630_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov2630_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov2630_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov2630_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while(0)

#define SET_OV7660_SENSOR(cam_ctx)  do {			\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_7660;	\
	cam_ctx->camera_functions->init = ov7660_init;		\
	cam_ctx->camera_functions->deinit = ov7660_deinit;	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov7660_set_capture_format;		\
	cam_ctx->camera_functions->start_capture =		\
			ov7660_start_capture;			\
	cam_ctx->camera_functions->stop_capture =		\
			ov7660_stop_capture;			\
	cam_ctx->camera_functions->sleep = ov7660_sleep;	\
	cam_ctx->camera_functions->wakeup = ov7660_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov7660_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov7660_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov7660_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov7660_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov7660_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov7660_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while(0)
#elif	defined(CONFIG_MACH_LITTLETON)
#define	SET_OV2620_SENSOR(cam_ctx) do {} while (0)
#define	SET_OV2630_SENSOR(cam_ctx) do {} while (0)
#define SET_OV7660_SENSOR(cam_ctx) do {} while (0)
#endif

#ifdef CONFIG_IMM
#define LUT_LUTDES_USE_ISRAM
#endif

#ifdef LUT_LUTDES_USE_ISRAM
#include <asm/arch/imm.h>
#endif

#ifdef	CONFIG_MACH_LITTLETON
#define SET_OV5623_SENSOR(cam_ctx)  do {                	\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_5623;	\
	cam_ctx->camera_functions->init = ov5623_init;     	\
	cam_ctx->camera_functions->deinit = ov5623_deinit;     	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov5623_set_capture_format; 		\
	cam_ctx->camera_functions->start_capture =		\
			ov5623_start_capture;	 		\
	cam_ctx->camera_functions->stop_capture =		\
			ov5623_stop_capture; 			\
	cam_ctx->camera_functions->sleep = ov5623_sleep;	\
	cam_ctx->camera_functions->wakeup = ov5623_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov5623_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov5623_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov5623_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov5623_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov5623_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov5623_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while(0)

#define SET_OV7673_SENSOR(cam_ctx)  do {                	\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_7673;	\
	cam_ctx->camera_functions->init = ov7673_init;     	\
	cam_ctx->camera_functions->deinit = ov7673_deinit;     	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov7673_set_capture_format; 		\
	cam_ctx->camera_functions->start_capture =		\
			ov7673_start_capture;	 		\
	cam_ctx->camera_functions->stop_capture =		\
			ov7673_stop_capture; 			\
	cam_ctx->camera_functions->sleep = ov7673_sleep;	\
	cam_ctx->camera_functions->wakeup = ov7673_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov7673_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov7673_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov7673_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov7673_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov7673_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov7673_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while(0)
#else
#define	SET_OV5623_SENSOR(cam_ctx) do {} while (0)
#define	SET_OV7673_SENSOR(cam_ctx) do {} while (0)
#endif

/* default value */
#define WIDTH_DEFT		176
#define HEIGHT_DEFT		144
#define FRAMERATE_DEFT		0

/* sensor capability*/
#define OV_2620_MAX_WIDTH	1600
#define OV_2620_MAX_HEIGHT	1200
#define OV_2620_MIN_WIDTH	2
#define OV_2620_MIN_HEIGHT	2

#define OV_2630_MAX_WIDTH	1600
#define OV_2630_MAX_HEIGHT	1200
#define OV_2630_MIN_WIDTH	2
#define OV_2630_MIN_HEIGHT	2

#define OV_7660_MAX_WIDTH	640
#define OV_7660_MAX_HEIGHT	480
#define OV_7660_MIN_WIDTH	88
#define OV_7660_MIN_HEIGHT	72

#define OV_5623_MAX_WIDTH	2560
#define OV_5623_MAX_HEIGHT	1920
#define OV_5623_MIN_WIDTH	2
#define OV_5623_MIN_HEIGHT	2

#define OV_7673_MAX_WIDTH	640
#define OV_7673_MAX_HEIGHT	480
#define OV_7673_MIN_WIDTH	88
#define OV_7673_MIN_HEIGHT	72

extern int pxa_i2c_write(u8 slaveaddr, const u8 * bytesbuf, u32 bytescount);

static int pxa_camera_minor = 0;
static p_camera_context_t g_camera_context;

#ifdef LUT_LUTDES_USE_ISRAM
static u32 zylcam_immid = 0;
#endif

struct pxa_camera_driver_status_t {
	int 		still_buf_ready;
	int 		video_buf_ready;
	int 		still_buf_submited;
	int 		video_buf_submited;
	int 		still_capture_started;
	int 		video_capture_started;
	struct 		list_head still_buf_head;
	struct 		list_head video_buf_head;
	struct 		list_head still_report_head;
	struct 		list_head video_report_head;
	unsigned int 	still_timeperframe_numerator;
	unsigned int 	video_timeperframe_numerator;
	unsigned int 	still_timeperframe_denominator;
	unsigned int 	video_timeperframe_denominator;

	int 		capture_mode;
	int 		*p_buf_ready;
	int 		*p_buf_submited;
	int 		*p_capture_started;
	struct 		list_head *p_buf_head;
	struct 		list_head *p_report_head;
	unsigned int 	*p_timeperframe_numerator;
	unsigned int 	*p_timeperframe_denominator;
	wait_queue_head_t camera_wait_q;
#ifdef CONFIG_DVFM
	wait_queue_head_t wait_outd0cs;
#endif
	int 		task_waiting;
	int 		driver_opened;

	int 		re_init_needed;
	int 		re_bufprepare_needed;
	int 		re_bufsubmit_needed;
	int 		re_formatset_needed;

	int 		i2c_inited;
	int 		suspended;
};
static struct pxa_camera_driver_status_t g_camdrv_status;

static int pxa_camera_open(struct inode *inode, struct file *file);
static int pxa_camera_close(struct inode *inode, struct file *file);
static int pxa_camera_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param);
static ssize_t pxa_camera_read(struct file *file, char __user * buf,
		size_t count, loff_t * ppos);
static int pxa_camera_mmap(struct file *file, struct vm_area_struct *vma);
static unsigned int pxa_camera_poll(struct file *file, poll_table * wait);
static void pxa_camera_release(struct video_device *dev);

/* Internal function */
static int pxa_camera_ioctl_streamon(struct pxa_camera_driver_status_t
		*p_camdrv_status);
static void pxa_camera_ioctl_streamoff(struct pxa_camera_driver_status_t
		*p_camdrv_status);
static int pxa_camera_reset(struct pxa_camera_driver_status_t *p_camdrv_status);
static int pxa_camera_get_framerate(struct pxa_camera_driver_status_t
		*p_camdrv_status);

static struct file_operations pxa_camera_fops = {
	.owner 		= THIS_MODULE,
	.open 		= pxa_camera_open,
	.release 	= pxa_camera_close,
	.ioctl 		= pxa_camera_ioctl,
	.read 		= pxa_camera_read,
	.mmap 		= pxa_camera_mmap,
	.poll 		= pxa_camera_poll,
	.llseek 	= no_llseek,
};

static struct video_device vd = {
	.name 		= "PXA Camera",
	.type 		= VID_TYPE_CAPTURE,
	.hardware 	= 50,		/* FIXME */
	.fops 		= &pxa_camera_fops,
	.release 	= pxa_camera_release,
	.minor 		= -1,
};

struct pxa_camera_format {
	char *name;		/* format description */
	int palette;		/* video4linux 1      */
	int fourcc;		/* video4linux 2      */
	int depth;		/* bit/pixel          */
};

const struct pxa_camera_format pxa_camera_formats[] = {
	{
		.name 		= "RGB565",
		.palette 	= VIDEO_PALETTE_RGB565,
		.fourcc 	= V4L2_PIX_FMT_RGB565X,
		.depth 	= 16,
	}, {
#if defined(CONFIG_CPU_PXA310)
		.name 		= "YCbCr4:2:0",
		.palette 	= VIDEO_PALETTE_YUV420,
		.fourcc 	= V4L2_PIX_FMT_YUV420,
		.depth 	= 16,
	}, {
#endif
		.name 		= "YCbCr4:2:2(Planar)",
		.palette 	= VIDEO_PALETTE_YUV422P,
		.fourcc 	= V4L2_PIX_FMT_YUV422P,
		.depth 		= 16,
	}, {
		.name 		= "RawRGGB 8 bit",
		.palette 	= VIDEO_PALETTE_RGGB8,
		.fourcc 	= V4L2_PIX_FMT_SRGGB8,
		.depth 		= 8,
	}, {
		.name 		= "RawRGGB 10 bit",
		.palette 	= VIDEO_PALETTE_RGGB10,
		.fourcc 	= V4L2_PIX_FMT_SRGGB10,
		.depth 		= 16,
	}
};

const unsigned int PXA_CAMERA_FORMATS = ARRAY_SIZE(pxa_camera_formats);

/* look up table: sensor input format <--> qci output format*/
struct pxa_camera_format_t {
	unsigned int input_format;
	unsigned int output_format;
};

static struct pxa_camera_format_t ov7660_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RGB565,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB565,
	},
#ifdef HW_IP_OV7660
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB888_PACKED,
	},
#else
	{
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	},
#endif
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
#if defined(CONFIG_CPU_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1
	},
};

static struct pxa_camera_format_t ov2620_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	},
#if 0				/*will support in MH B0 */
	{CAMERA_IMAGE_FORMAT_RAW10, CAMERA_IMAGE_FORMAT_RGB565},
	{CAMERA_IMAGE_FORMAT_RAW10, CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR},
#endif
	{
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1
	},
};

static struct pxa_camera_format_t ov2630_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB888_PACKED,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
#if defined(CONFIG_CPU_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
	},
};

static struct pxa_camera_format_t ov5623_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10, 
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10, 
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10, 
		.output_format	= CAMERA_IMAGE_FORMAT_RGB888_PACKED,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10, 
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
#if defined(CONFIG_CPU_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1, 
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
	},
};

static struct pxa_camera_format_t ov7673_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RGB565,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB565,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
#if defined(CONFIG_CPU_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1
	},
};

/*
 * buffer list
 */

struct buf_node {
	unsigned int io_type;           /* camera IO methods */
	struct list_head buf_head;	/* For buffer list */
	struct list_head report_head;	/* For report buffer list */
	void *vaddr;		/* vmap() return virtual address */
	struct page **pages;	/* physical pages */
	int page_num;		/* physical pages count */
	int buf_id;		/* buffer id to let driver access */
	int buf_index;		/* buffer index */
	int size;		/* buffer size */
	void *dma_desc_vaddr;	/* dma description virtual address */
	dma_addr_t dma_desc_paddr;	/* dma description physical address */
	int dma_desc_size;	/* dma description size */
	void *Y_vaddr;		/* Y virtual address */
	void *Cb_vaddr;		/* Cb virtual address */
	void *Cr_vaddr;		/* Cr virtual address */
	int fifo0_size;		/* fifo0 data transfer size */
	int fifo1_size;		/* fifo1 data transfer size */
	int fifo2_size;		/* fifo2 data transfer size */
};

struct semaphore buf_list_sem;
static spinlock_t report_list_lock;	/* Spin lock for report_list */
static spinlock_t cam_queue_lock;	/* Spin lock for queue */

/* page cache */

#define        MAX_PAGE_CACHE         256
struct page_cache_head {
	struct list_head page_list;
	int page_count;
	spinlock_t lock;
};
static struct page_cache_head pc_head;

#ifdef CONFIG_DVFM
static spinlock_t cam_status_lock;
static int pxa_camera_dvfm_notifier(unsigned int cmd, void *client_data,
		void *info);
static struct pxa3xx_fv_notifier dvfm_notifier = {
	.name 		= "pxa2xx-camera",
	.priority 	= 0,
	.notifier_call 	= pxa_camera_dvfm_notifier,
};
#endif

#ifdef CAMERA_DEBUG
static void camera_print_context(void)
{
	int i, j;
	p_camera_context_t cam_ctx = g_camera_context;
	p_camera_frame_buffer_info_t buffer;
	camera_frame_buffer_queue_t *queue;
	CI_DMAC_DESCRIPTOR_T *cur_des_virtual;
	unsigned int cur_des_physical;
	CI_DMAC_DESCRIPTOR_T *init_descriptor;
	unsigned int init_descriptor_physical;
	unsigned long flags;

	if (!g_camera_context) {
		printk(KERN_ALERT "cam_ctx == NULL!\n");
	}

	printk(KERN_ALERT "cam_ctx->sensor_type                 = %d\n",
			cam_ctx->sensor_type);
	printk(KERN_ALERT "cam_ctx->capture_mode                = %d\n",
			cam_ctx->capture_mode);
	printk(KERN_ALERT "cam_ctx->video_capture_width         = %d\n",
			cam_ctx->video_capture_width);
	printk(KERN_ALERT "cam_ctx->video_capture_height        = %d\n",
			cam_ctx->video_capture_height);
	printk(KERN_ALERT "cam_ctx->video_capture_scale         = %d\n",
			cam_ctx->video_capture_scale);
	printk(KERN_ALERT "cam_ctx->video_capture_input_format  = %d\n",
			cam_ctx->video_capture_input_format);
	printk(KERN_ALERT "cam_ctx->video_capture_output_format = %d\n",
			cam_ctx->video_capture_output_format);
	printk(KERN_ALERT "cam_ctx->still_capture_width         = %d\n",
			cam_ctx->still_capture_width);
	printk(KERN_ALERT "cam_ctx->still_capture_height        = %d\n",
			cam_ctx->still_capture_height);
	printk(KERN_ALERT "cam_ctx->still_capture_scale         = %d\n",
			cam_ctx->still_capture_scale);
	printk(KERN_ALERT "cam_ctx->still_capture_input_format  = %d\n",
			cam_ctx->still_capture_input_format);
	printk(KERN_ALERT "cam_ctx->still_capture_output_format = %d\n",
			cam_ctx->still_capture_output_format);
	printk(KERN_ALERT "cam_ctx->frame_rate                  = %d\n",
			cam_ctx->frame_rate);
	printk(KERN_ALERT "cam_ctx->phantom_buffer_virtual      = 0x%p\n",
			cam_ctx->phantom_buffer_virtual);
	printk(KERN_ALERT "cam_ctx->phantom_buffer_physical     = 0x%x\n",
			(unsigned int)cam_ctx->phantom_buffer_physical);
	printk(KERN_ALERT
			"cam_ctx->histogram_lut_buffer_virtual	 = 0x%p\n",
			cam_ctx->histogram_lut_buffer_virtual);
	printk(KERN_ALERT
			"cam_ctx->histogram_lut_buffer_physical	 = 0x%x\n",
			(unsigned int)cam_ctx->histogram_lut_buffer_physical);
	printk(KERN_ALERT
			"cam_ctx->histogram_lut_dma_descriptors_virtual = 0x%p\n",
			cam_ctx->histogram_lut_dma_descriptors_virtual);
	printk(KERN_ALERT
			"cam_ctx->histogram_lut_dma_descriptors_physical= 0x%x\n",
			(unsigned int)cam_ctx->
				histogram_lut_dma_descriptors_physical);
	printk(KERN_ALERT "cam_ctx->camera_functions            = 0x%x\n",
			(unsigned int)cam_ctx->camera_functions);
	printk(KERN_ALERT "cam_ctx->video_fifo0_transfer_size        = %d\n",
			cam_ctx->video_fifo0_transfer_size);
	printk(KERN_ALERT "cam_ctx->video_fifo1_transfer_size        = %d\n",
			cam_ctx->video_fifo1_transfer_size);
	printk(KERN_ALERT "cam_ctx->video_fifo2_transfer_size        = %d\n",
			cam_ctx->video_fifo2_transfer_size);

	printk(KERN_ALERT "cam_ctx->still_fifo0_transfer_size        = %d\n",
			cam_ctx->still_fifo0_transfer_size);
	printk(KERN_ALERT "cam_ctx->still_fifo1_transfer_size        = %d\n",
			cam_ctx->still_fifo1_transfer_size);
	printk(KERN_ALERT "cam_ctx->still_fifo2_transfer_size        = %d\n",
			cam_ctx->still_fifo2_transfer_size);

	printk(KERN_ALERT "cam_ctx->fifo0_transfer_size        = %d\n",
			cam_ctx->fifo0_transfer_size);
	printk(KERN_ALERT "cam_ctx->fifo1_transfer_size        = %d\n",
			cam_ctx->fifo1_transfer_size);
	printk(KERN_ALERT "cam_ctx->fifo2_transfer_size        = %d\n",
			cam_ctx->fifo2_transfer_size);

	if (cam_ctx->camera_functions) {
		printk(KERN_ALERT
				"cam_ctx->camera_functions->init = 0x%p\n",
				cam_ctx->camera_functions->init);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->deinit = 0x%p\n",
				cam_ctx->camera_functions->deinit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_capture_format= 0x%p\n",
				cam_ctx->camera_functions->
						set_capture_format);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"start_capture     = 0x%p\n",
				cam_ctx->camera_functions->
						start_capture);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"stop_capture      = 0x%p\n",
				cam_ctx->camera_functions->stop_capture);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->sleep = 0x%p\n",
				cam_ctx->camera_functions->sleep);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->wakeup = 0x%p\n",
				cam_ctx->camera_functions->wakeup);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->read_8bit = 0x%p\n",
				cam_ctx->camera_functions->read_8bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->write_8bit = 0x%p\n",
				cam_ctx->camera_functions->write_8bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->read_16bit = 0x%p\n",
				cam_ctx->camera_functions->read_16bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->write_16bit = 0x%p\n",
				cam_ctx->camera_functions->write_16bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->read_32bit = 0x%p\n",
				cam_ctx->camera_functions->read_32bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->write_32bit = 0x%p\n",
				cam_ctx->camera_functions->write_32bit);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_power_mode    = 0x%p\n",
				cam_ctx->camera_functions->set_power_mode);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_contrast      = 0x%p\n",
				cam_ctx->camera_functions->set_contrast);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_whitebalance  = 0x%p\n",
				cam_ctx->camera_functions->set_whitebalance);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_exposure      = 0x%p\n",
				cam_ctx->camera_functions->set_exposure);
		printk(KERN_ALERT
				"cam_ctx->camera_functions->"
				"set_zoom          = 0x%p\n",
				cam_ctx->camera_functions->set_zoom);
	}

	init_descriptor =
		(CI_DMAC_DESCRIPTOR_T *)((long)cam_ctx->phantom_buffer_virtual +
					  48);
	init_descriptor_physical = cam_ctx->phantom_buffer_physical + 48;
	printk(KERN_ALERT "init_descriptor_vir:         = 0x%p\n",
			init_descriptor);
	printk(KERN_ALERT "init_descriptor_physical:    = 0x%x\n",
			init_descriptor_physical);
	printk(KERN_ALERT "init_descriptor_vir->ddadr   = 0x%08x\n",
			(unsigned int)init_descriptor->ddadr);
	printk(KERN_ALERT "init_descriptor_vir->dsadr   = 0x%08x\n",
			(unsigned int)init_descriptor->dsadr);
	printk(KERN_ALERT "init_descriptor_vir->dtadr   = 0x%08x\n",
			(unsigned int)init_descriptor->dtadr);
	printk(KERN_ALERT "init_descriptor_vir->dcmd    = 0x%08x\n",
			(unsigned int)init_descriptor->dcmd);

	printk(KERN_ALERT "cam_ctx->frame_buffer_number = %d\n",
			cam_ctx->frame_buffer_number);
	for (i = 0; i < cam_ctx->frame_buffer_number; i++) {
		buffer = &cam_ctx->master_frame_buffer_list[i];
		printk(KERN_ALERT "buffer %d:0x%p\n", i, buffer);
		printk(KERN_ALERT "  frame_id           = 0x%x\n",
				buffer->frame_id);
		printk(KERN_ALERT "  buffer_vir_addr    = 0x%p\n",
				buffer->buffer_vir_addr);
		printk(KERN_ALERT "  buffer_size        = 0x%x\n",
				buffer->buffer_size);
		printk(KERN_ALERT "  dma_descriptors_virtual    = 0x%p\n",
				buffer->dma_descriptors_virtual);
		printk(KERN_ALERT "  pY                 = 0x%p\n", buffer->pY);
		printk(KERN_ALERT "  pCb                = 0x%p\n", buffer->pCb);
		printk(KERN_ALERT "  pCr                = 0x%p\n", buffer->pCr);
		printk(KERN_ALERT "  ch0_dma_desc_num   = %d\n",
				buffer->ch0_dma_desc_num);
		printk(KERN_ALERT "  ch1_dma_desc_num   = %d\n",
				buffer->ch1_dma_desc_num);
		printk(KERN_ALERT "  ch2_dma_desc_num   = %d\n",
				buffer->ch2_dma_desc_num);
		printk(KERN_ALERT "  ch0_phantom_dma_desc_num   = %d\n",
				buffer->ch0_phantom_dma_desc_num);
		printk(KERN_ALERT "  ch1_phantom_dma_desc_num   = %d\n",
				buffer->ch1_phantom_dma_desc_num);
		printk(KERN_ALERT "  ch2_phantom_dma_desc_num   = %d\n",
				buffer->ch2_phantom_dma_desc_num);

		cur_des_virtual = buffer->ch0_dma_desc_vir_addr[0];
		cur_des_physical = buffer->ch0_dma_desc_phy_addr[0];
		printk(KERN_ALERT
				"  -----------fifo0 dma chain--------------\n");
		for (j = 0; j < (buffer->ch0_dma_desc_num +
				buffer->ch1_phantom_dma_desc_num); j++) {
			printk(KERN_ALERT "  cur_des_virtual: 0x%p\n",
					cur_des_virtual);
			printk(KERN_ALERT "  cur_des_physical: 0x%x\n",
					cur_des_physical);
			printk(KERN_ALERT "    ddadr = 0x%08x\n",
					(unsigned int)cur_des_virtual->ddadr);
			printk(KERN_ALERT "    dsadr = 0x%08x\n",
					(unsigned int)cur_des_virtual->dsadr);
			printk(KERN_ALERT "    dtadr = 0x%08x\n",
					(unsigned int)cur_des_virtual->dtadr);
			printk(KERN_ALERT "    dcmd  = 0x%08x\n",
					(unsigned int)cur_des_virtual->dcmd);
			cur_des_virtual++;
			cur_des_physical += DMA_DESCRIPTOR_SIZE;
		}

		if (cam_ctx->fifo1_transfer_size != 0) {
			cur_des_virtual = buffer->ch1_dma_desc_vir_addr[0];
			cur_des_physical = buffer->ch1_dma_desc_phy_addr[0];
			if (cur_des_virtual) {
				printk(KERN_ALERT
					"  -------fifo1 dma chain---------\n");
				for (j = 0; j < (buffer->ch1_dma_desc_num +
					buffer->ch1_phantom_dma_desc_num); j++) {
					printk(KERN_ALERT
						"  cur_des_virtual: 0x%p\n",
						cur_des_virtual);
					printk(KERN_ALERT
						"  cur_des_physical: 0x%x\n",
						cur_des_physical);
					printk(KERN_ALERT
						"    ddadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						ddadr);
					printk(KERN_ALERT
						"    dsadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dsadr);
					printk(KERN_ALERT
						"    dtadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dtadr);
					printk(KERN_ALERT
						"    dcmd  = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dcmd);
					cur_des_virtual++;
					cur_des_physical += DMA_DESCRIPTOR_SIZE;
				}
			}
		}

		if (cam_ctx->fifo2_transfer_size != 0) {
			cur_des_virtual = buffer->ch2_dma_desc_vir_addr[0];
			cur_des_physical = buffer->ch2_dma_desc_phy_addr[0];
			if (cur_des_virtual) {
				printk(KERN_ALERT
					"  --------fifo2 dma chain---------\n");
				for (j = 0; j <	(buffer->ch2_dma_desc_num +
					buffer->ch2_phantom_dma_desc_num); j++) {
					printk(KERN_ALERT
						"  cur_des_virtual: 0x%p\n",
						cur_des_virtual);
					printk(KERN_ALERT
						"  cur_des_physical: 0x%x\n",
						cur_des_physical);
					printk(KERN_ALERT
						"    ddadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						ddadr);
					printk(KERN_ALERT
						"    dsadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dsadr);
					printk(KERN_ALERT
						"    dtadr = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dtadr);
					printk(KERN_ALERT
						"    dcmd  = 0x%08x\n",
						(unsigned int)cur_des_virtual->
						dcmd);
					cur_des_virtual++;
					cur_des_physical += DMA_DESCRIPTOR_SIZE;
				}
			}
		}
	}

	spin_lock_irqsave(&cam_queue_lock, flags);
	printk(KERN_ALERT "video capture queue:\n");
	queue = &cam_ctx->video_capture_buffer_queue;
	for (buffer = queue->head, i = 0; buffer;
			buffer = buffer->next_buffer, i++) {
		printk(KERN_ALERT "  buffer %d:0x%p\n", i, buffer);
	}

	printk(KERN_ALERT "still capture queue:\n");
	queue = &cam_ctx->still_capture_buffer_queue;
	for (buffer = queue->head, i = 0; buffer;
			buffer = buffer->next_buffer, i++) {
		printk(KERN_ALERT "  buffer %d:0x%p\n", i, buffer);
	}
	spin_unlock_irqrestore(&cam_queue_lock, flags);

	printk(KERN_ALERT "cam_ctx->ci_disable_complete = %d\n",
			cam_ctx->ci_disable_complete);
	printk(KERN_ALERT "cam_ctx->psu_enable          = %d\n",
			cam_ctx->psu_enable);
	printk(KERN_ALERT "cam_ctx->cgu_enable          = %d\n",
			cam_ctx->cgu_enable);
	printk(KERN_ALERT "cam_ctx->ssu_scale           = %d\n",
			cam_ctx->ssu_scale);
	printk(KERN_ALERT "cam_ctx->cmu_usage           = %d\n",
			cam_ctx->cmu_usage);
	printk(KERN_ALERT "cam_ctx->dma_running         = %d\n",
			cam_ctx->dma_running);
}

#define READ_REG(offset)	(offset)

void DumpReg_Int(unsigned int ci_reg_base)
{
	volatile unsigned int reg_val;
	int num_count = 0;
	num_count++;
	printk(KERN_DEBUG "KERN_DEBUG dump register on channel #0"
		"******************************************************\d\n\n",
		 num_count);
	reg_val = READ_REG(CIDADR0);
	printk(KERN_DEBUG "CIDADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR0);
	printk(KERN_DEBUG "CITADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR0);
	printk(KERN_DEBUG "CISADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD0);
	printk(KERN_DEBUG "CICMD0 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIDADR3);
	printk(KERN_DEBUG "CIDADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR3);
	printk(KERN_DEBUG "CITADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR3);
	printk(KERN_DEBUG "CISADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD3);
	printk(KERN_DEBUG "CICMD3 = 0X%08X\n", reg_val);

	return;

}

void DumpReg(void)
{
	volatile unsigned int reg_val;
	int num_count = 0;
	num_count++;

	printk(KERN_DEBUG "CI REG"
		"******************************************************%d\n\n",
		 num_count);
	reg_val = READ_REG(CICR0);
	printk(KERN_DEBUG "CICR0  = 0X%08X\n\n", reg_val);
	reg_val = READ_REG(CICR1);
	printk(KERN_DEBUG "CICR1  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICR2);
	printk(KERN_DEBUG "CICR2  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICR3);
	printk(KERN_DEBUG "CICR3  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICR4);
	printk(KERN_DEBUG "CICR4  = 0X%08X\n\n", reg_val);

	printk(KERN_DEBUG "*******CI status register\n");
	reg_val = READ_REG(CISR);
	printk(KERN_DEBUG "CISR = 0X%08X\n\n", reg_val);

	reg_val = READ_REG(CITOR);
	printk(KERN_DEBUG "CITOR  = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIBR0);
	printk(KERN_DEBUG "CIBR0  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIBR1);
	printk(KERN_DEBUG "CIBR1  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIBR2);
	printk(KERN_DEBUG "CIBR2  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIBR3);
	printk(KERN_DEBUG "CIBR3  = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIPSS);
	printk(KERN_DEBUG "CIPSS  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIPBUF);
	printk(KERN_DEBUG "CIPBUF = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIHST);
	printk(KERN_DEBUG "CIHST  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CISUM);
	printk(KERN_DEBUG "CISUM  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICCR);
	printk(KERN_DEBUG "CICCR  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CISSC);
	printk(KERN_DEBUG "CISSC  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMR);
	printk(KERN_DEBUG "CICMR  = 0X%08X\n", reg_val);

	reg_val = READ_REG(CICMC0);
	printk(KERN_DEBUG "CICMC0 = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMC1);
	printk(KERN_DEBUG "CICMC1 = 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMC2);
	printk(KERN_DEBUG "CICMC2 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIFR0);
	printk(KERN_DEBUG "CIFR0  = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIFR1);
	printk(KERN_DEBUG "CIFR1  = 0X%08X\n\n", reg_val);

	printk(KERN_DEBUG "*******CI FIFO status register\n");
	reg_val = READ_REG(CIFSR);
	printk(KERN_DEBUG "CIFSR  = 0X%08X\n\n", reg_val);

	printk(KERN_DEBUG "*******CI DMA control/status register\n");
	reg_val = READ_REG(CIDCSR0);
	printk(KERN_DEBUG "CIDCSR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDCSR1);
	printk(KERN_DEBUG "CIDCSR1= 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDCSR2);
	printk(KERN_DEBUG "CIDCSR2= 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDCSR3);
	printk(KERN_DEBUG "CIDCSR3= 0X%08X\n\n", reg_val);

	reg_val = READ_REG(CIDBR0);
	printk(KERN_DEBUG "CIDBR0 = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDBR1);
	printk(KERN_DEBUG "CIDBR1 = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDBR2);
	printk(KERN_DEBUG "CIDBR2 = 0X%08X\n", reg_val);
	reg_val = READ_REG(CIDBR3);
	printk(KERN_DEBUG "CIDBR3 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIDADR0);
	printk(KERN_DEBUG "CIDADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR0);
	printk(KERN_DEBUG "CITADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR0);
	printk(KERN_DEBUG "CISADR0= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD0);
	printk(KERN_DEBUG "CICMD0 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIDADR1);
	printk(KERN_DEBUG "CIDADR1= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR1);
	printk(KERN_DEBUG "CITADR1= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR1);
	printk(KERN_DEBUG "CISADR1= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD1);
	printk(KERN_DEBUG "CICMD1 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIDADR2);
	printk(KERN_DEBUG "CIDADR2= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR2);
	printk(KERN_DEBUG "CITADR2= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR2);
	printk(KERN_DEBUG "CISADR2= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD2);
	printk(KERN_DEBUG "CICMD2 = 0X%08X\n", reg_val);

	reg_val = READ_REG(CIDADR3);
	printk(KERN_DEBUG "CIDADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CITADR3);
	printk(KERN_DEBUG "CITADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CISADR3);
	printk(KERN_DEBUG "CISADR3= 0X%08X\n", reg_val);
	reg_val = READ_REG(CICMD3);
	printk(KERN_DEBUG "CICMD3 = 0X%08X\n", reg_val);

	return;
}

#else

#define READ_REG(offset) __raw_readl((unsigned int)ci_reg_base + (offset))
void DumpReg_Int(unsigned int ci_reg_base)
{
	return;
}

void DumpReg(void)
{
	return;
}
#endif

#ifdef CAMERA_DEBUG
void dbg_dump_report_list(struct list_head *dump_list)
{

	struct list_head *pos;
	struct buf_node *buf_node;

	printk(KERN_DEBUG "**********report list**********\n");
	printk(KERN_DEBUG "self = 0x%p, next = 0x%p, prev = 0x%p\n", dump_list,
			dump_list->next, dump_list->prev);
	list_for_each(pos, dump_list) {
		buf_node = list_entry(pos, struct buf_node, report_head);
		printk ("buf_id = %d, buf_index = %d,"
			"self = 0x%p, next = 0x%p, prev = 0x%p\n",
			 buf_node->buf_id, buf_node->buf_index,
			 pos, pos->next, pos->prev);
	}

	return;
}

void dbg_dump_queue(camera_frame_buffer_queue_t capture_buffer_queue)
{

	camera_frame_buffer_info_t *head;
	camera_frame_buffer_info_t *tail;

	printk(KERN_DEBUG "**********head queue**********\n");
	head = capture_buffer_queue.head;

	while (head != NULL) {
		printk(KERN_DEBUG "frame_id = %d,self = 0x%p, next_buffer = 0x%p\n",
				head->frame_id, head, head->next_buffer);
		head = head->next_buffer;
	}

	printk(KERN_DEBUG "**********tail queue**********\n");
	tail = capture_buffer_queue.tail;

	while (tail != NULL) {
		printk(KERN_DEBUG "frame_id = %d,self = 0x%p, next_buffer = 0x%p\n",
				tail->frame_id, tail, tail->next_buffer);
		tail = tail->next_buffer;
	}

	return;
}

void dbg_buffer_status(void)
{

	unsigned long flags;
	p_camera_context_t cam_ctx = g_camera_context;

	if (CAMERA_MODE_STILL == cam_ctx->capture_mode) {
		printk(KERN_DEBUG "capture mode: still\n");
	} else {
		printk(KERN_DEBUG "capture mode: video\n");
	}
	printk(KERN_DEBUG "cam_ctx->capture_mode = %d", cam_ctx->capture_mode);

	printk ("****************************** "
		"video buffer status******************************\n");
	printk ("****************************** "
		"video buffer status ******************************\n");

	spin_lock_irqsave(&report_list_lock, flags);
	dbg_dump_report_list(&(g_camdrv_status.video_report_head));
	spin_unlock_irqrestore(&report_list_lock, flags);

	spin_lock_irqsave(&cam_queue_lock, flags);
	dbg_dump_queue(cam_ctx->video_capture_buffer_queue);
	spin_unlock_irqrestore(&cam_queue_lock, flags);

	printk(KERN_DEBUG "\n\n************************** "
		"still buffer status ******************************\n");
	printk(KERN_DEBUG "****************************** "
		"still buffer status ******************************\n");

	spin_lock_irqsave(&report_list_lock, flags);
	dbg_dump_report_list(&(g_camdrv_status.still_report_head));
	spin_unlock_irqrestore(&report_list_lock, flags);

	spin_lock_irqsave(&cam_queue_lock, flags);
	dbg_dump_queue(cam_ctx->still_capture_buffer_queue);
	spin_unlock_irqrestore(&cam_queue_lock, flags);

	return;
}

#else
void dbg_dump_report_list(struct list_head dump_list)
{
	return;
}

void dbg_dump_queue(camera_frame_buffer_queue_t capture_buffer_queue)
{
	return;
}

void dbg_buffer_status(void)
{
	return;
}
#endif

static unsigned int camera_format_from_v4l2(__u32 fourcc)
{
	return (fourcc == V4L2_PIX_FMT_YUV422P) ?
			CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR :
#if defined(CONFIG_CPU_PXA310)
		(fourcc == V4L2_PIX_FMT_YUV420) ?
			CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR:
#endif
		(fourcc == V4L2_PIX_FMT_SRGGB8) ?
			CAMERA_IMAGE_FORMAT_RAW8 :
		(fourcc == V4L2_PIX_FMT_SRGGB10) ?
			CAMERA_IMAGE_FORMAT_RAW10 :
		(fourcc == V4L2_PIX_FMT_RGB565X) ?
			CAMERA_IMAGE_FORMAT_RGB565 :
		(fourcc == V4L2_PIX_FMT_RGB24) ?
			CAMERA_IMAGE_FORMAT_RGB888_PACKED :
			CAMERA_IMAGE_FORMAT_MAX + 1;
}

static __u32 camera_fromat_to_v4l2(unsigned int format)
{
	return (format == CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR) ?
			V4L2_PIX_FMT_YUV422P :
#if defined(CONFIG_CPU_PXA310)
		(format == CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR) ?
			V4L2_PIX_FMT_YUV420 :
#endif
		(format == CAMERA_IMAGE_FORMAT_RAW8) ?
			V4L2_PIX_FMT_SRGGB8 :
		(format == CAMERA_IMAGE_FORMAT_RAW10) ?
			V4L2_PIX_FMT_SRGGB10 :
		(format == CAMERA_IMAGE_FORMAT_RGB565) ?
			V4L2_PIX_FMT_RGB565X : 0xffffffff;
}

static unsigned int camera_default_input(unsigned int output_format)
{
	struct pxa_camera_format_t *format_lut;
	p_camera_context_t cam_ctx = g_camera_context;
	PRINTFUNC;

	if (CAMERA_TYPE_OMNIVISION_2620 == cam_ctx->sensor_type) {
		format_lut = ov2620_input_format_lut;
	} else if (CAMERA_TYPE_OMNIVISION_2630 == cam_ctx->sensor_type) {
		format_lut = ov2630_input_format_lut;
	} else if (CAMERA_TYPE_OMNIVISION_5623 == cam_ctx->sensor_type) {
		format_lut = ov5623_input_format_lut;
	} else {
		format_lut = ov7660_input_format_lut;
	}

	for (; format_lut->output_format != CAMERA_IMAGE_FORMAT_MAX + 1;
			format_lut++) {
		if (format_lut->output_format == output_format) {
			return format_lut->input_format;
		}
	}

	return CAMERA_IMAGE_FORMAT_MAX + 1;
}

static struct buf_node *camera_get_buffer_from_id(int buf_id)
{
	struct list_head *pos;
	struct buf_node *buf_node;
	PRINTFUNC;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_id == buf_id)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static struct buf_node *camera_get_buffer_from_index(int buf_index)
{
	struct list_head *pos;
	struct buf_node *buf_node;
	PRINTFUNC;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_index == buf_index)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static int camera_get_buffer_num(void)
{
	struct list_head *pos;
	int buf_num = 0;
	PRINTFUNC;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_num++;
	}
	return buf_num;
}

static ssize_t camera_get_buffer_size(void)
{
	struct buf_node *buf_node;
	ssize_t page_num;
	PRINTFUNC;

	if (!list_empty(g_camdrv_status.p_buf_head)) {
		buf_node =
			list_entry(g_camdrv_status.p_buf_head->next,
					struct buf_node, buf_head);
		page_num = buf_node->page_num;
	} else {
		page_num = 0;
	}

	return page_num * PAGE_SIZE;
}

static struct page *camera_alloc_page(void)
{
	unsigned long flags;
	struct page *page = NULL;

	if (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		pc_head.page_count--;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		page = alloc_page(GFP_KERNEL);
	}

	return page;
}

/* camera_free_page()
 *
 * Free page.
 * Param:
 *    page:    the page will be freed
 *    limit:
 *        0: The page will be added to camera page cache list.
 *           This will be very useful when app change capture mode
 *           and capture resolution dynamically. We needn't free all
 *           of the old pages and alloc new pages for new catpure
 *           mode/resolution. Just need alloc/free the delta pages.
 *        1: If the number of camera page cache list is lager than
 *           MAX_PAGE_CACHE, the page will be free using __free_page.
 *           Else the page will be added to page cache list.
 *
 */
static void camera_free_page(struct page *page, int limit)
{
	unsigned long flags;

	if (0 == limit) {
		spin_lock_irqsave(&pc_head.lock, flags);
		list_add_tail(&page->lru, &pc_head.page_list);
		pc_head.page_count++;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		if (pc_head.page_count < MAX_PAGE_CACHE) {
			spin_lock_irqsave(&pc_head.lock, flags);
			list_add_tail(&page->lru, &pc_head.page_list);
			pc_head.page_count++;
			spin_unlock_irqrestore(&pc_head.lock, flags);
		} else {
			atomic_set(&page->_count, 1);
			ClearPageReserved(page);
			__free_page(page);
		}
	}
}

static void camera_free_buffer_node(struct buf_node *buf_node, int limit)
{
	int i;
	struct page *page = NULL;
	PRINTFUNC;

	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */
	vunmap(buf_node->vaddr);
	if (buf_node->io_type == V4L2_MEMORY_USERPTR)
		goto done;
	for (i = 0; i < buf_node->page_num; i++) {
		page = buf_node->pages[i];
		camera_free_page(page, limit);
	}
done:
	kfree(buf_node->pages);
	kfree(buf_node);
}

static void camera_free_buffer_list(int capture_mode, int limit)
{
	struct buf_node *buf_node;
	unsigned long flags;
	struct list_head *p_buf_head;
	struct list_head *p_report_head;
	PRINTFUNC;

	if (CAMERA_MODE_STILL == capture_mode) {
		p_buf_head = &(g_camdrv_status.still_buf_head);
		p_report_head = &(g_camdrv_status.still_report_head);
	} else {
		p_buf_head = &(g_camdrv_status.video_buf_head);
		p_report_head = &(g_camdrv_status.video_report_head);
	}

	down_interruptible(&buf_list_sem);
	while (!list_empty(p_buf_head)) {
		buf_node =
			list_entry(p_buf_head->next, struct buf_node, buf_head);
		list_del_init(p_buf_head->next);
		dma_free_coherent(NULL, buf_node->dma_desc_size,
				buf_node->dma_desc_vaddr,
				buf_node->dma_desc_paddr);
		camera_free_buffer_node(buf_node, limit);
	}
	up(&buf_list_sem);

	/* empty the report list */
	spin_lock_irqsave(&report_list_lock, flags);
	while (!list_empty(p_report_head)) {
		list_del_init(p_report_head->next);
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

}

static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					(*page) = pte_page(*pte);
					ret = page_to_phys(*page);
					ret |= (addr & (PAGE_SIZE-1));
				}
			}
		}
	}
	return ret;
}


static int camera_alloc_buffer_node(struct buf_node **buf_node, unsigned long userptr, int size)
{
	int page_num;
	int i, j;
	unsigned int ret = 0;
	struct page *page;
	struct buf_node *buf;
	unsigned int vaddr = PAGE_ALIGN(userptr);
	PRINTFUNC;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		dbg("Not enough memory\n");
		return -ENOMEM;
	}

	page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	buf->pages =
		(struct page **)kzalloc(page_num * sizeof(long), GFP_KERNEL);
	if (!buf->pages) {
		dbg("Not enough memory\n");
		ret = -ENOMEM;
		goto alloc_node_error;
	}

	for (i = 0; i < page_num; i++) {
		if (vaddr){
			uva_to_pa(vaddr, &page);
			vaddr += PAGE_SIZE;
		} else {
			page = camera_alloc_page();

			if (!page) {
				dbg("Not enough memory\n");
				ret = -ENOMEM;
				goto alloc_pages_error;
			}
			atomic_set(&page->_count, 1);
			SetPageReserved(page);
       		}
		buf->pages[i] = page;
	}

	buf->page_num = page_num;
	buf->size = page_num * PAGE_SIZE;
	buf->buf_id = -1;
	buf->vaddr =
		vmap(buf->pages, buf->page_num, VM_MAP,
				pgprot_noncached(pgprot_kernel));

	memset(buf->vaddr, 0, buf->size);

	/* check if the memory map is OK. */
	if (!buf->vaddr) {
		dbg("vmap() failure\n");
		ret = -EFAULT;
		goto vmap_error;
	}

	*buf_node = buf;

	return ret;

vmap_error:
alloc_pages_error:
	for (j = 0; j < i; j++) {
		page = buf->pages[j];
		camera_free_page(page, 1);
	}
	kfree(buf->pages);
alloc_node_error:
	kfree(buf);
	return ret;
}

static void camera_get_fifo_size(p_camera_context_t cam_ctx,
		struct buf_node *buf_node, int buffer_type)
{
	/*
	 * caculate the fifo0-2 transfer size
	 */
	unsigned int capture_output_format;
	unsigned int capture_output_width;
	unsigned int capture_output_height;
	unsigned int frame_size;
	PRINTFUNC;

	capture_output_format = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_output_format :
		cam_ctx->still_capture_output_format;

	capture_output_width = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_width : cam_ctx->still_capture_width;

	capture_output_height = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_height : cam_ctx->still_capture_height;

	switch (capture_output_format) {
		case CAMERA_IMAGE_FORMAT_RAW10:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW9:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW8:
			frame_size = capture_output_width *
				capture_output_height;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB888_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 3;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB565:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size / 2;
			buf_node->fifo1_size = frame_size / 4;
			buf_node->fifo2_size = frame_size / 4;
			break;
#if defined(CONFIG_CPU_PXA310)
		case CAMERA_IMAGE_FORMAT_YCBCR420_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 3 / 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size / 2;
			buf_node->fifo1_size = frame_size / 8;
			buf_node->fifo2_size = frame_size / 8;
			break;
#endif
		default:
			break;
	}
	return;
}

static int camera_prepare_buffer(p_camera_context_t cam_ctx, unsigned long userptr, 
		unsigned int buf_len, unsigned int *buf_index)
{
	int ret = 0;
	int buf_type, buf_size, dma_desc_size;
	int j;
	void *dma_desc_virt;
	int *buf_phy_addr_array;
	int *dma_desc_phy_addr_array;

	dma_addr_t dma_desc_phy = 0;
	struct buf_node *buf_node = NULL;
	unsigned int dma_desc_phy_addr_array_size;
	unsigned long flags;
	PRINTFUNC;

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	if (mcam_get_buffer_size(cam_ctx, buf_type, &buf_size, &dma_desc_size)) {
		dbg("Get buffer size failure!\n");
		return -EFAULT;
	}
	if (userptr && buf_len < buf_size)
		return -EFAULT;

	dbg("buf_size = %d, dma_desc_size = %d\n", buf_size, dma_desc_size);
	buf_phy_addr_array =
		kzalloc(PAGE_ALIGN(buf_size) / PAGE_SIZE * sizeof(long),
				GFP_KERNEL);

	if (!buf_phy_addr_array) {
		dbg("No memory for record pages's physical address\n");
		return -ENOMEM;
	}

	dma_desc_phy_addr_array_size =
		(dma_desc_size + DMA_DESCRIPTOR_SIZE - 1) / DMA_DESCRIPTOR_SIZE;

	dma_desc_phy_addr_array =
		kzalloc(dma_desc_phy_addr_array_size * sizeof(long), GFP_KERNEL);

	if (!dma_desc_phy_addr_array) {
		dbg("No memory for record dma descriptors's physical address\n");
		kfree(buf_phy_addr_array);
		return -ENOMEM;
	}
	dma_desc_virt = dma_alloc_coherent(NULL, dma_desc_size,
		&dma_desc_phy, GFP_KERNEL);
	if (!dma_desc_virt) {
		dbg("Just alloc %d number buffer node\n", camera_get_buffer_num() - 1);
		goto exit;
	}
	ret = camera_alloc_buffer_node(&buf_node, userptr, buf_size);
	if (ret) {
		dbg("Alloc %dth buffer node failure\n", camera_get_buffer_num()); 
		dma_free_coherent(NULL, dma_desc_size, dma_desc_virt,
				dma_desc_phy);
		goto exit;
	}
	buf_node->io_type = userptr?V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;
	buf_node->buf_index = camera_get_buffer_num();
	buf_node->dma_desc_vaddr = dma_desc_virt;
	buf_node->dma_desc_paddr = dma_desc_phy;
	buf_node->dma_desc_size = dma_desc_size;

	dbg("dma_desc_virt = 0x%p, dma_desc_phy = 0x%08x\n",
			dma_desc_virt, dma_desc_phy);

	for (j = 0; j < buf_node->page_num; j++) {
		buf_phy_addr_array[j] =
			__pa(page_address(buf_node->pages[j]));
		/*dbg("buf_phy_addr_array[%d] = 0x%08x\n",
		  j, buf_phy_addr_array[j]); */
	}

	dma_desc_phy_addr_array[0] = buf_node->dma_desc_paddr;
	for (j = 1; j < dma_desc_phy_addr_array_size; j++) {
		dma_desc_phy_addr_array[j] =
			dma_desc_phy_addr_array[j - 1] +
			DMA_DESCRIPTOR_SIZE;
	}
	spin_lock_irqsave(&cam_queue_lock, flags);
	ret = mcam_prepare_buffer(cam_ctx,
			buf_node->vaddr,
			(void *)buf_phy_addr_array,
			buf_node->page_num,
			buf_size,
			buf_type,
			buf_node->dma_desc_vaddr,
			dma_desc_phy_addr_array,
			&buf_node->buf_id,
			&buf_node->Y_vaddr,
			&buf_node->Cb_vaddr,
			&buf_node->Cr_vaddr);
	spin_unlock_irqrestore(&cam_queue_lock, flags);
	if (ret) {
		dbg("Prepare %dth buffer node failure\n", buf_node->buf_index + 1);
		camera_free_buffer_node(buf_node, 1);
		dma_free_coherent(NULL, dma_desc_size, dma_desc_virt,
				dma_desc_phy);
		goto exit;
	}

	camera_get_fifo_size(cam_ctx, buf_node, buf_type);

	list_add_tail(&buf_node->buf_head, g_camdrv_status.p_buf_head);
exit: 
	kfree(buf_phy_addr_array);
	kfree(dma_desc_phy_addr_array);

	*(g_camdrv_status.p_buf_ready) = 1;

	dbg("test point in camera_prepare_buffers");
	TESTPOINT;
	if (buf_index)
		*buf_index = buf_node->buf_index;

	return ret;
}

static int camera_prepare_buffers(p_camera_context_t cam_ctx, int buf_num)
{
	int i;

	for (i = 0; i < buf_num; i++)
		camera_prepare_buffer(cam_ctx, 0, 0, NULL);

	return camera_get_buffer_num();
}

static int camera_submit_buffer(p_camera_context_t cam_ctx,
		unsigned int buf_indx)
{
	struct buf_node *buf_node;
	unsigned int buf_type;
	int ret;
	unsigned long flags;
	PRINTFUNC;

	buf_node = camera_get_buffer_from_index(buf_indx);

	if (!buf_node)
		goto exit;

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	spin_lock_irqsave(&cam_queue_lock, flags);
	ret = mcam_submit_buffer(cam_ctx, buf_node->buf_id, buf_type);
	spin_unlock_irqrestore(&cam_queue_lock, flags);
	if (ret) {
		dbg("Submit %dth buffer node failure\n", buf_indx);
		goto exit;
	}

	*(g_camdrv_status.p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static int camera_submit_buffers(p_camera_context_t cam_ctx)
{
	struct list_head *pos;
	struct buf_node *buf_node;
	unsigned int buf_type;
	int i = 0;
	unsigned long flags;
	PRINTFUNC;

	if (!*(g_camdrv_status.p_buf_ready)) {
		dbg("buffer not ready!\n");
		goto exit;
	}
	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);

		spin_lock_irqsave(&cam_queue_lock, flags);
		mcam_submit_buffer(cam_ctx, buf_node->buf_id, buf_type);
		spin_unlock_irqrestore(&cam_queue_lock, flags);

		i++;
	}

	if (i == 0) {
		goto exit;
	}

	*(g_camdrv_status.p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static int camera_start_capture(p_camera_context_t cam_ctx)
{

	PRINTFUNC;
	if (!*(g_camdrv_status.p_buf_submited)) {
		dbg("pxa_camera: buffer not submited!\n");
		goto exit;
	}

	mcam_set_interrupt_mask(cam_ctx, INT_MASK);

	if (mcam_set_capture_format(cam_ctx)) {
		goto exit;
	}

	cam_ctx->frame_rate = pxa_camera_get_framerate(&g_camdrv_status);
	dbg("cam_ctx->frame_rate  = %d\n", cam_ctx->frame_rate);
	mcam_set_capture_frame_rate(cam_ctx);

	dbg("start capture \n");

	if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {
		if (mcam_start_video_capture(cam_ctx)) {
			goto exit;
		}
	} else {
		if (mcam_capture_still_image(cam_ctx)) {
			goto exit;
		}
	}

	enable_irq(IRQ_CAMERA);

	return 0;

exit:

	return -EFAULT;
}

static void camera_stop_capture(p_camera_context_t cam_ctx)
{
	PRINTFUNC;
	/*
	 * stop video capture
	 * stop still capture
	 * Note: a workaround of camera drv for stopping still capture
	 * which has no such stop still capture primitives
	 */
	mcam_stop_video_capture(cam_ctx);
	disable_irq(IRQ_CAMERA);

	return;

}

void camera_desubmit_buffers(p_camera_context_t cam_ctx)
{
	PRINTFUNC;
	/*
	 * stop capture: a workaround of camera drv
	 * which has no such desubmit buffer primitives
	 */

	if (CAMERA_MODE_STILL == cam_ctx->capture_mode) {
		cam_ctx->still_capture_buffer_queue.head = NULL;
		cam_ctx->still_capture_buffer_queue.tail = NULL;
	} else {
		cam_ctx->video_capture_buffer_queue.head = NULL;
		cam_ctx->video_capture_buffer_queue.tail = NULL;
	}

	*(g_camdrv_status.p_buf_submited) = 0;

}

void camera_deprepare_buffers(p_camera_context_t cam_ctx)
{
	PRINTFUNC;
	camera_free_buffer_list(cam_ctx->capture_mode, 1);
	*(g_camdrv_status.p_buf_ready) = 0;

	return;
}

/***********************************************************************
 *
 * Init/Deinit APIs
 *
 ***********************************************************************/
#ifdef	CONFIG_MACH_LITTLETON
static int camera_init(p_camera_context_t camera_context)
{
	/* Notes: On Littleton, we set camera analog to 2.5v
	 * and camera io to 1.8v for ov5623 and ov7673.
	 */
	pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 2500);
	pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, 1800);

	/* enable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 1);

	return 0;
}
#elif defined(CONFIG_MACH_ZYLONITE)
static int camera_init(p_camera_context_t camera_context)
{
	PRINTFUNC;

	/* FIXME: init the Vcc for Camera analog. Should be done by system */
	if (CAMERA_TYPE_OMNIVISION_2620 == camera_context->sensor_type){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 3200);
	} else if (CAMERA_TYPE_OMNIVISION_2630 == camera_context->sensor_type){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 2800);
	} else if (CAMERA_TYPE_OMNIVISION_5623 == camera_context->sensor_type){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 2800);
	} else {
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 2800);
	}

	printk(KERN_DEBUG "%s: ckenA: 0x%x\n", __func__, CKENA);
	/* enable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 1);

	return 0;
}
#endif

static int camera_deinit(p_camera_context_t camera_context)
{
	int status;
	PRINTFUNC;

	if (!mcam_deinit(camera_context)) {
		status = 0;
	} else {
		status = -EFAULT;
	}

	camera_context->sensor_type = 0;

	/* disable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 0);

	return status;
}

static void pxa_camera_set_mode(struct pxa_camera_driver_status_t
		*p_camdrv_status, int capture_mode)
{
	PRINTFUNC;
	p_camdrv_status->capture_mode = capture_mode;
	if (CAMERA_MODE_VIDEO == capture_mode) {
		p_camdrv_status->p_buf_ready =
			&(p_camdrv_status->video_buf_ready);
		p_camdrv_status->p_buf_submited =
			&(p_camdrv_status->video_buf_submited);
		p_camdrv_status->p_capture_started =
			&(p_camdrv_status->video_capture_started);
		p_camdrv_status->p_buf_head =
			&(p_camdrv_status->video_buf_head);
		p_camdrv_status->p_report_head =
			&(p_camdrv_status->video_report_head);
		p_camdrv_status->p_timeperframe_numerator =
			&(p_camdrv_status->video_timeperframe_numerator);
		p_camdrv_status->p_timeperframe_denominator =
			&(p_camdrv_status->video_timeperframe_denominator);

	} else {
		p_camdrv_status->p_buf_ready =
			&(p_camdrv_status->still_buf_ready);
		p_camdrv_status->p_buf_submited =
			&(p_camdrv_status->still_buf_submited);
		p_camdrv_status->p_capture_started =
			&(p_camdrv_status->still_capture_started);
		p_camdrv_status->p_buf_head =
			&(p_camdrv_status->still_buf_head);
		p_camdrv_status->p_report_head =
			&(p_camdrv_status->still_report_head);
		p_camdrv_status->p_timeperframe_numerator =
			&(p_camdrv_status->still_timeperframe_numerator);
		p_camdrv_status->p_timeperframe_denominator =
			&(p_camdrv_status->still_timeperframe_denominator);

	}

	return;

}

static int camera_do_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg)
{
	int retval = 0;
	p_camera_context_t cam_ctx = g_camera_context;
	PRINTFUNC;

	if (!cam_ctx) {
		return -EINVAL;
	}

	switch (cmd) {

		case VIDIOC_QUERYCAP:
			{
				struct v4l2_capability *cap = arg;
				dbg("IOCTL CMD = VIDIOC_QUERYCAP\n");
				retval = 0;
				memset(cap, 0, sizeof(*cap));
				strcpy(cap->driver, "pxa camera");
				strcpy(cap->card, "");
				cap->version = PXA_CAMERA_VERSION;
				cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

				retval = 0;
				break;
			}

		case VIDIOC_ENUMINPUT:
			{
				struct v4l2_input *i = arg;
				unsigned int n;

				dbg("IOCTL CMD = VIDIOC_ENUMINPUT\n");
				n = i->index;
				if (n > SENSOR_NUMBER) {
					retval = -EINVAL;
					break;
				}

				memset(i, 0, sizeof(*i));
				i->index = n;
				i->type = V4L2_INPUT_TYPE_CAMERA;

				switch (n) {
					case OV2620_SENSOR:
						if(ov2620_detected != 0)
							strcpy(i->name, "Omnivision2620");
						break;

					case OV2630_SENSOR:
						if(ov2630_detected != 0)
							strcpy(i->name, "Omnivision2630");
						break;

					case OV7660_SENSOR:
						if(ov7660_detected != 0)
							strcpy(i->name, "Omnivision7660");
						break;

					case OV5623_SENSOR:
						if(ov5623_detected != 0)
							strcpy(i->name, "Omnivision5623");
						break;

					case OV7673_SENSOR:
						if(ov7673_detected != 0)
							strcpy(i->name, "Omnivision7673");
						break;

					default:
						break;
				}

				break;
			}

		case VIDIOC_ENUM_FMT:
			{
				struct v4l2_fmtdesc *f = arg;
				enum v4l2_buf_type type;
				int index;

				dbg("IOCTL CMD = VIDIOC_ENUM_FMT\n");
				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					
					retval = -EINVAL;
					break;
				}

				if (f->index >= PXA_CAMERA_FORMATS) {
					retval = -EINVAL;
					break;
				}

				type = f->type;
				index = f->index;

				memset(f, 0, sizeof(*f));
				f->index = index;
				f->type = type;
				f->pixelformat = pxa_camera_formats[index].fourcc;
				strlcpy(f->description,
					pxa_camera_formats[index].name,
					sizeof(f->description));

				break;
			}

		case VIDIOC_G_FMT:
			{
				struct v4l2_format *f = arg;

				dbg("IOCTL CMD = VIDIOC_G_FMT\n");

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {
					f->fmt.pix.pixelformat =
						camera_fromat_to_v4l2(cam_ctx->
						video_capture_output_format);
					f->fmt.pix.width =
						cam_ctx->video_capture_width;
					f->fmt.pix.height =
						cam_ctx->video_capture_height;
				} else {
					f->fmt.pix.pixelformat =
						camera_fromat_to_v4l2(cam_ctx->
						still_capture_output_format);
					f->fmt.pix.width =
						cam_ctx->still_capture_width;
					f->fmt.pix.height =
						cam_ctx->still_capture_height;
				}

				break;
			}
#if 1
		case VIDIOC_S_FMT:
			{
				struct v4l2_format *f = arg;
				unsigned int pixelformat;
				int capture_width;
				int capture_height;

				dbg("IOCTL CMD = VIDIOC_S_FMT\n");

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_TYPE_OMNIVISION_2620 ==
					cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
						cam_ctx->sensor_type) {
					if ((V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_CPU_PXA310)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)) {
#endif
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600)
						|| (f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_5623 ==
						cam_ctx->sensor_type) {
					if ((V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_CPU_PXA310)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)) {
#endif

						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 2560)
						|| (f->fmt.pix.height > 1920)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_7673 ==
						cam_ctx->sensor_type) {
					if ((V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_RGB565X !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_CPU_PXA310)
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)) {
#endif
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 640)
						|| (f->fmt.pix.height > 480)) {
						retval = -EINVAL;
						break;
					}
				} else {	/* OV7660 */
					if ((V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_RGB565X !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_CPU_PXA310)
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)) {
#endif
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 640)
						|| (f->fmt.pix.height > 480)) {
						retval = -EINVAL;
						break;
					}
				}

				pixelformat = camera_format_from_v4l2(
						f->fmt.pix.pixelformat);
				capture_width = f->fmt.pix.width;
				capture_height = f->fmt.pix.height;
				pxa_camera_reset(&g_camdrv_status);

				if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {

					cam_ctx->video_capture_output_format =
						camera_format_from_v4l2(f->
							fmt.pix.pixelformat);
					cam_ctx->video_capture_input_format =
						camera_default_input(cam_ctx->
							video_capture_output_format);
					cam_ctx->video_capture_width =
						f->fmt.pix.width;
					cam_ctx->video_capture_height =
						f->fmt.pix.height;

				} else {

					cam_ctx->still_capture_output_format =
						camera_format_from_v4l2(f->
							fmt.pix.pixelformat);
					cam_ctx->still_capture_input_format =
						camera_default_input(cam_ctx->
							still_capture_output_format);
					cam_ctx->still_capture_width =
						f->fmt.pix.width;
					cam_ctx->still_capture_height =
						f->fmt.pix.height;

				}

				break;
			}
#endif
		case VIDIOC_TRY_FMT:
			{
				struct v4l2_format *f = arg;

				dbg("IOCTL CMD = VIDIOC_TRY_FMT\n");
				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_TYPE_OMNIVISION_2620 ==
						cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
						cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_5623 ==
						cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 2560) ||
						(f->fmt.pix.height > 1920)) {
						retval = -EINVAL;
						break;
					}
				} else {
					if ((V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_RGB565X !=
							f->fmt.pix.pixelformat)) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 640) ||
						(f->fmt.pix.height > 480)) {
						retval = -EINVAL;
						break;
					}
				}
				break;
			}

		case VIDIOC_S_INPUT:
			{
				int *i = arg;
				int sensor_type;

				dbg("IOCTL CMD = VIDIOC_S_INPUT\n");
				if (*(g_camdrv_status.p_capture_started)) {
					pxa_camera_ioctl_streamoff(
						&g_camdrv_status);
				}

				switch (*i) {
					case OV2620_SENSOR:
						if(ov2620_detected == 0){
							printk(KERN_ERR "ov2620 sensor is not detected\n");
							retval = -EINVAL;
							return retval;
						}
						printk(KERN_ALERT "PXA_CAMERA:"
							"choose ov2620 as sensor\n");
						sensor_type =
							CAMERA_TYPE_OMNIVISION_2620;
						break;
					case OV2630_SENSOR:
						if(ov2630_detected == 0){
							printk(KERN_ERR "ov2630 sensor is not detected\n");
							retval = -EINVAL;
							return retval;
						}
						printk(KERN_ALERT "PXA_CAMERA:"
							"choose ov2630 as sensor\n");
						sensor_type =
							CAMERA_TYPE_OMNIVISION_2630;
						break;
					case OV5623_SENSOR:
						if (ov5623_detected == 0){
							printk(KERN_ERR "ov5623 sensor is not detected\n");
							retval = -EINVAL;
							return retval;
						}
						printk(KERN_ALERT "PXA_CAMERA:"
							"choose ov5623 as sensor\n");
						sensor_type = CAMERA_TYPE_OMNIVISION_5623;
						break;
					case OV7660_SENSOR:
						if(ov7660_detected == 0){
							printk(KERN_ERR "ov7660 sensor is not detected\n");
							retval = -EINVAL;
							return retval;
						}
						printk(KERN_ALERT "PXA_CAMERA:"
							"choose ov7660 as sensor\n");
						sensor_type =
							CAMERA_TYPE_OMNIVISION_7660;
						break;
					case OV7673_SENSOR:
						if(ov7673_detected == 0){
							printk(KERN_ERR "ov7673 sensor is not detected\n");
							retval = -EINVAL;
							return retval;
						}
						printk(KERN_ALERT "PXA_CAMERA:"
							"choose ov7673 as sensor\n");
						sensor_type =
							CAMERA_TYPE_OMNIVISION_7673;
						break;
					default:
						retval = -EINVAL;
						return retval;
				}

				mcam_deinit(cam_ctx);
				if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_2620) {
					SET_OV2620_SENSOR(cam_ctx);
				} else if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_2630) {
					SET_OV2630_SENSOR(cam_ctx);
				} else if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_5623) {
					SET_OV5623_SENSOR(cam_ctx);
				} else if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_7673) {
					SET_OV7673_SENSOR(cam_ctx);
				} else {
					SET_OV7660_SENSOR(cam_ctx);
				}
				mcam_init(cam_ctx);

				break;
			}

		case VIDIOC_G_INPUT:
			{
				int *i = arg;

				dbg("IOCTL CMD = VIDIOC_G_INPUT\n");
				if (CAMERA_TYPE_OMNIVISION_2620 ==
					cam_ctx->sensor_type) {
					*i = OV2620_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
					cam_ctx->sensor_type) {
					*i = OV2630_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_5623 ==
					cam_ctx->sensor_type) {
					*i = OV5623_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_7660 ==
					cam_ctx->sensor_type) {
					*i = OV7660_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_7673 ==
					cam_ctx->sensor_type) {
					*i = OV7673_SENSOR;
				} else {
					retval = -EINVAL;
				}
				break;
			}

		case VIDIOC_G_PARM:
			{
				struct v4l2_streamparm *parm = arg;

				dbg("IOCTL CMD = VIDIOC_G_PARM\n");
				if (parm->type !=
					V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_MODE_STILL ==
					cam_ctx->capture_mode) {
					parm->parm.capture.capturemode =
						V4L2_MODE_HIGHQUALITY;
				} else {
					parm->parm.capture.capturemode = 0;
				}
				parm->parm.capture.timeperframe.numerator =
					*(g_camdrv_status.p_timeperframe_numerator);
				parm->parm.capture.timeperframe.denominator =
					*(g_camdrv_status.p_timeperframe_denominator);

				break;
			}

		case VIDIOC_S_PARM:
			{

				struct v4l2_streamparm *parm = arg;
				int capture_mode;

				dbg("IOCTL CMD = VIDIOC_S_PARM\n");

				if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (V4L2_MODE_HIGHQUALITY ==
						parm->parm.capture.capturemode) {
					capture_mode = CAMERA_MODE_STILL;
				} else {
					capture_mode = CAMERA_MODE_VIDEO;
				}

				/* Spatial Scaling Unit(SSU) related functions */
				if (CI_SSU_SCALE_HALF ==
						parm->parm.capture.extendedmode) {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_HALF;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_HALF;
					}
				} else if (CI_SSU_SCALE_QUARTER ==
						parm->parm.capture.extendedmode) {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_QUATER;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_QUATER;
					}
				} else {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_DISABLE;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_DISABLE;
					}
				}

				if ((capture_mode != cam_ctx->capture_mode) ||
					(*(g_camdrv_status.p_timeperframe_numerator) !=
					parm->parm.capture.timeperframe.numerator) ||
					(*(g_camdrv_status.p_timeperframe_denominator) !=
					parm->parm.capture.timeperframe.denominator)) {

					if (*(g_camdrv_status.p_capture_started)) {
						pxa_camera_ioctl_streamoff
							(&g_camdrv_status);
					}

					if (capture_mode != cam_ctx->capture_mode) {
						cam_ctx->capture_mode = capture_mode;
						pxa_camera_set_mode(&g_camdrv_status,
								cam_ctx->
								capture_mode);
					}

					*(g_camdrv_status.p_timeperframe_numerator) =
						parm->parm.capture.timeperframe.numerator;
					*(g_camdrv_status.p_timeperframe_denominator) =
						parm->parm.capture.timeperframe.denominator;

				}

				break;

			}

		case VIDIOC_S_CTRL:
			{
				struct v4l2_control *ctrl = arg;

				dbg("IOCTL CMD = VIDIOC_S_CTRL\n");
				switch (ctrl->id) {
					case V4L2_CID_CONTRAST:
						retval = mcam_set_contrast_value(
							cam_ctx,
							SENSOR_MANUAL_CONTRAST,
							ctrl->value);
						break;
					case V4L2_CID_DO_WHITE_BALANCE:
						retval =
							mcam_set_white_balance_value(
							cam_ctx,
							SENSOR_MANUAL_WHITEBALANCE,
							ctrl->value);
						break;
					case V4L2_CID_EXPOSURE:
						retval =
							mcam_set_exposure_value(cam_ctx,
							SENSOR_MANUAL_EXPOSURE,
							ctrl->value);
						break;
					default:
						retval = -EINVAL;
						break;
				}
				break;
			}

		case VIDIOC_G_CTRL:
			{
				/* do nothing */
				dbg("IOCTL CMD = VIDIOC_G_CTRL\n");
				break;
			}

		case VIDIOC_QUERYBUF:
			{
				struct v4l2_buffer *buf = arg;
				int buf_size;

				dbg("IOCTL CMD = VIDIOC_QUERYBUF\n");
				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (buf->memory != V4L2_MEMORY_MMAP) {
					retval = -EINVAL;
					break;
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				buf_size = camera_get_buffer_size();
				buf->length = buf_size;
				buf->m.offset = buf->index * buf_size;

				break;
			}

		case VIDIOC_QBUF:
			{
				struct v4l2_buffer *buf = arg;

				unsigned long flags;

				dbg("IOCTL CMD = VIDIOC_QBUF\n");
				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}
				if ((buf->memory != V4L2_MEMORY_MMAP)
				       && (buf->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}
				 
				if ((buf->memory == V4L2_MEMORY_USERPTR)
						&& (buf->length > 0)){
					if (camera_prepare_buffer(cam_ctx,
						buf->m.userptr, buf->length, &buf->index)){
						retval = -EINVAL;
						break;
					}
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				dbg("buf address ox%p\n", buf);
				dbg("buf->index  =  %d\n", buf->index);

				spin_lock_irqsave(&report_list_lock, flags);
				retval = camera_submit_buffer(cam_ctx, buf->index);
				spin_unlock_irqrestore(&report_list_lock, flags);

				if (!retval) {
					dbg("submit buffer %d successfully!\n",
							buf->index);
				} else {
					dbg("submit buffer failed!\n");
				}

				break;
			}

		case VIDIOC_DQBUF:
			{
				struct v4l2_buffer *buf = arg;
				struct buf_node *buf_node;
				unsigned long flags;

				dbg("IOCTL CMD = VIDIOC_DQBUF\n");
				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}
				if ((buf->memory != V4L2_MEMORY_MMAP)
				       && (buf->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}

				spin_lock_irqsave(&report_list_lock, flags);

				if (!list_empty(g_camdrv_status.p_report_head)) {
					buf_node =
						list_entry(
						g_camdrv_status.p_report_head->
						next,
						struct buf_node,
						report_head);
					assert(buf_node);

					dbg("DQBUF: buf_node->index = %d\n",
							buf_node->buf_index);
					list_del(&buf_node->report_head);

					buf->index = buf_node->buf_index;

				} else
					retval = -EIO;
				spin_unlock_irqrestore(&report_list_lock, flags);

				break;
			}

		case VIDIOC_REQBUFS:
			{
				struct v4l2_requestbuffers *req = arg;
				int count;

				dbg("IOCTL CMD = VIDIOC_REQBUFS\n");
				if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if ((req->memory != V4L2_MEMORY_MMAP)
				       && (req->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}

				camera_deprepare_buffers(cam_ctx);
				 
				if((req->reserved[0] & 0x1) == YUV_NO_PADDING) 
				 	cam_ctx->align_type = YUV_NO_PADDING;
				else
					cam_ctx->align_type = YUV_HAVE_PADDING;	

				if (req->memory == V4L2_MEMORY_USERPTR)
					break;

				count = camera_prepare_buffers(cam_ctx,
					req->count);
				if (count < 0)
					retval = -EFAULT;
				else
					req->count = count;

				break;
			}

		case VIDIOC_STREAMON:
			{
				int *type = arg;

				dbg("IOCTL CMD = VIDIOC_STREAMON\n");
				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					dbg("VIDIOC_STREAMON failed!\n");
					retval = -EINVAL;
					break;
				}

				retval = pxa_camera_ioctl_streamon(
						&g_camdrv_status);

				break;
			}

		case VIDIOC_STREAMOFF:
			{
				int *type = arg;

				dbg("IOCTL CMD = VIDIOC_STREAMOFF\n");
				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				pxa_camera_ioctl_streamoff(&g_camdrv_status);

				break;
			}

			/* Application extended IOCTL.  */
			/* Register access interface    */

		case WCAM_VIDIOCGCIREG:
			{
				struct reg_set_s *reg = arg;
				dbg("IOCTL CMD = WCAM_VIDIOCGCIREG\n");
				reg->val2 =  __raw_readl(
					((unsigned int)cam_ctx->ci_reg_base
					  + (reg->val1)));
				break;

			}

		case WCAM_VIDIOCSCIREG:
			{
				struct reg_set_s *reg = arg;
				volatile unsigned int regVal;

				dbg("IOCTL CMD = WCAM_VIDIOCSCIREG\n");
				__raw_writel(reg->val2, (unsigned int)cam_ctx->
					ci_reg_base + reg->val1);
				regVal =
					__raw_readl((unsigned int)cam_ctx->
						ci_reg_base + reg->val1);
				if (regVal != reg->val2) {
					retval = -EFAULT;
					break;
				}
				break;
			}

		case WCAM_VIDIOCGCAMREG:
			{
				struct reg_set_s *reg = arg;
				volatile unsigned char regVal;

				dbg("IOCTL CMD = WCAM_VIDIOCGCAMREG\n");
				mcam_read_8bit(cam_ctx, (unsigned char)reg->val1,
					(unsigned char *)&(regVal));
				reg->val2 = regVal;

				break;

			}

		case WCAM_VIDIOCSCAMREG:
			{
				struct reg_set_s *reg = arg;
				dbg("IOCTL CMD = WCAM_VIDIOCSCAMREG\n");
				mcam_write_8bit(cam_ctx, (unsigned char)reg->val1,
					(unsigned char)reg->val2);
				break;
			}

		case WCAM_VIDIOCGHST:
			{
				struct hst_context_s *hst_ctx = arg;
				unsigned int color_type = CI_HISTO_GREEN1;
				unsigned short *ptemp;
				int i, count;
				unsigned int hst_sum = 0;

				dbg("IOCTL CMD = WCAM_VIDIOCGHST\n");

				if (HST_COLOR_RED == hst_ctx->color) {
					color_type = CI_HISTO_RED;
				} else if (HST_COLOR_BLUE == hst_ctx->color) {
					color_type = CI_HISTO_BLUE;
				} else if (HST_COLOR_GREEN1 == hst_ctx->color) {
					color_type = CI_HISTO_GREEN1;
				} else if (HST_COLOR_GREEN2 == hst_ctx->color) {
					color_type = CI_HISTO_GREEN2;
				}

				disable_irq(IRQ_CAMERA);
				mcam_get_histogram_info(cam_ctx, color_type,
					&(hst_ctx->size), &(hst_ctx->sum));
				enable_irq(IRQ_CAMERA);
				ptemp =	(unsigned short *)cam_ctx->
						histogram_lut_buffer_virtual;

				/* software work around for sum */
				count = hst_ctx->size / 2;
				for (i = 0; i < count; i++) {
					hst_sum += i * (ptemp[i]);
				}
				hst_ctx->sum = hst_sum;
				retval = copy_to_user((char __user *)hst_ctx->hst,
						(void *)(cam_ctx->
						 histogram_lut_buffer_virtual),
						hst_ctx->size);

				break;
			}

		default:
			{
				dbg("Invalid ioctl parameters.\n");
				retval = -ENOIOCTLCMD;
				break;
			}
	}

	return retval;
}

static irqreturn_t pxa_camera_irq(int irq, void *dev_id)
{
#if 1
	unsigned int int_state;
	int buf_id;
	struct buf_node *buf_node;
	unsigned long flags1, flags2;
	p_camera_context_t cam_ctx = g_camera_context;
	PRINTFUNC;

#ifdef CAMERA_DEBUG_IRQ
	static int irq_count = 0;
	irq_count++;
#endif

	DumpReg_Int(cam_ctx->ci_reg_base);

	int_state = mcam_get_interrupt_status(cam_ctx);
	/* printk("int_state = 0x%08x\n", int_state);
	   dbg("before clear: int_mask = 0x%08x, int_state = 0x%08x\n",
	   mcam_get_interrupt_mask(cam_ctx), int_state); */

	if (int_state & 0x90000000) {
		dbg("int_state&0x90000000 != 0\n");
		TESTPOINT;
		DumpReg();
	}


	/* DumpReg_Int(cam_ctx->ci_reg_base); */

	/* dbg_buffer_status(); */
	spin_lock_irqsave(&report_list_lock, flags1);
	spin_lock_irqsave(&cam_queue_lock, flags2);
 
	if (!mcam_get_filled_buffer(cam_ctx, &buf_id)) {
		dbg("end of frame\n");
		buf_node = camera_get_buffer_from_id(buf_id);
		if (&buf_node->report_head
			       	!= g_camdrv_status.p_report_head->next){
			list_add_tail(&buf_node->report_head,
				g_camdrv_status.p_report_head);
		}
		/* dbg_buffer_status(); */
		if (g_camdrv_status.task_waiting) {
			wake_up_interruptible(&
					(g_camdrv_status.
					 camera_wait_q));
			g_camdrv_status.task_waiting = 0;
		}
	}

	spin_unlock_irqrestore(&cam_queue_lock, flags2);
	spin_unlock_irqrestore(&report_list_lock, flags1);
 
	mcam_clear_interrupt_status(cam_ctx, int_state);
	/*dbg("after clear: int_mask = 0x%08x, int_state = 0x%08x\n",
	  mcam_get_interrupt_mask(cam_ctx), mcam_get_interrupt_status(cam_ctx));
	 */

#ifdef CAMERA_DEBUG_IRQ
	printk(KERN_DEBUG "irq count = %d\n", irq_count);
#endif

	return IRQ_HANDLED;
#endif
}

/********************************************************************************************
 * Application interface
 *******************************************************************************************/

static int pxa_camera_get_framerate(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{

	p_camera_context_t cam_ctx = g_camera_context;
	unsigned int sensor_timeperframe_numerator = 0;
	unsigned int sensor_timeperframe_denominator = 0;
	unsigned int framerate;

	PRINTFUNC;

	dbg("*(p_camdrv_status->p_timeperframe_denominator) = %d\n",
			*(p_camdrv_status->p_timeperframe_denominator));
	dbg("*(p_camdrv_status->p_timeperframe_numerator) = %d\n",
			*(p_camdrv_status->p_timeperframe_numerator));

	if ((0 == *(p_camdrv_status->p_timeperframe_denominator))
			|| (0 == *(p_camdrv_status->p_timeperframe_numerator))) {
		framerate = 0;
		return framerate;

	}

	if (CAMERA_TYPE_OMNIVISION_2620 == cam_ctx->sensor_type) {
		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_RAW10:
				if ((cam_ctx->capture_input_width <= 404)
						&& (cam_ctx->capture_input_height <= 302)) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 808)
						&& (cam_ctx->capture_input_height <= 604)) {
					/* SVGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 1616)
						&& (cam_ctx->capture_input_height <= 1208)) {
					/* UXGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 5;
				}
				break;
			default:
				break;
		}

	} else if (CAMERA_TYPE_OMNIVISION_2630 == cam_ctx->sensor_type) {
		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_RAW10:
				if ((cam_ctx->capture_input_width <= 400)
						&& (cam_ctx->capture_input_height <= 292)) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 800)
						&& (cam_ctx->capture_input_height <= 600)) {
					/* SVGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 1600)
						&& (cam_ctx->capture_input_height <= 1200)) {
					/* UXGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 5;
				}
				break;
			default:
				break;
		}

	} else if (CAMERA_TYPE_OMNIVISION_5623 == cam_ctx->sensor_type) {
		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_RAW10:
				if ((cam_ctx->capture_input_width <= 800)
						&& (cam_ctx->capture_input_height <= 600)) {
					/* CIF */
					sensor_timeperframe_numerator = 1;	
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 1600)
						&& (cam_ctx->capture_input_height <= 1200)) {
					/* UXGA */
					sensor_timeperframe_numerator = 2;	
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 2560)
						&& (cam_ctx->capture_input_height <= 1920)) {
					/* FULL */
					sensor_timeperframe_numerator = 2;	
					sensor_timeperframe_denominator = 5;
				}
				break;
			default:
				break;
		}

	} else {	/* ov7660 and ov7673 use the same framerate setting */

		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
			case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			case CAMERA_IMAGE_FORMAT_RGB565:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			case CAMERA_IMAGE_FORMAT_RAW8:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			default:
				break;
		}

	}

	dbg("*(p_camdrv_status->p_timeperframe_denominator) = %d\n",
			*(p_camdrv_status->p_timeperframe_denominator));
	dbg("sensor_timeperframe_numerator = %d\n",
			sensor_timeperframe_numerator);
	dbg("*(p_camdrv_status->p_timeperframe_numerator) = %d\n",
			*(p_camdrv_status->p_timeperframe_numerator));
	dbg("sensor_timeperframe_denominator = %d\n",
			sensor_timeperframe_denominator);

	framerate =
		((*(p_camdrv_status->p_timeperframe_numerator)) *
		 sensor_timeperframe_denominator) /
		((*(p_camdrv_status->p_timeperframe_denominator)) *
		 sensor_timeperframe_numerator);

	if (framerate == 0) {
		framerate = 0;
	} else if (framerate > 8) {
		framerate = 7;
	} else {
		framerate--;
	}

	return framerate;

}

static int pxa_camera_reset(struct pxa_camera_driver_status_t *p_camdrv_status)
{

	p_camera_context_t cam_ctx = g_camera_context;

	if (*(p_camdrv_status->p_capture_started)) {
		pxa_camera_ioctl_streamoff(p_camdrv_status);
	}

	if (*(p_camdrv_status->p_buf_submited)) {
		camera_desubmit_buffers(cam_ctx);
	}

	if (*(p_camdrv_status->p_buf_ready)) {
		camera_deprepare_buffers(cam_ctx);
	}

	return 0;
}

static int pxa_camera_ioctl_streamon(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{

	struct buf_node *buf_node;
	p_camera_context_t cam_ctx = g_camera_context;
	unsigned long flags;
	unsigned long camera_buffer_size;
	PRINTFUNC;

	if (!*(g_camdrv_status.p_buf_ready)) {
		camera_buffer_size = (CAMERA_MODE_STILL == cam_ctx->capture_mode)
		                     ?cam_ctx->still_capture_width * cam_ctx->still_capture_height
		                     :cam_ctx->video_capture_width * cam_ctx->video_capture_height;
		if (camera_buffer_size > 1920000) {
			/* only allocate one buffer for 1600*1200 above to save memory */
			if (camera_prepare_buffers(cam_ctx, 1) < 0) {
				return -EIO;
			}
		} else {
			if (camera_prepare_buffers(cam_ctx, 2) < 0) {
				return -EIO;
			}
		}
	}

	if (!*(g_camdrv_status.p_buf_submited)) {
		if (camera_submit_buffers(cam_ctx)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	/* Empty report head and put its buf into head queue */
	while (!list_empty(p_camdrv_status->p_report_head)) {
		printk(KERN_DEBUG " report head isn't empty\n");
		buf_node =
			list_entry(g_camdrv_status.p_report_head->next,
					struct buf_node, report_head);
		list_del_init(&buf_node->report_head);
		if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
			spin_unlock_irqrestore(&report_list_lock, flags);
			return -EIO;
		}
	}
	spin_unlock_irqrestore(&report_list_lock, flags);
	/* dbg_buffer_status(); */

	TESTPOINT;

	if (!*(g_camdrv_status.p_capture_started)) {
		if (camera_start_capture(cam_ctx)) {
			return -EIO;
		}
	}

	*(g_camdrv_status.p_capture_started) = 1;
	return 0;
}

static void pxa_camera_ioctl_streamoff(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{
	p_camera_context_t cam_ctx = g_camera_context;
	PRINTFUNC;

	camera_stop_capture(cam_ctx);
	*(g_camdrv_status.p_capture_started) = 0;

	return;
}

#ifdef CONFIG_DVFM
static void camera_notify_ipm(void)
{
#ifdef CONFIG_IPM
	pr_debug("send event to ipmd\n");
	ipm_event_notify(IPM_EVENT_DEVICE, IPM_EVENT_DEVICE_OUTD0CS, NULL, 0);
#endif
	wait_event_interruptible_timeout(g_camdrv_status.wait_outd0cs,
					(!(ACSR & 0x04000000)), HZ/100);
}
#else
static void camera_notify_ipm(void) {}
#endif

static int pxa_camera_open(struct inode *inode, struct file *file)
{
	int status = -1;
	dma_addr_t handle = 0;

	p_camera_context_t cam_ctx = g_camera_context;

	PRINTFUNC;

	if (g_camdrv_status.suspended) {
		return status;
	}
#ifdef CONFIG_DVFM
	spin_lock(&cam_status_lock);
#endif
	g_camdrv_status.driver_opened = 1;
#ifdef CONFIG_DVFM
	spin_unlock(&cam_status_lock);
	init_waitqueue_head(&(g_camdrv_status.wait_outd0cs));
	camera_notify_ipm();
#endif

	g_camdrv_status.still_buf_ready = 0;
	g_camdrv_status.video_buf_ready = 0;
	g_camdrv_status.still_buf_submited = 0;
	g_camdrv_status.video_buf_submited = 0;
	g_camdrv_status.still_capture_started = 0;
	g_camdrv_status.video_capture_started = 0;

	g_camdrv_status.still_timeperframe_numerator = 0;
	g_camdrv_status.video_timeperframe_numerator = 0;
	g_camdrv_status.still_timeperframe_denominator = 0;
	g_camdrv_status.video_timeperframe_denominator = 0;

	INIT_LIST_HEAD(&(g_camdrv_status.still_buf_head));
	INIT_LIST_HEAD(&(g_camdrv_status.video_buf_head));
	INIT_LIST_HEAD(&(g_camdrv_status.still_report_head));
	INIT_LIST_HEAD(&(g_camdrv_status.video_report_head));

	init_waitqueue_head(&(g_camdrv_status.camera_wait_q));
	g_camdrv_status.task_waiting = 0;

	/* set default value */
	cam_ctx->capture_mode = CAMERA_MODE_VIDEO;
	pxa_camera_set_mode(&g_camdrv_status, cam_ctx->capture_mode);

	INIT_LIST_HEAD(&pc_head.page_list);
	pc_head.page_count = 0;
	spin_lock_init(&pc_head.lock);
	init_MUTEX(&buf_list_sem);
	spin_lock_init(&report_list_lock);
	spin_lock_init(&cam_queue_lock);

	cam_ctx->video_capture_width = WIDTH_DEFT;
	cam_ctx->video_capture_height = HEIGHT_DEFT;
	cam_ctx->video_capture_scale = CAMERA_CAPTURE_SCALE_DISABLE;
	cam_ctx->video_capture_input_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
	cam_ctx->video_capture_output_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

	cam_ctx->still_capture_width = WIDTH_DEFT;
	cam_ctx->still_capture_height = HEIGHT_DEFT;
	cam_ctx->still_capture_scale = CAMERA_CAPTURE_SCALE_DISABLE;
	cam_ctx->still_capture_input_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
	cam_ctx->still_capture_output_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

	cam_ctx->frame_rate = FRAMERATE_DEFT;
	/* cam_ctx->ost_reg_base = (unsigned int)OST_BASE;
	   cam_ctx->gpio_reg_base = (unsigned int)GPIO_BASE; */
	cam_ctx->ci_reg_base = (unsigned int)(&(CICR0));
	/* cam_ctx->clk_reg_base = (unsigned int)CLK_BASE; */

	/*
	 * we alloc histogram/LUT buffer and its dma descriptor here
	 */
#ifdef LUT_LUTDES_USE_ISRAM
	/* cam_ctx->histogram_lut_buffer_virtual =
	   (u32)__ioremap(0x5c020000, 1024, 0, 1);
	   handle = 0x5c020000; */ /*for B0*/

	if ((cam_ctx->histogram_lut_buffer_virtual = imm_malloc(1024,
					IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, zylcam_immid))
			!= NULL) {
		handle = imm_get_physical((void *)cam_ctx->
				histogram_lut_buffer_virtual,
				zylcam_immid);
	} else {
		printk(KERN_DEBUG "can't mallco LUT from ISRAM by IMM successfully \n");
		goto alloc_histogram_buffer_error;
	}

#else
	cam_ctx->histogram_lut_buffer_virtual = dma_alloc_coherent(NULL,
			1024, &handle, GFP_KERNEL);
#endif

	if (!cam_ctx->histogram_lut_buffer_virtual) {
		dbg("Can't get memory for histogram buffer\n");
		goto alloc_histogram_buffer_error;
	} else
		cam_ctx->histogram_lut_buffer_physical = (volatile u32)handle;

#ifdef LUT_LUTDES_USE_ISRAM
	/* cam_ctx->histogram_lut_dma_descriptors_virtual =
	   (u32)__ioremap(0x5c020400, 16, 0, 1);
	   handle = 0x5c020400;
	 */  /* for B0*/

	if ((cam_ctx->histogram_lut_dma_descriptors_virtual = \
				imm_malloc(16,
					IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM,
					zylcam_immid)) != NULL) {

		handle = imm_get_physical(cam_ctx->
				histogram_lut_dma_descriptors_virtual,
				zylcam_immid);
	} else {
		printk
			("can't mallco LUT Des from ISRAM by IMM successfully \n");
		goto alloc_histogram_dma_desc_error;
	}
#else
	cam_ctx->histogram_lut_dma_descriptors_virtual =
		dma_alloc_coherent(NULL, 16, &handle, GFP_KERNEL);
#endif

	if (!cam_ctx->histogram_lut_dma_descriptors_virtual) {
		dbg("Can't get memory for phantom buffer\n");
		goto alloc_histogram_dma_desc_error;
	} else
		cam_ctx->histogram_lut_dma_descriptors_physical =
			(volatile u32)handle;

	/*
	 * We alloc phantom buffer here. The buffer node (list) will be
	 * alloc when application start capturing.
	 */
	cam_ctx->phantom_buffer_virtual = dma_alloc_coherent(NULL,
			PHANTOM_BUFFER_SIZE,
			&handle,
			GFP_KERNEL);
	if (!cam_ctx->phantom_buffer_virtual) {
		dbg("Can't get memory for phantom buffer\n");
		goto alloc_phantom_buffer_error;
	} else {
		cam_ctx->phantom_buffer_physical = (volatile u32)handle;
	}

	status = camera_init(cam_ctx);
	if (status) {
		goto camera_init_error;
	}
	printk(KERN_DEBUG "pxa_camera_open, status = %d \n", status);

	/* set interrupt mask */
	mcam_set_interrupt_mask(cam_ctx, INT_MASK);

	/* empty the report list */
	while (!list_empty(g_camdrv_status.p_report_head)) {
		dbg("no empty item in report head list \n");
		list_del_init(g_camdrv_status.p_report_head->next);
	}

	/* empty the head list */
	while (!list_empty(g_camdrv_status.p_buf_head)) {
		dbg("no empty item in report head list \n");
		list_del_init(g_camdrv_status.p_buf_head->next);
	}

	return status;

camera_init_error:
	dma_free_coherent(NULL, PHANTOM_BUFFER_SIZE,
			(void *)cam_ctx->phantom_buffer_virtual,
			cam_ctx->phantom_buffer_physical);

alloc_phantom_buffer_error:

#ifdef LUT_LUTDES_USE_ISRAM
	/* __iounmap(cam_ctx->histogram_lut_dma_descriptors_virtual); */
	imm_free((void *)cam_ctx->histogram_lut_dma_descriptors_virtual,
			zylcam_immid);

#else
	dma_free_coherent(NULL, 16,
			(void *)cam_ctx->
			histogram_lut_dma_descriptors_virtual,
			cam_ctx->histogram_lut_dma_descriptors_physical);
#endif

alloc_histogram_dma_desc_error:

#ifdef LUT_LUTDES_USE_ISRAM
	/* __iounmap(cam_ctx->histogram_lut_buffer_virtual); */
	imm_free((void *)cam_ctx->histogram_lut_buffer_virtual, zylcam_immid);
#else
	dma_free_coherent(NULL, 1024,
			(void *)cam_ctx->histogram_lut_buffer_virtual,
			cam_ctx->histogram_lut_buffer_physical);
#endif

alloc_histogram_buffer_error:
	return status;
}

static int pxa_camera_close(struct inode *inode, struct file *file)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct page *page;
	unsigned long flags;
	PRINTFUNC;

#ifdef CONFIG_DVFM
	spin_lock(&cam_status_lock);
#endif
	g_camdrv_status.driver_opened = 0;
#ifdef CONFIG_DVFM
	spin_unlock(&cam_status_lock);
#endif

	if (*(g_camdrv_status.p_capture_started)) {
		pxa_camera_ioctl_streamoff(&g_camdrv_status);
	}

	camera_deinit(cam_ctx);

	if (cam_ctx->phantom_buffer_virtual) {
		dma_free_coherent(NULL, PHANTOM_BUFFER_SIZE,
				(void *)cam_ctx->phantom_buffer_virtual,
				cam_ctx->phantom_buffer_physical);
	}
#ifdef LUT_LUTDES_USE_ISRAM
	if (cam_ctx->histogram_lut_dma_descriptors_virtual) {
		/* __iounmap(cam_ctx->histogram_lut_dma_descriptors_virtual); */
		imm_free((void *)cam_ctx->histogram_lut_dma_descriptors_virtual,
				zylcam_immid);
	}
#else
	if (cam_ctx->histogram_lut_dma_descriptors_virtual) {
		dma_free_coherent(NULL, 16,
				(void *)cam_ctx->
				histogram_lut_dma_descriptors_virtual,
				cam_ctx->
				histogram_lut_dma_descriptors_physical);
	}
#endif

#ifdef LUT_LUTDES_USE_ISRAM
	if (cam_ctx->histogram_lut_buffer_virtual) {
		/* __iounmap(cam_ctx->histogram_lut_buffer_virtual); */
		imm_free((void *)cam_ctx->histogram_lut_buffer_virtual,
				zylcam_immid);
	}
#else
	if (cam_ctx->histogram_lut_buffer_virtual) {
		dma_free_coherent(NULL, 1024,
				(void *)cam_ctx->histogram_lut_buffer_virtual,
				cam_ctx->histogram_lut_buffer_physical);
	}
#endif

	/* empty the report list */
	while (!list_empty(&(g_camdrv_status.still_report_head))) {
		list_del_init(g_camdrv_status.still_report_head.next);
	}
	while (!list_empty(&(g_camdrv_status.video_report_head))) {
		list_del_init(g_camdrv_status.video_report_head.next);
	}

	camera_free_buffer_list(CAMERA_MODE_STILL, 1);
	camera_free_buffer_list(CAMERA_MODE_VIDEO, 1);

	/* Free all the camera page cache */
	while (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		spin_unlock_irqrestore(&pc_head.lock, flags);

		atomic_set(&page->_count, 1);
		ClearPageReserved(page);
		__free_page(page);
	}
	pc_head.page_count = 0;

	return 0;
}

static ssize_t pxa_camera_read(struct file *file, char __user * buf,
		size_t count, loff_t * ppos)
{
	p_camera_context_t cam_ctx = g_camera_context;
	ssize_t ret = 0;
	struct buf_node *buf_node;
	char __user *tmp_buf = buf;
	unsigned long flags;
	PRINTFUNC;

	if (!*(g_camdrv_status.p_capture_started)) {
		if (pxa_camera_ioctl_streamon(&g_camdrv_status)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (list_empty(g_camdrv_status.p_report_head)) {
		int ret;
		g_camdrv_status.task_waiting = 1;
		spin_unlock_irqrestore(&report_list_lock, flags);
		ret = wait_event_interruptible(g_camdrv_status.camera_wait_q,
				!list_empty(g_camdrv_status.
					p_report_head));
		if (ret)
			return ret;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
	}

	spin_lock_irqsave(&report_list_lock, flags);
	buf_node =
		list_entry(g_camdrv_status.p_report_head->next, struct buf_node,
				report_head);
	if (!buf_node) {
		ret = -EINVAL;
		spin_unlock_irqrestore(&report_list_lock, flags);
		goto out;
	}

	list_del(&buf_node->report_head);
	spin_unlock_irqrestore(&report_list_lock, flags);

	dbg("fifo0 = %d, fifo1 = %d, fifo2 = %d\n", buf_node->fifo0_size,
			buf_node->fifo1_size, buf_node->fifo2_size);
	dbg("begin: read size = %d\n", buf_node->fifo0_size);
	if (copy_to_user(tmp_buf, buf_node->Y_vaddr, buf_node->fifo0_size)) {
		ret = -EFAULT;
		goto out;
	}
	dbg("end: read size = %d\n", buf_node->fifo0_size);

	tmp_buf += buf_node->fifo0_size;

	if (buf_node->fifo1_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cb_vaddr, buf_node->fifo1_size)) {
			ret = -EFAULT;
			goto out;
		}
		tmp_buf += buf_node->fifo1_size;
	}

	if (buf_node->fifo2_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cr_vaddr, buf_node->fifo2_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	ret =
		buf_node->fifo0_size + buf_node->fifo1_size + buf_node->fifo2_size;

out:
	camera_submit_buffer(cam_ctx, buf_node->buf_index);

	return ret;
}

static int pxa_camera_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long iterators = (unsigned long)vma->vm_start;
	unsigned long size = 0;
	int i, ret = 0;
	struct buf_node *buf_node;
	unsigned int offset;
	unsigned int buf_index;

	p_camera_context_t cam_ctx = g_camera_context;
	unsigned long camera_buffer_size;
	unsigned long pgprot = pgprot_noncached(PAGE_SHARED);
	PRINTFUNC;

	camera_buffer_size = (CAMERA_MODE_STILL == cam_ctx->capture_mode)
			     ?cam_ctx->still_capture_width * cam_ctx->still_capture_height
			     :cam_ctx->video_capture_width * cam_ctx->video_capture_height;
#define DCACHE_SIZE 32768
	if (camera_buffer_size > 2*DCACHE_SIZE)
		pgprot |= L_PTE_CACHEABLE | L_PTE_BUFFERABLE;
#ifdef	CACHEABLE_BUFFER
	pgprot |= L_PTE_CACHEABLE | L_PTE_BUFFERABLE;
#endif

	offset = vma->vm_pgoff << PAGE_SHIFT;
	buf_index = offset / camera_get_buffer_size();

	down_interruptible(&buf_list_sem);
	buf_node = camera_get_buffer_from_index(buf_index);
	for (i = 0; i < buf_node->page_num; i++) {
		if (remap_pfn_range(vma,
				iterators,
				(__pa(page_address(buf_node->pages[i]))) >>
				PAGE_SHIFT, PAGE_SIZE, pgprot)
				) {
			ret = -EFAULT;
			goto remap_page_error;
		}
		size += PAGE_SIZE;
		iterators += PAGE_SIZE;
	}
	up(&buf_list_sem);

	return ret;

remap_page_error:
	dbg("mmap failed!\n");
	/* FIXME: Maybe we should use other function to unmap */
	do_munmap(vma->vm_mm, vma->vm_start, size);
	return ret;
}

static unsigned int pxa_camera_poll(struct file *file, poll_table * wait)
{
	unsigned long flags;
	PRINTFUNC;

	if (!*(g_camdrv_status.p_capture_started)) {
		if (pxa_camera_ioctl_streamon(&g_camdrv_status)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(g_camdrv_status.p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll successfully!\n");
		return POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	g_camdrv_status.task_waiting = 1;
	poll_wait(file, &(g_camdrv_status.camera_wait_q), wait);

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(g_camdrv_status.p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll successfully!\n");
		return POLLIN | POLLRDNORM;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll failed!\n");
		return 0;
	}
}

static int pxa_camera_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param)
{
	return video_usercopy(inode, file, cmd, param, camera_do_ioctl);
}

static void pxa_camera_release(struct video_device *dev)
{
	printk(KERN_WARNING "pxa_camera_zl:  has no release callback. "
			"Please fix your driver for proper sysfs support, see "
			"http://lwn.net/Articles/36850/\n");

	return;
}

#ifdef CONFIG_PM
/*
 * Suspend the Camera Module.
 */
static int pxa_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	p_camera_context_t cam_ctx = g_camera_context;
	PRINTFUNC;

	dbg("LPM ENTERING.....\n");

	g_camdrv_status.suspended = 1;

	if (!(g_camdrv_status.driver_opened)) {
		if (g_camdrv_status.i2c_inited) {
			g_camdrv_status.i2c_inited = 0;
		}
		return 0;
	}

	if ((state.event == PM_EVENT_SUSPEND)) {
		dbg("Sleep ENTERING.....\n");
		printk(KERN_ALERT "PXA_CAMERA: camera suspend\n");
		mcam_suspend(cam_ctx);
		disable_irq(IRQ_CAMERA);
		if (g_camdrv_status.i2c_inited)
			g_camdrv_status.i2c_inited = 0;
		return 0;
	}

	return -1;
}

/*
 * Resume the Camera Module.
 */
static int pxa_camera_resume(struct platform_device *pdev)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct buf_node *buf_node;
	unsigned long flags;
	struct pxa_camera_driver_status_t *p_camdrv_status;

	int buf_type;
	PRINTFUNC;

	g_camdrv_status.suspended = 0;

	if (!(g_camdrv_status.driver_opened)) {
		if (!g_camdrv_status.i2c_inited) {
			g_camdrv_status.i2c_inited = 1;
		}
		return 0;
	}
	/* dbg_buffer_status(); */

	p_camdrv_status = &(g_camdrv_status);

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	/* init to output 3.2v temporarily.
	 * should be done by system 
	 */
#ifdef	CONFIG_MACH_ZYLONITE
	pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 3200);
#elif defined (CONFIG_MACH_LITTLETON)
	pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 2500);
	pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, 1800);
#endif

	/* enable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 1);

	spin_lock_irqsave(&report_list_lock, flags);
	/* Empty report head and put its buf into head queue */
	while (!list_empty(p_camdrv_status->p_report_head)) {
		printk(KERN_DEBUG " report head isn't empty\n");
		buf_node =
			list_entry(g_camdrv_status.p_report_head->next,
					struct buf_node, report_head);
		list_del_init(&buf_node->report_head);
		if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
			spin_unlock_irqrestore(&report_list_lock, flags);
			return -EIO;
		}
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	mcam_resume(cam_ctx);

	mcam_set_interrupt_mask(cam_ctx, INT_MASK);
	enable_irq(IRQ_CAMERA);

	if (cam_ctx->dma_running) {
		spin_lock_irqsave(&report_list_lock, flags);
		if (list_empty(g_camdrv_status.p_report_head)) {
			g_camdrv_status.task_waiting = 1;
			spin_unlock_irqrestore(&report_list_lock, flags);
			wait_event_interruptible(g_camdrv_status.
					camera_wait_q,
					!list_empty
					(g_camdrv_status.
					 p_report_head));
		} else
			spin_unlock_irqrestore(&report_list_lock, flags);
	}
	return 0;
}
#endif

#ifdef CONFIG_DVFM
static int check_dvfm_status(struct pxa3xx_fv_notifier_info *info)
{
	/* When camera is on, let system out of D0CS */
	if ((info->cur.d0cs == 1) && (info->next.d0cs == 0)) {
		return 0;
	}
#if 0	
	/* Only let cpu upgrade the frequency of CPU */
	if ((info->cur.d0cs == 0) && (info->next.d0cs == 0)) {
		if ((info->cur.xl * info->cur.xn)
			< (info->next.xl * info->next.xn)) {
			return 0;
		}
	}
#endif
	return -EINVAL;
}

static int
pxa_camera_dvfm_notifier(unsigned int cmd, void *client_data, void *info)
{
	struct pxa_camera_driver_status_t *camera_status = client_data;
	struct pxa3xx_fv_notifier_info *p = info;

	spin_lock(&cam_status_lock);
	switch (cmd) {
		case FV_NOTIFIER_QUERY_SET:
			/* When camera is enabled, system frequency can't be changed */
			if (camera_status->driver_opened) {
				if (check_dvfm_status(info)) {
					spin_unlock(&cam_status_lock);
					return -EPERM;
				}
			}
			break;

		case FV_NOTIFIER_PRE_SET:
			break;

		case FV_NOTIFIER_POST_SET:
			if (camera_status->driver_opened) {
				if ((p->cur.d0cs == 1) && (p->next.d0cs == 0))
					wake_up(&(g_camdrv_status.wait_outd0cs));
			}
			break;
	}

	spin_unlock(&cam_status_lock);
	return 0;
}
#endif

static int pxa_camera_probe(struct platform_device *pdev)
{

	g_camdrv_status.i2c_inited = 1;
	g_camdrv_status.suspended = 0;

	/* allocte camera context */
	g_camera_context = kzalloc(sizeof(camera_context_t), GFP_KERNEL);
	if (!g_camera_context) {
		dbg("Can't allocate buffer for" "camera control structure \n");
		return -ENOMEM;
	}

	/* allocte camera functions context */
	g_camera_context->camera_functions = kzalloc(sizeof(camera_function_t),
			GFP_KERNEL);
	if (!g_camera_context->camera_functions) {
		dbg("Can't allocate buffer for"
				"camera functions structure \n");
		goto malloc_camera_functions_err;
		return -ENOMEM;
	}

	if (video_register_device(&vd, VFL_TYPE_GRABBER, pxa_camera_minor) < 0) {
		printk(KERN_ALERT "PXA_CAMERA: video_register_device failed\n");
		goto register_video_error;
	} else {
		printk(KERN_ALERT "PXA_CAMERA: PXA Camera driver loaded for"
				"/dev/video%d \n", pxa_camera_minor);
	}

#ifdef CONFIG_DVFM
	spin_lock_init(&cam_status_lock);
	spin_lock(&cam_status_lock);
#endif
	g_camdrv_status.driver_opened = 0;
#ifdef CONFIG_DVFM
	spin_unlock(&cam_status_lock);
#endif
	/* request irq */
	if (request_irq(IRQ_CAMERA, pxa_camera_irq, 0, "PXA Camera", &vd)) {
		printk(KERN_ALERT
				"PXA_CAMERA: Camera interrupt register failed \n");
		goto register_video_error;
	}
	disable_irq(IRQ_CAMERA);

#ifdef CONFIG_DVFM
	dvfm_notifier.client_data = (void *)&g_camdrv_status;
	pxa3xx_fv_register_notifier(&dvfm_notifier);
#endif

	return 0;

register_video_error:
	if (g_camera_context->camera_functions) {
		kfree(g_camera_context->camera_functions);
	}

malloc_camera_functions_err:
	if (g_camera_context) {
		kfree(g_camera_context);
	}
	return -EIO;
}

static int pxa_camera_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DVFM
	pxa3xx_fv_unregister_notifier(&dvfm_notifier);
#endif

	g_camdrv_status.i2c_inited = 0;

	free_irq(IRQ_CAMERA, &vd);
	video_unregister_device(&vd);
	kfree(g_camera_context->camera_functions);
	kfree(g_camera_context);
	printk(KERN_ALERT "PXA_CAMERA: PXA Camera driver unloaded.\n");
	return 0;
}

static struct platform_driver pxa_camera_driver = {
	.driver = {
		.name = "pxa3xx-camera"
	},
	.probe 		= pxa_camera_probe,
	.remove 	= pxa_camera_remove,
#ifdef CONFIG_PM
	.suspend 	= pxa_camera_suspend,
	.resume 	= pxa_camera_resume,
#endif
};

static int __devinit pxa_camera_init(void)
{
#ifdef LUT_LUTDES_USE_ISRAM
	if ((zylcam_immid = imm_register_kernel("pxa_camera")) == 0) {
		printk(KERN_ERR "Zylonite Camera Driver - Error registering with IMM\n");
	}
#endif
	return platform_driver_register(&pxa_camera_driver);
}

static void __exit pxa_camera_exit(void)
{
	platform_driver_unregister(&pxa_camera_driver);
}

module_init(pxa_camera_init);
module_exit(pxa_camera_exit);

MODULE_DESCRIPTION("Zylonite/PXA3xx Camera Interface driver");
MODULE_LICENSE("GPL");
