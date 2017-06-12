/* Core MMC driver functions
 *
 * Copyright (c) 2002 Hewlett-Packard Company
 *   
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *  
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *  
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          6 May 2002 
 */

/*
 * mss_block.h - MMC/SD Card driver (block device driver) header file
 *
 * Copyright (C) 2006 Intel Corporation
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __MSS_BLOCK_H__
#define __MSS_BLOCK_H__


//#include "mss_core.h"

#define MSS_SHIFT	(3)
#define MSS_BLOCK_LENGTH	(512)

struct mss_block_device {
	struct request_queue	*mss_request_queue;
	struct gendisk		*mss_gendisk;
	struct mss_io_request	block_request;
	struct mss_card_device *card_dev;

	u32			flags;	/* for suspend/resume */
#define MMC_QUEUE_SUSPENDED	(1 << 1)
#define MMC_QUEUE_END		(1 << 0)
	int			usage;
	int			card_loaded;
	spinlock_t		dev_lock;
	spinlock_t		request_lock;
	int			changed;
	u8			lock_param[258];
	int			lock_result;
	struct completion	lock_wait;	
	struct completion	thread_complete;
	wait_queue_head_t	thread_wq;
	struct semaphore	thread_sem;
	struct request *	req;
};




#endif
