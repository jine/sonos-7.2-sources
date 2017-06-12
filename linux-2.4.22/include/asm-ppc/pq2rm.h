/*******************************************************************
 * pq2rm.h 2003/07/13 14:11:13
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

#ifndef PQRM_H

#define PQRM_H

#include <linux/list.h>
#include <asm/pq2defs.h>
#include <asm/resmgr.h>

#if RM_MAX_VARS < 5
    #error RM_MAX_VARS must be >= 5
#endif

/*
 * Resource management for PQ device family
 */

#define CONFIG_PQRM_DEBUG

#define PQRM_REG_DEBUG

#define PQ_RM_VAR_PQID   0

/* Minimum amount of DPRAM that can be kept as a separate
   free resource
*/
#define PQRM_MIN_MEM_SLICE 1

/* PQ object tree starts from objects registeted on
   PQII devices.
   That is, every object and resource in PQRM object
   hierarchy is associated with a PQII device
*/
int pq_register_object( int pqid, void *obj,
			const char *name,
			struct rm_obj **prmo );
int pq_get_pqii(struct rm_obj *rmo);


/*
 * Peripheral device management
 */
int pq_xcc_open( struct rm_obj *rmo, pq_xcc_id xcc, pq_protocol protocol );   /* Reserve xcc */
int pq_xcc_close( struct rm_obj *rmo, pq_xcc_id xcc );  /* Release xcc */
int pq_xcc_is_valid( int pqid, pq_xcc_id xcc );

/*
 * Parallel Port Pin management
 */
int pq_pin_reserve( struct rm_obj *rmo, pq_port_id port, __u32 pin_mask );
int pq_pin_release( struct rm_obj *rmo, pq_port_id port, __u32 pin_mask );
/* Reserve and program pin */
int pq_pin_set( struct rm_obj *rmo, pq_port_id port, __u32 pin_mask, __u32 spec );
/* Pin spec flags */
#define PQ_PORT_SET_PAR         0x00010000
#define PQ_PORT_SET_SOR         0x00020000
#define PQ_PORT_SET_ODR         0x00040000
#define PQ_PORT_SET_DIR         0x00080000
#define PQ_PORT_PAR_GP          0x00000000   /* PPAR: General purpose pin */
#define PQ_PORT_PAR_PERIPH      0x00000001   /* PPAR: dedicated pin */
#define PQ_PORT_SOR_0           0x00000000   /* PSOR: option 1 */
#define PQ_PORT_SOR_1           0x00000002   /* PSOR: option 2 */
#define PQ_PORT_DIR_OUT         0x00000004   /* PDIR: output */
#define PQ_PORT_DIR_IN          0x00000000   /* PDIR: input */
#define PQ_PORT_ODR_OUT         0x00000000   /* ODR:  output */
#define PQ_PORT_ODR_3STATE      0x00000008   /* ODR:  3-stated */

/* Alternative form of function for pin programming */
int pq_pin_program(struct rm_obj *rmo, pq_port_id port, __u32 pin_mask, int par, int sor, int odr, int dir);

/*
 * DPRAM management
 */
void *pq_intram_base(int pqid);

/* Allocate chunk of DPRAM of the given size and alignment.
   Try to allocate block with offset higher than start.
   If not possible - allocate anywhere.
*/
void *pq_dpram_alloc_ext(struct rm_obj *rmo, __u32 start, size_t size, size_t align);

/* Allocate chunk of DPRAM of the given size and alignment */
static inline void *pq_dpram_alloc(struct rm_obj *rmo, size_t size, size_t align)
{
   return pq_dpram_alloc_ext(rmo, 0, size, align);
}

void pq_dpram_free(struct rm_obj *rmo, void *addr);

/* Reserve DPRAM area. Returns address of the reserved area
   or NULL if reservation is not possible.
*/
void *pq_dpram_reserve(struct rm_obj *rmo, __u32 offset, size_t size);

/* Convert address to DPRAm offset */
__u16 pq_dpram_offset(int pqid, void *addr);

/*
 * BRG/clock management
 */
int pq_clock_connect(struct rm_obj *rmo, pq_xcc_id xcc, pq_clock_id clock, __u32 flags);
int pq_clock_disconnect(struct rm_obj *rmo, pq_xcc_id xcc, pq_clock_id clock);
int pq_clock_brg_start(struct rm_obj *rmo, pq_clock_id brg, __u32 hertz, __u32 flags);
int pq_clock_brg_stop(struct rm_obj *rmo, pq_clock_id brg);

#define PQRM_CLK_RX        0x00000001
#define PQRM_CLK_TX        0x00000002
#define PQRM_BRG_AUTO      0x00000010    /* Autobaud */
#define PQRM_BRG_ASYNC     0x00000020    /* Async clock */
#define PQRM_BRG_SRC_INT   0x00000000    /* Internal clock source - default */
#define PQRM_BRG_SRC_CLK3  0x00000100    /* CLK3 */
#define PQRM_BRG_SRC_CLK5  0x00000200    /* CLK5 */
#define PQRM_BRG_SRC_CLK9  0x00000400    /* CLK9 */
#define PQRM_BRG_SRC_CLK15 0x00000800    /* CLK15 */

/*
 * CPM timer management
 */

/* Timer ISR. The function does only application specific stuff.
   Timer interrupt is acknowledged by PQRM service.
*/
typedef void (*pq_tmr_isr_f)(void *parm);

/* Reserve and configure CPM timer.
   Only pq_xtmr1-4 can be used as the timer id.
 */
int pq_tmr_configure(struct rm_obj *rmo, pq_clock_id tmr, __u32 flags,
		     pq_tmr_isr_f isr, void *parm, struct rm_obj **p_res);
/* Start timer for period (us) */
int pq_tmr_start(struct rm_obj *res, __u32 period_usecs);
int pq_tmr_stop(struct rm_obj *res);
__u32 pq_tmr_timestamp(struct rm_obj *res);

/* Timer flags */
#define PQRM_TMR_PRESCALE_MASK 0x000000FF/* Timer prescale mask */
#define PQRM_TMR_IN_BUS        0x00000000    /* Input clock source for a timer is bus clock */
#define PQRM_TMR_IN_BUS16      0x00000100    /* Input clock source for a timer is bus clock/16 */
#define PQRM_TMR_IN_TIN        0x00000200    /* Input clock source for a timer is TIN pin */
#define PQRM_TMR_TOGGLE_TOUT   0x00000400    /* Toggle TOUT pin */
#define PQRM_TMR_ONESHOT       0x00000000    /* Timer is one shot (default) */
#define PQRM_TMR_PERIODIC      0x00001000    /* Timer is periodic */
#define PQRM_TMR_CASCADED      0x00002000    /* Cascaded timer */
#define PQRM_TMR_INTERRUPT     0x00004000    /* Generate interrupt when timer fires */
#define PQRM_TMR_AUTOSCALE     0x00008000    /* Scale timer automatically (prescale,BUS/BUS16) */


/* fixme: add functions to manage IRQs */

/* fixme: there should be a debug version of all functions that
   accepts 2 additional parameters: function name and line number
*/

#endif
