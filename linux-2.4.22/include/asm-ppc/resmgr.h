/*******************************************************************
 * resmgr.h 2003/07/13 14:11:13
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

#ifndef RESMGR_H

#define RESMGR_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/proc_fs.h>

/*
 * Hierarchical resource management
 */

#define RESMGR_DEBUG

/* Max number of object variables
 */
#define RM_MAX_VARS   5
#if RM_MAX_VARS < 3
    #error RM_MAX_VARS must be >= 3
#endif

/* Object variable */
typedef unsigned long rm_var;


#define RM_VAR_NOT_SET    0xFFFFFFFF
#define RM_PROC_DIR_NAME  "resmgr"
#define RM_OBJECT_MAJIC   (('r'<<24)|('m'<<16)|('o'<<8)|' ')
#define RM_RESOURCE_MAJIC (('r'<<24)|('m'<<16)|('r'<<8)|' ')
#define RM_TYPE_MAJIC     (('r'<<24)|('m'<<16)|('t'<<8)|' ')

/* Minimal alignment for rm_kmalloc
 */
#define RM_MIN_ALIGN      1

/* Enable /proc/resmgr/mem service
 */
#ifdef CONFIG_PROC_FS
#define RM_ENABLE_PROC_FS
#define RM_ENABLE_MEM_DEB
#else
#undef RM_ENABLE_PROC_FS
#undef RM_ENABLE_MEM_DEB
#endif

/* Object

   list usage:
   object: obj_list   - in-list of objects sharing the same parent
           child_list - list head of children of the given object
	   res_list   - list head of resources owned by the given object
	   parent     - object owning the given object

   resource_type:
           obj_list   - not used
	   child_list - list head of resources of the given type
	   res_list   - in-list of resources sharing the same owner
	   parent     - object owning the given type

   resource:
           obj_list   - in-list of resources of the same type
	   child_list - kludge: next points to object owning the resource
	   res_list   - in-list of resources sharing the same owner
	   parent     - resource type
 */
struct rm_obj
{
      struct list_head obj_list;
      struct list_head child_list;
      struct rm_obj *parent;
      char *name;
      void *obj;                  /* Opaque application pointer */
      struct proc_dir_entry *dir; /* Object directory in proc_fs */
      int users;                  /* Number of users. FFU */
      struct list_head res_list;  /* Resource list */
      int errno;                  /* Sticky errno */
      rm_var vars[RM_MAX_VARS];   /* Object variables */
      int nres;                   /* Number of resources associated with the object */
      int magic;                  /* Magic signature */
};


/* Resource operations */
struct rm_res_type_ops
{
      /* Alloc resource callback accepts new resource object
	 that contains information the resource type
	 and parameters p1, p2, p3. The callback is free to update
	 p1, p2 and p3 and also store additional information in
	 the resource object.
      */
      int (*alloc)(struct rm_obj *owner, struct rm_obj *new_res);

      /* Free internal resource associated with the resource object */
      int (*free)(struct rm_obj *owner, struct rm_obj *res);

      /* Format resource string for /proc_fs */
      int (*read_format)(struct rm_obj *res, char *buf, int count);

      /* Get resource owner name for /proc_fs */
      const char *(*get_owner_name)(struct rm_obj *res);
};


/* Resource type
 */
struct rm_resource_type
{
      struct rm_obj rmo;  /* Resource type itself is an object */
      struct rm_res_type_ops ops;
};

/*
 * APIs
 */

/* Resource management */
int rm_register_type(struct rm_obj *parent,
		     const char *name,
		     struct rm_res_type_ops *ops,
		     struct rm_resource_type **p_res_type);
int rm_alloc(struct rm_obj *rmo, struct rm_resource_type *res_type,
	     rm_var p1, rm_var p2, rm_var p3, struct rm_obj **p_res);
int rm_free(struct rm_obj *res);
int rm_find(struct rm_obj *rmo, struct rm_resource_type *res_type,
             rm_var p1, rm_var p2, rm_var p3, struct rm_obj **p_res);
struct rm_obj *rm_alloc_res_block(__u32 extra_size, __u32 flags);


/* Get resource owner
 */
static inline struct rm_obj *rm_owner(struct rm_obj *res)
{
   return (struct rm_obj *)res->child_list.next;
}

/* Set resource owner */
int rm_setowner(struct rm_obj *res, struct rm_obj *new_owner);

/*
 * All shared resources managed by pqrm are owned by objects.
 * Object must be registered before it can request a resource.
 * When object is unregistered, all resources it owns can be
 * deallocated automatically. The registered objects and the
 * resources they own can be tracked using
 * /proc/pqrm/objects
 * /proc/pqrm/resources
 */
int rm_register_object( struct rm_obj *parent, void *obj,
			const char *name, struct rm_obj **prmo );

/* Unregister object.
   If force==0 all object's resources must be deallocated
   beforehand.
   If force!=0 all object's resources are deallocated automatically
*/
int rm_unregister_object( struct rm_obj *rmo, int force );

/* User object to RM object mapping.
   NULL user object pointer corresponds to root RM object
*/
struct rm_obj *rm_uo2rmo(void *obj);

/* RM object to user object mapping */
static inline void *rm_rmo2uo(struct rm_obj *rmo)
{
   return rmo->obj;
}

/* Get sticky error code */
static inline int rm_errno(struct rm_obj *rmo)
{
   return rmo->errno;
}

/* Clear sticky error code */
static inline void rm_clear_errno(struct rm_obj *rmo)
{
   rmo->errno=0;
}

/* Set object's private variable */
int rm_set_var(struct rm_obj *rmo, int varnum, rm_var value);

/* Get object's private variable */
rm_var rm_get_var(struct rm_obj *rmo, int varnum);


/* The following functions are not part of the RM framework,
   but useful enough to deserve to be included in here
   flags is a combination of GFP_.. flags (kmalloc) and
   additional rm-specific flags RM_GFP_..
   Note!: area bigger than PAGE_SIZE may be not contiguous in physical memory.
          However, if memory is allocated for array of blocks with size =align
	  and align is ^2, it is guaranteened that all blocks are contiguous.
   Note2: addresses returned by rm_kmalloc can be converted using iopa
*/
#define RM_GFP_NOCACHE      0x00100000   /* Allocate in non-cacheable memory */
#define RM_GFP_CONTIG       0x00200000   /* Allocate contiguous memory region */

void *rm_kmalloc(struct rm_obj *rmo, size_t size, __u32 align, __u32 flags);
void rm_kfree(void *addr);

/*
 * Low-overhead tracer/profiler.
 */

#ifdef RM_ENABLE_PROC_FS
/* Initialize trace buffer */
int rm_trace_init(struct rm_obj *rmo,    /* Object owning the trace */
		  int max_entries,       /* Max number of trace entries to store */
		  int data_size,         /* Max size of data buffer associated with trace entry */
		  const char *format,    /* printf format applied to trace parameters p1, p2, p3 */
		  __u32 flags,           /* RM_TRACE_.. flags */
		  struct rm_obj **p_res);/* Trace resource object handle is returned here */
#define RM_TRACE_CIRCULAR         0x00000000 /* Circular trace buffer (default) */
#define RM_TRACE_STOP_WHEN_FULL   0x00000001 /* Stop tracing when trace buffer is full */
#define RM_TRACE_KEEP_AFTER_PRINT 0x00000002 /* Keep trace entries after printing. The default is flush */


/* Enable trace */
int rm_trace_enable(struct rm_obj *res);

/* Disable trace after additional cutoff entries are stored */
int rm_trace_disable(struct rm_obj *res, int cutoff);

/* Add trace entry */
int rm_trace_add(struct rm_obj *res, int point, __u32 p1, __u32 p2, __u32 p3,
		 int data_len, const void *data);

#else  /* RM_ENABLE_PROC_FS */

/* Trace is not supported in absence of proc_fs */
#define rm_trace_init(rmo, max_entries,data_size,format,flags,p_res) 0
#define rm_trace_enable(res) 0
#define rm_trace_disable(res,cutoff) 0
#define rm_trace_add(res,point,p1,p2,p3,data_len,) -EINVAL

#endif  /* RM_ENABLE_PROC_FS */


/*
 * A couple of defines that not really belong here
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#define irqreturn_t void
#define IRQ_HANDLED
#endif

#endif
