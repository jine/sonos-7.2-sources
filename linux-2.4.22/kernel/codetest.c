/*
-------------------------------------------------------------------------------|
|
| codetest.c - implementation of CodeTEST RTOS event hooks
|
-------------------------------------------------------------------------------|
*/
#include <linux/config.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/page.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm/system.h>


/* 
** HWIC definitions 
*/
#define CT_TASK_CREATE    (0x2a100000)
#define CT_TASK_ENTER     (0x2a200000)
#define CT_TASK_DELETE    (0x2a300000)
#define CT_TASK_EXIT      (0x2a400000)
#define CT_ISR_ENTER      (0x2a500000)
#define CT_ISR_EXIT       (0x2a600000)

#define CT_TASK_NAMES     	(0x00020000)
#define CT_TASK_IDS       	(0x00010000)
#define CT_TPP_OVERRUN_TAG	(0x2c400000)	

#ifdef CONFIG_CT_USEPCI
#include <linux/pci.h>

#define CT_VENDOR_ID           (0x11ab)
#define CT_DEVICE_ID           (0xf007)

#define _LITTLE_ENDIAN

enum
{
  CT_BAR_0,
  CT_BAR_1,
  CT_BAR_2,
  CT_BAR_3,
  CT_BAR_4,
  CT_BAR_5,
  CT_NUM_RESOURCES
};

#endif /* CONFIG_CT_USEPCI */

volatile unsigned long * ct_ctrl_port_ptr = NULL;
volatile unsigned long * ct_data_port_ptr = NULL;
void     ct_enable_hooks(void);
void     ct_disable_hooks(void);
extern unsigned long 	ct_get_phys_addr(void);
extern void 		ct_set_phys_addr( unsigned long addr );

#ifndef CONFIG_CT_HOOKS_ENABLED
#define CONFIG_CT_HOOKS_ENABLED  (0)
#endif

#ifdef CONFIG_CT_USEMICTOR
#define CT_PHYS_ADDR CONFIG_CT_ADDR
#else
#define CT_PHYS_ADDR 0
#endif

static unsigned long            ct_phys_addr = CT_PHYS_ADDR;
static unsigned long            ct_hooks_enabled = CONFIG_CT_HOOKS_ENABLED;

/* 
**    SWIC definitions
*/
#include "ctswic.h"

/* the kernel event tags are placed in SWIC buffer*/
unsigned ct_swic_enabled = 0;
/* the ISR event tags are ignored */
unsigned ct_filter_events = 1;
/* DEBUG -  force tags output to HWIC ports when SWIC enabled */
unsigned ct_force_hwic_enabled = 0;
/* reference to ring buffer defined in ctdriver */
struct Hook_Buffer * ct_hook_buffer = NULL;

#define HI_BIT_ONLY 0x80000000
#define HOOK_INC(x)  ((x) = (((x)+1) & (HOOK_BUFFER_SIZE-1)))

unsigned task_tag_count = 0 ;
static unsigned dropped = 0 ;
static pid_t deleted_thread_id = -1;


typedef unsigned long long U64;


void ctTaskTags(unsigned n_dtags, const u32 dtags[], u32 tag);
int ct_encode_taskname(const char* name, u32 buffer[8]);
u32 ctTagClock(void);

/* HWIC kernel exports */
EXPORT_SYMBOL(ct_enable_hooks);
EXPORT_SYMBOL(ct_disable_hooks);
EXPORT_SYMBOL(ct_get_phys_addr);
EXPORT_SYMBOL(ct_set_phys_addr);


/* SWIC kernel exports */
EXPORT_SYMBOL(ct_swic_enabled);
EXPORT_SYMBOL(ct_filter_events);
EXPORT_SYMBOL(ct_force_hwic_enabled);
EXPORT_SYMBOL(ct_hook_buffer);

/*
-------------------------------------------------------------------------------|
|
| ct_init() - perform physical -> kernel virtual mapping for the tag ports
|
-------------------------------------------------------------------------------|
*/
int ct_init( void )
{
#ifdef CONFIG_CT_USEPCI
  struct pci_dev * ct;
#endif
  int i;
  static int printed = 0;


  /*
  ** Check to see if the tag pointers have been initialized
  */
  if( ct_ctrl_port_ptr == NULL )
    {
#ifdef CONFIG_CT_USEPCI
      /*
      ** Check to see if the CodeTEST PCI adapter has been found
      */
      if( (ct = pci_find_device(CT_VENDOR_ID, CT_DEVICE_ID, 0)) == 0 )
	{
	  /*
	  ** Be polite and only print once every 4 billion tries
	  */
	  if( !printed++ )
	    {
	      printk( "CODETEST: PCI device not located\n" );
	    }
	  return -1;
	}

      printk( "CODETEST: found device \"%s\"\n", ct->name );
      printk( "CODETEST: resource \"%s\", start 0x%08x, end 0x%08x, flags 0x%08x\n",
	      ct->resource[CT_BAR_2].name,
	      ct->resource[CT_BAR_2].start,
	      ct->resource[CT_BAR_2].end,
	      ct->resource[CT_BAR_2].flags );
      /*
      ** Store the address of the PCI adapter tag port window
      */
      ct_phys_addr = ct->resource[CT_BAR_2].start;
#endif

      /*
      ** Map the physical addresses of the ports to kernel virtual addresses
      */
      printk( "CODETEST: attempting to map physical address 0x%08lx\n",(unsigned long) ct_phys_addr );
      ct_ctrl_port_ptr = (unsigned long *)ioremap( ct_phys_addr, PAGE_SIZE );
      ct_data_port_ptr = ct_ctrl_port_ptr + 1;
      printk( "CODETEST: physical address 0x%08lx mapped to kernel virtual address 0x%08lx\n",
	      ct_phys_addr, (unsigned long)ct_ctrl_port_ptr );
    }

  return 0;
}

/*
-------------------------------------------------------------------------------|
|
| ct_thread_event() - generic data/ctrl tag output function
|
-------------------------------------------------------------------------------|
*/
void ct_thread_event( unsigned long tag, struct task_struct * p )
{
#if 0
  unsigned long * tname;
  int i;
  char revname[16];
  int len;

  /*
  ** If enabled, check to see if the tag pointers have been initialized
  */

  if( ((ct_hooks_enabled == 0)&&(ct_force_hwic_enabled == 0)) ||
      ((ct_ctrl_port_ptr == NULL) && (ct_init() == -1)) )
    {
      return;
    }

  /*
  ** Reverse the name being sent to the CodeTEST probe since
  ** it goes on a stack
  */
  len = sizeof(p->comm);
  for( i = 0; i < len; i++ )
    {
      revname[i] = p->comm[len-i-1];
    }

  /*
  ** The name is sent in 4-byte chunks
  */
  len >>= 2;
  tname = (unsigned long *)revname;
  for( i = 0; i < len; i++ )
    {
      writel( *tname++, ct_data_port_ptr );
    }

  writel( p->pid, ct_data_port_ptr );
  writel( tag | CT_TASK_NAMES | ((len-1) & 0xffff),
	  ct_ctrl_port_ptr );
#endif
  int count;
  /* i = 0 changed 5/19/99 njh */
  int i = 1;
  union
  {
    unsigned char c[4];
    unsigned long l;
  } buffer;
  char* task_name = p->comm;

  /*
  ** If enabled, check to see if the tag pointers have been initialized
  */

  if( ((ct_hooks_enabled == 0)&&(ct_force_hwic_enabled == 0)) ||
      ((ct_ctrl_port_ptr == NULL) && (ct_init() == -1)) )
    {
      return;
    }

  while ( *task_name && ( i <= 32 )) task_name++, i++;

  /* Clear the tag buffer */
  buffer.l=0;

  /* Write out the rest of the string */
  switch (i%4)
    {
#ifdef _LITTLE_ENDIAN
    case 3:
      buffer.c[1] = *task_name--;
      i--;
      /* fall through */
    case 2:
      buffer.c[2] = *task_name--;
      i--;
      /* fall through */
    case 1:
      buffer.c[3] = *task_name--;
      i--;
      /* fall through */
#else
    case 3:
      buffer.c[2] = *task_name--;
      i--;
      /* fall through */
    case 2:
      buffer.c[1] = *task_name--;
      i--;
      /* fall through */
    case 1:
      buffer.c[0] = *task_name--;
      i--;
      /* fall through */
#endif
    }
  *ct_data_port_ptr = buffer.l;
  count = 0;

  /* Write out the the four byte blocks */

  while (i)
    {
#ifdef _LITTLE_ENDIAN
      buffer.c[0] = *task_name--;
      buffer.c[1] = *task_name--;
      buffer.c[2] = *task_name--;
      buffer.c[3] = *task_name--;
#else
      buffer.c[3] = *task_name--;
      buffer.c[2] = *task_name--;
      buffer.c[1] = *task_name--;
      buffer.c[0] = *task_name--;
#endif
      *ct_data_port_ptr = buffer.l;
      count++;
      i-=4;
    }
  *ct_data_port_ptr = p->pid;
  *ct_ctrl_port_ptr = tag | CT_TASK_NAMES | (count & 0xffff);
}



/*
-------------------------------------------------------------------------------|
|
| Accessor functions
|
-------------------------------------------------------------------------------|
*/


void ct_enable_hooks(void)
{
  ct_hooks_enabled = 1;
}


void ct_disable_hooks(void)
{
  ct_hooks_enabled = 0;
}


unsigned long ct_get_phys_addr(void)
{
  return ct_phys_addr;
}


void ct_set_phys_addr( unsigned long addr )
{
  ct_phys_addr = addr;
}

/* ---------------------------------------------------*

   Function:	ctCreateHook( const char* task_name, const u32 tId )

   Arguments:	task_name: The name of the task that is being created.
   		tId:	   The internal pid or task id.

*/
void ctCreateHook( const char* task_name, const u32 tId )
{
  u32 count;
  u32 tagBuf[9];
    
  count = ct_encode_taskname( task_name, tagBuf );
  tagBuf[count] = tId;
  ctTaskTags( count + 1, tagBuf, 
	      (CT_TASK_CREATE) | 
	      (CT_TASK_NAMES) |
	      ((count - 0x01)&0xffff) );
    
  return;
}

/* ---------------------------------------------------*

   Function: 	ctDeleteHook( const char* task_name, const u32 tId )

   Arguments: 	task_name: The name of the task that is being deleted.
   		tId:	   The internal pid or task id.

*/
void ctDeleteHook( const char* task_name, const u32 tId )
{
  u32 count;
  u32 tagBuf[9];
    
  count = ct_encode_taskname( task_name, tagBuf );
  tagBuf[count] = tId;
  ctTaskTags( count + 1, tagBuf, 
	      (CT_TASK_DELETE) |
	      (CT_TASK_NAMES) |
	      ((count - 0x01)&0xffff) );
    
  return;
}

/* ---------------------------------------------------*

   Function:	ctExitHook( const char* old_task_name, const u32 oldId) 

   Arguments:	old_task_name:	The name of the task that is being switched
   				from.
   		oldId:		The internal pid of the task that is being
				switched from.
*/
void ctExitHook( const char* old_task_name, const u32 oldId )
{
  u32 count;
  u32 tagBuf[9];
    
  count = ct_encode_taskname( old_task_name, tagBuf );
  tagBuf[count] = oldId;
  ctTaskTags( count + 1, tagBuf, 
	      (CT_TASK_EXIT) |
	      (CT_TASK_NAMES) |
	      ((count - 0x01)&0xffff) );

  return;
}

/* ---------------------------------------------------*

   Function:	ctEnterHook( const char* new_task_name, const u32 newId )

   Arguments:	new_task_name:	The name of the task that is being switched to.
		newId:		The internal pid of the task that is being
				switched to.
*/
void ctEnterHook(const char* new_task_name, const u32 newId )
{
  u32 count;
  u32 tagBuf[9];
        
  count = ct_encode_taskname( new_task_name, tagBuf );
  tagBuf[count] = newId;
  ctTaskTags( count + 1, tagBuf,
	      (CT_TASK_ENTER) |
	      (CT_TASK_NAMES) |
	      ((count - 0x01)&0xffff) );
    
  return;
}
/*
 This function is called to insert a Kernel event tags in the ring buffer
 Arguments: 	n_dtags - number of data tags for this event
		dtags 	- the data tags vector
		tag	- the control tag for this event    
*/
void ctTaskTags(unsigned n_dtags, const u32 dtags[], u32 tag)
{
  struct Hook_Buffer* hb = ct_hook_buffer ;
  u32* pdtags = (u32*)dtags;
  Q_Tag* qp ;
  /*unsigned long x;*/    
    
  task_tag_count++ ; /* statistic gathering */
  /* 
     hb should not be null, but this may be a kernal task
     so we are being extra careful
  */
  if (hb) {
    int n_tags = n_dtags + 1 ;
    /* compute number of empty slots in the ring buffer */
    int slots = ((hb->head-hb->tail) & (HOOK_BUFFER_SIZE-1)) ;

    if (slots == 0) {
      slots = HOOK_BUFFER_SIZE-1 ;
    }
    else
      slots-- ;
      
    /* 
       Disable hardware interrupts while writing tags to the hook buffer. Otherwise, with the 
       ISR hooks enabled, task and ISR tags could be interleaved.  
    */
    /*local_irq_save(x);*/

    if (slots < n_tags) {
      /* not enough room, flush all but first tag as its
	 not safe to mess with tag at head */
      int x = hb->head ;
      while(hb->buffer[x].a_time == HI_BIT_ONLY) {
	/* skip over data tags */
	HOOK_INC(x) ;
      }
      HOOK_INC(x) ; /* keep this control tag */
      hb->tail = x ;
      
      /* 
	 Insert a TPP OVERRUN tag to mark the drop of the full buffer
      */
      qp = &hb->buffer[hb->tail] ;
      qp->tag =  CT_TPP_OVERRUN_TAG; /* the control tag */
      qp->a_time = ctTagClock() & ~HI_BIT_ONLY ;

      HOOK_INC(hb->tail) ;
      
      dropped++ ;  /* stats */
    }

    {
      while(n_dtags > 0) 
	{
	  qp = &hb->buffer[hb->tail] ;
	  qp->tag = *pdtags ;
	  qp->a_time = HI_BIT_ONLY ;

	  HOOK_INC(hb->tail) ;
	  pdtags++ ;
	  n_dtags-- ;
	}
      qp = &hb->buffer[hb->tail] ;
      qp->tag = tag ; /* the control tag */
      qp->a_time = ctTagClock() & ~HI_BIT_ONLY ;

      HOOK_INC(hb->tail) ;
    }
    /* 
       Reenable hardware interrupts 
    */
    /*local_irq_restore(x);*/
  }
}

/* -------------------------------------------
int ct_encode_taskname(const char* name, u32 buffer[8])

 packs name into buffer as required for output as codetest tags.
 (TBD, find and reference appropriate doc)
 
  name is truncated to 32 chars.
  
   return value is number of words written into buffer[]
   
	
------------------------------------------------------- */


int ct_encode_taskname(const char* name, u32 buffer[8])
{
  size_t len ;
  int ret ; /* return value, number of words written into buffer */
  int index ; /* index for buffer [] */
  const char* p ;  /* walks name backwards in multiples of 4 */
	
  len = strlen(name) ;
  if (len > 31) len = 31 ;  /* truncate if name is too long */
  ret = (len+3)/4 ;
	
  p = name + (ret-1)*4;
	
  /* The first word is special as it is null padded if
     len is not a multiple of 4 */
	
  buffer[0] = 0 ;
  switch(len%4) {
  case 0 :
    p += 4 ;
    ret += 1 ;
    break ;
  case 3 :
    buffer[0] += p[2] << 8 ;
  case 2 :
    buffer[0] += p[1] << 16 ;
  case 1 :
    buffer[0] += p[0] << 24 ;
  }
  p -= 4 ;
	
  for(index = 1; index < ret ; index++, p -= 4) {
    buffer[index] = (p[0] << 24) + (p[1] << 16)
      + (p[2] << 8) + p[3] ;
  }
	
  return ret ;
}

#if defined(ARCH_X86)

static U64 rawClock(void)	/* Linux version */
{
    register U64 result asm ("eax");  /* this works, placing upper bits in edx! */
    asm( "rdtsc") ; /* long long value in edx:eax */
    return result;
}


/* 
Only 31 bits of the clock are used in a CodeTEST tag.
The absolute time to delta time computation in swtpp.c{.w},
assumes two adjacent tags have absolute times with difference less than
2^31, so it is desirable that the clock not roll over quickly.
Shifting the 64 bit clock by 11, will give a clock rate of
about 2^20 ticks per second on a 2GHz cpu. So, the 31 bit clock will
roll over in 34 minutes.
*/

#define CLOCKSHIFT   11

u32 ctTagClock(void) 
{
    return (rawClock()) >> CLOCKSHIFT;
}


#elif defined(ARCH_PPC)


static u32 rawClock(void) 
{
    register unsigned long low  asm("%r3");
    register unsigned long high asm("%r4");

    asm("mftb %r3 ");	/* Return value for PPC */
    asm("mftbu %r4");	/* Return upper value for PPC */

    return low & 0x7fffffff;
}

u32 ctTagClock(void) 
{
    return rawClock(); 
}
#endif

/*
-------------------------------------------------------------------------------|
|
| Thread event hooks
|
-------------------------------------------------------------------------------|
*/

/*
    These functions are called from kernel for task context changes and ISR events
*/
void ct_thread_create( struct task_struct * p )
{
  if( ct_hooks_enabled || ct_force_hwic_enabled ) 
    ct_thread_event( CT_TASK_CREATE, p );
    
  if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0))
    ctCreateHook(p->comm, p->pid);
}

void ct_thread_delete( struct task_struct * p )
{
  deleted_thread_id = p->pid;    
  
  if( ct_hooks_enabled || ct_force_hwic_enabled ) 
    ct_thread_event( CT_TASK_DELETE, p );
    
  if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0))
    ctDeleteHook(p->comm, p->pid);
}

void ct_thread_enter( struct task_struct * next )
{
  if( ct_hooks_enabled || ct_force_hwic_enabled ) 
    ct_thread_event( CT_TASK_ENTER, next );
    
  if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0))
    ctEnterHook(next->comm, next->pid);
}

void ct_thread_exit( struct task_struct * prev )
{
  /*
  ** If this exit comes after the thread have been deleted ignore this tag  
  ** This algorithm assumes that after a delete event always comes comes an 
  ** exit event for the deleted thread
  */
  if (prev->pid != deleted_thread_id) {
    if( ct_hooks_enabled || ct_force_hwic_enabled ) 
       ct_thread_event( CT_TASK_EXIT, prev );
    
    if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0))
       ctExitHook(prev->comm, prev->pid);
  }
  else{
    /* ignore the exit event */
    deleted_thread_id = -1;
  }
}


/*
-------------------------------------------------------------------------------|
|
| ISR hooks
|
-------------------------------------------------------------------------------|
*/
void ct_isr_enter( int irq )
{
  if( irq == 0 ) return;
  if( ct_hooks_enabled || ct_force_hwic_enabled ) 
    {
      /*
      ** Check to see if the tag pointers have been initialized
      */
      /*
      ** Initialization call removed because, on some xscale platforms,
      ** interrupts start before virtual memory has been initialized.
      ** JCG 12/2/02
      **
      if( ct_ctrl_port_ptr == NULL && ct_init() == -1 )
      */
      if( ct_ctrl_port_ptr == NULL )
	{
	  return;
	}

      *ct_ctrl_port_ptr = CT_ISR_ENTER | (irq & 0xffff);
    }
  if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0) && (ct_filter_events == 0))
    {
      ctTaskTags(0,NULL,(CT_ISR_ENTER)|(irq & 0xffff));
    }
      
}


void ct_isr_exit( int irq )
{
  if( irq == 0 ) return;
  
  if( ct_hooks_enabled || ct_force_hwic_enabled )      
    {
      /*
      ** Check to see if the tag pointers have been initialized
      */
      /*
      ** Initialization call removed because interrupts start before
      ** virtual memory has been initialized on some platforms
      ** JCG 12/2/02
      **
      if( ct_ctrl_port_ptr == NULL && ct_init() == -1 )
      */
      if( ct_ctrl_port_ptr == NULL )
	{
	  return;
	}

      *ct_ctrl_port_ptr = CT_ISR_EXIT | (irq & 0xffff);
    }
  if ((ct_hook_buffer != NULL) && (ct_swic_enabled != 0) && (ct_filter_events == 0))
    {
      ctTaskTags(0, NULL, (CT_ISR_EXIT)|(irq & 0xffff));
    }          
}
