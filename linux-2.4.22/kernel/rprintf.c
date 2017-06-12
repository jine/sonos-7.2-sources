#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/smp_lock.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/config.h>
#include <linux/proc_fs.h>
#include <linux/rprintf.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <asm/pci.h>

char *r_print_bgn;
char *r_print_end;
char *r_print_ptr;
char *r_phys_print_bgn;

int   r_print_enable = 0;

#define SPRINT_BUF_SIZE		1024
static char  r_print_buf[1024];			// temp sprint bfr
#define RPRINT_BUF_SIZE		4096

static
void 	rputs( char *sptr, int len )
{
	int tcnt;
	while( len != 0 ) {
		tcnt = r_print_end - r_print_ptr;
		if( tcnt > len )
			tcnt = len;

		memcpy( r_print_ptr,sptr,tcnt );
		sptr += tcnt;
		len -= tcnt;

		r_print_ptr += tcnt;
		if( r_print_ptr >= r_print_end )
			r_print_ptr = r_print_bgn;
	}
}

static int
rprintf_proc_read( char *page, char **start,off_t off, int count, int*eof, void *data )
{
	int len;
	int i = 0;
	char *tptr;
	char *sptr = r_print_ptr;
	char *eptr = r_print_ptr;

	do {
		for( tptr = sptr; *tptr != 0; tptr++ )
			if( tptr >= r_print_end )
				break;
		len = tptr - sptr;
		if( len > 0 && sptr != r_print_ptr ) {	// dont copy the first entry - its trashed
	 		memcpy( page+i, sptr, len );
			memcpy( page+i+len,"\n",1 );
			i += len + 1;			// acct for newline
		}
		sptr += len + 1;			// jump over null
		if( sptr >= r_print_end )
			sptr = r_print_bgn;
	} while( sptr != eptr );
		
        return( i );
}

static int
rprintf_proc_write(struct file *file, const char *buf,
    unsigned long count, void *data)
{
	int c;
	int rval0, rval1;
	struct timeval now;

	if( r_print_enable == 0 )
		return( 0 );

	do_gettimeofday( &now );
	rval0 = snprintf( r_print_buf,SPRINT_BUF_SIZE,"%02x%05x(%03d): ",
			(unsigned int)now.tv_sec&0xFF,(unsigned int)now.tv_usec,current->pid );
	rputs( r_print_buf,rval0 );

	rval1 = 0;
	while( count != 0 ) {

		c = min( (int)count,SPRINT_BUF_SIZE-1 );
		if( copy_from_user( r_print_buf,buf,c ) )
			return( -EFAULT );

		r_print_buf[c] = 0;
		rputs( r_print_buf,c+1 );

		buf += c;
		rval1 += c;
		count -= c;
	}
	return( rval0+rval1 );
}

static int
rprintf_proc_init( void )
{
	struct proc_dir_entry *dp;
    if( (dp = create_proc_entry( "kprintf", 0, 0 )) == 0 )
		return( -EIO );

	dp->read_proc = rprintf_proc_read;
	dp->write_proc = rprintf_proc_write;
	dp->data = 0;

	return( 0 );
}

static void
rprintf_proc_remove( void )
{
	remove_proc_entry( "kprintf",NULL );
}


void
rprintf_init()
{
	if( r_print_bgn == 0 ) {
		if( (r_print_bgn = (char *)pci_alloc_consistent( 0,RPRINT_BUF_SIZE,(dma_addr_t *)&r_phys_print_bgn)) == 0 ) {
			printk("Cant Allocated rprintf bfr: %d\n",RPRINT_BUF_SIZE );
		}
		r_print_end = r_print_bgn + RPRINT_BUF_SIZE;
		printk( "RPRINTF Virt: %x:%x Phys: %x:%x\n",(unsigned int)r_print_bgn,(unsigned int)r_print_end,
			(unsigned int)virt_to_phys(r_print_bgn), (unsigned int)virt_to_phys(r_print_end) );
	}
	r_print_ptr = r_print_bgn;
	if( rprintf_proc_init() == 0 ) {
		r_print_enable = 1;
		rprintf("Initialized");
	}
}

void
rprintf_release()
{
	rprintf_proc_remove();
	if( r_print_bgn != 0 ) {
		pci_free_consistent( 0,RPRINT_BUF_SIZE,(void *)r_print_bgn,(dma_addr_t)r_phys_print_bgn );
		r_print_bgn = 0;
	}
}

int
rprintf( const char *fmt, ... )
{
	va_list args;
	int rval0, rval1;
	struct timeval now;

	if( r_print_enable == 0 )
		return( 0 );

	do_gettimeofday( &now );
	rval0 = snprintf( r_print_buf,SPRINT_BUF_SIZE,"%02x%05x(%03d): ",(unsigned int)now.tv_sec&0xFF,
		(unsigned int)now.tv_usec,current->pid );
	rputs( r_print_buf,rval0 );

	va_start( args,fmt );
	rval1 = vsnprintf( r_print_buf,SPRINT_BUF_SIZE,fmt,args ) + 1;
	va_end( args );
	rputs( r_print_buf,rval1 );

	return( rval0+rval1 );
}
