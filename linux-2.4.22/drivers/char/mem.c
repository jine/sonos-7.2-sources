/*
 *  linux/drivers/char/mem.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Added devfs support. 
 *    Jan-11-1998, C. Scott Ananian <cananian@alumni.princeton.edu>
 *  Shared /dev/zero mmaping support, Feb 2000, Kanoj Sarcar <kanoj@sgi.com>
 */

#include <linux/config.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/tpqic02.h>
#include <linux/ftape.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/module.h>
#ifdef CONFIG_SH_SONOS_HH
#include <linux/rincon_pm.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#endif

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgalloc.h>
#ifdef CONFIG_SH_SONOS_HH
#include <asm/machvec.h>
#include <asm/rtc.h>
#include <linux/delay.h>
#endif

#ifdef CONFIG_I2C
extern int i2c_init_all(void);
#endif
#ifdef CONFIG_FB
extern void fbmem_init(void);
#endif
#ifdef CONFIG_PROM_CONSOLE
extern void prom_con_init(void);
#endif
#ifdef CONFIG_MDA_CONSOLE
extern void mda_console_init(void);
#endif
#if defined(CONFIG_S390_TAPE) && defined(CONFIG_S390_TAPE_CHAR)
extern void tapechar_init(void);
#endif
     
static ssize_t do_write_mem(struct file * file, void *p, unsigned long realp,
			    const char * buf, size_t count, loff_t *ppos)
{
	ssize_t written;

	written = 0;
#if defined(__sparc__) || defined(__mc68000__)
	/* we don't have page 0 mapped on sparc and m68k.. */
	if (realp < PAGE_SIZE) {
		unsigned long sz = PAGE_SIZE-realp;
		if (sz > count) sz = count; 
		/* Hmm. Do something? */
		buf+=sz;
		p+=sz;
		count-=sz;
		written+=sz;
	}
#endif
	if (copy_from_user(p, buf, count))
		return -EFAULT;
	written += count;
	*ppos += written;
	return written;
}


/*
 * This funcion reads the *physical* memory. The f_pos points directly to the 
 * memory location. 
 */
static ssize_t read_mem(struct file * file, char * buf,
			size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned long end_mem;
	ssize_t read;
	
	end_mem = __pa(high_memory);
	if (p >= end_mem)
		return 0;
	if (count > end_mem - p)
		count = end_mem - p;
	read = 0;
#if defined(__sparc__) || defined(__mc68000__)
	/* we don't have page 0 mapped on sparc and m68k.. */
	if (p < PAGE_SIZE) {
		unsigned long sz = PAGE_SIZE-p;
		if (sz > count) 
			sz = count; 
		if (sz > 0) {
			if (clear_user(buf, sz))
				return -EFAULT;
			buf += sz; 
			p += sz; 
			count -= sz; 
			read += sz; 
		}
	}
#endif
	if (copy_to_user(buf, __va(p), count))
		return -EFAULT;
	read += count;
	*ppos += read;
	return read;
}

static ssize_t write_mem(struct file * file, const char * buf, 
			 size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned long end_mem;

	end_mem = __pa(high_memory);
	if (p >= end_mem)
		return 0;
	if (count > end_mem - p)
		count = end_mem - p;
	return do_write_mem(file, __va(p), p, buf, count, ppos);
}

#ifndef pgprot_noncached

/*
 * This should probably be per-architecture in <asm/pgtable.h>
 */
static inline pgprot_t pgprot_noncached(pgprot_t _prot)
{
	unsigned long prot = pgprot_val(_prot);

#if defined(__i386__) || defined(__x86_64__)
	/* On PPro and successors, PCD alone doesn't always mean 
	    uncached because of interactions with the MTRRs. PCD | PWT
	    means definitely uncached. */ 
	if (boot_cpu_data.x86 > 3)
		prot |= _PAGE_PCD | _PAGE_PWT;
#elif defined(__powerpc__)
	prot |= _PAGE_NO_CACHE | _PAGE_GUARDED;
#elif defined(__mc68000__)
#ifdef SUN3_PAGE_NOCACHE
	if (MMU_IS_SUN3)
		prot |= SUN3_PAGE_NOCACHE;
	else
#endif
	if (MMU_IS_851 || MMU_IS_030)
		prot |= _PAGE_NOCACHE030;
	/* Use no-cache mode, serialized */
	else if (MMU_IS_040 || MMU_IS_060)
		prot = (prot & _CACHEMASK040) | _PAGE_NOCACHE_S;
#endif

	return __pgprot(prot);
}

#endif /* !pgprot_noncached */

/*
 * Architectures vary in how they handle caching for addresses 
 * outside of main memory.
 */
static inline int noncached_address(unsigned long addr)
{
#if defined(__i386__)
	/* 
	 * On the PPro and successors, the MTRRs are used to set
	 * memory types for physical addresses outside main memory, 
	 * so blindly setting PCD or PWT on those pages is wrong.
	 * For Pentiums and earlier, the surround logic should disable 
	 * caching for the high addresses through the KEN pin, but
	 * we maintain the tradition of paranoia in this code.
	 */
 	return !( test_bit(X86_FEATURE_MTRR, &boot_cpu_data.x86_capability) ||
		  test_bit(X86_FEATURE_K6_MTRR, &boot_cpu_data.x86_capability) ||
		  test_bit(X86_FEATURE_CYRIX_ARR, &boot_cpu_data.x86_capability) ||
		  test_bit(X86_FEATURE_CENTAUR_MCR, &boot_cpu_data.x86_capability) )
	  && addr >= __pa(high_memory);
#else
	return addr >= __pa(high_memory);
#endif
}

static int mmap_mem(struct file * file, struct vm_area_struct * vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	/*
	 * Accessing memory above the top the kernel knows about or
	 * through a file pointer that was marked O_SYNC will be
	 * done non-cached.
	 */
	if (noncached_address(offset) || (file->f_flags & O_SYNC))
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Don't try to swap out physical pages.. */
	vma->vm_flags |= VM_RESERVED;

	/*
	 * Don't dump addresses that are not real memory to a core file.
	 */
	if (offset >= __pa(high_memory) || (file->f_flags & O_SYNC))
		vma->vm_flags |= VM_IO;

	if (remap_page_range(vma->vm_start, offset, vma->vm_end-vma->vm_start,
			     vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

/*
 * This function reads the *virtual* memory as seen by the kernel.
 */
static ssize_t read_kmem(struct file *file, char *buf, 
			 size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	ssize_t read = 0;
	ssize_t virtr = 0;
	char * kbuf; /* k-addr because vread() takes vmlist_lock rwlock */
		
	if (p < (unsigned long) high_memory) {
		read = count;
		if (count > (unsigned long) high_memory - p)
			read = (unsigned long) high_memory - p;

#if defined(__sparc__) || defined(__mc68000__)
		/* we don't have page 0 mapped on sparc and m68k.. */
		if (p < PAGE_SIZE && read > 0) {
			size_t tmp = PAGE_SIZE - p;
			if (tmp > read) tmp = read;
			if (clear_user(buf, tmp))
				return -EFAULT;
			buf += tmp;
			p += tmp;
			read -= tmp;
			count -= tmp;
		}
#endif
		if (copy_to_user(buf, (char *)p, read))
			return -EFAULT;
		p += read;
		buf += read;
		count -= read;
	}

	if (count > 0) {
		kbuf = (char *)__get_free_page(GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		while (count > 0) {
			int len = count;

			if (len > PAGE_SIZE)
				len = PAGE_SIZE;
			len = vread(kbuf, (char *)p, len);
			if (!len)
				break;
			if (copy_to_user(buf, kbuf, len)) {
				free_page((unsigned long)kbuf);
				return -EFAULT;
			}
			count -= len;
			buf += len;
			virtr += len;
			p += len;
		}
		free_page((unsigned long)kbuf);
	}
 	*ppos = p;
 	return virtr + read;
}

extern long vwrite(char *buf, char *addr, unsigned long count);

/*
 * This function writes to the *virtual* memory as seen by the kernel.
 */
static ssize_t write_kmem(struct file * file, const char * buf, 
			  size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	ssize_t wrote = 0;
	ssize_t virtr = 0;
	char * kbuf; /* k-addr because vwrite() takes vmlist_lock rwlock */

	if (p < (unsigned long) high_memory) {
		wrote = count;
		if (count > (unsigned long) high_memory - p)
			wrote = (unsigned long) high_memory - p;

		wrote = do_write_mem(file, (void*)p, p, buf, wrote, ppos);

		p += wrote;
		buf += wrote;
		count -= wrote;
	}

	if (count > 0) {
		kbuf = (char *)__get_free_page(GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		while (count > 0) {
			int len = count;

			if (len > PAGE_SIZE)
				len = PAGE_SIZE;
			if (len && copy_from_user(kbuf, buf, len)) {
				free_page((unsigned long)kbuf);
				return -EFAULT;
			}
			len = vwrite(kbuf, (char *)p, len);
			count -= len;
			buf += len;
			virtr += len;
			p += len;
		}
		free_page((unsigned long)kbuf);
	}

 	*ppos = p;
 	return virtr + wrote;
}

#if defined(CONFIG_ISA) || !defined(__mc68000__)
static ssize_t read_port(struct file * file, char * buf,
			 size_t count, loff_t *ppos)
{
	unsigned long i = *ppos;
	char *tmp = buf;

	if (verify_area(VERIFY_WRITE,buf,count))
		return -EFAULT; 
	while (count-- > 0 && i < 65536) {
		if (__put_user(inb(i),tmp) < 0) 
			return -EFAULT;  
		i++;
		tmp++;
	}
	*ppos = i;
	return tmp-buf;
}

static ssize_t write_port(struct file * file, const char * buf,
			  size_t count, loff_t *ppos)
{
	unsigned long i = *ppos;
	const char * tmp = buf;

	if (verify_area(VERIFY_READ,buf,count))
		return -EFAULT;
	while (count-- > 0 && i < 65536) {
		char c;
		if (__get_user(c, tmp)) 
			return -EFAULT; 
		outb(c,i);
		i++;
		tmp++;
	}
	*ppos = i;
	return tmp-buf;
}
#endif

/*
 * This function reads the *virtual* memory as seen by the kernel.
 */
static ssize_t read_switch(struct file *file, char *buf, 
			 size_t count, loff_t *ppos)
{
	if (copy_to_user(buf, (char *)file->f_pos, 1))
		return -EFAULT;
 	return 1;
}

static ssize_t read_null(struct file * file, char * buf,
			 size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t write_null(struct file * file, const char * buf,
			  size_t count, loff_t *ppos)
{
	return count;
}

/*
 * For fun, we are using the MMU for this.
 */
static inline size_t read_zero_pagealigned(char * buf, size_t size)
{
	struct mm_struct *mm;
	struct vm_area_struct * vma;
	unsigned long addr=(unsigned long)buf;

	mm = current->mm;
	/* Oops, this was forgotten before. -ben */
	down_read(&mm->mmap_sem);

	/* For private mappings, just map in zero pages. */
	for (vma = find_vma(mm, addr); vma; vma = vma->vm_next) {
		unsigned long count;

		if (vma->vm_start > addr || (vma->vm_flags & VM_WRITE) == 0)
			goto out_up;
		if (vma->vm_flags & VM_SHARED)
			break;
		count = vma->vm_end - addr;
		if (count > size)
			count = size;

		zap_page_range(mm, addr, count, 0);
        	zeromap_page_range(addr, count, PAGE_COPY);

		size -= count;
		buf += count;
		addr += count;
		if (size == 0)
			goto out_up;
	}

	up_read(&mm->mmap_sem);
	
	/* The shared case is hard. Let's do the conventional zeroing. */ 
	do {
		unsigned long unwritten = clear_user(buf, PAGE_SIZE);
		if (unwritten)
			return size + unwritten - PAGE_SIZE;
		if (current->need_resched)
			schedule();
		buf += PAGE_SIZE;
		size -= PAGE_SIZE;
	} while (size);

	return size;
out_up:
	up_read(&mm->mmap_sem);
	return size;
}

static ssize_t read_zero(struct file * file, char * buf, 
			 size_t count, loff_t *ppos)
{
	unsigned long left, unwritten, written = 0;

	if (!count)
		return 0;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	left = count;

	/* do we want to be clever? Arbitrary cut-off */
	if (count >= PAGE_SIZE*4) {
		unsigned long partial;

		/* How much left of the page? */
		partial = (PAGE_SIZE-1) & -(unsigned long) buf;
		unwritten = clear_user(buf, partial);
		written = partial - unwritten;
		if (unwritten)
			goto out;
		left -= partial;
		buf += partial;
		unwritten = read_zero_pagealigned(buf, left & PAGE_MASK);
		written += (left & PAGE_MASK) - unwritten;
		if (unwritten)
			goto out;
		buf += left & PAGE_MASK;
		left &= ~PAGE_MASK;
	}
	unwritten = clear_user(buf, left);
	written += left - unwritten;
out:
	return written ? written : -EFAULT;
}

static int mmap_zero(struct file * file, struct vm_area_struct * vma)
{
	if (vma->vm_flags & VM_SHARED)
		return shmem_zero_setup(vma);
	if (zeromap_page_range(vma->vm_start, vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static ssize_t write_full(struct file * file, const char * buf,
			  size_t count, loff_t *ppos)
{
	return -ENOSPC;
}

/*
 * Special lseek() function for /dev/null and /dev/zero.  Most notably, you
 * can fopen() both devices with "a" now.  This was previously impossible.
 * -- SRB.
 */

static loff_t null_lseek(struct file * file, loff_t offset, int orig)
{
	return file->f_pos = 0;
}

/*
 * The memory devices use the full 32/64 bits of the offset, and so we cannot
 * check against negative addresses: they are ok. The return value is weird,
 * though, in that case (0).
 *
 * also note that seeking relative to the "end of file" isn't supported:
 * it has no meaning, so it returns -EINVAL.
 */
static loff_t memory_lseek(struct file * file, loff_t offset, int orig)
{
	switch (orig) {
		case 0:
			file->f_pos = offset;
			return file->f_pos;
		case 1:
			file->f_pos += offset;
			return file->f_pos;
		default:
			return -EINVAL;
	}
}

static int open_port(struct inode * inode, struct file * filp)
{
	return capable(CAP_SYS_RAWIO) ? 0 : -EPERM;
}

#ifdef CONFIG_SH_SONOS_HH
struct pmdevice *rincon_pmdevicelist=0;
extern unsigned long wall_jiffies;
extern struct timeval xtime;
void (*m16_sleep) (struct sleep_parms *)=0;
void (*m16_wake) (struct sleep_parms *)=0;
EXPORT_SYMBOL(m16_sleep);
EXPORT_SYMBOL(m16_wake);
#endif
#ifdef CONFIG_SH_SONOS_HH
extern void sh_bios_standby(void);
extern void sci_reset(void);
#define TMU_TSTR        0xffd80004      /* Byte access */
#define TMU_TCNT1	0xffd80018
#define TMU_TCR1	0xffd8001c
#define PM_CAPABLE_HH	MACH_HANDHELD_ES2
void pcibios_recover(void);
void nand_rincon_sleep(void);
void nand_rincon_wake(void);
void _cache_init(int);
static int rincon_pm_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,unsigned long arg)
{
	struct pmdevice *pmd;
	struct sleep_parms sp;
	struct timeval tv1,tv2;
	unsigned int jiffies_slept;
	sp.timeout=0;
	sp.flags=0;
	if ((!PM_CAPABLE_HH)||(!m16_sleep)) return -EOPNOTSUPP;
	switch (cmd) {
		case PMIOC_SLEEP:
			sp.timeout=0;
			sp.flags=arg;
			nand_rincon_sleep();
			pmd=rincon_pmdevicelist;
			while (pmd) {
				if (pmd->sleep) {
					pmd->sleep(pmd);
				}
				pmd=pmd->next;
			}
			if (m16_sleep) m16_sleep(&sp);
			cli();
			rtc_gettimeofday(&tv1);
#if 0
			ctrl_outw(0x06,TMU_TCR1);
			ctrl_outl(0xFFFFFFFF,TMU_TCNT1);
			ctrl_outb(2,TMU_TSTR);
			ctrl_inb(TMU_TSTR);
#endif
			_cache_init(0);
			sh_bios_standby();
			_cache_init(1);
#if 0
			st=0xFFFFFFFF-ctrl_inl(TMU_TCNT1);
#endif
			sci_reset();
			rtc_gettimeofday(&tv2);
			if ((tv2.tv_sec>xtime.tv_sec)||
			    ((tv2.tv_sec==xtime.tv_sec)&&(tv2.tv_usec>xtime.tv_usec))) {
				xtime=tv2;
			}
			if (tv2.tv_usec<tv1.tv_usec) {
				tv2.tv_sec--;
				tv2.tv_usec+=1000000;
			}
			tv2.tv_usec=tv2.tv_usec-tv1.tv_usec;
			tv2.tv_sec=tv2.tv_sec-tv1.tv_sec;
			jiffies_slept=(tv2.tv_usec*HZ)/1000000;
			jiffies_slept+=(tv2.tv_sec*HZ);
			(*(&jiffies))+=jiffies_slept;
			wall_jiffies=jiffies;
			ctrl_outb(1,TMU_TSTR);
			mark_bh(TIMER_BH);
        		if (TQ_ACTIVE(tq_timer))
                		mark_bh(TQUEUE_BH);
			sti();
			nand_rincon_wake();
			printk("Spent %ld seconds + %ld microseconds (%u jiffies) in standby\n",tv2.tv_sec,tv2.tv_usec,jiffies_slept);
			/* walltime / jiffies stuff goes here */
			if (m16_wake) m16_wake(&sp);
			pcibios_recover();
			pmd=rincon_pmdevicelist;
			while (pmd) {
				if (pmd->wake) {
					pmd->wake(pmd);
				}
				pmd=pmd->next;
			}
			return 0;
		default:
			return -EINVAL;
		}
	/*NOTREACHED*/
}
#endif
#ifdef CONFIG_SH_SONOS_HH
int rincon_pm_register(struct pmdevice *pmd)
{
	struct pmdevice *u=rincon_pmdevicelist;
	while (u) {
		if ((pmd==u)||((strcmp(pmd->name,u->name))==0)) {
			return -1;
		}
		u=u->next;
	}
	pmd->next=rincon_pmdevicelist;
	rincon_pmdevicelist=pmd;
	return 0;
}

int rincon_pm_unregister(struct pmdevice *pmd)
{
	struct pmdevice *prev;
	struct pmdevice *u;
	if (rincon_pmdevicelist==pmd) {
		rincon_pmdevicelist=rincon_pmdevicelist->next;
		pmd->next=0;
		return 0;
	}
	prev=rincon_pmdevicelist;
	u=rincon_pmdevicelist->next;
	while (u) {
		if (u==pmd) {
			prev->next=u->next;
			pmd->next=0;
			return 0;
		}
		prev=u;
		u=u->next;
	}
	return -1;
}
#else
int rincon_pm_register(struct pmdevice *pmd)
{
	return 0;
}
int rincon_pm_unregister(struct pmdevice *pmd)
{
	return 0;
}
#endif

EXPORT_SYMBOL(rincon_pm_register);
EXPORT_SYMBOL(rincon_pm_unregister);


static ssize_t kmsg_write(struct file * file, const char * buf,
			  size_t count, loff_t *ppos)
{
	char *tmp;
	ssize_t ret;

	tmp = kmalloc(count + 1, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;
	ret = -EFAULT;
	if (!copy_from_user(tmp, buf, count)) {
		tmp[count] = 0;
		ret = printk("%s", tmp);
		if (ret > count)
			/* printk can add a prefix */
			ret = count;
	}
	kfree(tmp);
	return ret;
}


struct page *kmem_vm_nopage(struct vm_area_struct *vma, unsigned long address, int write)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long kaddr;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *ptep, pte;
	struct page *page = NULL;

	/* address is user VA; convert to kernel VA of desired page */
	kaddr = (address - vma->vm_start) + offset;
	kaddr = VMALLOC_VMADDR(kaddr);

	spin_lock(&init_mm.page_table_lock);

	/* Lookup page structure for kernel VA */
	pgd = pgd_offset(&init_mm, kaddr);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		goto out;
	pmd = pmd_offset(pgd, kaddr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		goto out;
	ptep = pte_offset(pmd, kaddr);
	if (!ptep)
		goto out;
	pte = *ptep;
	if (!pte_present(pte))
		goto out;
	if (write && !pte_write(pte))
		goto out;
	page = pte_page(pte);
	if (!VALID_PAGE(page)) {
		page = NULL;
		goto out;
	}

	/* Increment reference count on page */
	get_page(page);

out:
	spin_unlock(&init_mm.page_table_lock);

	return page;
}

struct vm_operations_struct kmem_vm_ops = {
	nopage:		kmem_vm_nopage,
};

static int mmap_kmem(struct file * file, struct vm_area_struct * vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;

	/*
	 * If the user is not attempting to mmap a high memory address then
	 * the standard mmap_mem mechanism will work.  High memory addresses
	 * need special handling, as remap_page_range expects a physically-
	 * contiguous range of kernel addresses (such as obtained in kmalloc).
	 */
	if ((offset + size) < (unsigned long) high_memory)
		return mmap_mem(file, vma);

	/*
	 * Accessing memory above the top the kernel knows about or
	 * through a file pointer that was marked O_SYNC will be
	 * done non-cached.
	 */
	if (noncached_address(offset) || (file->f_flags & O_SYNC))
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Don't do anything here; "nopage" will fill the holes */
	vma->vm_ops = &kmem_vm_ops;

	/* Don't try to swap out physical pages.. */
	vma->vm_flags |= VM_RESERVED;

	/*
	 * Don't dump addresses that are not real memory to a core file.
	 */
	vma->vm_flags |= VM_IO;

	return 0;
}

#define zero_lseek	null_lseek
#define full_lseek      null_lseek
#define write_zero	write_null
#define read_full       read_zero
#define open_mem	open_port
#define open_kmem	open_mem

#define SW_ADDR0	0xB9000001
#define SW_ADDR1	0xB9000000
#define SW_ADDR2	0xB9000003
#define SW_ADDR3	0xB9000002

static struct file_operations mem_fops = {
	llseek:		memory_lseek,
	read:		read_mem,
	write:		write_mem,
	mmap:		mmap_mem,
	open:		open_mem,
};

static struct file_operations kmem_fops = {
	llseek:		memory_lseek,
	read:		read_kmem,
	write:		write_kmem,
	mmap:		mmap_kmem,
	open:		open_kmem,
};

static struct file_operations switch_fops = {
	read:		read_switch,
};

static struct file_operations null_fops = {
	llseek:		null_lseek,
	read:		read_null,
	write:		write_null,
};

#if defined(CONFIG_ISA) || !defined(__mc68000__)
static struct file_operations port_fops = {
	llseek:		memory_lseek,
	read:		read_port,
	write:		write_port,
	open:		open_port,
};
#endif

static struct file_operations zero_fops = {
	llseek:		zero_lseek,
	read:		read_zero,
	write:		write_zero,
	mmap:		mmap_zero,
};

static struct file_operations full_fops = {
	llseek:		full_lseek,
	read:		read_full,
	write:		write_full,
};

#ifdef CONFIG_SH_SONOS_HH
static struct file_operations pm_fops = {
	ioctl:		rincon_pm_ioctl,
};
#endif

static struct file_operations kmsg_fops = {
	write:		kmsg_write,
};

static int memory_open(struct inode * inode, struct file * filp)
{
	switch (MINOR(inode->i_rdev)) {
		case 1:
			filp->f_op = &mem_fops;
			break;
		case 2:
			filp->f_op = &kmem_fops;
			break;
		case 3:
			filp->f_op = &null_fops;
			break;
#if defined(CONFIG_ISA) || !defined(__mc68000__)
		case 4:
			filp->f_op = &port_fops;
			break;
#endif
		case 5:
			filp->f_op = &zero_fops;
			break;
		case 7:
			filp->f_op = &full_fops;
			break;
		case 8:
			filp->f_op = &random_fops;
			break;
		case 9:
			filp->f_op = &urandom_fops;
			break;
		case 10:
			filp->f_pos = SW_ADDR0;
			filp->f_op = &switch_fops;
			break;
		case 11:
			filp->f_pos = SW_ADDR1;
			filp->f_op = &switch_fops;
			break;
		case 12:
			filp->f_pos = SW_ADDR2;
			filp->f_op = &switch_fops;
			break;
		case 13:
			filp->f_pos = SW_ADDR3;
			filp->f_op = &switch_fops;
			break;
#ifdef CONFIG_SH_SONOS_HH
		case 15:
			filp->f_op = &pm_fops;
			break;
#endif
		case 16:
			filp->f_op = &kmsg_fops;
			break;
		default:
			return -ENXIO;
	}
	if (filp->f_op && filp->f_op->open)
		return filp->f_op->open(inode,filp);
	return 0;
}

void __init memory_devfs_register (void)
{
    /*  These are never unregistered  */
    static const struct {
	unsigned short minor;
	char *name;
	umode_t mode;
	struct file_operations *fops;
    } list[] = { /* list of minor devices */
	{1, "mem",     S_IRUSR | S_IWUSR | S_IRGRP, &mem_fops},
	{2, "kmem",    S_IRUSR | S_IWUSR | S_IRGRP, &kmem_fops},
	{3, "null",    S_IRUGO | S_IWUGO,           &null_fops},
#if defined(CONFIG_ISA) || !defined(__mc68000__)
	{4, "port",    S_IRUSR | S_IWUSR | S_IRGRP, &port_fops},
#endif
	{5, "zero",    S_IRUGO | S_IWUGO,           &zero_fops},
	{7, "full",    S_IRUGO | S_IWUGO,           &full_fops},
	{8, "random",  S_IRUGO | S_IWUSR,           &random_fops},
	{9, "urandom", S_IRUGO | S_IWUSR,           &urandom_fops},
	{10, "switch0", S_IRUGO | S_IWUSR,           &switch_fops},
	{11, "switch1", S_IRUGO | S_IWUSR,           &switch_fops},
	{12, "switch2", S_IRUGO | S_IWUSR,           &switch_fops},
	{13, "switch3", S_IRUGO | S_IWUSR,           &switch_fops},
#ifdef CONFIG_SH_SONOS_HH
	{15, "pm", S_IRUGO | S_IWUSR,                &pm_fops},
#endif
	{16, "kmsg", S_IWUSR,             &kmsg_fops},
    };
    int i;

    for (i=0; i<(sizeof(list)/sizeof(*list)); i++)
	devfs_register (NULL, list[i].name, DEVFS_FL_NONE,
			MEM_MAJOR, list[i].minor,
			list[i].mode | S_IFCHR,
			list[i].fops, NULL);
}

static struct file_operations memory_fops = {
	open:		memory_open,	/* just a selector for the real open */
};

int __init chr_dev_init(void)
{
	if (devfs_register_chrdev(MEM_MAJOR,"mem",&memory_fops))
		printk("unable to get major %d for memory devs\n", MEM_MAJOR);
	memory_devfs_register();
	rand_initialize();
#ifdef CONFIG_I2C
	i2c_init_all();
#endif
#if defined (CONFIG_FB)
	fbmem_init();
#endif
#if defined (CONFIG_PROM_CONSOLE)
	prom_con_init();
#endif
#if defined (CONFIG_MDA_CONSOLE)
	mda_console_init();
#endif
	tty_init();
#ifdef CONFIG_M68K_PRINTER
	lp_m68k_init();
#endif
	misc_init();
#if CONFIG_QIC02_TAPE
	qic02_tape_init();
#endif
#ifdef CONFIG_FTAPE
	ftape_init();
#endif
#if defined(CONFIG_S390_TAPE) && defined(CONFIG_S390_TAPE_CHAR)
	tapechar_init();
#endif
	return 0;
}

__initcall(chr_dev_init);
